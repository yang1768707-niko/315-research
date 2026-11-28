/* Compile the real NOR driver and vibration store against a stateful NOR/HAL.
   DMA retains the real source pointer until internal programming completes;
   every intervening HAL call verifies that the producer did not overwrite it. */
#include "qspi_nor.h"
#include "zigbee_vibration_store.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

TestRcc test_rcc;
TestDwt test_dwt;
TestCoreDebug test_debug;
uint32_t SystemCoreClock=64000000U;
QSPI_HandleTypeDef hqspi={.hdma=(void *)1};
static uint8_t flash[8U*1024U*1024U],status_reg=0x40,config2=2;
static QSPI_CommandTypeDef cmd,dma_cmd;
static const uint8_t *dma_source;
static uint8_t dma_copy[256];
static unsigned dma_stage,asleep,cs_low,events,quad_reads,sleep_fail,qe_fail,dma_fail;
static unsigned controller_state,wake_pulses;
static uint32_t tick;
static void unchanged(void) {
  if(dma_source)assert(!memcmp(dma_source,dma_copy,dma_cmd.NbData));
}
uint32_t HAL_GetTick(void){return tick;}
void HAL_Delay(uint32_t ms){tick+=ms;test_dwt.CYCCNT+=ms*64000U;}
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint32_t pins,uint32_t value){
  unchanged();
  if(port==GPIOA&&(pins&GPIO_PIN_2)) {
    if(value==GPIO_PIN_RESET)cs_low=1;
    else if(cs_low){asleep=0;cs_low=0;wake_pulses++;}
  }
}
void HAL_GPIO_Init(GPIO_TypeDef *port,const GPIO_InitTypeDef *gpio){(void)port;(void)gpio;unchanged();}
HAL_StatusTypeDef HAL_QSPI_Init(QSPI_HandleTypeDef *h){assert(h==&hqspi);controller_state=0;return HAL_OK;}
unsigned HAL_QSPI_GetState(QSPI_HandleTypeDef *h){assert(h==&hqspi);return controller_state;}
HAL_StatusTypeDef HAL_QSPI_Command(QSPI_HandleTypeDef *h,QSPI_CommandTypeDef *c,uint32_t timeout){
  (void)timeout;assert(h==&hqspi);unchanged();cmd=*c;
  assert(c->InstructionMode==QSPI_INSTRUCTION_1_LINE);
  if(asleep)return HAL_ERROR;
  switch(c->Instruction) {
    case 0x06:status_reg|=2;break;
    case 0x20:
      assert(status_reg&2);assert(c->Address+4096<=sizeof(flash));
      memset(flash+c->Address,0xff,4096);status_reg&=~2U;tick+=3;break;
    case 0xb9:
      assert(!dma_source);asleep=1;
      if(sleep_fail){sleep_fail=0;controller_state=HAL_QSPI_STATE_ERROR;return HAL_TIMEOUT;}
      break;
    case 0x6b:
      assert(c->AddressMode==QSPI_ADDRESS_1_LINE&&c->DataMode==QSPI_DATA_4_LINES&&c->DummyCycles==8);
      assert(status_reg&0x40);quad_reads++;break;
    case 0x03:assert(!"unexpected single-line array read");break;
    default:break;
  }
  return HAL_OK;
}
HAL_StatusTypeDef HAL_QSPI_Receive(QSPI_HandleTypeDef *h,uint8_t *out,uint32_t timeout){
  (void)timeout;assert(h==&hqspi);assert(!asleep);unchanged();
  switch(cmd.Instruction) {
    case 0x9f:out[0]=0xc2;out[1]=0x28;out[2]=0x17;break;
    case 0x05:out[0]=status_reg;break;
    case 0x15:out[0]=0;out[1]=config2;break;
    case 0x6b:assert(cmd.Address+cmd.NbData<=sizeof(flash));memcpy(out,flash+cmd.Address,cmd.NbData);break;
    default:assert(!"unexpected read opcode");
  }
  return HAL_OK;
}
static void program(const QSPI_CommandTypeDef *c,const uint8_t *data){
  assert(c->Instruction==0x38&&c->AddressMode==4&&c->DataMode==4);
  assert(c->NbData<=256-c->Address%256&&c->Address+c->NbData<=sizeof(flash));
  for(unsigned i=0;i<c->NbData;i++)flash[c->Address+i]&=data[i];
  status_reg&=~3U;
}
HAL_StatusTypeDef HAL_QSPI_Transmit(QSPI_HandleTypeDef *h,uint8_t *data,uint32_t timeout){
  (void)timeout;assert(h==&hqspi);unchanged();assert(status_reg&2);
  if(cmd.Instruction==1){status_reg=qe_fail?0:data[0];config2=data[2];}
  else program(&cmd,data);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_QSPI_AutoPolling(QSPI_HandleTypeDef *h,QSPI_CommandTypeDef *c,QSPI_AutoPollingTypeDef *p,uint32_t timeout){
  (void)c;(void)timeout;assert(h==&hqspi);unchanged();assert((status_reg&p->Mask)==p->Match);return HAL_OK;
}
HAL_StatusTypeDef HAL_QSPI_Transmit_DMA(QSPI_HandleTypeDef *h,uint8_t *data){
  assert(h==&hqspi&&!dma_source);assert(status_reg&2);
  if(dma_fail){dma_fail=0;return HAL_ERROR;}
  dma_cmd=cmd;dma_source=data;memcpy(dma_copy,data,cmd.NbData);dma_stage=1;status_reg|=1;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_QSPI_AutoPolling_IT(QSPI_HandleTypeDef *h,QSPI_CommandTypeDef *c,QSPI_AutoPollingTypeDef *p){
  assert(h==&hqspi&&c->Instruction==5&&p->Mask==1&&p->Match==0);
  unchanged();assert(dma_stage==2);dma_stage=3;return HAL_OK;
}
HAL_StatusTypeDef HAL_QSPI_Abort(QSPI_HandleTypeDef *h){assert(h==&hqspi);unchanged();dma_source=NULL;dma_stage=0;status_reg&=~3U;return HAL_OK;}
void QspiNor_ProgramEvent(void){events++;}
static void finish_tx(void){
  assert(dma_stage==1);unchanged();dma_stage=2;HAL_Delay(1);HAL_QSPI_TxCpltCallback(&hqspi);
}
static void finish_wip(void){
  assert(dma_stage==3);unchanged();HAL_Delay(1);program(&dma_cmd,dma_source);
  dma_source=NULL;dma_stage=0;HAL_QSPI_StatusMatchCallback(&hqspi);
}
static void progress(void){
  if(dma_stage==1)finish_tx();
  assert(ZigbeeVibrationStore_Service(0)!=HAL_ERROR);
  if(dma_stage==3)finish_wip();
  assert(ZigbeeVibrationStore_Service(0)!=HAL_ERROR);
}
static void test_driver(void){
  uint32_t id,capacity;uint8_t out[3072],data[256];memset(data,0x5a,sizeof(data));
  assert(QspiNor_Init(&id,&capacity)==HAL_OK&&id==0xc22817&&capacity==sizeof(flash));
  assert(QspiNor_Write(0,data,256)==HAL_OK);
  for(unsigned size=1;size<=3072;size=size==1?4:size==4?127:size==127?128:size==128?3072:3073)
    assert(QspiNor_Read(0,out,size)==HAL_OK);
  assert(!memcmp(out,data,256));
  assert(QspiNor_Read(capacity-1,out,2)==HAL_ERROR);
  assert(QspiNor_ProgramPageAsync(256,data,256)==HAL_OK);
  assert(QspiNor_Read(0,out,4)==HAL_BUSY&&QspiNor_Write(0,data,4)==HAL_BUSY);
  assert(QspiNor_EraseRange(4096,4096)==HAL_BUSY&&QspiNor_PrepareStop2()==HAL_BUSY);
  unsigned before=events;finish_tx();assert(events==before+1);
  assert(QspiNor_PrepareStop2()==HAL_BUSY&&QspiNor_ProgramService()==HAL_BUSY);
  assert(QspiNor_PrepareStop2()==HAL_BUSY);finish_wip();
  assert(QspiNor_PrepareStop2()==HAL_BUSY);assert(QspiNor_ProgramService()==HAL_OK);
  assert(QspiNor_Read(256,out,256)==HAL_OK&&!memcmp(out,data,256));
  sleep_fail=1;assert(QspiNor_PrepareStop2()==HAL_TIMEOUT&&asleep);
  unsigned wakes=wake_pulses;
  assert(QspiNor_PrepareStop2()==HAL_OK&&asleep&&wake_pulses==wakes+1);
  assert(QspiNor_PrepareStop2()==HAL_OK);assert(QspiNor_Read(256,out,4)==HAL_OK&&!asleep);
  assert(!memcmp(out,data,4));
  assert(QspiNor_PrepareStop2()==HAL_OK&&QspiNor_Init(&id,&capacity)==HAL_OK&&!asleep);
  status_reg=0;config2=0;qe_fail=1;
  assert(QspiNor_Init(NULL,NULL)==HAL_ERROR);
  assert(QspiNor_Read(0,out,4)==HAL_ERROR);qe_fail=0;
  assert(QspiNor_Init(NULL,NULL)==HAL_OK);
  dma_fail=1;assert(QspiNor_ProgramPageAsync(512,data,256)==HAL_ERROR);
  assert(QspiNor_Read(0,out,4)==HAL_OK);
}
static void test_capture(uint16_t window,uint32_t batch){
  const uint32_t count=(26667U*window+500U)/1000U;
  int16_t (*samples)[3]=malloc(count*6U);uint8_t *readback=malloc(count*6U);
  assert(samples&&readback);
  for(uint32_t i=0;i<count;i++)for(unsigned a=0;a<3;a++)samples[i][a]=(int16_t)(i*19U+a*137U);
  assert(ZigbeeVibrationStore_Begin(26667,window,batch)==HAL_OK);
  uint32_t consumed=0;
  while(consumed<count){
    uint16_t capacity=ZigbeeVibrationStore_AsyncCapacity();
    uint16_t n=(uint16_t)(1+(consumed*13U)%42U);
    if(n>capacity)n=capacity;
    if(n>count-consumed)n=(uint16_t)(count-consumed);
    if(!n){progress();continue;}
    assert(ZigbeeVibrationStore_AppendAsync(samples+consumed,n)==HAL_OK);consumed+=n;
    unchanged();
    /* Exercise filling the other page while DMA/WIP is deliberately held. */
    if(dma_source&&ZigbeeVibrationStore_AsyncCapacity()==0){
      uint32_t accepted=ZigbeeVibrationStore_SampleCount();
      if(consumed<count)assert(ZigbeeVibrationStore_AppendAsync(samples+consumed,1)==HAL_BUSY);
      assert(ZigbeeVibrationStore_SampleCount()==accepted);progress();
    }
  }
  while(ZigbeeVibrationStore_Service(1)==HAL_BUSY)progress();
  ZigbeeVibrationWriteStats_t stats;ZigbeeVibrationStore_GetWriteStats(&stats);
  assert(stats.bytes==count*6U&&stats.pages==(count*6U+255U)/256U);
  assert(stats.partial_pages==(count*6U%256U!=0)&&stats.erase_ms>0);
  assert(ZigbeeVibrationStore_Finish(NULL)==HAL_OK);
  assert(ZigbeeVibrationStore_ReadBatch(batch,0,count,readback)==1);
  assert(!memcmp(samples,readback,count*6U));
  /* A new RAM session must recover the same committed header and waveform. */
  assert(ZigbeeVibrationStore_Init(NULL,NULL)==HAL_OK);
  assert(ZigbeeVibrationStore_ReadBatch(batch,0,count,readback)==1);
  assert(!memcmp(samples,readback,count*6U));
  assert(QspiNor_PrepareStop2()==HAL_OK&&asleep);
  assert(ZigbeeVibrationStore_ReadBatch(batch,count-1,1,readback)==1&&!asleep);
  assert(!memcmp(samples[count-1],readback,6));free(samples);free(readback);
}
int main(void){
  memset(flash,0xff,sizeof(flash));test_driver();
  memset(flash,0xff,sizeof(flash));assert(ZigbeeVibrationStore_Init(NULL,NULL)==HAL_OK);
  test_capture(500,101);test_capture(1000,102);test_capture(1500,103);
#if (QSPI_NOR_PROGRAM_TIMING == 0)
  assert(test_debug.DEMCR==0&&test_dwt.CTRL==0);
#else
  assert(test_debug.DEMCR&CoreDebug_DEMCR_TRCENA_Msk);
  assert(QspiNor_GetProgramTiming().pages>0);
#endif
  printf("PASS: quad reads (short/long), busy guards, QE failure, DP timeout/retry/wake, DMA ownership, 3 capture lengths/CRC/reload; %u read commands.\n",quad_reads);
}
