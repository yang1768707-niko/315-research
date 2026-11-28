#ifndef NODE_CONSOLE_H
#define NODE_CONSOLE_H
/* Interrupts only buffer bytes; parsing, printing and Zigbee run as a task. */
#include "stm32wbxx_ll_usart.h"
#include "node_console_api.h"
#include "stm32_seq.h"
#include "app_conf.h"
#include "app_zigbee.h"
static volatile uint8_t node_console_bytes[128],node_console_head,node_console_tail,node_console_corrupt;
static volatile uint32_t node_console_overrun,node_console_frame,node_console_noise,node_console_parity,node_console_ring_full;
static volatile uint8_t node_console_started;
/* USART1 RX is owned here, not by one-byte HAL receive/rearm cycles. Drain the
 * hardware FIFO before HAL services TX. No printing or CPU2 calls in this IRQ. */
uint8_t NodeConsole_UART_IRQHandler(void)
{
    if(!node_console_started)return 0U;
    /* HAL owns TX only. A new RX error arriving after the FIFO drain must not
     * make HAL abort reception and disable our RX interrupts. */
    LL_USART_DisableIT_RXNE_RXFNE(USART1);
    LL_USART_DisableIT_ERROR(USART1);
    unsigned activity=0U;
    if(LL_USART_IsActiveFlag_ORE(USART1)){LL_USART_ClearFlag_ORE(USART1);node_console_overrun++;activity=1U;node_console_corrupt=1U;}
    if(LL_USART_IsActiveFlag_FE(USART1)){LL_USART_ClearFlag_FE(USART1);node_console_frame++;activity=1U;node_console_corrupt=1U;}
    if(LL_USART_IsActiveFlag_NE(USART1)){LL_USART_ClearFlag_NE(USART1);node_console_noise++;activity=1U;node_console_corrupt=1U;}
    if(LL_USART_IsActiveFlag_PE(USART1)){LL_USART_ClearFlag_PE(USART1);node_console_parity++;activity=1U;node_console_corrupt=1U;}
    while(LL_USART_IsActiveFlag_RXNE_RXFNE(USART1)) {
        uint8_t byte=LL_USART_ReceiveData8(USART1);
        uint8_t next=(uint8_t)((node_console_head+1U)%128U);
        if(next==node_console_tail){node_console_corrupt=1U;node_console_ring_full++;}
        else {node_console_bytes[node_console_head]=byte;node_console_head=next;}
        activity=1U;
    }
    HAL_UART_IRQHandler(&huart1);
    LL_USART_EnableIT_RXNE_RXFNE(USART1);
    LL_USART_EnableIT_ERROR(USART1);
    if(activity)UTIL_SEQ_SetTask(1U<<CFG_TASK_NODE_CONSOLE,CFG_SCH_PRIO_1);
    return 1U;
}
static void NodeConsole_PrintError(const char *reason)
{
    char msg[192];int n=snprintf(msg,sizeof(msg),
        "CONSOLE: %s; UART overrun=%lu frame=%lu noise=%lu parity=%lu ring_full=%lu\r\n",reason,
        (unsigned long)node_console_overrun,(unsigned long)node_console_frame,
        (unsigned long)node_console_noise,(unsigned long)node_console_parity,(unsigned long)node_console_ring_full);
    if(n>0)(void)HAL_UART_Transmit(&huart1,(uint8_t *)msg,
        (uint16_t)((size_t)n<sizeof(msg)?(size_t)n:sizeof(msg)-1U),100U);
}
static void NodeConsole_Process(void)
{
    static char line[24];static unsigned length,discard;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    if(node_console_corrupt){discard=1U;length=0U;node_console_corrupt=0U;
        node_console_tail=node_console_head;__set_PRIMASK(mask);
        NodeConsole_PrintError("RX data lost, input discarded; send Enter then zbping + Enter");return;}
    __set_PRIMASK(mask);
    while(node_console_tail!=node_console_head) {
        mask=__get_PRIMASK();__disable_irq();
        if(node_console_corrupt){node_console_tail=node_console_head;node_console_corrupt=0U;
            discard=1U;length=0U;__set_PRIMASK(mask);
            NodeConsole_PrintError("RX data lost, input discarded; send Enter then zbping + Enter");return;}
        uint8_t ch=node_console_bytes[node_console_tail];
        node_console_tail=(uint8_t)((node_console_tail+1U)%128U);__set_PRIMASK(mask);
        if(ch=='\r' || ch=='\n') {
            if(discard)NodeConsole_PrintError("invalid/overlong line discarded; use zbping + Enter");
            else if(length){line[length]=0;
                if(!strcmp(line,"zbping"))APP_ZIGBEE_CommTest();
                else {char msg[160];int n=snprintf(msg,sizeof(msg),"CONSOLE: unknown line len=%u HEX=",length);
                    for(unsigned i=0;i<length;i++)n+=snprintf(msg+n,sizeof(msg)-(size_t)n,"%02X ",(unsigned)(uint8_t)line[i]);
                    n+=snprintf(msg+n,sizeof(msg)-(size_t)n,"; use zbping + Enter\r\n");
                    (void)HAL_UART_Transmit(&huart1,(uint8_t *)msg,(uint16_t)n,100U);}}
            length=0U;discard=0U;
        } else if(!discard) {
            if(ch>=32U && ch<127U && length<sizeof(line)-1U)line[length++]=(char)ch;
            else {discard=1U;length=0U;}
        }
    }
}
static void NodeConsole_Init(void)
{
    UTIL_SEQ_RegTask(1U<<CFG_TASK_NODE_CONSOLE,UTIL_SEQ_RFU,NodeConsole_Process);
    node_console_started=1U;
    LL_USART_EnableIT_RXNE_RXFNE(USART1);
    LL_USART_EnableIT_ERROR(USART1);
}
#endif
