#ifndef NODE_UPLOAD_TRACE_H
#define NODE_UPLOAD_TRACE_H
/* Application callbacks store metadata only. Printing the bounded history is
 * deferred to the upload task; no extra radio packet/ACK is introduced. */
#define UT_RING_SIZE 24U
#include "upload_trace.h"
static UploadTraceRing nodeUploadHistory;
static uint32_t nodeUploadLastDump;
static uint8_t nodeUploadDumped,nodeUploadHaveReply;
static uint8_t nodeUploadLastReply[17];static unsigned nodeUploadLastReplySeq;static uint32_t nodeUploadLastReplyAt;
static uint16_t nodeUploadLastReplyShort;
static const char *nodeReplyKind(unsigned kind)
{
    switch(kind){case 1:return "批次许可";case 2:return "窗口描述确认";case 3:return "窗口校验结果";
    case 4:return "整批波形确认";case 5:return "振动特征确认";case 6:return "传感器数据校验";
    case 7:return "上传许可续期";case 8:return "批次已完成";default:return "未知回复";}
}
static const char *nodeTraceResultName(const uint8_t *p)
{
    if(p[1]==UT_NODE_TX)return "发送接口调用前";
    if(p[1]==UT_NODE_RESULT){
        switch(p[11]){case 0:return "发送成功";case 0xe9:return "未收到无线接收确认";
        case 0xe1:return "信道接入失败";case 0xa7:return "未收到APS确认";default:return "其他协议栈状态";}
    }
    switch(p[11]){case 0:return "通过";case 1:return "缺片补传";case 2:return "重申批次";
    case 3:return "拒绝";case 4:return "存储失败";case 5:return "重发窗口描述";
    case 6:return "窗口已完成";case 7:return "排队等待";default:return "其他回复";}
}
static void ZbApp_TracePacket(unsigned stage,unsigned cmd,int seq,const uint8_t *p,size_t n)
{
    UploadTraceRecord r;
    if(ut_packet(&r,(uint8_t)stage,APP_ElapsedMs(),ZbShortAddress(ZbApp.zb),(uint8_t)cmd,seq,p,n)){
        if(stage==UT_NODE_TX)r.p[11]=255;
        ut_push(&nodeUploadHistory,&r);
    }
}
static void ZbApp_TraceResult(unsigned seq,unsigned status,unsigned late)
{
    for(unsigned i=0;i<nodeUploadHistory.count;i++){
        unsigned pos=(nodeUploadHistory.head+UT_RING_SIZE-1U-i)%UT_RING_SIZE;
        UploadTraceRecord r=nodeUploadHistory.records[pos];
        if(r.p[1]!=UT_NODE_TX || r.p[9]!=(uint8_t)seq)continue;
        r.p[1]=UT_NODE_RESULT;r.p[11]=(uint8_t)status;ut_w32(r.p+2,APP_ElapsedMs());
        ut_w32(r.p+36,late);ut_push(&nodeUploadHistory,&r);return;
    }
}
static void ZbApp_TraceReply(const uint8_t *p,size_t n,unsigned seq)
{
    if(n!=17U || !p || p[0]!=2U)return;
    ZbApp_TracePacket(UT_NODE_RX,0x0D,(int)seq,p,n);
    /* No UART printing in the receive callback. Preserve the latest reply
     * separately so a long fragment run cannot overwrite its evidence. */
    memcpy(nodeUploadLastReply,p,17);nodeUploadLastReplySeq=seq;
    nodeUploadLastReplyAt=APP_ElapsedMs();nodeUploadHaveReply=1;
    nodeUploadLastReplyShort=ut_u16(nodeUploadHistory.records[(nodeUploadHistory.head+UT_RING_SIZE-1U)%UT_RING_SIZE].p+6);

}
static void ZbApp_TraceDump(unsigned reason)
{
    uint32_t now=APP_ElapsedMs();
    if(nodeUploadDumped && (uint32_t)(now-nodeUploadLastDump)<UT_COOLDOWN_MS)return;
    nodeUploadDumped=1;nodeUploadLastDump=now;
    ZbApp_Log("ZIGBEE: [收发快照] reason=%u records=%u overwritten=%lu short=0x%04x; local clock, bounded history\r\n",
        reason,nodeUploadHistory.count,(unsigned long)nodeUploadHistory.overwritten,ZbShortAddress(ZbApp.zb));
    if(nodeUploadHaveReply){const uint8_t *p=nodeUploadLastReply;
        ZbApp_Log("ZIGBEE: [最近回复收到] short=0x%04x 类型=%s(%u) batch=%lu window=%u seq=%u fingerprint=%08lX result=%u detail_low=%08lX at=%lu ms; received, not proof of application\r\n",
            nodeUploadLastReplyShort,nodeReplyKind(p[1]),p[1],(unsigned long)ut_u32(p+2),
            (p[1]==2U || p[1]==3U)?(unsigned)ut_u16(p+6)+1U:0U,nodeUploadLastReplySeq,
            (unsigned long)ut_fingerprint(p,17),p[8],(unsigned long)ut_u32(p+9),(unsigned long)nodeUploadLastReplyAt);
    }
    for(unsigned i=0;i<nodeUploadHistory.count;i++){
        unsigned pos=(nodeUploadHistory.head+UT_RING_SIZE-nodeUploadHistory.count+i)%UT_RING_SIZE;
        const uint8_t *p=nodeUploadHistory.records[pos].p;
        ZbApp_Log("ZIGBEE: [收发记录] stage=%s(%u) short=0x%04x batch=%lu window=%u fragment=%u cmd=0x%02x seq=%u fingerprint=%08lX result=%s(0x%02x) record=%lu at=%lu ms late=%lu\r\n",
            p[1]==UT_NODE_TX?"提交尝试":p[1]==UT_NODE_RESULT?"发送结果":"收到回复",p[1],ut_u16(p+6),(unsigned long)ut_u32(p+12),ut_u16(p+16)==UINT16_MAX?0U:(unsigned)ut_u16(p+16)+1U,
            p[19]&UT_FRAGMENT_VALID?(unsigned)p[18]+1U:0U,p[8],p[9],(unsigned long)ut_u32(p+20),nodeTraceResultName(p),p[11],
            (unsigned long)ut_u32(p+32),(unsigned long)ut_u32(p+2),(unsigned long)ut_u32(p+36));
    }
}
#endif
