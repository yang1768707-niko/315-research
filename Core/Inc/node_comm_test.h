#ifndef NODE_COMM_TEST_H
#define NODE_COMM_TEST_H
#include "comm_test_protocol.h"

#define PENDING_LINK_RETRY_MS 3000U
#define PENDING_LINK_FAILURE_LIMIT 2U

static struct {
    uint32_t token, started;
    uint8_t active, ready, timer, automatic, peer_result;
} comm_test = {.timer = 0xFFU};

static struct {
    uint32_t deadline, generation;
    uint8_t active, ready, timer, failures;
} pending_link = {.timer = 0xFFU};

static int PendingLink_Eligible(void)
{
    return APP_ZIGBEE_IsPendingAuthorization() && !admission_expired &&
           !ZbApp.rtc_join_active && !ZbApp.restore_pending &&
           !ZbApp.restore_recovery_active;
}

static void PendingLink_Schedule(uint32_t delay)
{
    pending_link.deadline = APP_ElapsedMs() + delay;
    ZbApp_StartTimer(pending_link.timer, delay);
}

static void PendingLink_RecordSuccess(void)
{
    if (!pending_link.active || !PendingLink_Eligible()) return;
    if (comm_test.active && comm_test.automatic) {
        comm_test.active = 0U;
        HW_TS_Stop(comm_test.timer);
    }
    pending_link.failures = 0U;
    PendingLink_Schedule(monitor_standby_random(&monitor_standby, 30000U, 45000U));
}

static void PendingLink_Start(void)
{
    if (!pending_link.ready || pending_link.active || !PendingLink_Eligible()) return;
    pending_link.active = 1U;
    PendingLink_RecordSuccess();
}

static void PendingLink_Stop(void)
{
    if (!pending_link.active) return;
    pending_link.active = 0U;
    pending_link.failures = 0U;
    pending_link.generation++;
    HW_TS_Stop(pending_link.timer);
    if (comm_test.active && comm_test.automatic) {
        comm_test.active = 0U;
        HW_TS_Stop(comm_test.timer);
    }
}

static const char *CommTest_Result(unsigned status)
{
    switch (status) {
    case 0: return "success";
    case 0xe9: return "no MAC receive acknowledgement";
    case 0xa7: return "no APS receive acknowledgement";
    case 0xe1: return "channel access failed";
    default: return "stack/local error (not proof of disconnection)";
    }
}

static const char *CommTest_Label(void)
{
    return comm_test.automatic ? "PENDING-LINK" : "COMM-TEST";
}

static void CommTest_Confirm(struct ZbZclCommandRspT *response, void *arg)
{
    uint32_t token = (uint32_t)(uintptr_t)arg;
    if (token != comm_test.token) return;
    unsigned status = response ? (unsigned)response->aps_status : 0xffU;
    ZbApp_Log("COMM-TEST: token=%lu stack-confirm=%s aps=0x%02x zcl=0x%02x elapsed=%lu ms current=%u\r\n",
        (unsigned long)token, CommTest_Result(status), status,
        response ? (unsigned)response->status : 0xffU,
        (unsigned long)(uint32_t)(APP_ElapsedMs() - comm_test.started),
        (unsigned)comm_test.active);
}

static void PendingLink_Confirm(struct ZbApsdeDataConfT *confirm, void *arg)
{
    uint32_t token = (uint32_t)(uintptr_t)arg;
    if (!comm_test.active || !comm_test.automatic || token != comm_test.token) return;
    unsigned status = confirm ? (unsigned)confirm->status : 0xffU;
    /* Count a missing peer ACK or a confirmed uplink followed by no echo.
     * A local failure or unknown result supplies no peer evidence. */
    comm_test.peer_result = confirm &&
        (status == ZB_STATUS_SUCCESS || link_health_missing_ack(status));
    ZbApp_Log("PENDING-LINK: token=%lu stack-confirm=%s aps=0x%02x elapsed=%lu ms\r\n",
        (unsigned long)token, CommTest_Result(status), status,
        (unsigned long)(uint32_t)(APP_ElapsedMs() - comm_test.started));
}

static int CommTest_Ready(void)
{
    return comm_test.ready && ZbApp.zb && ZbApp.control_server &&
           coordinator_endpoint && ZbApp.join_status == ZB_STATUS_SUCCESS &&
           !ZbApp.join_blocked_until_reset;
}

static int CommTest_Busy(void)
{
    return radio_sleep.held || ZbApp.radio_exclusive || ZbApp.persist_tx_hold ||
        ZbApp.startup_wait_active ||
        ZbApp.rtc_join_active || ZbApp.restore_recovery_active || upload_health.pending ||
        ZbSamplingCapture || ZbApp.audio_waiting || ZbApp.tx_in_flight || ZbApp.tx_count ||
        v2_tx_active(&ZbVibrationUpload);
}

static int CommTest_Send(uint8_t automatic)
{
    if (++comm_test.token == 0U) ++comm_test.token;
    uint8_t payload[COMM_TEST_REQUEST_LEN];
    comm_test_request(payload, comm_test.token);
    comm_test.automatic = automatic;
    comm_test.peer_result = 0U;
    comm_test.active = 1U;
    comm_test.started = APP_ElapsedMs();
    ZbApp_Log("%s: token=%lu REQUEST to coordinator; timeout=10000 ms\r\n",
        CommTest_Label(), (unsigned long)comm_test.token);
    unsigned status;
    void *arg = (void *)(uintptr_t)comm_test.token;
    if (automatic) {
        status = ZbApp_SendPendingLinkFrame(COMM_TEST_REQUEST, payload, sizeof(payload),
            PendingLink_Confirm, arg);
    } else {
        struct ZbZclClusterCommandReqT request = {0};
        request.dst.mode = ZB_APSDE_ADDRMODE_SHORT;
        request.dst.endpoint = coordinator_endpoint;
        request.cmdId = COMM_TEST_REQUEST;
        request.payload = payload;
        request.length = sizeof(payload);
        status = ZbZclClusterCommandReq(ZbApp.control_server, &request, CommTest_Confirm, arg);
    }
    ZbApp_Log("%s: token=%lu submit=%s status=0x%02x (not end-to-end receipt)\r\n",
        CommTest_Label(), (unsigned long)comm_test.token,
        status == 0U ? "accepted" : "rejected", status);
    if (status != 0U) {
        comm_test.active = 0U;
        return 0;
    }
    if (comm_test.active) ZbApp_StartTimer(comm_test.timer, COMM_TEST_TIMEOUT_MS);
    return 1;
}

static void CommTest_Expire(void)
{
    if (!comm_test.active) return;
    uint32_t elapsed = APP_ElapsedMs() - comm_test.started;
    if (elapsed < COMM_TEST_TIMEOUT_MS) {
        ZbApp_StartTimer(comm_test.timer, COMM_TEST_TIMEOUT_MS - elapsed);
        return;
    }
    comm_test.active = 0U;
    if (!comm_test.automatic) {
        ZbApp_Log("COMM-TEST: token=%lu TIMEOUT: no matching application echo in 10000 ms; no recovery/state change requested\r\n",
            (unsigned long)comm_test.token);
        return;
    }
    if (!pending_link.active || !PendingLink_Eligible()) return;
    if (comm_test.peer_result) pending_link.failures++;
    ZbApp_Log("PENDING-LINK: token=%lu TIMEOUT: no matching application echo; counted=%u failures=%u/%u\r\n",
        (unsigned long)comm_test.token, (unsigned)comm_test.peer_result,
        (unsigned)pending_link.failures, PENDING_LINK_FAILURE_LIMIT);
    if (pending_link.failures >= PENDING_LINK_FAILURE_LIMIT) {
        ZbApp_Log("PENDING-LINK: two peer checks failed; recovering the existing network\r\n");
        ZbApp_SetParentOnline(0U, ZB_APS_STATUS_NO_ACK);
        ZbApp_StartLinkRecovery();
    } else {
        PendingLink_Schedule(monitor_standby_random(&monitor_standby, 30000U, 45000U));
    }
}

static void CommTest_Task(void)
{
    CommTest_Expire();
    if (!pending_link.active) return;
    if (!PendingLink_Eligible()) {
        PendingLink_Stop();
        return;
    }
    int32_t remaining = (int32_t)(pending_link.deadline - APP_ElapsedMs());
    if (remaining > 0) {
        ZbApp_StartTimer(pending_link.timer, (uint32_t)remaining);
        return;
    }
    if (comm_test.active || !CommTest_Ready() || CommTest_Busy() ||
        !CommTest_Send(1U)) {
        PendingLink_Schedule(PENDING_LINK_RETRY_MS);
    }
}

static void CommTest_Timer(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_COMM_TEST, CFG_SCH_PRIO_1);
}

static void CommTest_Init(uint32_t seed)
{
    comm_test.token = seed;
    UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_COMM_TEST, UTIL_SEQ_RFU, CommTest_Task);
    comm_test.ready = HW_TS_Create(CFG_TIM_PROC_ID_ZIGBEE, &comm_test.timer,
        hw_ts_SingleShot, CommTest_Timer) == hw_ts_Successful;
    if (comm_test.ready) {
        pending_link.ready = HW_TS_Create(CFG_TIM_PROC_ID_ZIGBEE, &pending_link.timer,
            hw_ts_SingleShot, CommTest_Timer) == hw_ts_Successful;
    }
    if (!pending_link.ready) ZbApp_Log("PENDING-LINK: watchdog timer unavailable\r\n");
}

void APP_ZIGBEE_CommTest(void)
{
    ZbApp_Log("COMM-TEST: state join=0x%02x parent=%u authorized=%u config=%u monitor=%u ep=%u blocked=%u\r\n",
        (unsigned)ZbApp.join_status, (unsigned)ZbApp.parent_online, (unsigned)network_authorized,
        (unsigned)ZbApp.config_valid, (unsigned)ZbApp.monitor_enabled, (unsigned)coordinator_endpoint,
        (unsigned)ZbApp.join_blocked_until_reset);
    if (comm_test.active) {
        ZbApp_Log("COMM-TEST: BUSY: previous test still pending\r\n");
    } else if (!CommTest_Ready()) {
        ZbApp_Log("COMM-TEST: NOT READY: no usable current stack/endpoint; will not join or reset\r\n");
    } else if (CommTest_Busy()) {
        ZbApp_Log("COMM-TEST: BUSY: capture/upload/recovery/persistence in progress; retry when idle\r\n");
    } else {
        (void)CommTest_Send(0U);
    }
}

static void CommTest_Reply(const uint8_t *payload, uint16_t length)
{
    if (!payload || length != COMM_TEST_REPLY_LEN || payload[0] != COMM_TEST_VERSION) return;
    uint32_t token = (uint32_t)comm_test_token(payload);
    uint32_t elapsed = APP_ElapsedMs() - comm_test.started;
    if (!comm_test.active || token != comm_test.token || elapsed >= COMM_TEST_TIMEOUT_MS) {
        ZbApp_Log("COMM-TEST: token=%lu stale/unmatched echo ignored\r\n", (unsigned long)token);
        return;
    }
    comm_test.active = 0U;
    HW_TS_Stop(comm_test.timer);
    ZbApp_Log("%s: token=%lu PASS: coordinator received request and node received echo; RTT=%lu ms RSSI=%d dBm LQI=%u\r\n",
        CommTest_Label(), (unsigned long)token, (unsigned long)elapsed,
        (int)(int8_t)payload[5], (unsigned)payload[6]);
    if (comm_test.automatic) PendingLink_RecordSuccess();
}
#endif
