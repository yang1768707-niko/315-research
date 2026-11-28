#ifndef SENSOR_CLUSTERS_H
#define SENSOR_CLUSTERS_H
/* HY protocol v3: private IDs, ordinary ZCL headers (no assigned manufacturer
 * code is claimed). See the gateway's docs/hy-zcl-v3.md. */
#define HY_CONTROL_CLUSTER 0xFC10U
#define HY_DATA_CLUSTER 0xFC11U
#define HY_PROTOCOL_VERSION 3U
#define HY_CAPABILITIES 0x001FU
static unsigned hy_cluster_for_command(unsigned cmd)
{
    return ((cmd >= 0x86U && cmd <= 0x8CU) ||
            (cmd >= 0x92U && cmd <= 0x95U) || cmd == 0x0DU || cmd == 0xE1U)
        ? HY_DATA_CLUSTER : HY_CONTROL_CLUSTER;
}
#endif
