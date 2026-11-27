/* 统一应用层通讯协议：宏用于定义协议常量，功能函数用于公共收发。
 * 所有线上多字节整数均为小端，禁止直接发送 C 结构体以免受对齐/填充影响。
 * 8 字节帧头：版本、类别、功能码、标志、序号 LE16、载荷长度 LE16。
 * BLE：FE82/FE92 接收，FE81 控制/标量，FE91 波形。
 * Zigbee：FC10/E0 接收及响应，FC11/E1 数据/事件。
 * A 版网络控制、可靠 V2 波形上传保留原流程，不等同于 0x81 帧。
 */
#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/* 协议版本和容量上限；COMMAND_MAX 为含帧头的命令总长。 */
#define COMM_VERSION                 1U
#define COMM_HEADER_SIZE             8U
#define COMM_FRAME_MAX               244U
#define COMM_COMMAND_MAX             14U
#define COMM_ZB_CONTROL_COMMAND      0xE0U
#define COMM_ZB_DATA_COMMAND         0xE1U
#define COMM_ZB_FRAME_MAX            82U

/* 帧类别：命令、响应、数据、事件。 */
#define COMM_KIND_COMMAND            1U
#define COMM_KIND_RESPONSE           2U
#define COMM_KIND_DATA               3U
#define COMM_KIND_EVENT              4U

/* 上位机下发命令码及载荷格式。 */
#define COMM_CMD_SENSOR_ENABLE       0x20U /* 掩码 u16 + 状态 u8 */
#define COMM_CMD_WINDOW              0x21U /* 掩码 u16 + 窗口毫秒 u32；0 为持续采集 */
#define COMM_CMD_DECIMATION          0x22U /* 抽取倍数 u16，用于后续采集 */
#define COMM_CMD_STATUS              0x23U /* 无载荷，查询状态 */
#define COMM_CMD_WORK_MODE_QUERY     0x24U /* 无载荷，查询当前/保存的 BLE 工作模式 */
#define COMM_CMD_WORK_MODE_SET       0x25U /* mode:u8，保存后复位生效 */
/* MCU 上传的数据/事件功能码。 */
#define COMM_DATA_SCALARS            0x80U /* 批次 u32 + 多个 TLV */
#define COMM_DATA_WAVE               0x81U /* 批次 u32 + 原始起点 u32 + 点数 u16 + XYZ i16 */
#define COMM_DATA_FEATURES           0x82U /* 批次 u32 + 三轴各 5 个 i32，特征放大 1000 倍 */
#define COMM_DATA_STATUS             0x83U /* 状态 u8 + 掩码 u16 */
#define COMM_DATA_BATCH              0x84U /* 批次 u32 + 采样率 u32 + 抽取倍数 u16 + 灵敏度 u16 */
#define COMM_DATA_WORK_MODE          0x85U /* result:u8 + active:u8 + configured:u8 */
/* Zigbee 节点主动交换：结束/就绪事件与网关确认使用同一 code，按 kind 区分。 */
#define COMM_DATA_GATEWAY_EXCHANGE   0x90U
#define COMM_GATEWAY_REPORT_SIZE     13U /* token:u32,batch:u32,revision:u32,save_result:u8 */
#define COMM_GATEWAY_ACK_SIZE        15U /* token,batch,revision,result:u8,sleep_mask:u16 */
#define COMM_GATEWAY_SLEEP_ALLOWED   0x0001U
#define COMM_GATEWAY_COMMANDS_PENDING 0x0002U

/* 工作模式只控制 BLE 广播策略；两种链路均可使用统一命令查询/保存。 */
#define COMM_WORK_MODE_ONLINE        0U
#define COMM_WORK_MODE_INSPECTION    1U
#define COMM_WORK_MODE_UNKNOWN       0xFFU /* 命令校验失败，未读取模式时的响应占位 */
/* 标量 TLV 类型；长度字节表示值的字节数。 */
#define COMM_TLV_TEMPERATURE         1U /* i16，单位 0.01 摄氏度 */
#define COMM_TLV_MAGNETIC            2U /* XYZ 三个 i16，单位 0.1 微特斯拉 */
#define COMM_TLV_DIGITAL_SPL         3U /* u16，单位 0.01 dB SPL */
#define COMM_TLV_ANALOG_SPL          4U /* u16，单位 0.01 dB SPL */
/* 公共传感器位图，可按位或组合；SCALARS 不包含振动。 */
#define COMM_SENSOR_TEMPERATURE      0x0001U
#define COMM_SENSOR_MAGNETIC         0x0002U
#define COMM_SENSOR_DIGITAL_MIC      0x0004U
#define COMM_SENSOR_ANALOG_MIC       0x0008U
#define COMM_SENSOR_VIBRATION        0x0010U
#define COMM_SENSOR_SCALARS          0x000FU
#define COMM_SENSOR_ALL              0x001FU

/* 启停命令状态值：0 停止，1 启动。 */
#define COMM_STOP                    0U
#define COMM_START                   1U

/* 状态码：停止、采集中、完成、故障、FIFO 溢出。 */
#define COMM_STATUS_STOPPED          0U
#define COMM_STATUS_SAMPLING          1U
#define COMM_STATUS_COMPLETE          2U
#define COMM_STATUS_FAULT             3U
#define COMM_STATUS_OVERFLOW          4U

/* 默认不抽取，最高 16 倍；窗口上限限制在有符号 32 位计时差范围。 */
#define COMM_DEFAULT_DECIMATION      1U
#define COMM_MAX_DECIMATION          16U
#define COMM_MAX_WINDOW_MS           2147483647UL

/* 振动元信息：原始采样率 Hz，灵敏度微 g/LSB，需与硬件匹配。 */
#define COMM_VIB_RATE_HZ              26667UL
#define COMM_VIB_SENSITIVITY_UG       488U

/* 波形固定前缀 10 字节；特征载荷 4 + 3*5*4 = 64 字节。 */
#define COMM_WAVE_PREFIX_SIZE        10U
#define COMM_FEATURE_PAYLOAD_SIZE    64U

/* MCU 内部链路标识，不直接作为线上载荷发送。 */
typedef enum { COMM_LINK_BLE=0, COMM_LINK_ZIGBEE=1, COMM_LINK_COUNT=2 } CommLink;

/* 通用结果码。收发函数返回 OK 只表示本地接受/提交，不代表对端收到。
 * 命令响应中的 OK 表示操作完成，配置命令也已保存并回读；不能把入队成功当配置成功。 */
typedef enum {
  COMM_OK=0, COMM_BUSY=1, COMM_BAD_LENGTH=2, COMM_BAD_VALUE=3,
  COMM_UNSUPPORTED=4, COMM_NOT_READY=5, COMM_HARDWARE_ERROR=6,
  COMM_SAVE_FAILED=7 /* 参数已接收并应用，但保存/回读未成功，不能当作配置成功 */
} CommResult;
#define COMM_REVISION_UNKNOWN 0xFFFFFFFFUL

/* 来源包含链路和连接会话号，防止旧连接的命令在重连后执行。 */
typedef struct { CommLink link; uint32_t session; } CommSource;

/* 解码后的内部请求：sequence 请求序号，mask 公共掩码，factor 抽取倍数，
 * duration_ms 窗口毫秒，code 功能码，state 启停状态；只使用对应命令的字段。 */
typedef struct {
  CommSource source;
  uint16_t sequence, mask, factor;
  uint32_t duration_ms;
  uint8_t code, state, mode; /* mode 仅用于工作模式设置命令 */
} CommRequest;

/* 标量快照：valid_mask 指定有效字段，温度为 0.01 摄氏度，磁场为 0.1 微特斯拉，
 * 声压级为 0.01 dB SPL。无效字段不进入 TLV。 */
typedef struct {
  uint16_t valid_mask;
  int16_t temperature_centi_c, magnetic_0p1_ut[3];
  uint16_t digital_centi_db, analog_centi_db;
} CommScalars;

/* 传输回调参数为来源、完整帧指针、字节数；返回前必须复制或消费缓冲区。 */
typedef CommResult (*CommTransport)(const CommSource *, const uint8_t *, uint16_t);

/* 执行回调接收已校验请求，输出执行后的活动掩码，返回命令执行结果。 */
typedef CommResult (*CommExecutor)(const CommRequest *, uint16_t *active_mask);
/* 完成实际执行后通知应用层；队列重复请求不会重复触发，便于设置保存及版本跟踪。 */
typedef void (*CommExecutedObserver)(const CommRequest *, CommResult);
void Comm_RegisterExecutedObserver(CommExecutedObserver observer);
/* 在执行后、缓存/发送响应前完成配置提交。返回最终结果；响应附带 revision:u32。
 * BLE 或未关联网关轮次时 revision 为0；参数校验失败为 UNKNOWN。不能返回暂定成功。 */
typedef CommResult (*CommCommandFinalizer)(const CommRequest *, CommResult, uint32_t *revision);
typedef void (*CommResponseSubmitted)(const CommRequest *, CommResult, uint32_t revision);
void Comm_RegisterCommandFinalizer(CommCommandFinalizer finalizer, CommResponseSubmitted submitted);

/* 模式执行回调：仅由 Comm_Process 调用，输出当前/保存模式。
 * 模式最终响应载荷为 result、active、configured、revision，共 7 字节。 */
typedef CommResult (*CommWorkModeHandler)(const CommRequest *, uint8_t *active,
                                        uint8_t *configured);
void Comm_RegisterWorkModeHandler(CommWorkModeHandler handler);

/* 读取小端 u16；调用方保证输入至少有 2 字节。 */
uint16_t Comm_ReadU16(const uint8_t *p);

/* 读取小端 u32；调用方保证输入至少有 4 字节。 */
uint32_t Comm_ReadU32(const uint8_t *p);

/* 将 u16 写成 2 字节小端格式，不依赖内存对齐。 */
void Comm_WriteU16(uint8_t *p, uint16_t n);

/* 将 u32 写成 4 字节小端格式，调用方保证输出容量。 */
void Comm_WriteU32(uint8_t *p, uint32_t n);

/* 绑定公共命令执行器，清空队列、重复记录及发送序号。
 * 保留已注册的传输回调和会话号，兼容 BLE/Zigbee 各自的初始化顺序。 */
void Comm_Init(CommExecutor executor);

/* 注册链路发送回调：BLE 对接 GATT 通知，Zigbee 对接原发送队列。 */
void Comm_RegisterTransport(CommLink link, CommTransport transport);

/* 更新连接会话并清除该链路重复记录；排队的旧会话命令在处理时丢弃。 */
void Comm_SetSession(CommLink link, uint32_t session);

/* 只解析和校验命令，不操作硬件；检查帧头、长度、掩码、状态、窗口和抽取倍数。
 * 返回 COMM_OK 才表示参数有效，输出 request 为内部对象，不能直接发送。 */
CommResult Comm_DecodeCommand(const CommSource *source, const uint8_t *frame,
                              uint16_t length, CommRequest *request);

/* 公共接收入口：将命令复制入队，返回后原接收缓冲区可复用。
 * 同会话同序号同内容重发复用结果，同序号不同内容拒绝，队满返回 COMM_BUSY。
 * 完整帧头下的参数错误排队响应，残缺帧直接拒绝；实际执行由 Comm_Process 完成。 */
CommResult Comm_Receive(const CommSource *source, const uint8_t *frame, uint16_t length);

/* 在主循环/任务上下文执行命令并发送响应，不在中断中操作硬件。
 * 先执行有效会话的待处理命令，再发送队首响应，避免响应拥塞阻塞 STOP。
 * 配置提交完成后才缓存/发送最终结果；发送 BUSY 不重复执行或保存。 */
void Comm_Process(void);

/* 返回是否还有待执行命令或待发送响应，供任务调度使用。 */
uint8_t Comm_HasPending(void);

/* 拼装 8 字节公共帧头和载荷，flags 固定为 0；成功返回完整帧长度，失败返回 0。
 * 支持载荷与输出缓冲区重叠；序号由调用方指定，可用于保留待重发帧。 */
uint16_t Comm_EncodeFrame(uint8_t *out, uint16_t capacity, uint8_t kind,
                         uint8_t code, uint16_t sequence,
                         const uint8_t *payload, uint16_t length);

/* 提交已编码完整帧并带上当前会话，不自动拆包、不缓存失败帧。
 * BUSY 时调用方保留帧重试；链路适配器还会检查实际可发送长度。 */
CommResult Comm_SendFrame(CommLink link, const uint8_t *frame, uint16_t length);

/* 给载荷添加帧头，仅提交成功后递增该链路主动上传序号。
 * 使用局部缓冲区，传输回调返回前必须复制或消费数据。 */
CommResult Comm_Send(CommLink link, uint8_t kind, uint8_t code,
                     const uint8_t *payload, uint16_t length);

/* 编码载荷：批次 u32 + 多个 TLV；T、L 各 1 字节，L 为值的字节数。
 * 仅输出 valid_mask 选中的温度、磁场、数字声压、模拟声压，按此顺序排列。
 * 返回载荷长度，失败返回 0，最多需要 24 字节。 */
uint16_t Comm_EncodeScalars(uint8_t *out, uint16_t capacity, uint32_t batch,
                            const CommScalars *values);

/* 编码载荷：批次 u32 + 原始起点 u32 + 点数 u16 + XYZ i16。
 * first 为原始采样序号，count 为本包实际点数，每点按 X/Y/Z 排列占 6 字节。
 * 此处只打包不抽取，抽取倍数通过批次消息说明。返回载荷长度，失败返回 0。 */
uint16_t Comm_EncodeWave(uint8_t *out, uint16_t capacity, uint32_t batch,
                         uint32_t first, const int16_t (*xyz)[3], uint16_t count);

/* 编码载荷：批次 u32 + X/Y/Z 各 5 个 i32 特征，共 64 字节。
 * 按调用方提供的特征顺序写入，不在这里计算特征或改变缩放值。 */
uint16_t Comm_EncodeFeatures(uint8_t *out, uint16_t capacity, uint32_t batch,
                            const int32_t features[3][5]);

/* 标量发送便捷接口：编码批次和 TLV 后调用公共发送函数。 */
CommResult Comm_SendScalars(CommLink link, uint32_t batch, const CommScalars *values);

/* 波形发送便捷接口；当前实时流用 EncodeWave + SendFrame 保留 BUSY 待发帧。 */
CommResult Comm_SendWave(CommLink link, uint32_t batch, uint32_t first,
                         const int16_t (*xyz)[3], uint16_t count);

/* 发送已有振动特征，当前由 Zigbee 快照发布路径调用。 */
CommResult Comm_SendFeatures(CommLink link, uint32_t batch, const int32_t features[3][5]);

/* 发送状态事件，载荷为状态 u8 + 传感器掩码 u16。 */
CommResult Comm_SendStatus(CommLink link, uint8_t status, uint16_t mask);

/* 发送批次元信息：批次、原始采样率、抽取倍数、灵敏度。
 * 采样率与灵敏度取协议宏，必须与振动硬件配置一致。 */
CommResult Comm_SendBatch(CommLink link, uint32_t batch, uint16_t factor);

/* 主动上报模式状态，使用统一事件帧 0x85，载荷与模式命令响应相同。 */
CommResult Comm_SendWorkMode(CommLink link, CommResult result,
                             uint8_t active, uint8_t configured);

/* 旧 BLE 标量掩码转公共掩码：旧 bit3 温度，bit0 磁场，bit1/2 两类麦克风。
 * 振动使用独立状态，不在 BLE 标量掩码映射中处理。 */
uint16_t Comm_MaskFromBle(uint8_t legacy);

/* 公共标量掩码转旧 BLE 驱动位图；振动位不参与转换。 */
uint8_t Comm_MaskToBle(uint16_t mask);

/* 转换 A 版旧配置：bit0 振动，bit1/2 数字/模拟麦克风。
 * 旧采集默认包含温度与磁场，返回值总带这两位；不能用于解释新协议掩码。 */
uint16_t Comm_MaskFromZigbee(uint8_t legacy);
#endif
