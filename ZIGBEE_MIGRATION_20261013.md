# Zigbee 功能回迁至 BLE/Zigbee 工程（2026-10-13）

来源：`C:/Users/Administrator/STM32-Projects-1.91/Zigbee`，使用当前工作目录内容，包含尚未提交的修改。
目标：`C:/Users/Administrator/STM32-Projects-1.91/STM32WB55RGV6-BLE-Zigbee`。

## 已迁移

- 新节点授权、网络恢复、安全重入、离网清理及 Zigbee NVM 持久化。
- 默认关闭在线监测；授权后首次关闭监测的 10 分钟待机、3 秒配置查询和空闲状态上报；后续 5 分钟低功耗查询。
- 单节点/分组应用后的采样参数查询、传感器启用选择、麦克风采集窗口、原始波形上传频率。
- RTC 采样周期、跳过错过的采样期限而不突发补采。
- QSPI 波形记录、计数式振动采集、FIFO DMA、SRAM 弹性缓存。
- V2 压缩、分块申请、现有时隙上传、缺片补传、CRC 验证。
- 先上传原始波形，后计算/发送特征；波形与特征都确认后才释放 Flash 记录。
- 当前源工程 Zigbee 发射功率目标 +6 dBm 及读回诊断；没有改变 BLE 发射功率。
- 当前源工程的串口诊断日志。

这是现有功能迁移，不是新的并发上传方案：源工程的 RTT 判定、时隙参数、重试策略保持原样，
因此本次编译成功不能证明既有上传速度/时隙拒绝问题已经解决。

## 蓝牙保留范围与共用资源

`app_ble.c/.h`、`custom_stm.c/.h`、`ble_conf.h`、`ble_dbg_conf.h`与备份逐字节一致。
BLE 名称仍为 `WB55_DT1`，广播、连接参数、GATT 服务、通知格式及 FE92 控制命令不变。
共享 `custom_app.c` 中 23 个 BLE/公共函数已按函数体与备份核对一致。

不新增 BLE 与 Zigbee 同时争用传感器的行为：继续使用目标工程原来的 BLE 优先机制。
手机连接时中止未完成的 Zigbee 采集，保留已经提交的 Flash 记录；等待 Zigbee 异步启动/恢复、
APS 确认和取包回调结束后才暂停 Zigbee MAC。断开后恢复 Zigbee、重新申请上传许可。
蓝牙连接时将麦克风窗口恢复为原来的 500 ms，不沿用 Zigbee 配置的长窗口。

共用的主程序、引脚初始化、中断、STOP2 恢复、BSP 与链接脚本未被覆盖。
CPU2 应保留适合本工程的 BLE/Zigbee 双协议无线固件，不要改烧源工程所用的纯 Zigbee RFD 固件。
本次没有安装 CPU2 固件、烧录板卡、擦除内部 Flash 或擦除外部 Flash。

## 备份

`C:/Users/Administrator/STM32-Projects-1.91/power-backups/20261013-zigbee-to-ble-zigbee`

保存了修改前目标工程的全部源码与工程配置，未备份可重新生成的 Debug/Release 构建目录。
来源 Zigbee 工程没有修改；TI 协调器和网关前后端也没有修改。

## 修改文件（14 个）

| 文件 | 改动 |
| --- | --- |
| `STM32_WPAN/App/app_zigbee.c` | 替换为当前 Zigbee 业务；保留双协议固件检查、蓝牙优先的 MAC 暂停/恢复接口，并增加上传缓冲与异步恢复的安全交接。 |
| `STM32_WPAN/App/app_zigbee.h` | 增加授权、休眠状态查询；保留原蓝牙通知接口。 |
| `STM32_WPAN/App/custom_app.c` | 只合并 Zigbee 采集/缓存相关代码；BLE 连接通知、控制命令、数据发送与预览实现保持原样。 |
| `STM32_WPAN/App/custom_app.h` | 扩展 Zigbee 快照、采样参数接口。 |
| `Core/Inc/app_conf.h` | 增加 Zigbee 任务与低功耗请求 ID；BLE 配置宏不变。 |
| `Core/Inc/hw_conf.h` | 定时器容量由 10 增加到 16，容纳双协议与新的 Zigbee 定时器。 |
| `Core/Src/app_entry.c` | 增加 Zigbee NVM 所需的 CPU2 Flash SEM7 仲裁；保留原 BLE 初始化及 NVM 事件。 |
| `Middlewares/Sensors/Analog microphone/analog_microphone.c` | 增加可配置采集窗口；保留原 BSP 供电控制与 BLE 默认窗口。 |
| `Middlewares/Sensors/Analog microphone/analog_microphone.h` | 声明采集窗口配置接口。 |
| `Middlewares/Sensors/Digital microphone/digital_microphone.c` | 增加可配置采集窗口；BLE 仍使用原 500 ms 窗口。 |
| `Middlewares/Sensors/Digital microphone/digital_microphone.h` | 声明采集窗口配置接口。 |
| `Middlewares/Sensors/Vibrometer/vibrometer.c` | 增加 Zigbee FIFO 异步 DMA 读取；保留 BLE 的阻塞 FIFO 读取与初始化行为。 |
| `Middlewares/Sensors/Vibrometer/vibrometer.h` | 声明异步读取和中止接口。 |
| `.cproject` | 将旧 F: 盘传感器包含目录改为工程相对路径，Debug/Release 均可在当前目录构建。 |

## 新增功能文件（11 个）

- `Core/Inc/monitor_standby.h`
- `Core/Inc/sampling_cadence.h`
- `Core/Inc/vibration_timeslots.h`
- `Core/Inc/vibration_tx.h`
- `Core/Inc/vibration_wire.h`
- `Core/Inc/vibration_codec.h`
- `Core/Inc/vibration_features.h`
- `Core/Inc/zigbee_vibration_store.h`
- `Core/Inc/qspi_nor.h`
- `Core/Src/zigbee_vibration_store.c`
- `Core/Src/qspi_nor.c`

另新增本说明和 `tests/verify-zigbee-migration.ps1`。
CubeIDE 完整构建自动更新了 `.settings/language.settings.xml` 和 Debug/Release 下的构建清单、
对象、依赖、映射文件与产物，这些是生成文件，不是额外的业务逻辑修改。

## 验证结果

- CubeIDE Debug / Release 完整构建通过：0 errors、0 warnings；最后的安全交接修正也重新编译、链接通过。
- Debug：text 204280 B，data 3432 B，bss 159640 B。
- Release：text 136616 B，data 3429 B，bss 159616 B。
- RAM 链接未溢出；Debug 的堆起点为 0x20025B78，至保留的 4 KB 主栈之前约剩 37 KB。
  这不是运行期峰值内存测试。
- 应用链接区继续限定为 0x08000000–0x0806FFFF；NVM 区为 0x08070000–0x0807FFFF。
- 保留性检查通过：12 个文件字节不变，11 个 Zigbee 依赖内容与源工程一致，23 个 BLE/公共函数体不变。
- 实际空口、BLE 稳定性、STOP2 电流和吞吐量未通过软件编译代替板测。

构建产物：
`Debug/STM32WB55RGV6-BLE-Zigbee.elf/.hex/.bin`；
`Release/STM32WB55RGV6-BLE-Zigbee.elf/.hex/.bin`。

## 上板验收

1. CubeIDE 刷新目标工程，选择 Debug 或 Release；烧录相应 CPU1 镜像。节点调试串口保持 921600、8N1。
2. 保留双协议 CPU2，确认 BLE 广播名、连接、原传感器数据流及振动预览正常。
3. 开放 Zigbee 入网并授权，验证默认监测关闭、3 秒查询/状态更新，再开启监测。
4. 验证波形分块、缺片补传和特征入库；关闭监测后验证原采集结束、查询配置及 STOP2。
5. 在 Zigbee 采集和上传期间分别连接手机：检查 BLE 不抢用尚在 DMA 中的外设；
   断开后检查 Zigbee 恢复、已保存批次继续处理。
6. 验证断电恢复及删除节点；涉及凭据的迁移以实际节点/协调器日志为准。

CubeMX 重新生成代码可能覆盖定制业务，生成后必须核对差异。

## 2026-10-13：补齐 QSPI DMA 和中断依赖

上板日志显示 `samples=162/13334, flash_err=2, wrote=0`。
首次迁移没有同步 QSPI DMA 的底层配置，异步页写入检测到 `hqspi.hdma == NULL`，
因此在首个 Flash 页提交时失败，并非无线上传或后端 ACK 超时。
前文“主程序、中断、MSP 文件不变”仅描述首次迁移；本次对其中的 QSPI 部分作以下必要补充。

本次修改：

- `Core/Src/main.c`：增加 `hdma_quadspi`，启用 DMA1 通道 6 中断，优先级 5。
- `Core/Src/stm32wbxx_hal_msp.c`：配置 QSPI DMA 请求、Normal/字节模式、内存到外设，
  调用 `__HAL_LINKDMA`；启用 QUADSPI 中断，并补齐反初始化。
- `Core/Src/stm32wbxx_it.c`：新增 DMA1 通道 6 和 QUADSPI 处理函数，分别分发到 HAL。
- `Core/Inc/stm32wbxx_it.h`：增加上述处理函数声明。
- `STM32WB55RGV6-BLE-Zigbee.ioc`：增加第 6 个 DMA 请求及两个中断配置。
- `STM32_WPAN/App/app_zigbee.c`：仅更新启动标识为
  `ble-zigbee-qspi-dma-20261013`，便于确认已烧录修复版；业务逻辑不变。
- `tests/verify-zigbee-migration.ps1`：原来的完整文件不变检查改为包含 QSPI 范围检查。
- 新增 `tests/verify-qspi-dma.ps1`：检查 DMA 绑定、中断链路、通道无冲突、
  既有 .ioc 属性不变及其他外设/中断函数不变。
- 本说明文件更新。

DMA1 通道 1～5 的 SPI、SAI、ADC、UART 配置不变；BLE 应用、服务与配置不变。
QSPI 支持代码放在 CubeMX 对应的生成区域，不在保留的 USER CODE 区域重复定义，
避免以后重新生成时出现重复的中断函数或重复初始化。
本次未运行 CubeMX 全工程重新生成，避免覆盖已有定制业务。

备份：`C:/Users/Administrator/STM32-Projects-1.91/power-backups/20261013-qspi-dma-fix`。

验证：Debug 与 Release 编译、链接通过，没有编译错误/警告；
Debug 为 text 205512 B、data 3432 B、bss 159736 B；
Release 为 text 137280 B、data 3429 B、bss 159712 B。
两个新中断均为链接产物中的强符号，不再落到启动文件的弱 Default_Handler。
回归检查通过：6 个通道无冲突、58 个其他外设/中断函数保持一致，
并确认该检查会拒绝修复前缺少 DMA 的版本。
原 Zigbee 依赖及 BLE 保留检查继续通过。ELF、HEX、BIN 已更新。

重新烧录 CPU1 后，用原 500 ms 振动配置验证 `samples=13334`、`flash_err=0`、
`wrote` 大于 0，然后确认波形分块和特征均完成上传。
这是验收目标，不是已获得的上板结果；本次没有硬件烧录、Flash 擦除或 CPU2 固件变更。

