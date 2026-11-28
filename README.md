# 315-research

STM32WB55RGV6 BLE / Zigbee 节点固件，使用 STM32CubeIDE 工程。仓库根目录即工程根目录，工程名为 `STM32WB55RGV6-BLE-Zigbee`。

## GitHub Desktop 协作

首次获取工程，在 GitHub Desktop 中选择 File → Clone repository，仓库地址为 `https://github.com/yang1768707-niko/315-research.git`。

已有本地工程可通过 File → Add local repository 添加。主分支为 `main`，远程仓库名为 `origin`。

每次开发前先 Fetch origin 并拉取最新提交。各成员从最新 main 创建功能分支，完成后 Commit、Publish branch / Push origin，再通过 Pull Request 合并。修改同一文件时，应先协调负责范围，再处理合并冲突。

## 导入与构建

1. 安装 STM32CubeIDE，通过 File → Import → General → Existing Projects into Workspace 选择本仓库根目录。
2. 使用工程名 `STM32WB55RGV6-BLE-Zigbee`，选择 Debug 配置并重新构建。本次固件已使用 GNU Tools for STM32 13.3.rel1 构建验证。
3. 检查本机 ST-Link 调试配置。已有 `.launch` 文件包含原机器的日志路径，可在本机调试设置中调整或重新创建调试配置。

仓库包含 Core、STM32_WPAN、Drivers、Middlewares、Utilities、CubeIDE 配置、链接脚本和依赖静态库。Debug、Release、工作区元数据及编译产物由 `.gitignore` 排除，克隆后由本机构建生成。

## 当前分支：BLE / QSPI 融合版本

本次从已测试的原节点工程同步源码。保留现用 Zigbee 活跃接收、通信测试、外设休眠、RTC 长时间休眠和网关配置交互；合入 Git 版本的 BLE 绑定、工作模式和延迟复位处理。QSPI 使用统一四线数据读取、双缓冲页写入、完成事件调度，保留深度掉电与唤醒检查。

- [相对旧 Git 的修改对照、融合范围与测试步骤](MERGE_BLE_QSPI_20261127.md)
- [下载后旧采集记录处理](DOWNLOAD_FLASH_CLEAR.md)
- [外部 Flash 缓存策略](FLASH_CACHE_POLICY.md)
- [BLE 工作模式、绑定与自动重启](BLE_WORK_MODE.md)

BLE 采集命令最终响应为 16 字节，工作模式响应为 15 字节，模式事件及 FE82 快照为 11 字节。Zigbee 保留现用网关协议，以 `Core/Inc/gateway_exchange_wire.h`、`control_sync.h` 和 `STM32_WPAN/App/app_zigbee.c` 为准。协调器与网关继续使用配套现用版本。

`COMM_PROTOCOL.md` 和 `GATEWAY_EXCHANGE.md` 保留导入 Git 版本的设计记录，其中通用 Zigbee 命令入口与本次保留的 Zigbee 实现不同，不能据此要求现用网关改为旧设计。

在 CubeIDE 中使用 `BLE-QSPI 本地融合测试` 下载配置，它指向本工程的 Debug ELF，未设置全片擦除命令。串口为 921600、8N1。克隆后需在自己的电脑上构建，`Debug` 等产物不提交到仓库；同名工程在一个 IDE workspace 中只导入一份。

## 主机回归

以下测试编译实际源码或从中抽取函数，不访问硬件。安装宿主 GCC 并加入 PATH 后运行：

```powershell
node tests/run-ble-work-mode-tests.cjs
node tests/run-qspi-store-tests.cjs
```

也可用 `NATIVE_CC` 指定宿主 GCC 可执行文件。若使用 Zig，将 `NATIVE_CC` 指向 `zig.exe`，并设置 `NATIVE_CC_ARGS='["cc"]'`。BLE 模式测试另保留 LCC64 支持，可用 `LCC_ROOT` 指定目录。

QSPI 模拟覆盖短/长四线读取、QE 失败、并发忙保护、掉电超时重试、DMA 页缓冲所有权、三种完整采集长度的写入/CRC/重新加载，以及 STOP2 准备失败的拒绝和重试。BLE 模式测试覆盖保存/发送失败、断线、延迟复位和绑定任务。

保留的旧 `tests/run-comm-tests.ps1`、`test_comm_protocol.c`、`mock_comm_acquisition.c`、`test_gateway_exchange.c` 针对导入 Git 版本的通用 Zigbee 入口，不能作为本分支当前 Zigbee 实现的验收测试。无线吞吐、真实配对、RTC 唤醒和功耗仍需实机验证。

蓝牙专家：刘子锐