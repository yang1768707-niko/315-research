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

## 通信与功能说明

- [统一通信协议及配置 App 适配要求](COMM_PROTOCOL.md)
- [BLE 工作模式、绑定与自动重启](BLE_WORK_MODE.md)
- [网关交互与配置保存](GATEWAY_EXCHANGE.md)

BLE 和 Zigbee 采集命令最终响应均为 16 字节，统一工作模式命令响应为 15 字节，模式事件及 FE82 快照为 11 字节。App 与网关修改时请同步核对协议字段、结果码、请求序号和配置版本。

## 主机回归

模式与绑定测试通过实际源码抽取运行，使用宿主 GCC 时可设置 `NATIVE_CC`。采集和网关回归脚本可通过 `-LccRoot` 指定本机 LCC64 目录。

```powershell
node tests/run-ble-work-mode-tests.cjs
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run-comm-tests.ps1 -LccRoot "<本机 LCC64 目录>"
```

主机测试覆盖协议、采集控制、工作模式、延迟重启、绑定任务及网关交互。无线吞吐、真实配对、断电保持和传感器数据完整性需要实机验证。
