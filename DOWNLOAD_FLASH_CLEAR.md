# 每次下载后逻辑丢弃旧采集记录

使用本工程的 `STM32WB55RGV6-BLE-Zigbee Debug.launch` 下载完整ELF。工程已有 `fDownload=true`、`incremental_flashing=false`、下载校验以及 Connect under reset；已移除启动命令里的内部Flash全片擦除。刷新工程后使用这个共享启动配置，不要选择旧的 BLE / BLE-2.0 工程配置。

2026-10-27起，每次实际下载，包括重复下载同一个ELF，都会恢复固件内的一次性清理标志。首次运行只将最多16个采集槽位的旧记录标志置为无效并读回确认，完成后启动BLE/Zigbee及采集。不擦除整个外部Flash，旧波形字节保留但不再参与上传；下次采集使用槽位时才擦除该槽位所需区域。正常复位、断电上电、休眠唤醒保留本次下载后新采集的数据。仅构建、连接调试器或执行Reset，没有重新下载时，不触发清理。

下载后如果停在main断点，按Resume/F8，让程序执行清理。串口先显示：

```text
FLASH-CLEAR: download detected; invalidating old collection records (no external erase)
FLASH-CLEAR: complete; old records invalidated; payload retained; marker committed
```

不再执行原先约109秒的整片擦除与遍历校验；实际启动耗时需上板测量。之后普通重启显示：

```text
FLASH-CLEAR: already completed for this download; cached data retained
```

任何记录写入、读回、标志提交失败都会打印FAILED并停止启动；重新复位可重试。清理中途断电，下一次上电重新让全部记录失效，成功前不开始采集。若提示CPU2 already running，采用硬件复位/断电上电，保证清理在CPU2启动前执行；不会在无线核运行时绕过双核Flash仲裁。

## 实现与边界

- 链接脚本将CPU1程序区缩为444 KiB，独立保留 `0x0806F000–0x0806FFFF` 的4 KiB页；链接断言防止程序、标志、Zigbee NVM重叠。
- ELF/HEX包含 `.download_reset` 的8字节ARMED标志（不是NOLOAD）。启动从真实Flash读取它，不使用版本号或RAM变量判断是否下载。
- 调用现有 `ZigbeeVibrationStore_ResetRecords()`，每槽只写4字节无效magic并读回确认，最多16槽。记录全部失效后才擦除内部标志专用页并写入DONE。没有在已编程内部Flash双字上二次写入，避免ECC限制。
- Zigbee网络NVM仍在内部Flash `0x08070000–0x0807FFFF`，本流程不会擦除它。外部旧波形和元数据字节仍在，但记录无效、槽位管理状态为空闲。全片物理数据清除不再是本流程的行为。
- 标志提交早于 `MX_APPE_Init()`，并拒绝在C2BOOT已置位时操作；现有Zigbee NVM驱动与其双核仲裁没有更改。
- 若自行制作仅含部分地址的镜像，漏掉 `.download_reset`，或者启用其他下载器的跳过/增量下载优化，不能保证每次下载触发。请使用本工程的完整ELF和共享下载配置。
- STM32工程当前默认make未启用自动HEX输出。IDE优先下载最新ELF；需要HEX时必须从该ELF重新导出，不能沿用旧HEX。

已通过Debug/Release构建及主机隔离测试：真实16槽记录失效、旧波形不变、清理过程没有外部擦除、下次使用槽位时擦除、同一镜像重复下载、普通重启保留、中断重试、HAL错误、标志页范围与CPU2排除。没有执行硬件烧录，实际启动耗时和连续两次下载需在板上验证。
