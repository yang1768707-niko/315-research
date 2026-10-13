param(
  [string]$SourceRoot = 'C:/Users/Administrator/STM32-Projects-1.91/Zigbee',
  [string]$BackupRoot = 'C:/Users/Administrator/STM32-Projects-1.91/power-backups/20261013-zigbee-to-ble-zigbee'
)
$ErrorActionPreference = 'Stop'
$targetRoot = Split-Path $PSScriptRoot -Parent
function Read-Normalized([string]$path) { [IO.File]::ReadAllText($path).Replace("`r`n","`n") }
function Assert-True($value, [string]$message) { if (!$value) { throw $message } }
function Read-CFunction([string]$content, [string]$name) {
  $pattern = '(?m)^(?:static\s+)?[A-Za-z_]\w*(?:[ \t]+|[ \t]*\*[ \t]*)' +
    [regex]::Escape($name) + '\([^;]*?\)\s*(?:__attribute__\(\([^;]*?\)\)\s*)?\{'
  $match = [regex]::Match($content, $pattern)
  Assert-True $match.Success "Missing function: $name"
  $begin = $content.IndexOf('{', $match.Index)
  $depth = 0
  for ($index=$begin; $index -lt $content.Length; $index++) {
    if ($content[$index] -eq '{') { $depth++ }
    if ($content[$index] -eq '}') {
      $depth--
      if ($depth -eq 0) { return $content.Substring($match.Index, $index-$match.Index+1) }
    }
  }
  throw "Unclosed function: $name"
}
$unchanged = @(
 'STM32_WPAN/App/app_ble.c','STM32_WPAN/App/app_ble.h',
 'STM32_WPAN/App/custom_stm.c','STM32_WPAN/App/custom_stm.h',
 'STM32_WPAN/App/ble_conf.h','STM32_WPAN/App/ble_dbg_conf.h',
 'Core/Src/stm32_lpm_if.c',
 'Drivers/BSP/stm32wb55rgv6_bsp.c','STM32WB55RGVX_BLE_ZIGBEE_FLASH.ld'
)
foreach ($file in $unchanged) {
  Assert-True ((Get-FileHash (Join-Path $BackupRoot $file)).Hash -eq
    (Get-FileHash (Join-Path $targetRoot $file)).Hash) "Unexpected change: $file"
}
& (Join-Path $PSScriptRoot 'verify-qspi-dma.ps1')
$copied = @(
 'Core/Inc/monitor_standby.h','Core/Inc/sampling_cadence.h',
 'Core/Inc/vibration_timeslots.h','Core/Inc/vibration_wire.h',
 'Core/Inc/vibration_codec.h','Core/Inc/vibration_features.h',
 'Core/Inc/zigbee_vibration_store.h','Core/Inc/qspi_nor.h',
 'Core/Src/zigbee_vibration_store.c','Core/Src/qspi_nor.c'
)
foreach ($file in $copied) {
  Assert-True ((Read-Normalized (Join-Path $SourceRoot $file)) -ceq
    (Read-Normalized (Join-Path $targetRoot $file))) "Zigbee dependency mismatch: $file"
}
$gatewayTx = 'C:/Users/Administrator/Documents/Codex/Zigbee-Gateway/gateway-core/include/vibration_tx.h'
Assert-True ((Read-Normalized $gatewayTx) -ceq
  (Read-Normalized (Join-Path $targetRoot 'Core/Inc/vibration_tx.h'))) 'Single-node TX copies differ'
$old = Read-Normalized (Join-Path $BackupRoot 'STM32_WPAN/App/custom_app.c')
$new = Read-Normalized (Join-Path $targetRoot 'STM32_WPAN/App/custom_app.c')
$bleFunctions = @('Custom_STM_App_Notification','Custom_APP_Notification','Custom_APP_TxPoolAvailable','Custom_APP_PrepareStop2','Custom_APP_ResumeFromStop2','DT_StartSensorStream','DT_StopSensorStream','DT_ApplySensorEnableMask','DT_SensorIdToMask','DT_StartVibStream','DT_StopVibStream','DT_UpdateStatusLeds','DT_ToggleVibStream','DT_SensorTimerCallback','DT_ProcessBleAudio','DT_LogAudioHealth','DT_HandleSensorCtrl','DT_HandleVibCtrl','DT_SendTask','DT_Log','DT_VibLogStatistics','DT_VibInitHardware','DT_VibBuildPacket')
foreach ($name in $bleFunctions) {
  Assert-True ((Read-CFunction $old $name) -ceq (Read-CFunction $new $name)) "BLE function changed: $name"
}
$zigbee = Read-Normalized (Join-Path $targetRoot 'STM32_WPAN/App/app_zigbee.c')
$handoff = Read-CFunction $zigbee 'ZbApp_RadioControlTask'
Assert-True ($handoff.Contains('persist_restore_inflight != 0U')) 'Restore must drain before MAC pause'
Assert-True ($handoff.Contains('|| upload_inflight)')) 'Upload callbacks must drain before MAC pause'
Assert-True (!$zigbee.Contains('ZbApp_PollUploadReply') -and !$zigbee.Contains('V2_TIMED_ACK_LEN')) 'Timed upload polling must be removed'
Assert-True ($handoff.Contains('ZbApp.radio_retry_pending = 1U;')) 'Deferred handoff must be retried'
Assert-True (!$handoff.Contains('ZigbeeVibrationStore_Acknowledge(') -and
  !$handoff.Contains('ZigbeeVibrationStore_ResetRecords(')) 'BLE handoff must retain Flash data'
Assert-True ($zigbee.Contains('INFO_STACK_TYPE_BLE_ZIGBEE_RFD_DYNAMIC')) 'Dual-protocol firmware check missing'
Assert-True ((Read-CFunction $new 'DT_AbortZigbeeSnapshot').Contains('DigitalMicrophone_ConfigureWindow(500U)')) 'BLE microphone default must be restored'
$oldConfig = Read-Normalized (Join-Path $BackupRoot 'Core/Inc/app_conf.h')
$newConfig = Read-Normalized (Join-Path $targetRoot 'Core/Inc/app_conf.h')
$bleDefines = '(?m)^#define (?:CFG_BLE_|CFG_GAP_|CFG_TX_POWER|CFG_FAST_CONN_|CFG_LP_CONN_|L2CAP_|CFG_BONDING_|CFG_FIXED_PIN|CFG_USED_FIXED_PIN|CFG_PRIVACY|CFG_IDENTITY_).*$'
Assert-True (([regex]::Matches($oldConfig,$bleDefines).Value -join "`n") -ceq
  ([regex]::Matches($newConfig,$bleDefines).Value -join "`n")) 'BLE settings changed'
Write-Output ("PASS: {0} unchanged files, {1} Zigbee dependencies, {2} preserved BLE functions; ownership guards verified." -f
  $unchanged.Count,$copied.Count,$bleFunctions.Count)
Write-Output 'Source checks only: actual radio, sleep, capture and throughput still require board tests.'

