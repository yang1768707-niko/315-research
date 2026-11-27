param(
  [string]$LccRoot = 'D:\Program Files\MATLAB\R2024b\sys\lcc64\lcc64'
)
$ErrorActionPreference = 'Stop'
$project = Split-Path $PSScriptRoot -Parent
$outDir = Join-Path $project 'Debug\comm-tests'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $project 'STM32_WPAN\App\custom_app.c'))
$match = [regex]::Match($source, 'static CommResult DT_CommExecute\(const CommRequest \*r,uint16_t \*active\)\s*\{')
if (!$match.Success) { throw 'Production executor definition not found' }
$start = $source.IndexOf('{', $match.Index)
$depth = 0
$end = $start
for (; $end -lt $source.Length; $end++) {
  if ($source[$end] -eq '{') { $depth++ }
  if ($source[$end] -eq '}') { $depth--; if ($depth -eq 0) { break } }
}
if ($depth -ne 0) { throw 'Unclosed executor definition' }
[IO.File]::WriteAllText((Join-Path $outDir 'comm_executor_under_test.inc'),
  $source.Substring($match.Index, $end-$match.Index+1), [Text.Encoding]::ASCII)
$transportMatch = [regex]::Match($source, 'static CommResult DT_CommBleSend\(const CommSource \*source,const uint8_t \*frame,uint16_t length\)\s*\{')
if (!$transportMatch.Success) { throw 'Production BLE transport not found' }
$start = $source.IndexOf('{', $transportMatch.Index)
$depth = 0
for ($end=$start; $end -lt $source.Length; $end++) {
  if ($source[$end] -eq '{') { $depth++ }
  if ($source[$end] -eq '}') { $depth--; if ($depth -eq 0) { break } }
}
if ($depth -ne 0) { throw 'Unclosed BLE transport definition' }
[IO.File]::AppendAllText((Join-Path $outDir 'comm_executor_under_test.inc'),
  "`n" + $source.Substring($transportMatch.Index, $end-$transportMatch.Index+1),
  [Text.Encoding]::ASCII)
# Extract production settings functions; mock only backup registers and peripherals.
$gatewaySource = [IO.File]::ReadAllText((Join-Path $project 'STM32_WPAN\App\app_zigbee.c'))
$gatewayFunctions = @([regex]::Match($gatewaySource,'#define GATEWAY_SETTINGS_MAGIC[^\r\n]*').Value)
foreach($exportName in @('Custom_APP_ProtocolExportSettings','Custom_APP_ProtocolImportSettings')) {
  $m=[regex]::Match($source,'(?:void|uint8_t) '+$exportName+'\([^;]*?\)\s*\{')
  if(!$m.Success){throw "Missing settings function $exportName"}
  $start=$source.IndexOf('{',$m.Index);$depth=0
  for($end=$start;$end -lt $source.Length;$end++) {
    if($source[$end] -eq '{'){$depth++}
    if($source[$end] -eq '}'){$depth--;if(!$depth){break}}
  }
  $gatewayFunctions += $source.Substring($m.Index,$end-$m.Index+1)
}
foreach ($functionName in @('ZbApp_GatewayBegin','ZbApp_GatewayChanged','ZbApp_CommonCommandDone','APP_ZIGBEE_WaitingGateway','ZbApp_GatewaySaveSettings','ZbApp_GatewayLoadSettings','ZbApp_FinalizeCommand','ZbApp_FinalResponseSubmitted','ZbApp_ReleaseMonitorStandbyLock','ZbApp_ApplyMonitorSwitch','ZbApp_ScheduleMonitorStandby','ZbApp_SendLinkStatus','APP_ZIGBEE_GatewayProcess')) {
  $functionMatch=[regex]::Match($gatewaySource, '(?:static )?(?:void|uint8_t|CommResult|enum ZclStatusCodeT) '+$functionName+'\([^;]*?\)\s*\{')
  if (!$functionMatch.Success) { throw "Missing production function $functionName" }
  $start=$gatewaySource.IndexOf('{',$functionMatch.Index);$depth=0
  for($end=$start;$end -lt $gatewaySource.Length;$end++) {
    if($gatewaySource[$end] -eq '{'){$depth++}
    if($gatewaySource[$end] -eq '}'){$depth--;if(!$depth){break}}
  }
  $gatewayFunctions += $gatewaySource.Substring($functionMatch.Index,$end-$functionMatch.Index+1)
}
# Use the actual legacy 0x08 branch, not a test copy of its intended behavior.
$startMatch=[regex]::Match($gatewaySource,'if \(header->cmdId == ZB_CMD_REPORT_START\)\s*\{')
if(!$startMatch.Success){throw 'Missing legacy REPORT_START branch'}
$start=$gatewaySource.IndexOf('{',$startMatch.Index);$depth=0
for($end=$start;$end -lt $gatewaySource.Length;$end++) {
  if($gatewaySource[$end] -eq '{'){$depth++}
  if($gatewaySource[$end] -eq '}'){$depth--;if(!$depth){break}}
}
$gatewayFunctions += 'static enum ZclStatusCodeT invoke_legacy_report_start(void) '+$gatewaySource.Substring($start,$end-$start+1)
[IO.File]::WriteAllText((Join-Path $outDir 'gateway_settings_under_test.inc'),
  ($gatewayFunctions -join "`n"), [Text.UTF8Encoding]::new($false))
$compiler = Join-Path $LccRoot 'bin\lcc64.exe'
$linker = Join-Path $LccRoot 'bin\lcclnk64.exe'
if (!(Test-Path $compiler)) { throw "Set -LccRoot to the installed LCC64 directory: $LccRoot" }
$includes = @(('-I' + (Join-Path $LccRoot 'include64')), '-ICore\Inc',
              '-ISTM32_WPAN\App', '-IDebug\comm-tests')
Push-Location $project
try {
foreach ($unit in @('Core\Src\comm_protocol.c','tests\test_comm_protocol.c','tests\mock_comm_acquisition.c','tests\test_gateway_exchange.c')) {
  $obj = 'Debug\comm-tests\' + ([IO.Path]::GetFileNameWithoutExtension($unit)) + '.obj'
  & $compiler @includes ('-Fo' + $obj) $unit
  if ($LASTEXITCODE -ne 0) { throw "Compile failed: $unit" }
}
foreach ($test in @('test_comm_protocol','mock_comm_acquisition','test_gateway_exchange')) {
  $exe = '.\Debug\comm-tests\' + $test + '.exe'
  & $linker ('-L' + (Join-Path $LccRoot 'lib64')) '-o' $exe 'Debug\comm-tests\comm_protocol.obj' ('Debug\comm-tests\' + $test + '.obj')
  if ($LASTEXITCODE -ne 0) { throw "Link failed: $test" }
  & $exe
  if ($LASTEXITCODE -ne 0) { throw "Test failed: $test" }
}
} finally { Pop-Location }
