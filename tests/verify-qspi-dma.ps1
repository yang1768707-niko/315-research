param(
  [string]$ProjectRoot = (Split-Path $PSScriptRoot -Parent),
  [string]$BackupRoot = 'C:/Users/Administrator/STM32-Projects-1.91/power-backups/20261013-qspi-dma-fix'
)
$ErrorActionPreference = 'Stop'
function Read-Text([string]$path) { [IO.File]::ReadAllText($path).Replace("`r`n","`n") }
function Assert-True($value, [string]$message) { if (!$value) { throw $message } }
function Read-Function([string]$content, [string]$name) {
  $pattern = '(?m)^(?:static\s+)?[A-Za-z_]\w*(?:[ \t]+|[ \t]*\*[ \t]*)' +
    [regex]::Escape($name) + '\([^;]*?\)\s*\{'
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
$main = Read-Text (Join-Path $ProjectRoot 'Core/Src/main.c')
$msp = Read-Text (Join-Path $ProjectRoot 'Core/Src/stm32wbxx_hal_msp.c')
$irq = Read-Text (Join-Path $ProjectRoot 'Core/Src/stm32wbxx_it.c')
$header = Read-Text (Join-Path $ProjectRoot 'Core/Inc/stm32wbxx_it.h')
$init = Read-Function $msp 'HAL_QSPI_MspInit'
foreach ($statement in @(
  'hdma_quadspi.Instance = DMA1_Channel6;',
  'hdma_quadspi.Init.Request = DMA_REQUEST_QUADSPI;',
  'hdma_quadspi.Init.Direction = DMA_MEMORY_TO_PERIPH;',
  'hdma_quadspi.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;',
  'hdma_quadspi.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;',
  'hdma_quadspi.Init.Mode = DMA_NORMAL;',
  'HAL_DMA_Init(&hdma_quadspi)',
  '__HAL_LINKDMA(hqspi, hdma, hdma_quadspi);',
  'HAL_NVIC_EnableIRQ(QUADSPI_IRQn);'
)) { Assert-True $init.Contains($statement) "Missing QSPI setup: $statement" }
Assert-True $main.Contains('DMA_HandleTypeDef hdma_quadspi;') 'QSPI DMA storage missing'
Assert-True (Read-Function $main 'MX_DMA_Init').Contains('HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);') 'DMA interrupt disabled'
foreach ($name in @('DMA1_Channel6_IRQHandler','QUADSPI_IRQHandler')) {
  Assert-True $header.Contains("void $name(void);") "IRQ declaration missing: $name"
  Assert-True ([regex]::Matches($irq,"(?m)^void $name\(void\)").Count -eq 1) "IRQ must have exactly one definition: $name"
}
Assert-True (Read-Function $irq 'DMA1_Channel6_IRQHandler').Contains('HAL_DMA_IRQHandler(&hdma_quadspi);') 'DMA handler missing'
Assert-True (Read-Function $irq 'QUADSPI_IRQHandler').Contains('HAL_QSPI_IRQHandler(&hqspi);') 'QSPI handler missing'
Assert-True (Read-Function $msp 'HAL_QSPI_MspDeInit').Contains('HAL_DMA_DeInit(hqspi->hdma);') 'DMA deinit missing'

# Every pre-existing property except the request count must remain identical.
function Read-Ioc([string]$path) {
  $properties = @{}
  foreach ($line in (Read-Text $path).Split("`n")) {
    $split = $line.IndexOf('=')
    if ($split -gt 0) {
      $key = $line.Substring(0,$split)
      Assert-True (!$properties.ContainsKey($key)) "Duplicate .ioc key: $key"
      $properties[$key] = $line.Substring($split+1)
    }
  }
  return $properties
}
$oldIoc = Read-Ioc (Join-Path $BackupRoot 'STM32WB55RGV6-BLE-Zigbee.ioc')
$newIoc = Read-Ioc (Join-Path $ProjectRoot 'STM32WB55RGV6-BLE-Zigbee.ioc')
foreach ($key in $oldIoc.Keys) {
  if ($key -eq 'Dma.RequestsNb') { continue }
  Assert-True ($newIoc[$key] -ceq $oldIoc[$key]) "Unrelated .ioc configuration changed: $key"
}
Assert-True ($newIoc['Dma.RequestsNb'] -eq '6') 'Expected 6 DMA requests'
Assert-True ($newIoc['Dma.Request5'] -eq 'QUADSPI') 'QSPI request missing'
Assert-True ($newIoc['Dma.QUADSPI.5.Instance'] -eq 'DMA1_Channel6') 'Wrong QSPI DMA channel'
Assert-True ($newIoc['Dma.QUADSPI.5.Direction'] -eq 'DMA_MEMORY_TO_PERIPH') 'Wrong QSPI DMA direction'
Assert-True ($newIoc['Dma.QUADSPI.5.Mode'] -eq 'DMA_NORMAL') 'Wrong QSPI DMA mode'
Assert-True ($newIoc['NVIC.DMA1_Channel6_IRQn'].StartsWith('true\:5\:0')) 'Wrong DMA IRQ setting'
Assert-True ($newIoc['NVIC.QUADSPI_IRQn'].StartsWith('true\:5\:0')) 'Wrong QSPI IRQ setting'
$channels = @{}
for ($request=0; $request -lt 6; $request++) {
  $name = $newIoc["Dma.Request$request"]
  $channel = $newIoc["Dma.$name.$request.Instance"]
  Assert-True ($null -ne $channel) "Missing DMA channel: $name"
  Assert-True (!$channels.ContainsKey($channel)) "DMA channel conflict: $channel"
  $channels[$channel]=$name
}
# No changes to any other peripheral initializer or existing interrupt body.
$preserved = 0
foreach ($rel in @('Core/Src/main.c','Core/Src/stm32wbxx_hal_msp.c','Core/Src/stm32wbxx_it.c')) {
  $old = Read-Text (Join-Path $BackupRoot $rel)
  $new = Read-Text (Join-Path $ProjectRoot $rel)
  $names = [regex]::Matches($old,'(?m)^(?:static\s+)?(?:void|int)\s+([A-Za-z_]\w*)\([^;]*?\)\s*\{')
  foreach ($match in $names) {
    $name = $match.Groups[1].Value
    if ($name -in @('MX_DMA_Init','HAL_QSPI_MspInit','HAL_QSPI_MspDeInit')) { continue }
    Assert-True ((Read-Function $old $name) -ceq (Read-Function $new $name)) "Unrelated function changed: $name"
    $preserved++
  }
}
Write-Output "PASS: QSPI DMA binding, transfer/status IRQs, 6 non-conflicting channels, unchanged existing .ioc settings, $preserved preserved peripheral/IRQ functions."
Write-Output 'Source consistency check only; Flash transfer completion must still be verified on hardware.'

