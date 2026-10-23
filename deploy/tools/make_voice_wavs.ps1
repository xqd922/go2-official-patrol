<#
.SYNOPSIS
  Generate Chinese announcement WAV files for a station audio/ directory
  using the built-in Windows SAPI voice - offline, no TTS service.

.DESCRIPTION
  Reads a UTF-8 JSON file mapping speech_id -> announcement text
  (default: deploy/site.example/voice_lines.json) and writes one 16-bit PCM
  WAV per id. The result is what voice_player plays on Ubuntu (aplay) or on
  Windows (Media.SoundPlayer fallback), so the same files work for the
  offline demo and for a real robot.

  Run with Windows PowerShell 5.1 (powershell.exe); PowerShell 7 cannot load
  System.Speech.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File deploy/tools/make_voice_wavs.ps1 `
      -OutDir deploy/site.example/audio

.EXAMPLE
  # demo station: keep the beeps for the other ids, speak only the turn warning
  powershell -ExecutionPolicy Bypass -File deploy/tools/make_voice_wavs.ps1 `
      -OutDir tools/offline_demo/site-demo/audio -Ids turn_careful
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$OutDir,
    [string]$Lines = "",
    [string]$Ids = "",
    [string]$Voice = "",
    [int]$Rate = 0,
    [int]$Volume = 100
)

$ErrorActionPreference = "Stop"

try {
    Add-Type -AssemblyName System.Speech
} catch {
    throw "System.Speech unavailable. Run this with Windows PowerShell (powershell.exe), not PowerShell 7."
}

if (-not $Lines) {
    $Lines = Join-Path $PSScriptRoot "..\site.example\voice_lines.json"
}
if (-not (Test-Path -LiteralPath $Lines)) {
    throw "announcement file not found: $Lines"
}

$map = Get-Content -Raw -Encoding UTF8 -LiteralPath $Lines | ConvertFrom-Json

$speech = New-Object System.Speech.Synthesis.SpeechSynthesizer
if (-not $Voice) {
    $zh = $speech.GetInstalledVoices() |
        Where-Object { $_.VoiceInfo.Culture.Name -like "zh-*" } |
        Select-Object -First 1
    if (-not $zh) {
        throw "no zh-CN voice installed; install one in Windows 语言设置 or pass -Voice <name>"
    }
    $Voice = $zh.VoiceInfo.Name
}
$speech.SelectVoice($Voice)
$speech.Rate = $Rate
$speech.Volume = $Volume

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$outFull = (Resolve-Path -LiteralPath $OutDir).Path

$names = if ($Ids) { $Ids -split "," | ForEach-Object { $_.Trim() } }
         else { $map.PSObject.Properties.Name }

$count = 0
foreach ($name in $names) {
    $prop = $map.PSObject.Properties[$name]
    if (-not $prop -or -not $prop.Value) {
        Write-Warning "skip '$name': no text in $Lines"
        continue
    }
    $text = [string]$prop.Value
    $path = Join-Path $outFull "$name.wav"
    $speech.SetOutputToWaveFile($path)
    $speech.Speak($text)
    $speech.SetOutputToNull()
    $size = (Get-Item -LiteralPath $path).Length
    Write-Host ("[make_voice_wavs] {0}.wav  <- {1}  ({2:N0} bytes)" -f $name, $text, $size)
    $count++
}
$speech.Dispose()
Write-Host "[make_voice_wavs] voice=$Voice  generated $count file(s) in $outFull"
