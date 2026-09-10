$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$adcSource = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\adc.c') -Raw
$timSource = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\tim.c') -Raw

function Assert-Contains {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -notmatch $Pattern) {
        throw $Message
    }
}

function Assert-NotContains {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -match $Pattern) {
        throw $Message
    }
}

Assert-Contains $adcSource 'InjectedNbrOfConversion\s*=\s*2' `
    'ADC1 injected sequence must contain exactly two conversions.'
Assert-Contains $adcSource 'InjectedChannel\s*=\s*ADC_CHANNEL_5[\s\S]*InjectedRank\s*=\s*ADC_INJECTED_RANK_1' `
    'PB1/ADC1_INP5 must be injected rank 1.'
Assert-Contains $adcSource 'InjectedChannel\s*=\s*ADC_CHANNEL_9[\s\S]*InjectedRank\s*=\s*ADC_INJECTED_RANK_2' `
    'PB0/ADC1_INP9 must be injected rank 2.'
Assert-NotContains $adcSource 'sConfigInjected\.InjectedChannel\s*=\s*ADC_CHANNEL_16' `
    'PA0/ADC1_INP16 must not remain in the injected current sequence.'
Assert-Contains $adcSource 'ExternalTrigInjecConv\s*=\s*ADC_EXTERNALTRIGINJEC_T8_TRGO2' `
    'ADC1 injected conversions must be triggered by TIM8 TRGO2.'
Assert-Contains $adcSource 'ExternalTrigInjecConvEdge\s*=\s*ADC_EXTERNALTRIGINJECCONV_EDGE_RISING' `
    'ADC1 injected conversions must use one TIM8 TRGO2 rising edge per PWM period.'

Assert-Contains $timSource 'MasterOutputTrigger2\s*=\s*TIM_TRGO2_OC4REF' `
    'TIM8 TRGO2 must be sourced from OC4REF.'
Assert-Contains $timSource 'OCMode\s*=\s*TIM_OCMODE_PWM2[\s\S]*Pulse\s*=\s*11399[\s\S]*TIM_CHANNEL_4' `
    'TIM8 CH4 must generate the internal sample point at about 95 percent of ARR.'

Write-Output 'H7 current-sampling configuration checks passed'
