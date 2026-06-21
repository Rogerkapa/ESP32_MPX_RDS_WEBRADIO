#pragma once

#include <stddef.h>
#include <stdint.h>

enum class AudioInputSource : uint8_t
{
    NJOY = 0,
    SUNSHINE,
    SUNSHINE_ALT,
    VIRGIN_ROCK_2K,
    VIRGIN_CLASSIC,
    ADC,
    TEST_LR,
    ADC_MONO,
    ZERO_DAC,
    ADC_RAW_MONO,
    COUNT
};

bool webRadioInit();
void webRadioLoop();
void webRadioPollSourceButton();
bool webRadioUsingStream();
AudioInputSource webRadioCurrentSource();
const char *webRadioCurrentSourceName();

// Preenche buffer interleaved: L,R,L,R...
bool webRadioGetInterleaved(float *outInterleaved, size_t frames);
