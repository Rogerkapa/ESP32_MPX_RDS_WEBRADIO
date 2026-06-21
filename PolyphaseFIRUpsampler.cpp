/*
 * =====================================================================================
 *
 *                      
 *                  Polyphase FIR Upsampler Implementation
 *
 * =====================================================================================
 *
 * File:         PolyphaseFIRUpsampler.cpp
 * Description:  High-performance 4x upsampling using polyphase FIR filter
 *
 * Purpose:
 *   This module implements a computationally-efficient 4x upsampler that converts
 *   Config::SAMPLE_RATE_ADC stereo audio to Config::SAMPLE_RATE_DAC for FM multiplex synthesis. It uses a polyphase
 *   decomposition of a 128-tap FIR filter to achieve high-quality anti-imaging
 *   filtering while minimizing computational cost.
 *
 * Algorithm: Polyphase FIR Filtering
 *   Traditional upsampling inserts zeros between samples, then applies a lowpass
 *   filter to remove imaging artifacts. However, this wastes 75% of multiplications
 *   on zero-valued samples. The polyphase approach reorganizes the filter into
 *   4 sub-filters (phases), each operating at the input rate (ADC rate), eliminating
 *   wasted multiply-accumulate operations.
 *
 * Filter Design:
 *   * Design method: Kaiser-windowed sinc (Aapprox80 dB)
 *   * Total taps: 128
 *   * Taps per phase: 32 (128 / 4 = 32)
 *   * Passband: 0-16 kHz (FM audio with a little extra top-end)
 *   * Transition band: ~16-19 kHz
 *   * Stopband: >=19 kHz (pilot protection, anti-imaging)
 *   * Latency: 47.5 samples @ SAMPLE_RATE_ADC approx 0.99 ms (when 48 kHz)
 *
 * Polyphase Decomposition:
 *   The 128-tap prototype filter H(z) is decomposed into 4 polyphase components:
 *     H(z) = E0(z^4) + z^-1*E1(z^4) + z^-2*E2(z^4) + z^-3*E3(z^4)
 *
 *   Each En(z) is a 24-tap filter operating at the ADC rate. For each input sample,
 *   we compute 4 output samples by convolving with each of the 4 phase filters.
 *
 * Computational Cost:
 *   Traditional approach: 128 taps x SAMPLE_RATE_DAC
 *   Polyphase approach:   32 taps x 4 phases x SAMPLE_RATE_ADC
 *   Speedup: ~4x reduction (eliminates multiply-by-zero operations)
 *
 * Circular Buffer Management:
 *   Input samples are stored in a circular delay line. To avoid boundary checks
 *   during convolution, the buffer uses mirrored wraparound: each sample is
 *   written to two locations (index and index - kTapsPerPhase). This ensures
 *   the convolution window always has contiguous valid data.
 *
 * ESP32 SIMD Optimization:
 *   The dot-product inner loop uses ESP32-S3 dsps_dotprod_f32_aes3(), which
 *   leverages hardware SIMD instructions for 2-4x speedup over scalar code.
 *
 * =====================================================================================
 */

#include "PolyphaseFIRUpsampler.h"

#include "Config.h"

#include "DSPCompat.h"

#include <cmath>

#include <cstring>

// ==================================================================================
//                       PROTOTYPE COEFFICIENT DESIGN (RUNTIME)
// ==================================================================================

namespace {

// Approximate modified Bessel function of the first kind, order 0: I0(x)
static inline float bessel_i0f(float x)
{
    float y = (x * x) * 0.25f;
    float sum = 1.0f;
    float t = y;
    for (int k = 1; k <= 10; ++k)
    {
        sum += t;
        t *= y / (static_cast<float>(k) * static_cast<float>(k));
    }
    return sum;
}

// Design a lowpass prototype using Kaiser-windowed sinc
static void design_prototype(float* h, std::size_t taps, float fs_out,
                             float f_pass_hz, float f_stop_hz, std::size_t L)
{
    // Cutoff at midpoint of transition band
    float f_c_hz = 0.5f * (f_pass_hz + f_stop_hz);
    float fc = f_c_hz / fs_out; // normalized (0..0.5)
    const float pi = 3.14159265358979323846f;

    // Kaiser window beta for ~80 dB sidelobe attenuation
    const float A = 80.0f;
    const float beta = 0.1102f * (A - 8.7f); // for A > 50 dB
    const float i0beta = bessel_i0f(beta);

    float m = static_cast<float>(taps - 1) * 0.5f;
    for (std::size_t n = 0; n < taps; ++n)
    {
        float k = static_cast<float>(n) - m;
        float x = (2.0f * static_cast<float>(n)) / static_cast<float>(taps - 1) - 1.0f; // -1..1
        float w = bessel_i0f(beta * sqrtf(fmaxf(0.0f, 1.0f - x * x))) / i0beta;

        float hn;
        if (fabsf(k) < 1e-6f)
        {
            hn = 2.0f * fc;
        }
        else
        {
            float arg = 2.0f * pi * fc * k;
            hn = sinf(arg) / (pi * k);
        }

        // Windowed sinc and scale by upsampling factor to maintain unity gain
        h[n] = w * hn * static_cast<float>(L);
    }
}

} // namespace

PolyphaseFIRUpsampler::PolyphaseFIRUpsampler() : state_index_(kTapsPerPhase)
{
    std::memset(phase_coeffs_, 0, sizeof(phase_coeffs_));
    reset();
}

void PolyphaseFIRUpsampler::initialize()
{
    initPhaseCoeffs();
    reset();
}

void PolyphaseFIRUpsampler::reset()
{
    std::memset(state_L_, 0, sizeof(state_L_));
    std::memset(state_R_, 0, sizeof(state_R_));
    state_index_ = static_cast<int>(kTapsPerPhase);
}

void PolyphaseFIRUpsampler::process(const float* __restrict input, float* __restrict output,
                                    std::size_t frames)
{
    if (!input || !output || frames == 0)
        return;

    for (std::size_t frame = 0; frame < frames; ++frame)
    {
        float in_L = input[frame * 2 + 0];
        float in_R = input[frame * 2 + 1];

        state_L_[state_index_]                                   = in_L;
        state_R_[state_index_]                                   = in_R;
        state_L_[state_index_ - static_cast<int>(kTapsPerPhase)] = in_L;
        state_R_[state_index_ - static_cast<int>(kTapsPerPhase)] = in_R;

        const float* __restrict baseL =
            &state_L_[state_index_ - static_cast<int>(kTapsPerPhase - 1)];
        const float* __restrict baseR =
            &state_R_[state_index_ - static_cast<int>(kTapsPerPhase - 1)];

        std::size_t out_base = frame * kUpsampleFactor * 2;
        for (std::size_t phase = 0; phase < kPhases; ++phase)
        {
            float yL = 0.0f;
            float yR = 0.0f;
            DSP_DOTPROD_F32(baseL, phase_coeffs_[phase], &yL,
                            static_cast<int>(kTapsPerPhase));
            DSP_DOTPROD_F32(baseR, phase_coeffs_[phase], &yR,
                            static_cast<int>(kTapsPerPhase));

            output[out_base + phase * 2 + 0] = yL;
            output[out_base + phase * 2 + 1] = yR;
        }

        ++state_index_;
        if (state_index_ >= static_cast<int>(kTapsPerPhase * 2))
            state_index_ = static_cast<int>(kTapsPerPhase);
    }
}

void PolyphaseFIRUpsampler::initPhaseCoeffs()
{
    // Design prototype for current output rate and desired band edges
    alignas(16) float proto[kTaps];
    const float fs_out = static_cast<float>(Config::SAMPLE_RATE_DAC);
    const float f_pass = 16000.0f;                       // preserve audio to 16 kHz
    const float f_stop = 19000.0f;                       // begin strong attenuation at pilot
    design_prototype(proto, kTaps, fs_out, f_pass, f_stop, kUpsampleFactor);

    // Reorganize into polyphase coefficients (oldest -> newest order per phase)
    for (std::size_t phase = 0; phase < kPhases; ++phase)
    {
        for (std::size_t t = 0; t < kTapsPerPhase; ++t)
        {
            std::size_t src_idx = (kTapsPerPhase - 1 - t) * kPhases + phase;
            phase_coeffs_[phase][t] = proto[src_idx];
        }
    }
}
