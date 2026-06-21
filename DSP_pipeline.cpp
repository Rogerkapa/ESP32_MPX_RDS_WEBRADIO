/*
 * =====================================================================================
 *
 *
 * =====================================================================================
 *
 * File:         DSP_pipeline.cpp
 * Description:  Implementation of the FM stereo encoding pipeline
 *
 * Processing Flow:
 *   ADC/WebRadio -> Upsample/LPF -> Pre-emphasis -> Stereo Matrix ->
 *   NCO Carriers -> MPX Synthesis -> RDS Injection -> DAC
 *
 * =====================================================================================
 */

#include "DSP_pipeline.h"

#include "Console.h"
#include "DisplayManager.h"
#include "ErrorHandler.h"
#include "RDSAssembler.h"
#include "RDSSynth.h"
#include "TaskBaseClass.h"
#include "WebRadioInput.h"

#include <Arduino.h>
#include <esp32-hal-cpu.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/task.h>

#include <algorithm>
#include <cmath>
#include <math.h>

// ==================================================================================
//                          CONSTRUCTOR
// ==================================================================================

DSP_pipeline::DSP_pipeline(IHardwareDriver *hardware_driver)
    : hardware_driver_(hardware_driver),
      pilot_19k_(19000.0f, static_cast<float>(Config::SAMPLE_RATE_DAC)),
      mpx_synth_(Config::PILOT_AMP, Config::DIFF_AMP)
{
    stats_.reset();
}

void DSP_pipeline::requestReset()
{
    reset_requested_ = true;
}

// ---------------- Runtime control state ----------------
static volatile bool s_rds_enable = Config::ENABLE_RDS_57K;
static volatile bool s_stereo_enable = Config::ENABLE_STEREO_SUBCARRIER_38K;
static volatile bool s_preemph_enable = Config::ENABLE_PREEMPHASIS;
static volatile bool s_pilot_auto = Config::PILOT_MUTE_ON_SILENCE;
static volatile bool s_pilot_enable = Config::ENABLE_STEREO_PILOT_19K;
static volatile float s_pilot_thresh = Config::SILENCE_RMS_THRESHOLD;
static volatile uint32_t s_pilot_hold_ms = Config::SILENCE_HOLD_MS;

bool g_dsp_pilot_muted_shadow = false;

void DSP_pipeline::setStereoEnable(bool en)
{
    s_stereo_enable = en;
}

bool DSP_pipeline::getStereoEnable()
{
    return s_stereo_enable;
}

void DSP_pipeline::setRdsEnable(bool en)
{
    s_rds_enable = en;
}

bool DSP_pipeline::getRdsEnable()
{
    return s_rds_enable;
}

void DSP_pipeline::setPreemphEnable(bool en)
{
    s_preemph_enable = en;
}

bool DSP_pipeline::getPreemphEnable()
{
    return s_preemph_enable;
}

void DSP_pipeline::setPilotAuto(bool en)
{
    s_pilot_auto = en;
}

bool DSP_pipeline::getPilotAuto()
{
    return s_pilot_auto;
}

void DSP_pipeline::setPilotThresh(float thr)
{
    s_pilot_thresh = thr;
}

float DSP_pipeline::getPilotThresh()
{
    return s_pilot_thresh;
}

void DSP_pipeline::setPilotHold(uint32_t ms)
{
    s_pilot_hold_ms = ms;
}

uint32_t DSP_pipeline::getPilotHold()
{
    return s_pilot_hold_ms;
}

void DSP_pipeline::setPilotEnable(bool en)
{
    s_pilot_enable = en;
}

bool DSP_pipeline::getPilotEnable()
{
    return s_pilot_enable;
}

bool DSP_pipeline::getPilotActive()
{
    extern bool g_dsp_pilot_muted_shadow;
    return s_pilot_enable && !g_dsp_pilot_muted_shadow;
}

// ==================================================================================
//                          INITIALIZATION
// ==================================================================================

bool DSP_pipeline::begin()
{
    if (Config::ENABLE_BOOT_INFO_LOGS)
    {
        Console::enqueuef(LogLevel::INFO, "ESP32-S3 Audio DSP: %u Hz -> %u Hz",
                          (unsigned)Config::SAMPLE_RATE_ADC,
                          (unsigned)Config::SAMPLE_RATE_DAC);
        Console::enqueuef(LogLevel::INFO, "DSP_pipeline running on Core %d", xPortGetCoreID());
    }

    if (!hardware_driver_ || !hardware_driver_->isReady())
    {
        Console::enqueue(LogLevel::ERROR,
                         "Hardware driver not ready (initialize via SystemContext first)");
        return false;
    }

    preemphasis_.configure(Config::PREEMPHASIS_ALPHA, Config::PREEMPHASIS_GAIN);

    upsampler_.initialize();

    if (Config::ENABLE_RDS_57K)
    {
        rds_synth_.configure(static_cast<float>(Config::SAMPLE_RATE_DAC));
    }

    stats_.reset();
    uint64_t now = esp_timer_get_time();
    stats_.start_time_us = now;
    stats_.last_print_us = now;

    last_above_thresh_us_ = esp_timer_get_time();
    pilot_muted_ = false;
    mpx_synth_.setPilotAmp(Config::PILOT_AMP);

    if (Config::ENABLE_BOOT_INFO_LOGS)
    {
        Console::enqueue(LogLevel::INFO, "System Ready - Starting Audio Processing");
    }

    return true;
}

void DSP_pipeline::performSoftReset()
{
    mpx_synth_.setPilotAmp(0.0f);

    preemphasis_.reset();
    upsampler_.reset();
    rds_synth_.reset();
    pilot_19k_.reset();

    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    memset(tx_buffer_, 0, sizeof(tx_buffer_));
    memset(rx_f32_, 0, sizeof(rx_f32_));
    memset(tx_f32_, 0, sizeof(tx_f32_));
    memset(pilot_buffer_, 0, sizeof(pilot_buffer_));
    memset(subcarrier_buffer_, 0, sizeof(subcarrier_buffer_));
    memset(mono_buffer_, 0, sizeof(mono_buffer_));
    memset(diff_buffer_, 0, sizeof(diff_buffer_));
    memset(mpx_buffer_, 0, sizeof(mpx_buffer_));
    memset(carrier57_buffer_, 0, sizeof(carrier57_buffer_));
    memset(rds_buffer_, 0, sizeof(rds_buffer_));

    stats_.reset();
    last_above_thresh_us_ = esp_timer_get_time();
    pilot_muted_ = false;
    normalyser_source_ = AudioInputSource::COUNT;
    normalyser_start_us_ = 0;
    normalyser_sum_sq_ = 0.0;
    normalyser_frames_ = 0;
    normalyser_gain_ = 1.0f;
    normalyser_applied_gain_ = 1.0f;
    normalyser_frozen_ = false;

    if (hardware_driver_)
    {
        hardware_driver_->reset();
    }

    mpx_synth_.setPilotAmp(Config::PILOT_AMP);
}

// ==================================================================================
//                    ORCHESTRATION HELPER METHODS
// ==================================================================================

bool DSP_pipeline::readAndConvertAudio(std::size_t &frames_read,
                                       float &l_peak,
                                       float &r_peak,
                                       float &l_rms,
                                       float &r_rms,
                                       uint32_t &rx_wait_us,
                                       uint32_t cpu_mhz,
                                       uint32_t &deint_us)
{
    using namespace Config;

    auto synth_silence = [&](std::size_t frames)
    {
        frames_read = frames;
        l_peak = r_peak = 0.0f;
        l_rms = r_rms = 0.0f;

        std::size_t samples = frames * 2;
        for (std::size_t i = 0; i < samples; ++i)
        {
            rx_f32_[i] = 0.0f;
        }

        rx_wait_us = 0;
        deint_us = 0;
        vTaskDelay(1);
        return true;
    };

    if (!Config::ENABLE_AUDIO)
    {
        return synth_silence(Config::BLOCK_SIZE);
    }

    // ==================================================================================
    // WEB RADIO INPUT
    // ==================================================================================
    if (Config::USE_WEBRADIO && webRadioUsingStream())
    {
        if (hardware_driver_->isInputEnabled())
            hardware_driver_->setInputEnabled(false);

        frames_read = Config::BLOCK_SIZE;

        uint32_t tr0_web = cpu_mhz ? ESP.getCycleCount() : 0;

        bool got = webRadioGetInterleaved(rx_f32_, frames_read);

        if (cpu_mhz)
        {
            uint32_t tr1_web = ESP.getCycleCount();
            rx_wait_us = (tr1_web - tr0_web) / cpu_mhz;
        }
        else
        {
            rx_wait_us = 0;
        }

        if (!got)
        {
            return synth_silence(Config::BLOCK_SIZE);
        }

        float l_sum_sq = 0.0f;
        float r_sum_sq = 0.0f;

        l_peak = 0.0f;
        r_peak = 0.0f;

        for (std::size_t f = 0; f < frames_read; ++f)
        {
            float vl = rx_f32_[(f * 2) + 0];
            float vr = rx_f32_[(f * 2) + 1];

            l_sum_sq += vl * vl;
            r_sum_sq += vr * vr;

            l_peak = std::max(l_peak, fabsf(vl));
            r_peak = std::max(r_peak, fabsf(vr));
        }

        l_rms = (frames_read > 0) ? sqrtf(l_sum_sq / static_cast<float>(frames_read)) : 0.0f;
        r_rms = (frames_read > 0) ? sqrtf(r_sum_sq / static_cast<float>(frames_read)) : 0.0f;

        deint_us = 0;

        return true;
    }

    if (webRadioCurrentSource() == AudioInputSource::TEST_LR)
    {
        if (hardware_driver_->isInputEnabled())
            hardware_driver_->setInputEnabled(false);

        static float tone_phase_a = 0.0f;
        static float tone_phase_b = 0.0f;
        constexpr float two_pi = 6.28318530717958647692f;
        const float phase_inc_a = Config::TEST_TONE_FREQ_A_HZ / (float)Config::SAMPLE_RATE_ADC;
        const float phase_inc_b = Config::TEST_TONE_FREQ_B_HZ / (float)Config::SAMPLE_RATE_ADC;

        frames_read = Config::BLOCK_SIZE;
        l_peak = r_peak = 0.0f;
        float l_sum_sq = 0.0f;
        float r_sum_sq = 0.0f;

        for (std::size_t f = 0; f < frames_read; ++f)
        {
            float tone_l = sinf(two_pi * tone_phase_a) * Config::TEST_TONE_AMP;
            float tone_r = sinf(two_pi * tone_phase_b) * Config::TEST_TONE_AMP;

            tone_phase_a += phase_inc_a;
            if (tone_phase_a >= 1.0f)
                tone_phase_a -= 1.0f;

            tone_phase_b += phase_inc_b;
            if (tone_phase_b >= 1.0f)
                tone_phase_b -= 1.0f;

            float l = tone_l;
            float r = tone_r;

            rx_f32_[(f * 2) + 0] = l;
            rx_f32_[(f * 2) + 1] = r;

            float al = fabsf(l);
            float ar = fabsf(r);
            if (al > l_peak)
                l_peak = al;
            if (ar > r_peak)
                r_peak = ar;

            l_sum_sq += l * l;
            r_sum_sq += r * r;
        }

        l_rms = sqrtf(l_sum_sq / (float)frames_read);
        r_rms = sqrtf(r_sum_sq / (float)frames_read);
        rx_wait_us = 0;
        deint_us = 0;
        return true;
    }

    // ==================================================================================
    // ADC / I2S INPUT
    // ==================================================================================
    const bool adc_mono_left = (webRadioCurrentSource() == AudioInputSource::ADC_MONO ||
                                webRadioCurrentSource() == AudioInputSource::ADC_RAW_MONO);
    if (!hardware_driver_->isInputEnabled() && !hardware_driver_->setInputEnabled(true))
    {
        return synth_silence(Config::BLOCK_SIZE);
    }

    size_t bytes_read = 0;
    uint32_t tr0 = cpu_mhz ? ESP.getCycleCount() : 0;

    bool ok = hardware_driver_->read(rx_buffer_,
                                     sizeof(rx_buffer_),
                                     bytes_read,
                                     Config::I2S_READ_TIMEOUT_MS);

    if (cpu_mhz)
    {
        uint32_t tr1 = ESP.getCycleCount();
        rx_wait_us = (tr1 - tr0) / cpu_mhz;
    }
    else
    {
        rx_wait_us = 0;
    }

    if (!ok || bytes_read == 0)
    {
        return synth_silence(Config::BLOCK_SIZE);
    }

    const std::size_t frame_bytes = 2 * static_cast<std::size_t>(BYTES_PER_SAMPLE);
    std::size_t valid_frames = bytes_read / frame_bytes;
    if (valid_frames == 0)
    {
        return synth_silence(Config::BLOCK_SIZE);
    }
    if (valid_frames > Config::BLOCK_SIZE)
        valid_frames = Config::BLOCK_SIZE;

    // O DAC/MPX deve receber sempre blocos completos. Se o RX devolver uma
    // leitura parcial, seguramos o ultimo sample valido para evitar um buraco
    // curto no fluxo de saida, que aparece como click no MPX/FFT.
    static int32_t s_adc_hold_l = 0;
    static int32_t s_adc_hold_r = 0;
    s_adc_hold_l = rx_buffer_[(valid_frames - 1) * 2 + 0];
    s_adc_hold_r = rx_buffer_[(valid_frames - 1) * 2 + 1];
    for (std::size_t f = valid_frames; f < Config::BLOCK_SIZE; ++f)
    {
        rx_buffer_[(f * 2) + 0] = s_adc_hold_l;
        rx_buffer_[(f * 2) + 1] = s_adc_hold_r;
    }

    frames_read = Config::BLOCK_SIZE;

    uint32_t tc0 = cpu_mhz ? ESP.getCycleCount() : 0;

    float l_sum_sq = 0.0f;
    float r_sum_sq = 0.0f;

    l_peak = 0.0f;
    r_peak = 0.0f;

    for (std::size_t f = 0; f < frames_read; ++f)
    {
        std::size_t iL = (f * 2) + 0;
        std::size_t iR = (f * 2) + 1;

        float vl = static_cast<float>(rx_buffer_[iL]) / Q31_FULL_SCALE;
        float vr = adc_mono_left ? vl : (static_cast<float>(rx_buffer_[iR]) / Q31_FULL_SCALE);

        if (!Config::ENABLE_AUDIO)
        {
            vl = 0.0f;
            vr = 0.0f;
        }

        rx_f32_[iL] = vl;
        rx_f32_[iR] = vr;

        l_sum_sq += vl * vl;
        r_sum_sq += vr * vr;

        l_peak = std::max(l_peak, std::fabs(vl));
        r_peak = std::max(r_peak, std::fabs(vr));
    }

    l_rms = (frames_read > 0) ? sqrtf(l_sum_sq / static_cast<float>(frames_read)) : 0.0f;
    r_rms = (frames_read > 0) ? sqrtf(r_sum_sq / static_cast<float>(frames_read)) : 0.0f;

    if (cpu_mhz)
    {
        uint32_t tc1 = ESP.getCycleCount();
        deint_us = (tc1 - tc0) / cpu_mhz;
    }
    else
    {
        deint_us = 0;
    }

    return true;
}

void DSP_pipeline::updateVUMeters(float l_peak,
                                  float r_peak,
                                  float l_rms,
                                  float r_rms,
                                  std::size_t frames_read)
{
    static uint64_t s_last_vu_us = 0;
    uint64_t now_us = esp_timer_get_time();

    if ((now_us - s_last_vu_us) < Config::VU_UPDATE_INTERVAL_US)
        return;

    s_last_vu_us = now_us;

    VUSample vu{};
    vu.l_rms = l_rms;
    vu.r_rms = r_rms;
    vu.l_peak = l_peak;
    vu.r_peak = r_peak;

    vu.l_dbfs = (l_rms > 0.0f)
                    ? (20.0f * log10f(std::min(l_peak, Config::DBFS_REF) / Config::DBFS_REF))
                    : -120.0f;

    vu.r_dbfs = (r_rms > 0.0f)
                    ? (20.0f * log10f(std::min(r_peak, Config::DBFS_REF) / Config::DBFS_REF))
                    : -120.0f;

    vu.frames = static_cast<uint32_t>(frames_read);
    vu.ts_us = static_cast<uint32_t>(now_us & 0xFFFFFFFFu);

    DisplayManager::enqueue(vu);
}

void DSP_pipeline::applyStartupNormalyser(std::size_t frames_read,
                                          float &l_peak,
                                          float &r_peak,
                                          float &l_rms,
                                          float &r_rms)
{
    if (!Config::STARTUP_NORMALYSER_ENABLED || frames_read == 0)
        return;

    AudioInputSource src = webRadioCurrentSource();
    uint64_t now_us = esp_timer_get_time();

    // A NJOY e a fonte de teste sao referencias. Saem intactas: sem medicao,
    // sem correcao, sem hipotese de o normalyser alterar o nivel original.
    if (src == AudioInputSource::NJOY ||
        src == AudioInputSource::ADC ||
        src == AudioInputSource::TEST_LR ||
        src == AudioInputSource::ADC_MONO ||
        src == AudioInputSource::ZERO_DAC ||
        src == AudioInputSource::ADC_RAW_MONO)
    {
        if (src != normalyser_source_)
        {
            Console::enqueuef(LogLevel::INFO,
                              "Startup normalyser bypass: source=%s gain=1.00",
                              webRadioCurrentSourceName());
        }
        normalyser_source_ = src;
        normalyser_gain_ = 1.0f;
        normalyser_applied_gain_ = 1.0f;
        normalyser_frozen_ = true;
        return;
    }

    if (src != normalyser_source_)
    {
        normalyser_source_ = src;
        normalyser_start_us_ = now_us;
        normalyser_sum_sq_ = 0.0;
        normalyser_frames_ = 0;
        normalyser_gain_ = 1.0f;
        normalyser_applied_gain_ = Config::STARTUP_NORMALYSER_MEASURING_GAIN;
        normalyser_frozen_ = false;
        Console::enqueuef(LogLevel::INFO,
                          "Startup normalyser measuring source %s",
                          webRadioCurrentSourceName());
    }

    if (!normalyser_frozen_)
    {
        for (std::size_t f = 0; f < frames_read; ++f)
        {
            float l = rx_f32_[(f * 2) + 0];
            float r = rx_f32_[(f * 2) + 1];
            normalyser_sum_sq_ += 0.5 * ((double)l * (double)l + (double)r * (double)r);
        }
        normalyser_frames_ += (uint32_t)frames_read;

        if (!normalyser_frozen_ &&
            (now_us - normalyser_start_us_) >= ((uint64_t)Config::STARTUP_NORMALYSER_MEASURE_MS * 1000ULL) &&
            normalyser_frames_ > 0)
        {
            float measured_rms = sqrtf((float)(normalyser_sum_sq_ / (double)normalyser_frames_));

            if (measured_rms >= Config::STARTUP_NORMALYSER_MIN_INPUT_RMS &&
                measured_rms > Config::STARTUP_NORMALYSER_TARGET_RMS)
            {
                normalyser_gain_ = Config::STARTUP_NORMALYSER_TARGET_RMS / measured_rms;
                if (normalyser_gain_ < Config::STARTUP_NORMALYSER_MIN_GAIN)
                    normalyser_gain_ = Config::STARTUP_NORMALYSER_MIN_GAIN;
                if (normalyser_gain_ > Config::STARTUP_NORMALYSER_MAX_GAIN)
                    normalyser_gain_ = Config::STARTUP_NORMALYSER_MAX_GAIN;
            }
            else
            {
                normalyser_gain_ = 1.0f;
            }

            normalyser_frozen_ = true;
            Console::enqueuef(LogLevel::INFO,
                              "Startup normalyser frozen: source=%s rms=%.3f measuring=%.2f gain=%.2f",
                              webRadioCurrentSourceName(),
                              (double)measured_rms,
                              (double)Config::STARTUP_NORMALYSER_MEASURING_GAIN,
                              (double)normalyser_gain_);
        }
    }

    float target_gain = normalyser_frozen_
                            ? normalyser_gain_
                            : Config::STARTUP_NORMALYSER_MEASURING_GAIN;

    if (target_gain < 0.0f)
        target_gain = 0.0f;

    const float ramp_ms = (Config::STARTUP_NORMALYSER_GAIN_RAMP_MS > 0)
                              ? (float)Config::STARTUP_NORMALYSER_GAIN_RAMP_MS
                              : 1.0f;
    float max_step = ((float)frames_read * 1000.0f) /
                     ((float)Config::SAMPLE_RATE_ADC * ramp_ms);
    if (max_step < 0.0001f)
        max_step = 0.0001f;

    if (normalyser_applied_gain_ < target_gain)
    {
        normalyser_applied_gain_ += max_step;
        if (normalyser_applied_gain_ > target_gain)
            normalyser_applied_gain_ = target_gain;
    }
    else if (normalyser_applied_gain_ > target_gain)
    {
        normalyser_applied_gain_ -= max_step;
        if (normalyser_applied_gain_ < target_gain)
            normalyser_applied_gain_ = target_gain;
    }

    if (normalyser_applied_gain_ >= 0.9999f)
        return;

    float l_sum_sq = 0.0f;
    float r_sum_sq = 0.0f;
    l_peak = 0.0f;
    r_peak = 0.0f;

    for (std::size_t f = 0; f < frames_read; ++f)
    {
        float l = rx_f32_[(f * 2) + 0] * normalyser_applied_gain_;
        float r = rx_f32_[(f * 2) + 1] * normalyser_applied_gain_;

        if (l > 1.0f)
            l = 1.0f;
        else if (l < -1.0f)
            l = -1.0f;

        if (r > 1.0f)
            r = 1.0f;
        else if (r < -1.0f)
            r = -1.0f;

        rx_f32_[(f * 2) + 0] = l;
        rx_f32_[(f * 2) + 1] = r;

        l_sum_sq += l * l;
        r_sum_sq += r * r;
        l_peak = std::max(l_peak, fabsf(l));
        r_peak = std::max(r_peak, fabsf(r));
    }

    l_rms = sqrtf(l_sum_sq / (float)frames_read);
    r_rms = sqrtf(r_sum_sq / (float)frames_read);
}

void DSP_pipeline::convertFloatToInt32(std::size_t frames_read)
{
    using namespace Config;

    std::size_t out_samples = frames_read * UPSAMPLE_FACTOR * 2;

    constexpr float Q31_SCALE = 2147483647.0f;

    for (std::size_t i = 0; i < out_samples; ++i)
    {
        float v = tx_f32_[i];

        if (v > 1.0f)
            v = 1.0f;
        else if (v < -1.0f)
            v = -1.0f;

        tx_buffer_[i] = static_cast<int32_t>(v * Q31_SCALE);
    }
}

void DSP_pipeline::writeToDAC(std::size_t frames_read)
{
    using namespace Config;

    size_t bytes_to_write = frames_read * UPSAMPLE_FACTOR * 2 * BYTES_PER_SAMPLE;
    size_t bytes_written = 0;
    uint64_t now_us = esp_timer_get_time();
    static uint64_t s_last_i2s_error_log_us = 0;
    static uint64_t s_last_underrun_log_us = 0;
    constexpr uint64_t LOG_THROTTLE_US = 2000000ULL;

    const uint8_t *write_ptr = reinterpret_cast<const uint8_t *>(tx_buffer_);
    bool write_ok = true;

    while (bytes_written < bytes_to_write)
    {
        size_t chunk_written = 0;
        size_t remaining = bytes_to_write - bytes_written;

        if (!hardware_driver_->write(reinterpret_cast<const int32_t *>(write_ptr + bytes_written),
                                     remaining,
                                     chunk_written,
                                     Config::I2S_WRITE_TIMEOUT_MS))
        {
            write_ok = false;
            break;
        }

        if (chunk_written == 0)
        {
            write_ok = false;
            break;
        }

        bytes_written += chunk_written;
    }

    if (!write_ok)
    {
        if ((now_us - s_last_i2s_error_log_us) >= LOG_THROTTLE_US)
        {
            s_last_i2s_error_log_us = now_us;

            auto derr = hardware_driver_->getLastError();
            int perr = hardware_driver_->getErrorStatus();
            const char *err_name = esp_err_to_name(static_cast<esp_err_t>(perr));
            char details[96];

            switch (derr)
            {
            case DriverError::Timeout:
                snprintf(details, sizeof(details), "I2S TX timeout (esp:%s)", err_name);
                ErrorHandler::logError(ErrorCode::TIMEOUT, "DSP_pipeline::writeToDAC", details);
                break;

            case DriverError::InvalidArgument:
                snprintf(details, sizeof(details), "I2S TX invalid arg (esp:%s)", err_name);
                ErrorHandler::logError(ErrorCode::INVALID_PARAM, "DSP_pipeline::writeToDAC", details);
                break;

            case DriverError::InvalidState:
            case DriverError::NotInitialized:
                snprintf(details, sizeof(details), "I2S TX not ready (esp:%s)", err_name);
                ErrorHandler::logError(ErrorCode::I2S_NOT_INITIALIZED,
                                       "DSP_pipeline::writeToDAC",
                                       details);
                break;

            default:
                snprintf(details, sizeof(details), "I2S TX error (esp:%s)", err_name);
                ErrorHandler::logError(ErrorCode::I2S_WRITE_ERROR,
                                       "DSP_pipeline::writeToDAC",
                                       details);
                break;
            }
        }

        ++stats_.errors;
    }

    if (bytes_written != bytes_to_write &&
        (now_us - s_last_underrun_log_us) >= LOG_THROTTLE_US)
    {
        s_last_underrun_log_us = now_us;
        Console::enqueuef(LogLevel::WARN,
                          "Underrun (wrote %u/%u bytes)",
                          (unsigned)bytes_written,
                          (unsigned)bytes_to_write);
    }

    ++stats_.loops_completed;
}

void DSP_pipeline::updatePerformanceMetrics(uint32_t total_us, std::size_t frames_read)
{
    using namespace Config;
    (void)total_us;
    uint64_t now = esp_timer_get_time();

    if (STATS_PRINT_INTERVAL_US > 0 && now - stats_.last_print_us >= STATS_PRINT_INTERVAL_US)
    {
        stats_.last_print_us = now;

        float available_us = (frames_read * 1'000'000.0f) /
                             static_cast<float>(SAMPLE_RATE_ADC);

        float compute_us = static_cast<float>(stats_.total.current) -
                           static_cast<float>(stats_.stage_i2s_rx_wait.current);

        if (compute_us < 0.0f)
            compute_us = 0.0f;

        float cpu_usage = (available_us > 0.0f) ? (compute_us / available_us * 100.0f) : 0.0f;
        float cpu_headroom = 100.0f - cpu_usage;

        printPerformance(frames_read, available_us, cpu_usage, cpu_headroom);
    }

}

// ==================================================================================
//                          MAIN AUDIO PROCESSING LOOP
// ==================================================================================

void DSP_pipeline::process()
{
    if (reset_requested_)
    {
        performSoftReset();
        reset_requested_ = false;
    }

    using namespace Config;

    if (webRadioCurrentSource() == AudioInputSource::ZERO_DAC)
    {
        if (hardware_driver_->isInputEnabled())
            hardware_driver_->setInputEnabled(false);

        memset(tx_buffer_, 0, sizeof(tx_buffer_));
        writeToDAC(Config::BLOCK_SIZE);
        return;
    }

    if (webRadioCurrentSource() == AudioInputSource::ADC_RAW_MONO)
    {
        std::size_t frames_read = 0;
        float l_peak = 0.0f;
        float r_peak = 0.0f;
        float l_rms = 0.0f;
        float r_rms = 0.0f;
        uint32_t rx_wait_us_local = 0;
        uint32_t deint_us_local = 0;

        if (!readAndConvertAudio(frames_read,
                                 l_peak,
                                 r_peak,
                                 l_rms,
                                 r_rms,
                                 rx_wait_us_local,
                                 0,
                                 deint_us_local))
        {
            return;
        }

        updateVUMeters(l_peak, r_peak, l_rms, r_rms, frames_read);

        constexpr float q31_scale = 2147483647.0f;
        for (std::size_t f = 0; f < frames_read; ++f)
        {
            float v = rx_f32_[(f * 2) + 0];
            if (v > 0.9999999f)
                v = 0.9999999f;
            else if (v < -1.0f)
                v = -1.0f;

            int32_t q = static_cast<int32_t>(v * q31_scale);
            for (std::size_t phase = 0; phase < Config::UPSAMPLE_FACTOR; ++phase)
            {
                std::size_t out = ((f * Config::UPSAMPLE_FACTOR) + phase) * 2;
                tx_buffer_[out + 0] = q;
                tx_buffer_[out + 1] = q;
            }
        }

        writeToDAC(frames_read);
        return;
    }

    constexpr bool profile_audio =
        Config::ENABLE_AUDIO_PROFILING || (Config::STATS_PRINT_INTERVAL_US > 0);
    uint32_t cpu_mhz = profile_audio ? getCpuFrequencyMhz() : 0;
    uint32_t t_start = profile_audio ? ESP.getCycleCount() : 0;

    std::size_t frames_read = 0;
    float l_peak = 0.0f;
    float r_peak = 0.0f;
    float l_rms = 0.0f;
    float r_rms = 0.0f;

    uint32_t rx_wait_us_local = 0;
    uint32_t deint_us_local = 0;

    if (!readAndConvertAudio(frames_read,
                             l_peak,
                             r_peak,
                             l_rms,
                             r_rms,
                             rx_wait_us_local,
                             cpu_mhz,
                             deint_us_local))
    {
        return;
    }

    uint32_t t0 = 0;
    uint32_t t1 = 0;

    if (profile_audio)
    {
        stats_.stage_i2s_rx_wait.update(rx_wait_us_local);
        stats_.stage_int_to_float.update(deint_us_local);
    }

    // Normalyser de arranque: mede por tempo, congela o ganho e aplica antes
    // do upsampler/pre-emphasis/MPX.
    applyStartupNormalyser(frames_read, l_peak, r_peak, l_rms, r_rms);

    updateVUMeters(l_peak, r_peak, l_rms, r_rms, frames_read);

    // STAGE 2: Upsample + audio low-pass
    t0 = profile_audio ? ESP.getCycleCount() : 0;
    upsampler_.process(rx_f32_, tx_f32_, frames_read);
    t1 = profile_audio ? ESP.getCycleCount() : 0;

    if (profile_audio)
    {
        uint32_t stage_upsample_us = (t1 - t0) / cpu_mhz;
        stats_.stage_upsample.update(stage_upsample_us);
    }

    std::size_t samples = frames_read * Config::UPSAMPLE_FACTOR;

    // STAGE 3: Pre-emphasis at DAC/MPX rate, after the audio low-pass.
    if (s_preemph_enable)
    {
        t0 = t1;
        preemphasis_.process(tx_f32_, samples);
        t1 = profile_audio ? ESP.getCycleCount() : 0;

        if (profile_audio)
        {
            uint32_t stage2_us = (t1 - t0) / cpu_mhz;
            stats_.stage_preemphasis.update(stage2_us);
        }
    }

    // STAGE 4: Stereo matrix
    t0 = t1;

    stereo_matrix_.process(tx_f32_, mono_buffer_, diff_buffer_, samples);

    t1 = profile_audio ? ESP.getCycleCount() : 0;

    if (profile_audio)
    {
        uint32_t stage_matrix_us = (t1 - t0) / cpu_mhz;
        stats_.stage_matrix.update(stage_matrix_us);
    }

    // STAGE 5: MPX synthesis + RDS
    t0 = t1;
    const bool adc_mono_direct = (webRadioCurrentSource() == AudioInputSource::ADC_MONO);

    if (!adc_mono_direct && s_pilot_auto && s_pilot_enable)
    {
        const float level = std::max(l_rms, r_rms);
        const uint64_t now_us = esp_timer_get_time();
        const uint64_t hold_us = static_cast<uint64_t>(s_pilot_hold_ms) * 1000ULL;

        if (level >= s_pilot_thresh)
        {
            last_above_thresh_us_ = now_us;
            if (pilot_muted_)
            {
                pilot_muted_ = false;
            }
        }
        else
        {
            if (!pilot_muted_ && (now_us - last_above_thresh_us_) >= hold_us)
            {
                pilot_muted_ = true;
            }
        }
    }

    bool need19 = (!adc_mono_direct && Config::ENABLE_STEREO_PILOT_19K && s_pilot_enable);
    bool need38 = (!adc_mono_direct && Config::ENABLE_AUDIO && s_stereo_enable);
    bool need57 = (!adc_mono_direct && s_rds_enable);

    if (need19 || need38 || need57)
    {
        pilot_19k_.generate_harmonics(pilot_buffer_,
                                      subcarrier_buffer_,
                                      carrier57_buffer_,
                                      samples);
    }

    {
        float desired_amp =
            (!adc_mono_direct && s_pilot_enable && !pilot_muted_) ? Config::PILOT_AMP : 0.0f;
        float desired_diff_amp =
            (!adc_mono_direct && s_stereo_enable) ? Config::DIFF_AMP : 0.0f;
        mpx_synth_.setPilotAmp(desired_amp);
        mpx_synth_.setDiffAmp(desired_diff_amp);
        g_dsp_pilot_muted_shadow = pilot_muted_;
    }

    mpx_synth_.process(mono_buffer_,
                       diff_buffer_,
                       pilot_buffer_,
                       subcarrier_buffer_,
                       mpx_buffer_,
                       samples);

    if (!adc_mono_direct && s_rds_enable)
    {
        uint32_t t_r0 = profile_audio ? ESP.getCycleCount() : 0;

        rds_synth_.processBlockWithCarrier(carrier57_buffer_,
                                           Config::RDS_AMP,
                                           rds_buffer_,
                                           samples);

        for (std::size_t i = 0; i < samples; ++i)
        {
            mpx_buffer_[i] += rds_buffer_[i];
        }

        if (profile_audio)
        {
            uint32_t t_r1 = ESP.getCycleCount();
            uint32_t rds_us = (t_r1 - t_r0) / cpu_mhz;
            stats_.stage_rds.update(rds_us);
        }
    }

    for (std::size_t i = 0; i < samples; ++i)
    {
        float mpx = mpx_buffer_[i];

        tx_f32_[(i * 2) + 0] = mpx;
        tx_f32_[(i * 2) + 1] = mpx;
    }

    t1 = profile_audio ? ESP.getCycleCount() : 0;

    if (profile_audio)
    {
        uint32_t stage_mpx_us = (t1 - t0) / cpu_mhz;
        stats_.stage_mpx.update(stage_mpx_us);
    }

    // STAGE 7: Float to int
    t0 = t1;

    convertFloatToInt32(frames_read);

    t1 = profile_audio ? ESP.getCycleCount() : 0;

    if (profile_audio)
    {
        uint32_t stage4_us = (t1 - t0) / cpu_mhz;
        stats_.stage_float_to_int.update(stage4_us);
    }

    // STAGE 7b: I2S DAC write
    writeToDAC(frames_read);

    // STAGE 8: Metrics
    if (profile_audio)
    {
        uint32_t t_end = t1;
        uint32_t total_us = (t_end - t_start) / cpu_mhz;
        stats_.total.update(total_us);
        updatePerformanceMetrics(total_us, frames_read);
    }

#if DIAGNOSTIC_PRINT_INTERVAL > 0
    int32_t peak_adc = Diagnostics::findPeakAbs(rx_buffer_, frames_read * 2);

    int32_t peak_after_pre = 0;

    for (std::size_t i = 0; i < frames_read * 2; ++i)
    {
        float v = rx_f32_[i];
        v = std::min(0.9999999f, std::max(-1.0f, v));
        rx_buffer_[i] = static_cast<int32_t>(v * 2147483647.0f);
    }

    peak_after_pre = Diagnostics::findPeakAbs(rx_buffer_, frames_read * 2);

    int32_t peak_after_fir =
        Diagnostics::findPeakAbs(tx_buffer_, frames_read * Config::UPSAMPLE_FACTOR * 2);

    ++diagnostic_counter_;

    if (diagnostic_counter_ >= DIAGNOSTIC_PRINT_INTERVAL)
    {
        diagnostic_counter_ = 0;

        float pre_db =
            20.0f * log10f(static_cast<float>(peak_after_pre) / static_cast<float>(peak_adc));

        float total_db =
            20.0f * log10f(static_cast<float>(peak_after_fir) / static_cast<float>(peak_adc));

        printDiagnostics(frames_read,
                         peak_adc,
                         peak_after_pre,
                         peak_after_fir,
                         pre_db,
                         total_db);
    }
#endif
}

// ==================================================================================
//                          FREERTOS TASK MANAGEMENT
// ==================================================================================

bool DSP_pipeline::startTaskInstance(int core_id,
                                     UBaseType_t priority,
                                     uint32_t stack_words)
{
    return TaskBaseClass::spawnTaskFor(this,
                                       "audio",
                                       stack_words,
                                       priority,
                                       core_id,
                                       DSP_pipeline::taskTrampoline,
                                       &task_handle_);
}

void DSP_pipeline::taskTrampoline(void *arg)
{
    auto *self = static_cast<DSP_pipeline *>(arg);

    if (!self->begin())
    {
        Console::enqueue(LogLevel::ERROR, "DSP_pipeline begin() failed");
        vTaskDelete(nullptr);
        return;
    }

    for (;;)
    {
        self->process();
    }
}

bool DSP_pipeline::startTask(IHardwareDriver *hardware_driver,
                             int core_id,
                             UBaseType_t priority,
                             uint32_t stack_words)
{
    static DSP_pipeline s_instance(hardware_driver);
    return s_instance.startTaskInstance(core_id, priority, stack_words);
}

// ==================================================================================
//                          PERFORMANCE LOGGING
// ==================================================================================

void DSP_pipeline::printPerformance(std::size_t frames_read,
                                    float available_us,
                                    float cpu_usage,
                                    float cpu_headroom)
{
    using namespace Config;

    if (!Console::shouldLog(LogLevel::INFO))
        return;

    (void)frames_read;

    Console::enqueue(LogLevel::INFO, "========================================");
    Console::enqueue(LogLevel::INFO, "Performance Stats");
    Console::enqueue(LogLevel::INFO, "========================================");

    Console::enqueuef(LogLevel::INFO, "Loops completed: %u", stats_.loops_completed);
    Console::enqueuef(LogLevel::INFO, "Errors: %u", stats_.errors);

    float uptime_s = (esp_timer_get_time() - stats_.start_time_us) / 1000000.0f;
    Console::enqueuef(LogLevel::INFO, "Uptime: %.1f seconds", uptime_s);

    Console::enqueue(LogLevel::INFO, "----------------------------------------");
    Console::enqueue(LogLevel::INFO, "Processing time:");

    Console::enqueuef(LogLevel::INFO,
                      "  Total (incl. I/O waits): %.2f us",
                      (double)stats_.total.current);

    {
        float compute_us =
            (float)stats_.total.current - (float)stats_.stage_i2s_rx_wait.current;

        if (compute_us < 0.0f)
            compute_us = 0.0f;

        Console::enqueuef(LogLevel::INFO,
                          "  Compute (excl. I/O waits): %.2f us",
                          (double)compute_us);
    }

    Console::enqueuef(LogLevel::INFO, "  Min: %.2f us", (double)stats_.total.min);
    Console::enqueuef(LogLevel::INFO, "  Max: %.2f us", (double)stats_.total.max);
    Console::enqueuef(LogLevel::INFO, "  Available: %.2f us", (double)available_us);
    Console::enqueuef(LogLevel::INFO, "CPU usage: %.1f%%", (double)cpu_usage);
    Console::enqueuef(LogLevel::INFO, "CPU headroom: %.1f%%", (double)cpu_headroom);

    Console::enqueue(LogLevel::INFO, "----------------------------------------");
    Console::enqueue(LogLevel::INFO, "Per-Stage Breakdown:");

    Console::enqueue(LogLevel::INFO, "  1a. I2S RX wait (block):");
    Console::enqueuef(LogLevel::INFO,
                      "     Cur: %6.2f us  Min: %6.2f us  Max: %6.2f us",
                      (double)stats_.stage_i2s_rx_wait.current,
                      (double)stats_.stage_i2s_rx_wait.min,
                      (double)stats_.stage_i2s_rx_wait.max);

    Console::enqueue(LogLevel::INFO, "  1b. Deinterleave (int->float):");
    Console::enqueuef(LogLevel::INFO,
                      "     Cur: %6.2f us  Min: %6.2f us  Max: %6.2f us",
                      (double)stats_.stage_int_to_float.current,
                      (double)stats_.stage_int_to_float.min,
                      (double)stats_.stage_int_to_float.max);

    Console::enqueue(LogLevel::INFO, "  2. Upsample 4x (FIR):");
    Console::enqueuef(LogLevel::INFO,
                      "     Cur: %6.2f us  Min: %6.2f us  Max: %6.2f us",
                      (double)stats_.stage_upsample.current,
                      (double)stats_.stage_upsample.min,
                      (double)stats_.stage_upsample.max);

    Console::enqueue(LogLevel::INFO, "  3. Pre-emphasis:");
    Console::enqueuef(LogLevel::INFO,
                      "     Cur: %6.2f us  Min: %6.2f us  Max: %6.2f us",
                      (double)stats_.stage_preemphasis.current,
                      (double)stats_.stage_preemphasis.min,
                      (double)stats_.stage_preemphasis.max);

    Console::enqueue(LogLevel::INFO, "  4. Stereo matrix:");
    Console::enqueuef(LogLevel::INFO,
                      "     Cur: %6.2f us  Min: %6.2f us  Max: %6.2f us",
                      (double)stats_.stage_matrix.current,
                      (double)stats_.stage_matrix.min,
                      (double)stats_.stage_matrix.max);

    Console::enqueue(LogLevel::INFO, "  5. MPX synthesis:");
    Console::enqueuef(LogLevel::INFO,
                      "     Cur: %6.2f us  Min: %6.2f us  Max: %6.2f us",
                      (double)stats_.stage_mpx.current,
                      (double)stats_.stage_mpx.min,
                      (double)stats_.stage_mpx.max);

    Console::enqueue(LogLevel::INFO, "  6. RDS injection:");
    Console::enqueuef(LogLevel::INFO,
                      "     Cur: %6.2f us  Min: %6.2f us  Max: %6.2f us",
                      (double)stats_.stage_rds.current,
                      (double)stats_.stage_rds.min,
                      (double)stats_.stage_rds.max);

    Console::enqueue(LogLevel::INFO, "  7. Conversion (float->int):");
    Console::enqueuef(LogLevel::INFO,
                      "     Cur: %6.2f us  Min: %6.2f us  Max: %6.2f us",
                      (double)stats_.stage_float_to_int.current,
                      (double)stats_.stage_float_to_int.min,
                      (double)stats_.stage_float_to_int.max);

    Console::enqueue(LogLevel::INFO, "----------------------------------------");
    Console::enqueuef(LogLevel::INFO, "Free heap: %u bytes", (unsigned)ESP.getFreeHeap());
    Console::enqueuef(LogLevel::INFO, "Min free heap: %u bytes", (unsigned)ESP.getMinFreeHeap());
    Console::enqueue(LogLevel::INFO, "========================================");
}

// ==================================================================================
//                          DIAGNOSTIC LOGGING
// ==================================================================================

void DSP_pipeline::printDiagnostics(std::size_t frames_read,
                                    int32_t peak_adc,
                                    int32_t peak_pre,
                                    int32_t peak_fir,
                                    float pre_db,
                                    float total_db)
{
    using namespace Config;

    (void)frames_read;

    Console::enqueue(LogLevel::INFO, "=== SIGNAL LEVEL DIAGNOSTIC ===");

    Console::enqueuef(LogLevel::INFO,
                      "ADC Peak: %d (%.1f%%)",
                      peak_adc,
                      (peak_adc / 2147483647.0f) * 100.0f);

    Console::enqueuef(LogLevel::INFO,
                      "After Pre: %d (%.1f%%)  Pre Gain: %.2f dB",
                      peak_pre,
                      (peak_pre / 2147483647.0f) * 100.0f,
                      pre_db);

    Console::enqueuef(LogLevel::INFO,
                      "After FIR: %d (%.1f%%)  Total Gain: %.2f dB",
                      peak_fir,
                      (peak_fir / 2147483647.0f) * 100.0f,
                      total_db);
}

// =====================================================================================
//                                END OF FILE
// =====================================================================================
