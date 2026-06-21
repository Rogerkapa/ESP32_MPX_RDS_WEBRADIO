/*
 * =====================================================================================
 *
 *                     
 *
 * =====================================================================================
 *
 * File:         Config.h
 * Description:  Centralized configuration for audio DSP pipeline
 *
 * Purpose:
 *   This header contains all configurable parameters for the stereo encoder.
 *   Modify these values to adapt the system to different hardware or requirements.
 *
 * Configuration Categories:
 *   * GPIO Pin Assignments
 *   * Sample Rates and Timing
 *   * DSP Algorithm Parameters
 *   * FM Stereo MPX Levels
 *   * Diagnostic Options
 *   * VU Meter Display Settings
 *
 * =====================================================================================
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "secrets.h"

// ==================================================================================
//                         TARGET PLATFORM DETECTION (CENTRAL)
// ==================================================================================
// Define exactly one of PROJ_TARGET_ESP32 or PROJ_TARGET_ESP32S3 for project-wide use.
// Prefer ESP-IDF's CONFIG_IDF_TARGET_* if available; otherwise use SoC capabilities
// as a heuristic. Avoid hard errors here to keep Arduino builds smooth.

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#if __has_include(<soc/soc_caps.h>)
#include <soc/soc_caps.h>
#endif
#endif

// Manual override hook (can be defined in .ino or build flags)
// #define PROJ_TARGET_ESP32
// #define PROJ_TARGET_ESP32S3

#if !defined(PROJ_TARGET_ESP32) && !defined(PROJ_TARGET_ESP32S3)
#if defined(CONFIG_IDF_TARGET_ESP32)
#define PROJ_TARGET_ESP32
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define PROJ_TARGET_ESP32S3
#elif defined(SOC_USB_SERIAL_JTAG_SUPPORTED) && (SOC_USB_SERIAL_JTAG_SUPPORTED == 1)
// Heuristic: present on ESP32-S3, not on classic ESP32
#define PROJ_TARGET_ESP32S3
#pragma message("Detected ESP32-S3 via SOC caps")
#else
// Fallback default to keep compilation flowing
#define PROJ_TARGET_ESP32S3
#pragma message(                                                                                   \
    "Unknown target; defaulting to ESP32-S3. Define PROJ_TARGET_ESP32 or PROJ_TARGET_ESP32S3 to override.")
#endif
#endif

// ==================================================================================
//                           DIAGNOSTIC CONFIGURATION
// ==================================================================================

/**
 * Diagnostic Print Interval
 *
 * Controls how often detailed diagnostic information is printed to serial.
 * Set to 0 to disable diagnostic printing (recommended for production).
 *
 * Values:
 *   0     = Disabled (no diagnostic output, best performance)
 *   1-N   = Print diagnostics every N blocks (increases Serial traffic)
 *
 * Note: Performance logging (every 5 seconds) is independent and always enabled.
 */
#define DIAGNOSTIC_PRINT_INTERVAL 0

namespace Config
{
// ==================================================================================
//                              LOGGING
// ==================================================================================
/**
 * Verbose boot information.
 *
 * Keep disabled for normal operation to avoid unnecessary serial traffic during startup.
 * Errors and warnings are still logged.
 */
constexpr bool ENABLE_BOOT_INFO_LOGS = false;

// ==================================================================================
//                              FIRMWARE METADATA
// ==================================================================================
/**
 * Firmware Version String
 *
 * Semantic version or tag for the firmware build. Update this when making
 * a release so it is reported in SYST:VERSION? and on the splash screen.
 */
constexpr const char *FIRMWARE_VERSION = "1.1.0";

// ==================================================================================
//                              SPLASH SCREEN SETTINGS
// ==================================================================================
/**
 * Splash image vertical offset (pixels)
 *
 * Controls the Y position where the splash image starts.
 * Set this to trim the vertical placement if the image is shorter than 240 px.
 */
constexpr int SPLASH_TOP_Y = 70;

/**
 * Splash image height (pixels)
 *
 * Set this to the actual height of the LOGO image in SplashScreen.h.
 * Width is assumed to be 320 px.
 */
constexpr int SPLASH_HEIGHT = 133;

/**
 * Splash hold time.
 *
 * Keep this short so the main VU display takes over quickly after boot.
 */
constexpr uint32_t SPLASH_HOLD_MS = 1800u;

// ==================================================================================
//                              UI COLOR PALETTE (RGB565)
// ==================================================================================
/**
 * UI color palette for the on-device display (ST7789 - RGB565).
 * Adjust to taste or theme; values are 16-bit RGB565.
 */
namespace UI
{
constexpr uint16_t COLOR_BG = 0x0000;     // Black
constexpr uint16_t COLOR_TEXT = 0xFFFF;   // White
constexpr uint16_t COLOR_DIM = 0x7BEF;    // Mid gray
constexpr uint16_t COLOR_ACCENT = 0xFD20; // Amber/Orange accent
constexpr uint16_t COLOR_GOOD = 0x07E0;   // Green
constexpr uint16_t COLOR_WARN = 0xFD20;   // Amber/Orange
constexpr uint16_t COLOR_MUTED = 0x4208;  // Dark gray
} // namespace UI

// ==================================================================================
//                              INPUT / OUTPUT ASSIGNMENTS
// ==================================================================================
//
// Pin Configuration for I2S Audio Interfaces
//
// Hardware Setup:
//   * MCLK (e.g., 22.5792/24.576 MHz) is shared between ADC and DAC
//   * ADC operates at SAMPLE_RATE_ADC (MCLK / 512)
//   * DAC operates at SAMPLE_RATE_DAC (MCLK / 128)
//   * Both use standard I2S protocol (Philips format)
//
// Pin Usage:
//   - BCK (Bit Clock): Carries serial audio data clock
//   - LRCK (Left/Right Clock): Word select (L/R channel indicator)
//   - DOUT/DIN: Serial audio data (24-bit samples)
//   - MCLK: Master clock for ADC/DAC synchronization

#if defined(PROJ_TARGET_ESP32)

// ==================================================================================
// Classic ESP32 I/O configuration
// ==================================================================================
// Note: Adjust to your board wiring as needed.

// ---- Master Clock (Shared) ----
constexpr int PIN_MCLK = 0; // MCLK output (if supported on your module)

// ---- DAC Interface (I2S TX @ SAMPLE_RATE_DAC) ----
constexpr int PIN_DAC_BCK = 26;  // DAC bit clock (BCK)
constexpr int PIN_DAC_LRCK = 25; // DAC word select (LRCK / WS)
constexpr int PIN_DAC_DOUT = 22; // DAC serial data output

// ---- ADC Interface (I2S RX @ SAMPLE_RATE_ADC) ----
constexpr int PIN_ADC_BCK = 32;  // ADC bit clock (BCK) - shared with DAC BCK
constexpr int PIN_ADC_LRCK = 33; // ADC word select (LRCK / WS) - shared with DAC LRCK
constexpr int PIN_ADC_DIN = 34;  // ADC serial data input (input-only pin)

// ---- SPI Pin Assignments for ST7789 ----
constexpr int TFT_SCK = 18;  // VSPI SCK
constexpr int TFT_MOSI = 23; // VSPI MOSI
constexpr int TFT_DC = 2;    // Data/Command select (adjust as needed)
constexpr int TFT_CS = 5;    // Chip select (GPIO5 typical)
constexpr int TFT_RST = 16;  // Hardware reset (adjust as needed)
constexpr int TFT_BL = -1;   // TFT Backlight Control Pin (-1 if backlight is hard-wired to power)

#elif defined(PROJ_TARGET_ESP32S3)

// ==================================================================================
// ESP32-S3 I/O configuration
// ==================================================================================
// ---- Master Clock (Shared) ----
constexpr int PIN_MCLK = 8; // 24.576 MHz MCLK output (shared by ADC and DAC)

// ---- DAC Interface (I2S TX @ SAMPLE_RATE_DAC) ----
constexpr int PIN_DAC_BCK = 9;   // DAC bit clock (BCK)
constexpr int PIN_DAC_LRCK = 11; // DAC word select (LRCK / WS)
constexpr int PIN_DAC_DOUT = 10; // DAC serial data output

// ---- ADC Interface (I2S RX @ SAMPLE_RATE_ADC) ----
constexpr int PIN_ADC_BCK = 4;  // ADC bit clock (BCK)
constexpr int PIN_ADC_LRCK = 6; // ADC word select (LRCK / WS)
constexpr int PIN_ADC_DIN = 5;  // ADC serial data input

// ---- SPI Pin Assignments for ST7789 ----
constexpr int TFT_SCK = 40;  // SPI clock (SCLK)
constexpr int TFT_MOSI = 41; // SPI data out (MOSI / SDI)
constexpr int TFT_DC = 42;   // Data/Command select (DC / RS)
constexpr int TFT_CS = 1;    // Chip select (CS)
constexpr int TFT_RST = 2;   // Hardware reset (RST)
constexpr int TFT_BL = -1;   // TFT Backlight Control Pin (-1 if backlight is hard-wired to power)

#endif

// ==================================================================================
//                       ST7789 TFT DISPLAY CONFIGURATION
// ==================================================================================

/**
 * Enable/Disable VU Display
 *
 * Set to false to disable TFT display initialization and rendering.
 * Useful for testing audio pipeline without display hardware.
 *
 * Value: true (display enabled)
 */
constexpr bool VU_DISPLAY_ENABLED = true;

/**
 * Display Rotation
 *
 * ST7789 orientation setting:
 *   0 = Portrait (240x320)
 *   1 = Landscape (320x240)
 *   2 = Portrait flipped
 *   3 = Landscape flipped
 *
 * Value: 1 (landscape mode for horizontal VU bars)
 */
constexpr int TFT_ROTATION = 1;

/**
 * ST7789 color inversion.
 *
 * Keep this setting identical for splash and main display so colors match.
 * If your panel shows negative/inverted colors, change this value.
 */
constexpr bool TFT_INVERT_DISPLAY = true;

// ==================================================================================
//                          SAMPLE RATES AND TIMING
// ==================================================================================

/**
 * Input Sample Rate (ADC)
 *
 * The rate at which audio samples are captured from the I2S ADC.
 *
 * Value: 48,000 Hz for WebRadio streams that decode to 48 kHz PCM.
 */
constexpr uint32_t SAMPLE_RATE_ADC = 48000;

static constexpr bool USE_WEBRADIO = true;

static constexpr const char *WEBRADIO_URL_NJOY =
    "http://icecast.ndr.de/ndr/njoy/live/mp3/128/stream.mp3";
static constexpr const char *WEBRADIO_URL_SUNSHINE =
    "http://stream.sunshine-live.de/live/mp3-128/stream.sunshine-live.de/";
static constexpr const char *WEBRADIO_URL_SUNSHINE_ALT =
    "http://sunsl.streamabc.net/sunsl-sslsimulcast-mp3-192-4434053";
static constexpr const char *WEBRADIO_URL_VIRGIN_ROCK_2K =
    "http://icy.unitedradio.it/Virgin_02.mp3";
static constexpr const char *WEBRADIO_URL_VIRGIN_CLASSIC =
    "http://icy.unitedradio.it/VirginRockClassics.mp3";

// Mantido para compatibilidade com codigo antigo: fonte inicial = NJOY.
static constexpr const char *WEBRADIO_URL = WEBRADIO_URL_NJOY;

// Botao BOOT do ESP32-S3. Toque curto: NJOY -> Sunshine -> Sunshine ALT
// -> Virgin 2K -> Virgin Classic -> ADC -> TEST L/R -> ADC MONO -> ZERO DAC -> ADC RAW MONO.
// Nao manter pressionado durante reset/upload.
static constexpr int INPUT_SOURCE_BUTTON_PIN = 0;
static constexpr uint32_t INPUT_SOURCE_BUTTON_DEBOUNCE_MS = 350;
static constexpr uint32_t INPUT_SOURCE_BUTTON_STARTUP_IGNORE_MS = 3000;

// Fonte de teste: dois tons continuos em mono, bons para observar no FFT.
static constexpr float TEST_TONE_FREQ_A_HZ = 1500.0f;
static constexpr float TEST_TONE_FREQ_B_HZ = 9000.0f;
static constexpr float TEST_TONE_AMP = 0.16f;

// Ativar apenas para diagnostico do stream/buffer PCM. Em uso normal fica false
// para nao carregar a Serial nem interferir com o tempo real.
static constexpr bool WEBRADIO_DEBUG_LOGS = false;

static constexpr const char *WIFI_SSID = SECRET_WIFI_SSID;
static constexpr const char *WIFI_PASS = SECRET_WIFI_PASS;

// ==================================================================================
//                              NTP / RDS CLOCK
// ==================================================================================
static constexpr bool NTP_TIME_SYNC_ENABLED = true;
static constexpr const char *NTP_TIMEZONE_PORTUGAL = "WET0WEST,M3.5.0/1,M10.5.0";
static constexpr const char *NTP_SERVER_1 = "pool.ntp.org";
static constexpr const char *NTP_SERVER_2 = "time.google.com";
static constexpr const char *NTP_SERVER_3 = "time.cloudflare.com";
static constexpr uint32_t NTP_SYNC_TIMEOUT_MS = 10000;
static constexpr uint32_t NTP_RETRY_INTERVAL_MS = 60000;
static constexpr uint32_t NTP_RDS_UPDATE_INTERVAL_MS = 30000;

// ==================================================================================
//                              WEBRADIO TASK
// ==================================================================================
// O decode MP3/WebRadio corre no Core 1 para deixar o Core 0 dedicado ao DSP/MPX.
// Prioridade 4 da tempo ao audio sem passar por cima da task critica do DSP.
static constexpr int WEBRADIO_CORE = 1;
static constexpr uint32_t WEBRADIO_PRIORITY = 5;
static constexpr uint32_t WEBRADIO_STACK_WORDS = 6144;
static constexpr int WEBRADIO_COPY_BURST = 12;
static constexpr int WEBRADIO_RECOVERY_COPY_BURST = 32;
static constexpr uint32_t WEBRADIO_RETRY_INTERVAL_MS = 5000;

/**
 * Upsampling Factor
 *
 * Ratio between output and input sample rates: DAC / ADC (e.g., 4x when 48->192 kHz)
 * This determines the polyphase FIR filter configuration.
 *
 * Value: 4x upsampling
 */
constexpr std::size_t UPSAMPLE_FACTOR = 4;

/**
 * Output Sample Rate (DAC)
 *
 * The rate at which processed audio is output to the I2S DAC.
 * 4x higher than input to accommodate FM multiplex bandwidth (38KHz and 57KHz subcarriers).
 *
 * Value: 4x ADC sampling rate (typ. 176,400 Hz or 192,000 Hz)
 */
constexpr uint32_t SAMPLE_RATE_DAC = SAMPLE_RATE_ADC * UPSAMPLE_FACTOR;

/**
 * Audio Block Size
 *
 * Number of stereo sample pairs processed per iteration.
 * Smaller blocks reduce latency but increase overhead.
 * Larger blocks improve efficiency but increase latency.
 *
 * Current: 64 samples approx 1.45 ms @ 44.1 kHz (scales with SAMPLE_RATE_ADC)
 *          (64 samples / SAMPLE_RATE_ADC seconds; approx1.33 ms at 48 kHz)
 */
constexpr std::size_t BLOCK_SIZE = 64;

/**
 * Bit Depth Configuration
 *
 * I2S audio format: 24-bit samples in 32-bit words (MSB-aligned)
 *   * BITS_PER_SAMPLE: Actual audio precision (24 bits)
 *   * BYTES_PER_SAMPLE: Memory allocation per sample (4 bytes = 32 bits)
 */
constexpr std::size_t BITS_PER_SAMPLE = 24; // 24-bit audio precision
constexpr std::size_t BYTES_PER_SAMPLE = 4; // 32-bit word (4 bytes)

/**
 * Fixed-Point to Float Conversion Factor
 *
 * Used to convert 32-bit signed integers to normalized float [-1.0, +1.0]
 * Q31 format: int32 / 2^31 = float
 *
 * Value: 2^31 = 2,147,483,648
 */
constexpr float Q31_FULL_SCALE = 2147483648.0f;

// ==================================================================================
//                          AUDIO I/O TIMEOUTS (MS)
// ==================================================================================
/**
 * I2S read/write timeouts in milliseconds. Prevents the audio task from blocking
 * indefinitely if a peripheral stalls. Values should exceed a typical block period
 * (approx64 / SAMPLE_RATE_ADC ms; e.g., approx1.45 ms at 44.1 kHz),
 * remain small to preserve
 * real-time behavior.
 */
constexpr uint32_t I2S_READ_TIMEOUT_MS = 5;  // RX timeout per read
constexpr uint32_t I2S_WRITE_TIMEOUT_MS = 10; // TX timeout per write

// ==================================================================================
//                     FREERTOS TASK ALLOCATION (CORE & PRIORITY)
// ==================================================================================
//
// This section centralizes all task allocation decisions: core assignment and priority.
// Each task has three configuration parameters:
//   * CORE: Which CPU core (0 = audio, 1 = I/O)
//   * PRIORITY: FreeRTOS priority level (0-25, higher = more priority)
//   * STACK_WORDS: Stack memory in 32-bit words
//
// Core Assignment Strategy:
//   Core 0: Audio processing (DSP_pipeline only - highest priority, no interruptions)
//   Core 1: I/O operations (Console, VU Meter, RDS - can block on I/O)
//
// Priority Strategy (FreeRTOS ESP32):
//   Level 6: Audio DSP (real-time, cannot block, highest priority)
//   Level 2: Console (CLI + logs, medium priority)
//   Level 1: VU Meter, RDS (display/RDS generation, low priority, can block)
//   Level 0: Idle task (system)

/** ---- CONSOLE TASK (Serial owner + log draining) ---- */
/** Console: Core Assignment (0 or 1) */
constexpr int CONSOLE_CORE = 1; // Core selection for Console task

/** Console: Task Priority */
constexpr uint32_t CONSOLE_PRIORITY = 2; // Medium priority

/** Console: Stack Size (32-bit words) */
constexpr uint32_t CONSOLE_STACK_WORDS =
    8192; // 32 KB (increase to avoid stack canary on heavy console parsing)

/** Console: Message Queue Capacity */
constexpr std::size_t CONSOLE_QUEUE_LEN = 256; // Larger buffer for verbose init

/** ---- VU METER TASK ---- */
/** VU Meter: Core Assignment (0 or 1) */
constexpr int VU_CORE = 1; // Core selection for VU task

/** VU Meter: Task Priority */
constexpr uint32_t VU_PRIORITY = 1; // Low priority

/** VU Meter: Stack Size (32-bit words) */
constexpr uint32_t VU_STACK_WORDS = 4096; // 16 KB

/** VU Meter: Sample Queue (Mailbox Pattern) */
constexpr std::size_t VU_QUEUE_LEN = 1; // Mailbox: holds only latest

/** ---- RDS ASSEMBLER TASK ---- */
/** RDS Assembler: Core Assignment (0 or 1) */
constexpr int RDS_CORE = 1; // Core selection for RDS assembler

/** RDS Assembler: Task Priority */
constexpr uint32_t RDS_PRIORITY = 1; // Low priority

/** RDS Assembler: Stack Size (32-bit words) */
constexpr uint32_t RDS_STACK_WORDS = 4096; // 16 KB

/** RDS Assembler: Bit Queue Capacity */
constexpr std::size_t RDS_BIT_QUEUE_LEN = 1024; // 1024 bits buffer

/** ---- DSP PIPELINE TASK ---- */
/** DSP Pipeline: Core Assignment (0 or 1) */
constexpr int DSP_CORE = 0; // Core selection for DSP pipeline

/** DSP Pipeline: Task Priority */
constexpr uint32_t DSP_PRIORITY = 6; // Highest priority

/** DSP Pipeline: Stack Size (32-bit words) */
constexpr uint32_t DSP_STACK_WORDS = 12288; // 48 KB (DSP buffers)

// ==================================================================================
//                         PRE-EMPHASIS FILTER PARAMETERS
// ==================================================================================
//
// FM Broadcasting Standard: Pre-emphasis compensates for high-frequency noise
// by boosting treble before transmission. The receiver applies de-emphasis
// to restore flat response while reducing noise.
//
// Standard: 50 us time constant (Europe) or 75 us (North America)

/**
 * Pre-emphasis Time Constant
 *
 * Determines the corner frequency of the high-pass pre-emphasis filter.
 * European FM broadcasting standard.
 *
 * Formula: fc = 1 / (2pi x tau) = 1 / (2pi x 50us) approx 3.18 kHz
 *
 * Value: 50 microseconds (EU standard)
 */
constexpr float PREEMPHASIS_TIME_CONSTANT_US = 50.0f;

/**
 * Pre-emphasis Filter Coefficient (Alpha)
 *
 * High-pass leaky differentiator coefficient calculated from time constant.
 * Derived from: alpha = exp(-1 / (tau x fs))
 *
 * The coefficient depends on the DAC sampling rate (SAMPLE_RATE_DAC), because
 * pre-emphasis is applied after the 4x upsampler in the 192 kHz MPX domain. It is
 * computed at compile time. Implemented via a 5th-order series approximation
 * to avoid pulling non-constexpr math into configuration.
 */

constexpr float exp_approx(float x)
{
    // 1 + x + x^2/2! + x^3/3! + x^4/4! + x^5/5!
    return 1.0f +
           x * (1.0f + x * (0.5f + x * (1.0f / 6.0f + x * (1.0f / 24.0f + x * (1.0f / 120.0f)))));
}
constexpr float PREEMPHASIS_ALPHA =
    exp_approx(-1.0f / (PREEMPHASIS_TIME_CONSTANT_US * 1.0e-6f * SAMPLE_RATE_DAC));

/**
 * Pre-emphasis Output Gain
 *
 * Normalizes the 50 us pre-emphasis filter to unity gain at DC/low frequencies.
 * The boost above the 3.18 kHz time-constant corner then follows the standard
 * FM pre-emphasis curve without hiding program-level attenuation inside this
 * filter. Overall MPX/DAC level should be calibrated separately.
 */
constexpr float PREEMPHASIS_GAIN = 1.0f / (1.0f - PREEMPHASIS_ALPHA);

// ==================================================================================
//                         STARTUP NORMALYSER
// ==================================================================================
// A NJOY fica como referencia e nao e levantada. O normalyser so atenua fontes
// demasiado fortes, mede por alguns segundos e depois congela o ganho.
constexpr bool STARTUP_NORMALYSER_ENABLED = true;
constexpr uint32_t STARTUP_NORMALYSER_MEASURE_MS = 6000;
constexpr float STARTUP_NORMALYSER_TARGET_RMS = 0.11f;
constexpr float STARTUP_NORMALYSER_MIN_INPUT_RMS = 0.01f;
constexpr float STARTUP_NORMALYSER_MIN_GAIN = 0.20f;
constexpr float STARTUP_NORMALYSER_MAX_GAIN = 1.00f;

// Durante a medicao inicial, as web radios nao-referencia saem mais baixas
// para evitar clip. Depois da medicao usa-se apenas o ganho final calculado.
constexpr float STARTUP_NORMALYSER_MEASURING_GAIN = 0.50f;
constexpr uint32_t STARTUP_NORMALYSER_GAIN_RAMP_MS = 1000;

// ==================================================================================
//                          FM STEREO MPX LEVELS
// ==================================================================================
//
// FM Multiplex Signal Composition:
//   * Mono (L+R): 0-15 kHz, full bandwidth
//   * Pilot tone: 19 kHz, 9% modulation
//   * Stereo subcarrier: 23-53 kHz (38 kHz +/- 15 kHz), DSB-SC with L-R

/**
 * Feature toggles for MPX components - used for debugging
 *
 * ENABLE_AUDIO: Includes program audio into the composite (mono and L-R)
 * ENABLE_STEREO_PILOT_19K: Includes the 19 kHz pilot tone
 * ENABLE_RDS_57K:   Includes the 57 kHz RDS subcarrier
 * ENABLE_STEREO_SUBCARRIER_38K: Enables the 38 kHz stereo subcarrier (L-R DSB)
 */
constexpr bool ENABLE_AUDIO = true;
constexpr bool ENABLE_STEREO_PILOT_19K = true;
constexpr bool ENABLE_RDS_57K = true;
constexpr bool ENABLE_STEREO_SUBCARRIER_38K = true;

/**
 * Enable/Disable Pre-emphasis
 *
 * For measurement and A/B testing. When false, the pre-emphasis stage is
 * bypassed entirely and audio proceeds unmodified from the ADC domain.
 */
constexpr bool ENABLE_PREEMPHASIS = true;

/**
 * Pilot Tone Amplitude
 *
 * The 19 kHz pilot tone indicates stereo transmission to the receiver.
 * FM standard: 9% +/- 1% of total modulation.
 *
 * Value: 0.09 (9% modulation depth)
 */
constexpr float PILOT_AMP = 0.09f;

/**
 * Stereo Subcarrier Amplitude (L-R)
 *
 * The difference signal (L-R) is modulated onto a 38 kHz suppressed carrier.
 * Amplitude is chosen to maintain proper stereo separation without overmodulation.
 *
 * Value: 0.90 (90% of available modulation depth) according to
 * https://www.itu.int/dms_pubrec/itu-r/rec/bs/R-REC-BS.450-4-201910-I%21%21PDF-E.pdf
 */

constexpr float DIFF_AMP = 1.0f;

// ==================================================================================
//                       PILOT AUTO-MUTE ON SILENCE
// ==================================================================================
/**
 * Automatically mute 19 kHz pilot after sustained input silence.
 *
 * Purpose: When program audio is silent for a configurable duration, the pilot can
 * be disabled to avoid stereo hiss on receivers. As soon as
 * audio reappears above the threshold, the pilot is re-enabled immediately.
 */
constexpr bool PILOT_MUTE_ON_SILENCE = false;

/**
 * Silence detection threshold (RMS, linear). Default: 0.001 (~ -60 dBFS)
 */
constexpr float SILENCE_RMS_THRESHOLD = 0.002f;

/**
 * Hold time before muting pilot (milliseconds). Default: 3000 ms (3 seconds)
 */
constexpr uint32_t SILENCE_HOLD_MS = 3000u;

// ==================================================================================
//                          PERFORMANCE MONITORING
// ==================================================================================

/**
 * Statistics Print Interval
 *
 * How often performance statistics are logged to Serial.
 * Includes: CPU usage, processing time, headroom, peak levels.
 *
 * Set to 0 for normal operation.
 */
constexpr uint64_t STATS_PRINT_INTERVAL_US = 0ULL; // 0 disables serial performance stats

/**
 * Per-stage DSP profiling.
 *
 * Keep false for normal operation: avoids cycle-counter reads in the audio loop.
 * Enable only when profiling CPU time per DSP stage.
 */
constexpr bool ENABLE_AUDIO_PROFILING = false;

// ==================================================================================
//                          VU METER CONFIGURATION
// ==================================================================================

/**
 * VU Update Interval
 *
 * How often the DSP_pipeline sends VU sample data to the display task.
 * Faster updates provide more responsive meters but increase queue traffic.
 *
 * Current: 50 ms = 20 Hz update rate.
 * Lower rate keeps SPI display work from starving WiFi/WebRadio decoding.
 *
 * Value: 50,000 microseconds (50 ms)
 */
constexpr uint64_t VU_UPDATE_INTERVAL_US = 50000ULL;

/**
 * LCD VU bargraph frame interval.
 *
 * The ST7789 bargraph is SPI-heavy. Keep this modest when WebRadio is enabled
 * so the WiFi/MP3 pipeline has enough CPU time.
 */
constexpr uint32_t VU_BARGRAPH_FRAME_INTERVAL_MS = 100;

/**
 * dBFS Reference Level
 *
 * Full-scale reference for dBFS calculations.
 * In float audio (range -1.0 to +1.0), full scale = 1.0
 *
 * Formula: dBFS = 20 x log10(sample / DBFS_REF)
 *
 * Value: 1.0 (normalized float full scale)
 */
constexpr float DBFS_REF = 1.0f;

// ==================================================================================
//                          VU METER CALIBRATION
// ==================================================================================

/**
 * VU Meter dB Offset
 *
 * Shifts the entire dB scale to adjust VU meter sensitivity.
 * Applied to measured dBFS values before display mapping.
 *
 * Use Cases:
 *   * Compensate for digital headroom in audio pipeline
 *   * Adjust meter sensitivity for different source levels
 *   * Make quieter signals more visible on the display
 *
 * Effect on Display:
 *   Offset = 0.0:  True dBFS scale (-40 to +3 dB)
 *                  Only loud signals move the meters
 *
 *   Offset = 10.0: Scale shifts up by 10 dB
 *                  -30 dBFS input shows as -20 dB (more sensitive)
 *
 *   Offset = -10.0: Scale shifts down by 10 dB
 *                   -30 dBFS input shows as -40 dB (less sensitive)
 *
 * Typical Values:
 *   0.0  = No adjustment (default, accurate dBFS)
 *   5.0  = Moderate boost (good for moderate signals)
 *   10.0 = High boost (good for quiet signals)
 *
 * Value: 0.0 dB (no offset, true dBFS scale)
 */
constexpr float VU_DB_OFFSET = 0.0f;

/**
 * VU Meter Mode Selection
 *
 * Determines which audio metric drives the VU bar length:
 *
 *   true  = Peak mode (recommended)
 *           * Shows instantaneous peak levels
 *           * Responds immediately to transients
 *           * Better for monitoring clipping
 *           * Current setting
 *
 *   false = RMS mode
 *           * Shows average power (RMS = Root Mean Square)
 *           * Smoother, less responsive to brief peaks
 *           * Better represents perceived loudness
 *           * May miss brief clipping events
 *
 * Value: true (peak mode)
 */
constexpr bool VU_USE_PEAK_FOR_BAR = true;

/**
 * Draw animated VU bargraph on the ST7789.
 * Mostra nivel L/R no LCD; a taxa de desenho e limitada por VU_BARGRAPH_FRAME_INTERVAL_MS.
 * Disable for WebRadio timing tests or lowest display load.
 */
constexpr bool DISPLAY_SHOW_VU_BARGRAPH = true;

/**
 * Draw NTP local time line on the ST7789.
 * A hora vem do NTP em fuso Portugal e tambem alimenta o grupo RDS CT.
 * Disable for WebRadio timing tests or lowest display load.
 */
constexpr bool DISPLAY_SHOW_TIME_LINE = true;

/**
 * Scroll RadioText/metadata on the ST7789.
 * Quando chegam metadados ICY, mostra "PLAY NOW - ..." em scroll no LCD.
 * Disable to draw a static one-time text line instead.
 */
constexpr bool DISPLAY_SCROLL_RT_TEXT = true;

/**
 * Display RDS Status Bar
 *
 * When true, shows a compact one-line bar with PI, PTY and flags (ST/RDS/PIL/TP/TA/MS).
 * Ajuda a confirmar no LCD que piloto, RDS e flags estao ativos.
 * Disable if you experience instability with your display driver.
 */
constexpr bool DISPLAY_SHOW_RDS_STATUS_BAR = true;

// ==================================================================================
//                          RDS (RADIO DATA SYSTEM)
// ==================================================================================

/** RDS injection amplitude (fraction of full-scale MPX). Typical 0.02-0.04 */
// Standard conservative injection. Raise only if a weak/old receiver struggles.
constexpr float RDS_AMP = 0.027f;

/** RDS symbol rate (RDS bps) */
constexpr float RDS_SYMBOL_RATE = 1187.5f;

// Final RDS narrowing filter, equivalent in purpose to JMPX's
// BandPassHanning(57000 - 2400, 57000 + 2400). Runs after 57 kHz modulation and
// before mixing RDS into the MPX buffer.
constexpr bool ENABLE_RDS_57K_BPF = true;
constexpr float RDS_57K_BPF_Q = 6.0f;

// ==================================================================================
//                         DSP CORE BUFFER CONFIGURATION
// ==================================================================================

/**
 * NCO (Numerically Controlled Oscillator) Lookup Table Size
 *
 * The number of entries in the pre-computed sine wave table used for
 * efficient frequency generation. Larger tables reduce quantization error
 * but increase memory usage.
 *
 * Value: 1024 entries (typical for embedded systems)
 */
constexpr std::size_t NCO_TABLE_SIZE = 1024;

// ==================================================================================
//                    POLYPHASE FIR UPSAMPLER CONFIGURATION
// ==================================================================================

/**
 * FIR Filter Tap Count
 *
 * Total number of filter coefficients in the polyphase FIR upsampler.
 * Larger tap counts reduce aliasing but increase processing overhead.
 *
 * Value: 128 taps (32 taps per phase x 4 phases)
 */
constexpr std::size_t FIR_TAPS = 128;

/**
 * FIR Taps Per Phase
 *
 * Number of filter coefficients per upsampling phase.
 * Derived from: FIR_TAPS / UPSAMPLE_FACTOR = 128 / 4
 *
 * Value: 32 taps per phase
 */
constexpr std::size_t FIR_TAPS_PER_PHASE = 32;

// ==================================================================================
//                           I2S DRIVER CONFIGURATION
// ==================================================================================

/**
 * I2S Bits Per Sample (Wire Format)
 *
 * The number of bits transmitted per I2S sample.
 * Standard for 24-bit audio: 32-bit words (24-bit data + 8-bit padding)
 *
 * Value: 32 bits
 */
constexpr uint32_t I2S_BITS_PER_SAMPLE = 32;

/**
 * I2S Number of Channels
 *
 * Stereo audio: 2 channels (Left + Right)
 * Affects DMA configuration and data layout.
 *
 * Value: 2 (stereo)
 */
constexpr uint32_t I2S_CHANNELS = 2;

/**
 * I2S MCLK Multiple for TX (DAC Output)
 *
 * Master clock divider for DAC output clock generation.
 * MCLK = Sample_Rate x MCLK_Multiple_TX
 * Example: At 192 kHz: 192000 x 128 = 24.576 MHz MCLK
 *
 * Value: 128
 */
constexpr uint32_t I2S_MCLK_MULTIPLE_TX = 128;

/**
 * I2S MCLK Multiple for RX (ADC Input)
 *
 * Master clock divider for ADC input clock generation.
 * MCLK = Sample_Rate x MCLK_Multiple_RX
 * Example: At 48 kHz: 48000 x 512 = 24.576 MHz MCLK
 *
 * Value: 512
 */
constexpr uint32_t I2S_MCLK_MULTIPLE_RX = 512;

/**
 * I2S Bit Clock Divisor
 *
 * Divider for generating bit clock (BCK) from MCLK.
 * Used in DMA clock configuration calculations.
 *
 * Value: 64
 */
constexpr uint32_t I2S_BCK_DIVISOR = 64;

/**
 * I2S DMA Buffer Lengths (samples per buffer)
 *
 * Align these with DSP block sizes to avoid repeating descriptor rollovers
 * that can cause periodic artifacts. Revert to previous values (e.g., 240)
 * if needed.
 *
 * TX (DAC): 256 samples -> matches 64-frame block x 4x upsample
 * RX (ADC):  64 samples -> matches 64-frame input block
 */
constexpr int I2S_DMA_LEN_TX = 512; // extra TX/DAC margin for transient CPU jitter
constexpr int I2S_DMA_LEN_RX = 256; // more RX margin for ADC reads

// (Duplicate FreeRTOS task/display constants removed - see earlier definitions.)

} // namespace Config

// =====================================================================================
//                                END OF FILE
// =====================================================================================
