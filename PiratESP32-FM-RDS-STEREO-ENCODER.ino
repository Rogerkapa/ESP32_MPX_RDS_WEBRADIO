/*
 * =====================================================================================
 *
 *                     Main INO
 *
 * =====================================================================================
 *
 * File:         PiratESP32-FM-RDS-STEREO-ENCODER.ino
 * Description:  Entry point for ESP32 FM stereo encoder with real-time DSP
 *
 * Overview:
 *   This application implements a professional-grade FM stereo encoder using the
 *   ESP32's dual-core architecture. It receives stereo audio at Config::SAMPLE_RATE_ADC via I2S,
 *   processes it through a multi-stage DSP pipeline, and outputs FM-multiplexed
 *   stereo at Config::SAMPLE_RATE_DAC for transmission.
 *
 * Core Architecture:
 *   CORE 0 (Real-Time Audio):
 *     * DSP_pipeline task (priority 6 - highest)
 *     * Handles all audio I/O and DSP processing
 *     * Must maintain strict timing for glitch-free audio
 *
 *   CORE 1 (Non-Real-Time):
 *     * Console task (priority 2)
 *     * VU Meter task (priority 1)
 *     * Handles serial output and display rendering
 *     * Cannot interrupt audio processing
 *
 * Task Priorities:
 *   Priority 6: DSP_pipeline  (highest - real-time audio)
 *   Priority 2: Console      (medium - CLI + diagnostics)
 *   Priority 1: VU Meter     (low - visual feedback)
 *   Priority 0: Idle         (system idle task)
 *
 * Memory Allocation:
 *   DSP_pipeline: 12,288 bytes stack (complex DSP operations)
 *   Console:      4,096 bytes stack (string formatting)
 *   VU Meter:     4,096 bytes stack (graphics rendering)
 *
 * Signal Flow:
 *   1. Audio enters via I2S RX (ADC-rate stereo)
 *   2. DSP_pipeline processes 64-sample blocks (~1.45 ms @ 44.1 kHz)
 *   3. DSP pipeline: upsample/low-pass -> pre-emphasis -> MPX synthesis
 *   4. Output via I2S TX (DAC-rate stereo)
 *   5. VU meters display real-time levels on ST7789 TFT
 *   6. Console handles CLI and outputs diagnostics to Serial (115200 baud)
 *
 * =====================================================================================
 */

#include "Config.h"
#include "Console.h"
#include "DSP_pipeline.h"
#include "DisplayManager.h"
#include "ESP32I2SDriver.h"
#include "RDSAssembler.h"
#include "SystemContext.h"
#include "WebRadioInput.h"

// ==================================================================================
//                              ARDUINO SETUP
// ==================================================================================

/**
 * Initialize hardware and start FreeRTOS tasks
 *
 * Uses SystemContext (IoC Container) to manage all module initialization.
 * This centralized approach ensures:
 *    Consistent startup order
 *    Proper dependency injection
 *    Clear lifecycle management
 *    Easy to test with mock drivers
 *
 * Core Assignment Strategy:
 *    Core 0: Dedicated to audio processing (no interruptions)
 *    Core 1: Handles all I/O (serial, display, logging)
 *    This separation ensures audio processing is never blocked by I/O
 */


// ==================================================================================
//                              ARDUINO SETUP
// ==================================================================================

void setup()
{
    // ---- Initialize Serial Communication ----
    Serial.begin(115200);
    delay(300);

    if (Config::ENABLE_BOOT_INFO_LOGS)
    {
        Serial.println();
        Serial.println("Booting ESP32 S3...");
    }

    // ---- Optional Splash Screen (DisplayManager owns the LCD driver) ----
    DisplayManager::showSplash();

    // ---- Initialize System Context with Dependency Injection ----
    static ESP32I2SDriver hw_driver;

    SystemContext &system = SystemContext::getInstance();

    bool initialized =
        system.initialize(&hw_driver,
                          Config::DSP_CORE,
                          Config::DSP_PRIORITY,
                          Config::DSP_STACK_WORDS,
                          Config::ENABLE_RDS_57K);

    if (!initialized)
    {
        Console::printOrSerial(LogLevel::ERROR, "FATAL: System initialization failed!");

        while (true)
        {
            Serial.println("FATAL: System initialization failed!");
            delay(1000);
        }
    }

    // ---- Start WebRadio AFTER system/console/tasks are alive ----
    if (Config::USE_WEBRADIO)
    {
        if (Config::ENABLE_BOOT_INFO_LOGS)
        {
            Console::enqueue(LogLevel::INFO, "Starting WebRadio input...");
        }

        bool wr_ok = webRadioInit();

        if (wr_ok)
        {
            if (Config::ENABLE_BOOT_INFO_LOGS)
            {
                Console::enqueue(LogLevel::INFO, "WebRadio input started");
            }
        }
        else
        {
            Console::enqueue(LogLevel::ERROR, "WebRadio input failed to start");
        }
    }

    if (Config::ENABLE_BOOT_INFO_LOGS)
    {
        Console::enqueue(LogLevel::INFO, "Arduino setup() complete");
    }
}

// ==================================================================================
//                              ARDUINO LOOP
// ==================================================================================

void loop()
{
    if (Config::USE_WEBRADIO)
    {
        webRadioLoop();
    }

    vTaskDelay(1);
}

// =====================================================================================
//                                END OF FILE
// =====================================================================================
