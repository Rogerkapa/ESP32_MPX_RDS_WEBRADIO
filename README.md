#  ESP32-S3 FM RDS Stereo Encoder


Software FM stereo encoder with RDS, WebRadio input, external ADC input and ST7789 TFT monitoring, running on ESP32-S3 with Arduino ESP32 core.

This project is based on the original **PiratESP32 DSP BASED RDS STEREO ENCODER FOR FM RADIO** project and keeps the same core idea: generate the full FM multiplex signal in software.

The current tree is the evolved ESP32-S3 version used for:

- MP3 WebRadio decoding and ICY metadata
- FM stereo MPX synthesis
- RDS PS/RT/CT generation
- ST7789 TFT splash/VU display
- External I2S ADC input
- Diagnostic local sources for DAC/ADC/noise testing

![In action](Picture.jpeg)

## Credits

Original project and architecture:

- MarcFinns / PiratESP32-FM-RDS-STEREO-ENCODER
- ESP32 Arduino Core by Espressif Systems
- FreeRTOS, integrated in the ESP32 SDK
- Arduino_GFX library for the ST7789 display
- ESP32-audio-tools / Helix MP3 decoder for WebRadio playback

This working branch was extended and debugged with AI-assisted development using Anthropic Claude Code, OpenAI Codex and user hardware testing.

## Main Features

- Real-time FM stereo MPX generation at 192 kHz
- WebRadio MP3 input with ICY metadata for LCD and RDS RT
- Source selection with the ESP32-S3 BOOT button
- Sources:
  - NJOY
  - Sunshine Live 128 kbps
  - Sunshine Live 192 kbps
  - Virgin Radio Rock 2K
  - Virgin Radio Rock Classic
  - External ADC
  - Test tones
  - ADC mono diagnostic
  - Zero DAC diagnostic
  - ADC raw mono diagnostic
- NTP time sync, used for RDS Clock-Time group
- RDS PI/PTY/TP/TA/MS, PS, RT and CT
- RDS PTY set to POP for current radio/audio sources
- RDS final 57 kHz narrowing filter using cascaded biquads
- 19 kHz pilot, 38 kHz L-R and 57 kHz RDS generated coherently from one NCO
- Stereo matrix:
  - mono = (L + R) / 2
  - diff = (L - R) / 2
- 4x polyphase FIR upsampler, 128 taps
- 50 us European FM pre-emphasis
- Startup normalyser for strong WebRadio streams
- ST7789 320x240 TFT splash, VU meter, time and scrolling "PLAY NOW" text
- WiFi credentials separated into `secrets.h` so the project can be published without the local password

## Current DSP Chain

Normal audio/WebRadio path:

```text
WebRadio MP3 / ADC input
        |
48 kHz stereo float
        |
Startup normalyser / temporary measuring attenuation for non-reference streams
        |
4x polyphase FIR upsampler and audio low-pass
        |
50 us pre-emphasis
        |
Stereo matrix: (L+R)/2 and (L-R)/2
        |
NCO coherent carriers: 19 kHz, 38 kHz, 57 kHz
        |
MPX mix: mono + pilot + L-R DSB-SC + RDS BPSK
        |
192 kHz I2S TX to DAC, same MPX on both DAC channels
```

The old 19 kHz notch filter is no longer active in this branch, make clipping on some sources

## Hardware

### Main Board

- ESP32-S3 N16R8 recommended
- OPI PSRAM enabled in Arduino board settings
- 16 MB flash
- CPU 240 MHz
- Flash QIO 80 MHz
- Partition: No OTA / Large App

### Audio

- I2S DAC, for example PCM5102A
- I2S ADC, for example PCM1808 / WM8782S style board
- ESP32-S3 provides MCLK
- DAC output is the final MPX signal

### Display

- ST7789 320x240 SPI TFT
- Backlight is hard-wired, not controlled by firmware
- Color inversion is enabled by `Config::TFT_INVERT_DISPLAY`

## ESP32-S3 Pinout

### I2S TX to DAC

```text
MCLK  GPIO8   -> DAC SCK / MCLK
BCK   GPIO9   -> DAC BCK
LRCK  GPIO11  -> DAC LCK / LRCK / WS
DOUT  GPIO10  -> DAC DIN
GND   GND
3V3   3V3
```

### I2S RX from ADC

```text
MCLK  GPIO8   -> ADC SCK / MCLK
BCK   GPIO4   -> ADC BCK
LRCK  GPIO6   -> ADC LRC / LRCK / WS
DIN   GPIO5   <- ADC OUT / DOUT
GND   GND
3V3   3V3
```

### ST7789 TFT

```text
SCK   GPIO40  -> TFT SCK / CLK
MOSI  GPIO41  -> TFT MOSI / SDA / SDI
DC    GPIO42  -> TFT DC / RS
CS    GPIO1   -> TFT CS
RST   GPIO2   -> TFT RST
BL    hard-wired, not used by firmware
MISO  not used
```

### Source Button

```text
BOOT  GPIO0
```

Short press advances through the input sources. Do not hold BOOT during upload/reset.

## Source Order

```text
NJOY
Sunshine 128
Sunshine 192
Virgin 2K
Virgin Classic
ADC
TEST L/R
ADC MONO
ZERO DAC
ADC RAW MONO
```

`ZERO DAC` sends absolute digital zero to the DAC. It is useful to separate DAC/ground noise from DSP or source noise.

`ADC RAW MONO` sends ADC left channel directly to both DAC channels, repeated to 192 kHz, without MPX, pilot, RDS, pre-emphasis or FIR. It is a diagnostic path.

## WiFi Secrets

Local credentials are stored in:

```text
secrets.h
```

This file is ignored by git.

For GitHub or a clean machine, use:

```text
secrets.example.h
```

Copy it to `secrets.h` and fill in:

```cpp
static constexpr const char *SECRET_WIFI_SSID = "YOUR_WIFI_SSID";
static constexpr const char *SECRET_WIFI_PASS = "YOUR_WIFI_PASSWORD";
```

`Config.h` reads:

```cpp
static constexpr const char *WIFI_SSID = SECRET_WIFI_SSID;
static constexpr const char *WIFI_PASS = SECRET_WIFI_PASS;
```

## Important Config Values

Current MPX levels:

```cpp
PILOT_AMP = 0.09f;
DIFF_AMP  = 1.0f;
RDS_AMP   = 0.027f;
```

Current sample rates:

```cpp
SAMPLE_RATE_ADC = 48000;
UPSAMPLE_FACTOR = 4;
SAMPLE_RATE_DAC = 192000;
BLOCK_SIZE      = 64;
```

Current upsampler:

```cpp
FIR_TAPS = 128;
FIR_TAPS_PER_PHASE = 32;
```

Current normalyser behavior:

```cpp
STARTUP_NORMALYSER_ENABLED = true;
STARTUP_NORMALYSER_MEASURE_MS = 6000;
STARTUP_NORMALYSER_TARGET_RMS = 0.11f;
STARTUP_NORMALYSER_MEASURING_GAIN = 0.50f;
STARTUP_NORMALYSER_GAIN_RAMP_MS = 1000;
```

NJOY is treated as the reference stream and is not raised by the normalyser. Other WebRadio streams are measured at their real level, but while measuring they are sent to the DAC with temporary attenuation to avoid clipping. After measurement, the calculated gain is applied with a short ramp.

## Task Layout

### Core 0

- `DSP_pipeline`
- Priority 6
- Real-time audio/DSP/MPX/I2S TX

### Core 1

- `WebRadioInput`
- Priority 5
- MP3 decode, HTTP stream and PCM ringbuffer

- `Console`
- Priority 2
- Serial CLI and logging

- `DisplayManager`
- Priority 1
- ST7789 VU display

- `RDSAssembler`
- Priority 1
- RDS group/bit generation

## File Structure

Main Arduino entry:

```text
PiratESP32-FM-RDS-STEREO-ENCODER.ino
```

Core modules:

```text
Config.h                 Central configuration
secrets.h                Local WiFi credentials, ignored by git
secrets.example.h        Public credentials template
DSP_pipeline.*           Audio DSP chain and MPX generation
WebRadioInput.*          WebRadio streams, source button, PCM ringbuffer
ESP32I2SDriver.*         Hardware driver wrapper
I2SDriver.*              ESP32 I2S setup
DisplayManager.*         ST7789 splash/VU/time/RDS text display
RDSAssembler.*           RDS group builder and bit queue
RDSSynth.*               RDS baseband shaping, 57 kHz modulation/filter
NCO.*                    Coherent 19/38/57 kHz oscillator
StereoMatrix.*           (L+R)/2 and (L-R)/2 matrix
PolyphaseFIRUpsampler.*  4x FIR upsampler
PreemphasisFilter.*      FM pre-emphasis
Console.*                SCPI/JSON serial console
TimeSync.*               NTP for Portugal and RDS CT
```


## Serial Console

Serial runs at 115200 baud.

Useful examples:

```text
RDS:PS "xxxxxxxx"
RDS:PTY POP_MUSIC
RDS:RT "Artist - Title"
AUDIO:STEREO 1
AUDIO:STEREO 0
AUDIO:PREEMPH 1
PILOT:ENABLE 1
RDS:ENABLE 1
SYST:LOG:LEVEL INFO
SYST:LOG:LEVEL OFF
```

Redirect messages such as:

```text
URLStream.h : 336 - Redirected to:
```

are normal for WebRadio servers. They mean the stream URL was redirected to the active CDN endpoint.

## Build Notes

Known good Arduino settings for the current ESP32-S3 setup:

```text
Board: ESP32-S3 N16R8 or equivalent
CPU Frequency: 240 MHz
Flash: 16 MB
PSRAM: OPI PSRAM enabled
Partition: No OTA / Large App
Events run on: Core 1
Upload speed: 921600 usually works
```

Required libraries include:

```text
Arduino_GFX_Library
ESP32-audio-tools
ESP-DSP
ESP32 Arduino Core
```

The project currently uses Arduino IDE/classic ESP32 Arduino core paths on the development machine. `arduino-cli` may not be installed.

## Troubleshooting

### No WiFi

- Check `secrets.h`
- Confirm `SECRET_WIFI_SSID` and `SECRET_WIFI_PASS`
- Do not commit `secrets.h`

### WebRadio plays briefly then stops

- Confirm PSRAM is enabled in Arduino board settings
- Confirm partition is No OTA / Large App
- Check Serial for stream timeout, connect failed or memory allocation errors

### No MPX/audio at DAC

- Check DAC I2S pins
- Check MCLK GPIO8 at 24.576 MHz
- Check BCK GPIO9 at 12.288 MHz
- Check LRCK GPIO11 at 192 kHz

### ADC source noise

- Ground the ADC input before the coupling capacitor to separate board/input noise from firmware noise
- Use `ZERO DAC` and `ADC RAW MONO` sources for comparison
- Add input impedance/termination as required by the ADC board

### Display colors inverted

- Toggle `TFT_INVERT_DISPLAY` in `Config.h`

## Notes

- This is an experimental educational project.
- Check local regulations before transmitting any RF signal.
- The DAC outputs MPX baseband. Use appropriate filtering, level calibration and legal RF equipment.

## License

This project is provided as-is for educational and non-commercial use. Refer to individual library licenses for third-party components.

## References

- [FM broadcasting](https://en.wikipedia.org/wiki/FM_broadcasting)
- [Radio Data System](https://en.wikipedia.org/wiki/Radio_Data_System)
- [ESP32 I2S documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)






