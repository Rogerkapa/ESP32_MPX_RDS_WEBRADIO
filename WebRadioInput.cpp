#include "WebRadioInput.h"
#include "Config.h"
#include "DisplayManager.h"
#include "RDSAssembler.h"
#include "TimeSync.h"

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

#include "AudioTools.h"
#include "AudioTools/Communication/AudioHttp.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"

using namespace audio_tools;

// =====================================================
// AudioTools objects
// =====================================================

// ICYStream faz a ligacao HTTP e separa os metadados ICY do audio MP3.
static ICYStream url(Config::WIFI_SSID, Config::WIFI_PASS);

// Sink custom: recebe PCM ja descodificado pelo Helix.
// O AudioTools escreve aqui bytes PCM 16-bit; depois guardamos no ringbuffer.
class PipelineSink : public Print
{
public:
    size_t write(uint8_t b) override
    {
        return write(&b, 1);
    }

    size_t write(const uint8_t *data, size_t len) override;
};

static PipelineSink sink;
static MP3DecoderHelix mp3;
static EncodedAudioStream decoder(&sink, &mp3);
static StreamCopy copier(decoder, url);
static TaskHandle_t web_radio_task_handle = nullptr;
static volatile bool web_radio_task_running = false;
static void logWebRadioStats();
static inline size_t rb_frame_count();
static inline void rb_reset();

struct SourceDef
{
    AudioInputSource id;
    const char *name;
    const char *url;
    bool is_stream;
    uint8_t pty;
};

static constexpr SourceDef SOURCES[] = {
    {AudioInputSource::NJOY, "NJOY", Config::WEBRADIO_URL_NJOY, true, 10},
    {AudioInputSource::SUNSHINE, "SUNSHINE 128", Config::WEBRADIO_URL_SUNSHINE, true, 10},
    {AudioInputSource::SUNSHINE_ALT, "SUNSHINE 192", Config::WEBRADIO_URL_SUNSHINE_ALT, true, 10},
    {AudioInputSource::VIRGIN_ROCK_2K, "VIRGIN 2K", Config::WEBRADIO_URL_VIRGIN_ROCK_2K, true, 11},
    {AudioInputSource::VIRGIN_CLASSIC, "VIRGIN CLASSIC", Config::WEBRADIO_URL_VIRGIN_CLASSIC, true, 11},
    {AudioInputSource::ADC, "ADC", nullptr, false, 10},
    {AudioInputSource::TEST_LR, "TEST L/R", nullptr, false, 0},
    {AudioInputSource::ADC_MONO, "ADC MONO", nullptr, false, 10},
    {AudioInputSource::ZERO_DAC, "ZERO DAC", nullptr, false, 0},
    {AudioInputSource::ADC_RAW_MONO, "ADC RAW MONO", nullptr, false, 10},
};

static volatile uint8_t current_source = (uint8_t)AudioInputSource::NJOY;
static volatile uint8_t requested_source = (uint8_t)AudioInputSource::NJOY;
static bool stream_open = false;
static uint32_t next_stream_retry_ms = 0;

static const SourceDef &sourceDef(uint8_t idx)
{
    if (idx >= (uint8_t)AudioInputSource::COUNT)
        idx = 0;
    return SOURCES[idx];
}

// Buffer PCM entre o decoder MP3 (produtor) e o DSP (consumidor).
// 48 kHz stereo: 20480 frames ~= 426 ms de folga.
// O ESP precisa de mais margem que um PC para absorver jitter HTTP/MP3.
static constexpr size_t PCM_FRAMES = 20480;
static constexpr size_t PCM_SAMPLES = PCM_FRAMES * 2;
static constexpr size_t PCM_PREBUFFER_FRAMES = 15360;

// Histerese do produtor: quando enche ate HIGH, o decoder espera ate baixar a LOW.
// Isto evita overwrite/cortes e deixa o TCP/MP3 aplicar back-pressure naturalmente.
// Mantemos o ringbuffer mais cheio para absorver pequenos atrasos do decoder/rede.
static constexpr size_t PCM_HIGH_WATER_FRAMES = 19456;
static constexpr size_t PCM_LOW_WATER_FRAMES = 15360;

// =====================================================
// ICY metadata: current track for RDS RT and LCD scroll
// =====================================================

static portMUX_TYPE meta_mux = portMUX_INITIALIZER_UNLOCKED;
static char meta_pending_title[128] = {0};
static char meta_current_title[128] = {0};
static volatile bool meta_title_pending = false;

static void sanitizeTitle(const char *in, int len, char *out, size_t out_sz)
{
    if (!out || out_sz == 0)
        return;

    out[0] = '\0';
    if (!in || len <= 0)
        return;

    size_t pos = 0;
    bool last_space = true;

    // O RDS/LCD lidam melhor com texto ASCII limpo; removemos aspas,
    // caracteres de controlo e espacos repetidos.
    for (int i = 0; i < len && in[i] != '\0' && pos < out_sz - 1; ++i)
    {
        unsigned char c = (unsigned char)in[i];

        if (c == '\'' || c == '"')
            continue;

        if (c < 0x20 || c >= 0x7F)
            c = ' ';

        if (c == ' ')
        {
            if (last_space)
                continue;
            last_space = true;
        }
        else
        {
            last_space = false;
        }

        out[pos++] = (char)c;
    }

    while (pos > 0 && out[pos - 1] == ' ')
        --pos;

    out[pos] = '\0';
}

static void onIcyMetadata(MetaDataType type, const char *str, int len)
{
    if (type != Title || !str || len <= 0)
        return;

    char clean[sizeof(meta_pending_title)];
    sanitizeTitle(str, len, clean, sizeof(clean));

    if (clean[0] == '\0')
        return;

    // Este callback vem do fluxo de audio. Guardamos apenas uma copia curta e
    // publicamos depois fora da critical section para nao bloquear o decoder.
    portENTER_CRITICAL(&meta_mux);
    if (strncmp(clean, meta_pending_title, sizeof(meta_pending_title)) != 0 &&
        strncmp(clean, meta_current_title, sizeof(meta_current_title)) != 0)
    {
        strncpy(meta_pending_title, clean, sizeof(meta_pending_title) - 1);
        meta_pending_title[sizeof(meta_pending_title) - 1] = '\0';
        meta_title_pending = true;
    }
    portEXIT_CRITICAL(&meta_mux);
}

static void publishPendingMetadata()
{
    char title[sizeof(meta_pending_title)];
    bool has_update = false;

    portENTER_CRITICAL(&meta_mux);
    if (meta_title_pending)
    {
        strncpy(title, meta_pending_title, sizeof(title) - 1);
        title[sizeof(title) - 1] = '\0';
        strncpy(meta_current_title, title, sizeof(meta_current_title) - 1);
        meta_current_title[sizeof(meta_current_title) - 1] = '\0';
        meta_title_pending = false;
        has_update = true;
    }
    portEXIT_CRITICAL(&meta_mux);

    if (!has_update)
        return;

    // Uma unica fonte para LCD e RDS RadioText: evita textos diferentes nos dois.
    char line[160];
    snprintf(line, sizeof(line), "PLAY NOW - %s", title);

    RDSAssembler::setRT(line);
    DisplayManager::setDisplayRT(line);
}

static void publishSourceName()
{
    const SourceDef &src = sourceDef(current_source);
    RDSAssembler::setPTY(src.pty);
    RDSAssembler::setMS(src.is_stream);

    char line[80];
    snprintf(line, sizeof(line), "SOURCE - %s PTY %u", src.name, (unsigned)src.pty);
    RDSAssembler::setRT(line);
    DisplayManager::setDisplayRT(line);
    Console::enqueuef(LogLevel::INFO,
                      "Audio input source: %s PTY=%u",
                      src.name,
                      (unsigned)src.pty);
}

static bool openCurrentStream()
{
    const SourceDef &src = sourceDef(current_source);

    rb_reset();
    url.end();
    stream_open = false;
    next_stream_retry_ms = 0;

    if (!src.is_stream)
    {
        publishSourceName();
        return true;
    }

    decoder.begin();
    url.setMetadataCallback(onIcyMetadata);

    bool ok = url.begin(src.url, "audio/mp3");
    stream_open = ok;
    publishSourceName();

    if (!ok)
    {
        Console::enqueuef(LogLevel::ERROR, "WebRadio source failed: %s", src.name);
        next_stream_retry_ms = millis() + Config::WEBRADIO_RETRY_INTERVAL_MS;
    }

    return ok;
}

static void retryCurrentStreamIfDue()
{
    if (!sourceDef(current_source).is_stream || stream_open || next_stream_retry_ms == 0)
        return;

    uint32_t now = millis();
    if ((int32_t)(now - next_stream_retry_ms) < 0)
        return;

    Console::enqueuef(LogLevel::WARN,
                      "Retrying WebRadio source: %s",
                      sourceDef(current_source).name);
    openCurrentStream();
}

static void applyRequestedSource()
{
    uint8_t req = requested_source;
    if (req >= (uint8_t)AudioInputSource::COUNT)
        req = 0;

    if (req == current_source && (stream_open || !sourceDef(current_source).is_stream))
        return;

    current_source = req;
    openCurrentStream();
}

static void webRadioTask(void *arg)
{
    (void)arg;
    web_radio_task_running = true;

    for (;;)
    {
        applyRequestedSource();
        retryCurrentStreamIfDue();

        if (!sourceDef(current_source).is_stream || !stream_open)
        {
            publishPendingMetadata();
            timeSyncLoop();
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        size_t copied = 0;
        size_t fill_before = rb_frame_count();
        int burst = (fill_before < PCM_LOW_WATER_FRAMES)
                        ? Config::WEBRADIO_RECOVERY_COPY_BURST
                        : Config::WEBRADIO_COPY_BURST;

        // Cada copy() puxa dados do stream, descodifica MP3 e escreve PCM no sink.
        // Sunshine 192 precisa de mais rajadas para manter o ringbuffer cheio.
        for (int i = 0; i < burst; ++i)
        {
            size_t n = copier.copy();
            copied += n;
            if (n == 0)
                break;
        }

        publishPendingMetadata();
        timeSyncLoop();
        logWebRadioStats();

        size_t fill_after = rb_frame_count();

        if (copied == 0)
            vTaskDelay(pdMS_TO_TICKS(1));
        else if (fill_after < PCM_LOW_WATER_FRAMES)
            taskYIELD();
        else if (fill_after < PCM_HIGH_WATER_FRAMES)
            taskYIELD();
        else
            vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =====================================================
// PCM ringbuffer interleaved int16: L,R,L,R...
// =====================================================

static int16_t pcm_ring[PCM_SAMPLES];

static volatile size_t pcm_wr = 0;
static volatile size_t pcm_rd = 0;
static portMUX_TYPE pcm_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t pcm_pending_byte = 0;
static bool pcm_has_pending_byte = false;
static bool pcm_playing = false;
static volatile uint32_t pcm_underruns = 0;
static volatile uint32_t pcm_producer_waits = 0;
static float pcm_resample_pos = 0.0f;
static int pcm_resample_rate = 0;

static inline size_t rb_next(size_t p)
{
    return (p + 1) % PCM_SAMPLES;
}

static inline size_t rb_count_snapshot()
{
    size_t wr = pcm_wr;
    size_t rd = pcm_rd;
    if (wr >= rd)
        return wr - rd;
    return PCM_SAMPLES - rd + wr;
}

static inline size_t rb_frame_count()
{
    return rb_count_snapshot() / 2;
}

static void logWebRadioStats()
{
    if (!Config::WEBRADIO_DEBUG_LOGS)
        return;

    static uint32_t last_stats_ms = 0;
    static int last_rate = 0;
    static int last_channels = 0;
    uint32_t now_ms = millis();

    MP3FrameInfo info = mp3.audioInfoEx();
    if (info.samprate > 0 &&
        (info.samprate != last_rate || info.nChans != last_channels))
    {
        last_rate = info.samprate;
        last_channels = info.nChans;
        Console::enqueuef(LogLevel::INFO,
                          "WebRadio MP3: %d Hz, %d ch, %d bps",
                          info.samprate,
                          info.nChans,
                          info.bitrate);
    }

    if (now_ms - last_stats_ms >= 5000)
    {
        last_stats_ms = now_ms;
        Console::enqueuef(LogLevel::INFO,
                          "WebRadio PCM: fill=%u/%u frames, underruns=%u, producer_waits=%u",
                          (unsigned)rb_frame_count(),
                          (unsigned)PCM_FRAMES,
                          (unsigned)pcm_underruns,
                          (unsigned)pcm_producer_waits);
    }
}

static inline void rb_reset()
{
    // Chamado no arranque do WebRadio: limpa indices e estatisticas do buffer.
    portENTER_CRITICAL(&pcm_mux);
    pcm_wr = 0;
    pcm_rd = 0;
    pcm_pending_byte = 0;
    pcm_has_pending_byte = false;
    pcm_playing = false;
    pcm_underruns = 0;
    pcm_producer_waits = 0;
    pcm_resample_pos = 0.0f;
    pcm_resample_rate = 0;
    portEXIT_CRITICAL(&pcm_mux);

}

static inline void rb_push_sample_blocking(int16_t sample)
{
    bool counted_full = false;

    for (;;)
    {
        // Back-pressure: se o decoder esta demasiado adiantado, espera que o
        // DSP consuma audio suficiente antes de aceitar mais PCM.
        if (rb_frame_count() >= PCM_HIGH_WATER_FRAMES)
        {
            if (!counted_full)
            {
                ++pcm_producer_waits;
                counted_full = true;
            }

            while (rb_frame_count() > PCM_LOW_WATER_FRAMES)
                vTaskDelay(pdMS_TO_TICKS(1));
        }

        size_t n = rb_next(pcm_wr);
        if (n != pcm_rd)
        {
            pcm_ring[pcm_wr] = sample;
            pcm_wr = n;
            return;
        }

        if (!counted_full)
        {
            ++pcm_producer_waits;
            counted_full = true;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static inline size_t rb_write_bytes_as_samples(const uint8_t *data, size_t len)
{
    if (!data || len == 0)
        return 0;

    size_t i = 0;

    // O Helix escreve bytes; o DSP precisa de samples int16 little endian.
    // Se chegou um byte solto no bloco anterior, juntamos com o primeiro atual.
    if (pcm_has_pending_byte)
    {
        int16_t v = (int16_t)((uint16_t)pcm_pending_byte |
                              ((uint16_t)data[0] << 8));
        rb_push_sample_blocking(v);
        pcm_has_pending_byte = false;
        i = 1;
    }

    for (; (i + 1) < len; i += 2)
    {
        int16_t v = (int16_t)((uint16_t)data[i] |
                              ((uint16_t)data[i + 1] << 8));
        rb_push_sample_blocking(v);
    }

    // Guarda byte impar para completar o sample na proxima chamada.
    if (i < len)
    {
        pcm_pending_byte = data[i];
        pcm_has_pending_byte = true;
    }

    return len;
}

static inline size_t rb_pop_frames(int16_t *out, size_t frames)
{
    if (!out || frames == 0)
        return 0;

    size_t got_frames = 0;

    size_t available_frames = rb_count_snapshot() / 2;
    if (available_frames > frames)
        available_frames = frames;

    for (; got_frames < available_frames; ++got_frames)
    {
        out[got_frames * 2 + 0] = pcm_ring[pcm_rd];
        pcm_rd = rb_next(pcm_rd);

        out[got_frames * 2 + 1] = pcm_ring[pcm_rd];
        pcm_rd = rb_next(pcm_rd);
    }

    return got_frames;
}

static inline bool rb_peek_frame(size_t frame_offset, int16_t &l, int16_t &r)
{
    size_t available_frames = rb_frame_count();
    if (frame_offset >= available_frames)
        return false;

    size_t idx = (pcm_rd + frame_offset * 2) % PCM_SAMPLES;
    l = pcm_ring[idx];
    idx = rb_next(idx);
    r = pcm_ring[idx];
    return true;
}

static inline void rb_drop_frames(size_t frames)
{
    size_t available_frames = rb_frame_count();
    if (frames > available_frames)
        frames = available_frames;

    for (size_t i = 0; i < frames * 2; ++i)
        pcm_rd = rb_next(pcm_rd);
}

// =====================================================
// PipelineSink write()
// =====================================================

size_t PipelineSink::write(const uint8_t *data, size_t len)
{
    // AudioTools/Helix entrega PCM 16-bit little endian.
    return rb_write_bytes_as_samples(data, len);
}

// =====================================================
// Init / loop
// =====================================================

bool webRadioInit()
{
    AudioLogger::instance().begin(Serial, AudioLogger::Warning);
    rb_reset();
    pinMode(Config::INPUT_SOURCE_BUTTON_PIN, INPUT_PULLUP);

    // A ligacao WiFi fica aqui para garantir que o stream e o NTP arrancam juntos.
    if (WiFi.status() != WL_CONNECTED)
    {
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASS);

        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(250);
            if (millis() - t0 > 20000)
            {
                Console::enqueuef(LogLevel::ERROR,
                                  "WebRadio WiFi connect timeout, status=%d heap=%u",
                                  (int)WiFi.status(),
                                  (unsigned)ESP.getFreeHeap());
                return false;
            }
        }
    }

    (void)timeSyncInit();

    copier.setDelayOnNoData(1);

    // Inicia a fonte atual. Por defeito arranca em NJOY; depois o botao BOOT
    // pode alternar entre streams e ADC.
    if (!openCurrentStream())
    {
        Console::enqueuef(LogLevel::ERROR,
                          "WebRadio stream open failed, heap=%u",
                          (unsigned)ESP.getFreeHeap());
        return false;
    }

    if (!web_radio_task_handle)
    {
        BaseType_t ok = xTaskCreatePinnedToCore(webRadioTask,
                                                "webradio",
                                                Config::WEBRADIO_STACK_WORDS,
                                                nullptr,
                                                Config::WEBRADIO_PRIORITY,
                                                &web_radio_task_handle,
                                                Config::WEBRADIO_CORE);
        if (ok != pdPASS)
        {
            web_radio_task_handle = nullptr;
            web_radio_task_running = false;
            Console::enqueue(LogLevel::WARN, "WebRadio task creation failed; using loop fallback");
        }
        else
        {
            if (Config::ENABLE_BOOT_INFO_LOGS)
            {
                Console::enqueuef(LogLevel::INFO,
                                  "WebRadio task started on Core %d priority %u",
                                  Config::WEBRADIO_CORE,
                                  (unsigned)Config::WEBRADIO_PRIORITY);
            }
        }
    }

    return true;
}

void webRadioPollSourceButton()
{
    static bool initialized = false;
    static bool was_pressed = false;
    static uint32_t last_change_ms = 0;

    bool pressed = (digitalRead(Config::INPUT_SOURCE_BUTTON_PIN) == LOW);
    uint32_t now = millis();

    if (!initialized)
    {
        initialized = true;
        was_pressed = pressed;
        last_change_ms = now;
        return;
    }

    // No ESP32-S3 a tecla BOOT e GPIO0. Durante flash/reset ela pode estar
    // pressionada; ignoramos o arranque para nao saltar de NJOY sozinho.
    if (now < Config::INPUT_SOURCE_BUTTON_STARTUP_IGNORE_MS)
    {
        was_pressed = pressed;
        last_change_ms = now;
        return;
    }

    if (!pressed && was_pressed &&
        (now - last_change_ms) >= Config::INPUT_SOURCE_BUTTON_DEBOUNCE_MS)
    {
        uint8_t next = requested_source + 1;
        if (next >= (uint8_t)AudioInputSource::COUNT)
            next = 0;
        requested_source = next;
        last_change_ms = now;
    }

    was_pressed = pressed;
}

bool webRadioUsingStream()
{
    return sourceDef(current_source).is_stream;
}

AudioInputSource webRadioCurrentSource()
{
    return (AudioInputSource)current_source;
}

const char *webRadioCurrentSourceName()
{
    return sourceDef(current_source).name;
}

void webRadioLoop()
{
    webRadioPollSourceButton();

    if (!web_radio_task_running)
    {
        applyRequestedSource();
        retryCurrentStreamIfDue();

        if (!sourceDef(current_source).is_stream || !stream_open)
        {
            publishPendingMetadata();
            timeSyncLoop();
            vTaskDelay(pdMS_TO_TICKS(20));
            return;
        }

        int burst = (rb_frame_count() < PCM_LOW_WATER_FRAMES)
                        ? Config::WEBRADIO_RECOVERY_COPY_BURST
                        : Config::WEBRADIO_COPY_BURST;

        for (int i = 0; i < burst; ++i)
        {
            if (copier.copy() == 0)
                break;
        }
        publishPendingMetadata();
        timeSyncLoop();
    }
}

// =====================================================
// Output para o DSP_pipeline
// =====================================================

bool webRadioGetInterleaved(float *outInterleaved, size_t frames)
{
    if (!outInterleaved || frames == 0)
        return false;

    MP3FrameInfo info = mp3.audioInfoEx();
    int source_rate = (info.samprate > 0) ? info.samprate : (int)Config::SAMPLE_RATE_ADC;
    if (source_rate != pcm_resample_rate)
    {
        pcm_resample_rate = source_rate;
        pcm_resample_pos = 0.0f;
        if (Config::WEBRADIO_DEBUG_LOGS)
        {
            Console::enqueuef(LogLevel::INFO,
                              "WebRadio resampler: %d -> %u Hz",
                              source_rate,
                              (unsigned)Config::SAMPLE_RATE_ADC);
        }
    }

    float step = (float)source_rate / (float)Config::SAMPLE_RATE_ADC;
    if (step < 0.5f || step > 2.0f)
        step = 1.0f;

    int16_t pcm_block[Config::BLOCK_SIZE * 2];
    size_t max_frames = frames;
    if (max_frames > Config::BLOCK_SIZE)
        max_frames = Config::BLOCK_SIZE;

    size_t available_frames = rb_frame_count();
    size_t needed_frames = (step == 1.0f)
                               ? max_frames
                               : ((size_t)(pcm_resample_pos + step * (float)max_frames) + 2);

    if (!pcm_playing)
    {
        // No arranque, espera por um prebuffer para evitar cortes logo no inicio.
        if (available_frames < PCM_PREBUFFER_FRAMES)
        {
            // Nao conta como underrun: ainda estamos a encher o buffer inicial.
            for (size_t f = 0; f < frames; ++f)
            {
                outInterleaved[f * 2 + 0] = 0.0f;
                outInterleaved[f * 2 + 1] = 0.0f;
            }
            return false;
        }
        pcm_playing = true;
    }

    if (available_frames < needed_frames)
    {
        // Se o stream falhar momentaneamente, entrega silencio e volta ao
        // prebuffer. O objetivo e evitar chegar aqui mantendo o ring mais cheio.
        ++pcm_underruns;
        pcm_playing = false;
        for (size_t f = 0; f < frames; ++f)
        {
            outInterleaved[f * 2 + 0] = 0.0f;
            outInterleaved[f * 2 + 1] = 0.0f;
        }
        return false;
    }

    if (step != 1.0f)
    {
        for (size_t f = 0; f < max_frames; ++f)
        {
            size_t i0 = (size_t)pcm_resample_pos;
            size_t i1 = i0 + 1;
            float frac = pcm_resample_pos - (float)i0;

            int16_t l0 = 0, r0 = 0, l1 = 0, r1 = 0;
            rb_peek_frame(i0, l0, r0);
            rb_peek_frame(i1, l1, r1);

            float l = (float)l0 + ((float)l1 - (float)l0) * frac;
            float r = (float)r0 + ((float)r1 - (float)r0) * frac;
            outInterleaved[f * 2 + 0] = l / 32768.0f;
            outInterleaved[f * 2 + 1] = r / 32768.0f;

            pcm_resample_pos += step;
        }

        size_t drop = (size_t)pcm_resample_pos;
        rb_drop_frames(drop);
        pcm_resample_pos -= (float)drop;

        for (size_t f = max_frames; f < frames; ++f)
        {
            outInterleaved[f * 2 + 0] = 0.0f;
            outInterleaved[f * 2 + 1] = 0.0f;
        }

        return true;
    }

    size_t gotFrames = rb_pop_frames(pcm_block, max_frames);

    for (size_t f = 0; f < gotFrames; ++f)
    {
        outInterleaved[f * 2 + 0] = (float)pcm_block[f * 2 + 0] / 32768.0f;
        outInterleaved[f * 2 + 1] = (float)pcm_block[f * 2 + 1] / 32768.0f;
    }

    // Se nao chegou audio suficiente, preenche o resto com silencio.
    for (size_t f = gotFrames; f < frames; ++f)
    {
        outInterleaved[f * 2 + 0] = 0.0f;
        outInterleaved[f * 2 + 1] = 0.0f;
    }
    if (gotFrames < frames)
    {
        ++pcm_underruns;
        pcm_playing = false;
    }

    return gotFrames > 0;
}
