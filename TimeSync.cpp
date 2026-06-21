#include "TimeSync.h"

#include "Config.h"
#include "Console.h"
#include "RDSAssembler.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_sntp.h>
#include <time.h>

namespace
{
bool s_ntp_configured = false;
bool s_ntp_stopped_after_sync = false;
bool s_time_valid = false;
int s_last_rds_minute = -1;
uint32_t s_last_attempt_ms = 0;
uint32_t s_last_loop_ms = 0;

long dateToJdn(int year, int month, int day)
{
    // Julian Day Number usado para calcular diferencas de dia entre UTC e hora local.
    int a = (14 - month) / 12;
    int y = year + 4800 - a;
    int m = month + 12 * a - 3;
    return day + (153 * m + 2) / 5 + 365L * y + y / 4 - y / 100 + y / 400 - 32045;
}

int localOffsetHalfHours(const tm &local_tm, const tm &utc_tm)
{
    // O RDS CT codifica o fuso horario em passos de 30 minutos.
    // Portugal fica 0 no inverno e +2 meios-blocos no horario de verao.
    const int local_year = local_tm.tm_year + 1900;
    const int utc_year = utc_tm.tm_year + 1900;

    long local_day = dateToJdn(local_year, local_tm.tm_mon + 1, local_tm.tm_mday);
    long utc_day = dateToJdn(utc_year, utc_tm.tm_mon + 1, utc_tm.tm_mday);

    int local_minutes = local_tm.tm_hour * 60 + local_tm.tm_min;
    int utc_minutes = utc_tm.tm_hour * 60 + utc_tm.tm_min;
    int offset_minutes = (int)((local_day - utc_day) * 1440L) + local_minutes - utc_minutes;

    if (offset_minutes >= 0)
        return (offset_minutes + 15) / 30;
    return (offset_minutes - 15) / 30;
}

bool updateRdsClockFromSystem(bool log_first_sync)
{
    time_t now = time(nullptr);
    if (now < 1704067200)
        return false;

    tm local_tm{};
    tm utc_tm{};
    localtime_r(&now, &local_tm);
    gmtime_r(&now, &utc_tm);

    if ((local_tm.tm_year + 1900) < 2024)
        return false;

    // Atualizar o CT uma vez por minuto chega; evita trafego desnecessario
    // entre a task de WebRadio e o assembler RDS.
    int minute_key = local_tm.tm_hour * 60 + local_tm.tm_min;
    if (minute_key == s_last_rds_minute && s_time_valid)
        return true;

    s_last_rds_minute = minute_key;
    int offset_half_hours = localOffsetHalfHours(local_tm, utc_tm);

    RDSAssembler::setClock(local_tm.tm_year + 1900,
                           local_tm.tm_mon + 1,
                           local_tm.tm_mday,
                           local_tm.tm_hour,
                           local_tm.tm_min,
                           (int8_t)offset_half_hours);

    if (Config::ENABLE_BOOT_INFO_LOGS && !s_time_valid && log_first_sync)
    {
        Console::enqueuef(LogLevel::INFO,
                          "NTP time synced: %04d-%02d-%02d %02d:%02d UTC%+d:%02d",
                          local_tm.tm_year + 1900,
                          local_tm.tm_mon + 1,
                          local_tm.tm_mday,
                          local_tm.tm_hour,
                          local_tm.tm_min,
                          offset_half_hours / 2,
                          (offset_half_hours & 1) ? 30 : 0);
    }

    s_time_valid = true;
    return true;
}
} // namespace

bool timeSyncInit()
{
    if (!Config::NTP_TIME_SYNC_ENABLED)
        return false;

    if (WiFi.status() != WL_CONNECTED)
        return false;

    s_last_attempt_ms = millis();

    if (!s_ntp_configured)
    {
        configTzTime(Config::NTP_TIMEZONE_PORTUGAL,
                     Config::NTP_SERVER_1,
                     Config::NTP_SERVER_2,
                     Config::NTP_SERVER_3);
        s_ntp_configured = true;
    }

    uint32_t t0 = millis();
    while ((millis() - t0) < Config::NTP_SYNC_TIMEOUT_MS)
    {
        if (updateRdsClockFromSystem(true))
        {
            // O NTP so serve para acertar a hora no arranque. Depois disso,
            // o relogio interno do ESP mantem a hora e alimenta o RDS CT.
            if (!s_ntp_stopped_after_sync)
            {
                esp_sntp_stop();
                s_ntp_stopped_after_sync = true;
            }
            return true;
        }
        delay(100);
    }

    Console::enqueue(LogLevel::WARN, "NTP time sync failed; CT will keep default clock");
    return false;
}

void timeSyncLoop()
{
    if (!Config::NTP_TIME_SYNC_ENABLED)
        return;

    uint32_t now_ms = millis();
    if ((now_ms - s_last_loop_ms) < Config::NTP_RDS_UPDATE_INTERVAL_MS)
        return;
    s_last_loop_ms = now_ms;

    if (!s_time_valid &&
        WiFi.status() == WL_CONNECTED &&
        (now_ms - s_last_attempt_ms) >= Config::NTP_RETRY_INTERVAL_MS)
    {
        (void)timeSyncInit();
        return;
    }

    // Sem trafego NTP aqui: so copia a hora interna atual para o grupo CT.
    (void)updateRdsClockFromSystem(false);
}

bool timeSyncIsValid()
{
    return s_time_valid;
}
