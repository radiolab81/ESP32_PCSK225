// main.cpp - PL225/PCSK225 ("e-CzasPL") Dekoder-Portierung für den
// Ur-ESP32 (Original-Xtensa-ESP32, ESP-IDF 6.1).
//
// -------------------------------------------------------------------------
// Signalweg
// -------------------------------------------------------------------------
// Ein SSB-Empfänger (z.B. auf 224 kHz USB, oder ein SI4732 im SSB-Modus)
// liefert am NF-/Kopfhörer-Ausgang ein reelles Mono-Audiosignal, in dem
// der 225 kHz Träger als stabiler Ton bei der eingestellten BFO-Frequenz
// (Default hier: 1000 Hz, siehe kBfoFrequencyHz) erscheint. Dieses Audio
// wird per ADC eingelesen und läuft dann durch dieselbe Dekoder-Kette wie
// im PC-Originalprojekt (siehe pl225_decoder.hpp für die Herleitung, warum
// SSB-Mischung statt AM-Hüllkurvendemodulation nötig ist):
//
//   ADC (roh, unipolar 0..4095) --normalize--> float [-1..1] NF-Samples
//     --> MonoToIqDownconverter (Software-BFO-Mischung + Tiefpass)
//     --> std::complex<float> Basisband-IQ --> Dezimierung
//     --> pl225::Pl225Decoder::processBlock()
//     --> IDecodedTimeSink --> Ausgabe auf der seriellen Konsole (UART0)
//
// -------------------------------------------------------------------------
// Hardware / Beschaltung
// -------------------------------------------------------------------------
// - ADC1, Kanal 6 = GPIO34 (ADC1 bleibt auch bei WLAN-Betrieb nutzbar;
//   GPIO34 ist ein reiner Eingang, ideal für ein hochohmiges Audiosignal).
// - Das NF-Signal des Empfängers ist bipolar, der ESP32-ADC kann aber nur
//   0..~3.1V (unipolar) messen. Nötig ist daher eine AC-Kopplung
//   (Koppelkondensator) plus Vorspannung auf die Mitte des ADC-Bereichs
//   (Spannungsteiler 2x 10kOhm von 3.3V nach GND, Mittelabgriff über den
//   Koppelkondensator ans Audiosignal) UND ein Anti-Aliasing-Tiefpass
//   direkt vor dem ADC-Pin.
// - Lautstärke so einstellen, dass der Ton den ADC nicht übersteuert, aber
//   deutlich über dem Rauschen liegt (siehe die periodische DC-Bias-/AC-
//   Spitzenwert-Diagnose weiter unten).
//
// -------------------------------------------------------------------------
// Bekannte Einschränkung ggü. reiner IQ-Aufnahme
// -------------------------------------------------------------------------
// Dieser Signalweg funktioniert NUR mit echtem SSB-Empfang (BFO-Mischung
// erhält die Phaseninformation). Ein normaler AM-Envelope-Empfänger
// liefert KEINE Phaseninformation - PL225 ist dann grundsätzlich nicht
// dekodierbar (Signaltheorie, keine Einschränkung dieser Portierung).
//
// -------------------------------------------------------------------------
// Performance-Hinweise (siehe README für die volle Herleitung)
// -------------------------------------------------------------------------
// Drei Anpassungen waren nötig, damit der Decoder auf dem Ur-ESP32 in
// Echtzeit mithält:
//   1. Dezimierung vor dem Decoder (kDecimationFactor) - reduziert sowohl
//      dessen internen ~1s-Verlaufspuffer als auch die Rahmenlänge, die
//      computeAdaptiveThreshold() bei jedem Sync-Versuch neu scannt.
//   2. sdkconfig.defaults setzt CONFIG_COMPILER_OPTIMIZATION_PERF=y (-O2) -
//      der ESP-IDF-Default (-Og) reichte nicht aus.
//   3. MonoToIqDownconverter nutzt einen NCO per komplexer Rotation statt
//      sin()/cos() pro Sample (siehe dortiger Kommentar) - die einzige
//      bewusste funktionale Änderung an einer sonst unveränderten
//      PC-Original-Datei, nötig weil picolibcs sin()/cos() auf dem Xtensa-
//      Kern ohne Hardware-Transzendenten-Unterstützung deutlich teurer
//      sind als auf einem PC.

#include <cstdio>
#include <cstring>
#include <vector>

#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pl225_decoder.hpp"
#include "mono_to_iq_downconverter.hpp"
#include "time_signal_decoder.hpp"

namespace {

static const char* TAG = "PL225_PCSK225";

// ---- Konfiguration --------------------------------------------------
constexpr adc_channel_t kAdcChannel = ADC_CHANNEL_6;   // GPIO34 (ADC1)
constexpr uint32_t kAdcSampleRateHz = 20000;           // NF-Abtastrate (ADC)
constexpr double   kBfoFrequencyHz  = 1000.0;          // Empfänger-BFO/Anzeige
constexpr double   kLowpassCutoffHz = 300.0;           // Downconverter-Tiefpass

// Dezimierung vor dem Decoder: reduziert dessen ~1s-Verlaufspuffer und die
// pro Sync-Versuch gescannte Rahmenlänge (kFrameBits*samplesPerBit_) auf
// ein für den Ur-ESP32 handhabbares Maß. Getestet (per host_sim/ gegen die
// Referenz-WAV, auch mit reduzierter Amplitude + simuliertem ADC-
// Rauschen) bis Faktor 80 erfolgreich; 20 ist bewusst konservativer
// gewählt, um Marge für realen, stärker verrauschten Empfang zu behalten.
constexpr unsigned kDecimationFactor = 20;
constexpr uint32_t kDecoderSampleRateHz = kAdcSampleRateHz / kDecimationFactor; // 1000 Hz

// ADC-Continuous-Treiber liest in Frames aus 2-Byte "digi"-Ergebnissen.
constexpr size_t kAdcFrameSamples = 256;  // Samples pro DMA-Frame (~12.8ms bei 20kHz)
constexpr size_t kAdcFrameBytes = kAdcFrameSamples * SOC_ADC_DIGI_RESULT_BYTES;
constexpr size_t kAdcPoolFrames = 4;      // interner DMA-Puffer (Frames)

adc_continuous_handle_t g_adcHandle = nullptr;

// ---- Sink: gibt dekodierte Zeit auf der seriellen Konsole aus -------
class SerialTimeSink : public timesignal::IDecodedTimeSink {
public:
    void onDecodedTime(const timesignal::DecodedTime& t) override {
        printf("[%s] Zeitpunkt dekodiert: 20%02d-%02d-%02d %02d:%02d:%02d UTC",
               t.sourceTag.c_str(), t.year2, t.month, t.day, t.hour, t.minute, t.second);
        if (t.dateValid) {
            printf(" (Wochentag %d)", t.weekday);
        }
        printf(" TZ=+%dh", t.tzOffsetHours);
        if (t.leapSecondAnnounced) {
            printf(" LEAP-SECOND(%s angekuendigt)", t.leapSecondIsRemoval ? "Entfernung" : "Einfuegung");
        }
        if (!t.transmitterStateText.empty()) {
            printf(" Status='%s'", t.transmitterStateText.c_str());
        }
        printf("\n");
        fflush(stdout);
    }

    void onDecodeError(const std::string& reason) override {
        ESP_LOGW(TAG, "Dekodierfehler: %s", reason.c_str());
    }

    void onBitDecoded(int bitIndex, int bitValue) override {
        // Hinweis: Pl225Decoder ruft dies aktuell nicht auf
        // - bleibt dennoch implementiert, falls sich
        // das in einer zukünftigen Decoder-Version ändert.
        ESP_LOGD(TAG, "Bit %d = %d", bitIndex, bitValue);
    }
};

// ---- ADC-Continuous-Treiber initialisieren ---------------------------
void initAdc() {
    adc_continuous_handle_cfg_t handleCfg = {};
    handleCfg.max_store_buf_size = kAdcFrameBytes * kAdcPoolFrames;
    handleCfg.conv_frame_size = kAdcFrameBytes;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handleCfg, &g_adcHandle));

    adc_digi_pattern_config_t patternCfg = {};
    patternCfg.atten = ADC_ATTEN_DB_12;   // voller ~0..3.1V Bereich
    patternCfg.channel = kAdcChannel;
    patternCfg.unit = ADC_UNIT_1;
    patternCfg.bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t adcCfg = {};
    adcCfg.pattern_num = 1;
    adcCfg.adc_pattern = &patternCfg;
    adcCfg.sample_freq_hz = kAdcSampleRateHz;
    adcCfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    adcCfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2; // Ur-ESP32: Typ2-Format
    ESP_ERROR_CHECK(adc_continuous_config(g_adcHandle, &adcCfg));

    ESP_ERROR_CHECK(adc_continuous_start(g_adcHandle));
    ESP_LOGI(TAG, "ADC gestartet: GPIO34 (ADC1_CH6), %u Hz, 12 Bit", (unsigned)kAdcSampleRateHz);
}

// Wandelt einen rohen ADC-Frame (Typ2-Format) in normierte float-Samples
// [-1..1] um. Der Nullpunkt (Bias-Mittelspannung) wird laufend als
// gleitender Mittelwert nachgeführt, damit Bauteiltoleranzen der externen
// Vorspannung keine feste Kalibrierung erfordern.
float g_dcEstimate = 2048.0f; // Start: Mitte des 12-Bit-Bereichs
constexpr float kDcTrackingAlpha = 0.0005f; // sehr langsam (>> 20ms Bitdauer)

// Diagnose: AC-Spitzenwert (Betrag) seit dem letzten Log-Zeitpunkt - zeigt,
// ob überhaupt ein Wechselsignal (der BFO-Ton) am ADC ankommt.
float g_acPeakSinceLastLog = 0.0f;

// Diagnose gegen Datenverlust/DMA-Überlauf: zählt tatsächlich verarbeitete
// ADC-Rohsamples mit. Bleibt (verarbeitete Samples / Abtastrate) hinter der
// real vergangenen Zeit zurück, gehen Samples verloren - das würde die
// über Sekunden nötige PSK-Phasenverfolgung zerstören, ohne dass eine kurze
// Stichprobe das zeigen würde.
uint64_t g_totalSamplesProcessed = 0;

void adcFrameToFloatSamples(const uint8_t* data, size_t len, std::vector<float>& out) {
    out.clear();
    size_t count = len / SOC_ADC_DIGI_RESULT_BYTES;
    out.reserve(count);
    const auto* results = reinterpret_cast<const adc_digi_output_data_t*>(data);
    for (size_t i = 0; i < count; ++i) {
        uint32_t raw = results[i].type2.data; // 12-Bit-Rohwert, Typ2-Format
        g_dcEstimate += kDcTrackingAlpha * (static_cast<float>(raw) - g_dcEstimate);
        float centered = static_cast<float>(raw) - g_dcEstimate;
        float normalized = centered / 2048.0f; // normieren auf ca. [-1..1]
        float absNormalized = normalized < 0 ? -normalized : normalized;
        if (absNormalized > g_acPeakSinceLastLog) {
            g_acPeakSinceLastLog = absNormalized;
        }
        out.push_back(normalized);
    }
}

} // namespace

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "PL225/PCSK225-Dekoder fuer Ur-ESP32 startet...");

    initAdc();

    SerialTimeSink sink;
    // Decoder arbeitet mit der dezimierten Rate (kDecoderSampleRateHz) -
    // begrenzt sowohl seinen internen Verlaufspuffer als auch die pro
    // Sync-Versuch gescannte Rahmenlänge auf ein für den Ur-ESP32
    // handhabbares Maß (siehe Konfigurationskommentar oben).
    pl225::Pl225Decoder decoder(sink, kDecoderSampleRateHz);
    // Downconverter (NCO-Mischung + Tiefpass) läuft mit der vollen
    // ADC-Rate - die Dezimierung passiert danach, im Hauptloop unten.
    pl225::MonoToIqDownconverter downconverter(kAdcSampleRateHz, kBfoFrequencyHz, kLowpassCutoffHz);

    std::vector<uint8_t> adcBuf(kAdcFrameBytes);
    std::vector<float> nfSamples;
    nfSamples.reserve(kAdcFrameSamples);
    std::vector<std::complex<float>> decimatedIq;
    decimatedIq.reserve(kAdcFrameSamples / kDecimationFactor + 1);
    unsigned decimationCounter = 0;

    int64_t lastLevelLogUs = esp_timer_get_time();
    const int64_t startUs = lastLevelLogUs; // Referenz fuer die Datenverlust-Diagnose

    while (true) {
        uint32_t bytesRead = 0;
        esp_err_t err = adc_continuous_read(g_adcHandle, adcBuf.data(), adcBuf.size(),
                                             &bytesRead, 1000 /* ms Timeout */);
        if (err != ESP_OK) {
            if (err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "adc_continuous_read Fehler: %s", esp_err_to_name(err));
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // genuiner Yield-Punkt: nichts zu tun
            continue;
        }

        adcFrameToFloatSamples(adcBuf.data(), bytesRead, nfSamples);
        if (nfSamples.empty()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        g_totalSamplesProcessed += nfSamples.size();

        std::vector<std::complex<float>> iqSamples = downconverter.process(nfSamples);

        // Dezimierung: nur jedes kDecimationFactor-te (bereits tiefpass-
        // gefilterte) IQ-Sample an den Decoder weitergeben.
        decimatedIq.clear();
        for (const auto& s : iqSamples) {
            if (decimationCounter == 0) {
                decimatedIq.push_back(s);
            }
            decimationCounter = (decimationCounter + 1) % kDecimationFactor;
        }
        if (decimatedIq.empty()) {
            // Sollte bei vollen Bloecken praktisch nie eintreten - falls
            // doch, kurzer Yield statt Endlosschleife.
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint64_t blockEndMs = static_cast<uint64_t>(esp_timer_get_time() / 1000);
        decoder.processBlock(decimatedIq, blockEndMs);

        // Periodischer, garantierter Yield als Sicherheitsnetz fuer den
        // Task-Watchdog (IDLE0 muss gelegentlich laufen koennen). Bewusst
        // nur alle kWatchdogSafetyYieldEveryNBlocks Durchlaeufe (~640ms),
        // nicht bei jedem Block - ein Delay bei JEDEM Block wuerde den
        // Durchsatz spuerbar unter Echtzeit druecken (siehe README).
        static unsigned iterationCounter = 0;
        constexpr unsigned kWatchdogSafetyYieldEveryNBlocks = 50;
        if (++iterationCounter >= kWatchdogSafetyYieldEveryNBlocks) {
            iterationCounter = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Periodische Diagnose: DC-Bias/AC-Pegel (Hardware-Beschaltung
        // pruefen) und Datenverlust-Check (verarbeitete Samples vs.
        // Echtzeit - sollte nahe 0 bleiben, siehe README).
        int64_t nowUs = esp_timer_get_time();
        if (nowUs - lastLevelLogUs > 10 * 1000 * 1000) {
            double wallClockElapsedS = (nowUs - startUs) / 1e6;
            double sampleDerivedElapsedS = (double)g_totalSamplesProcessed / (double)kAdcSampleRateHz;
            double driftS = wallClockElapsedS - sampleDerivedElapsedS;
            ESP_LOGI(TAG, "ADC DC-Bias ~ %.0f / 4095 (Soll: nahe 2048) | AC-Spitzenwert: %.4f | "
                          "Datenverlust-Drift: %.3fs (Soll: nahe 0 und stabil)",
                     (double)g_dcEstimate, (double)g_acPeakSinceLastLog, driftS);
            g_acPeakSinceLastLog = 0.0f;
            lastLevelLogUs = nowUs;
        }
    }
}
