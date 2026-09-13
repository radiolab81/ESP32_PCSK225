# PL225 / PCSK225 (e-CzasPL) Dekoder für den Ur-ESP32

Testportierung des PL225-Dekoders aus [Timesignal_decoder](.) auf den
Original-ESP32 (Xtensa, ESP-IDF 6.1). Die eigentliche Dekoder-/DSP-Logik
(`main/pl225/*`) ist **unverändert** aus dem PC-Projekt übernommen (reines
Standard-C++17, keine Plattformabhängigkeiten) - portiert wurde nur die
Audio-Eingangskette (ADC statt WAV-Datei) und die Ausgabe (serielle Konsole
statt stdout).

## Warum ADC und nicht "einfach AM"?

PL225 ist eine **Phasenmodulation**, keine Amplitudentastung. Ein normaler
Hüllkurven-AM-Empfang zerstört die Phaseninformation unwiderruflich (siehe
`main/pl225/pl225_decoder.hpp`, Kopfkommentar). Es wird daher ein **SSB-
Empfänger** benötigt (z.B. Empfang auf 224 kHz USB oder ein SI4732 im
SSB-Modus), dessen NF-/Kopfhörerausgang den 225-kHz-Träger als stabilen
Ton bei der eingestellten BFO-Frequenz (hier per Default 1000 Hz)
ausgibt. Dieser Ton enthält die Phasenmodulation weiterhin vollständig -
genau das nutzt `MonoToIqDownconverter` (ebenfalls unverändert aus dem
Originalprojekt übernommen), um daraus per Software-BFO-Mischung wieder
ein komplexes Basisband-IQ-Signal zu gewinnen, das dann exakt wie im
PC-Projekt an `Pl225Decoder::processBlock()` geht.

## Hardware

- **Ur-ESP32** (Original-Xtensa-ESP32; ESP32-S2/S3/C3 haben andere
  ADC-Peripherie und sind mit diesem Code nicht getestet).
- NF-Eingang: **GPIO34 (ADC1, Kanal 6)**.
- Der ESP32-ADC kann nur 0..~3,1V (unipolar) messen, das NF-Signal des
  Empfängers ist aber bipolar. Nötig ist daher eine einfache
  **AC-Kopplung + Vorspannung ("Bias-Tee")**

  Die Empfänger-Lautstärke so einstellen, dass der
  Ton den ADC nicht übersteuert - die Firmware gibt alle 10s den
  aktuellen DC-Mittelwert (Sollbereich: nahe 2048 von 4095) auf der
  Konsole aus, damit sich die Beschaltung/Aussteuerung kontrollieren
  lässt.

## Bauen und flashen (ESP-IDF 6.1)

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Konfiguration

In `main/main.cpp`:

- `kAdcChannel` / GPIO: welcher ADC1-Pin verwendet wird.
- `kAdcSampleRateHz`: NF-Abtastrate (Default 20 kHz, reicht für den
  50-Bit/s-PL225-Takt deutlich).
- `kBfoFrequencyHz`: muss zur tatsächlich am Empfänger eingestellten
  BFO-/Empfangsfrequenz passen (Default 1000 Hz, wie im PC-Projekt-Beispiel
  `225kHz_IQ.wav`/README).
- `kLowpassCutoffHz`: Tiefpass-Grenzfrequenz nach der Software-BFO-Mischung
  (Default 300 Hz, wie im Original).

## Ausgabe

Erfolgreich dekodierte Zeittelegramme erscheinen auf der seriellen Konsole
(UART0, 115200 Baud), z.B.:

```
[PL225] Zeitpunkt dekodiert: 2026-09-05 06:22:42 UTC (Wochentag 6) TZ=+2h Status='Normalbetrieb'
I (270493) PL225: ADC DC-Bias ~ 1777 / 4095 (Soll: nahe 2048) | AC-Spitzenwert: 0.0245 | Datenverlust-Drift: 0.003s (Soll: nahe 0 und stabil)
[PL225] Zeitpunkt dekodiert: 2026-09-05 06:22:45 UTC (Wochentag 6) TZ=+2h Status='Normalbetrieb'
[PL225] Zeitpunkt dekodiert: 2026-09-05 06:22:51 UTC (Wochentag 6) TZ=+2h Status='Normalbetrieb'
I (280503) PL225: ADC DC-Bias ~ 1777 / 4095 (Soll: nahe 2048) | AC-Spitzenwert: 0.0222 | Datenverlust-Drift: 0.001s (Soll: nahe 0 und stabil)
[PL225] Zeitpunkt dekodiert: 2026-09-05 06:23:03 UTC (Wochentag 6) TZ=+2h Status='Normalbetrieb'
I (290513) PL225: ADC DC-Bias ~ 1777 / 4095 (Soll: nahe 2048) | AC-Spitzenwert: 0.0225 | Datenverlust-Drift: -0.002s (Soll: nahe 0 und stabil)
W (292183) PL225: Dekodierfehler: PL225: Header-Byte != 0x60 - verworfen (evtl. anderer Nachrichtentyp, z.B. 'ENEA'-Lichtsteuerung auf demselben Kanal)
[PL225] Zeitpunkt dekodiert: 2026-09-05 06:23:09 UTC (Wochentag 6) TZ=+2h Status='Normalbetrieb'
W (298183) PL225: Dekodierfehler: PL225: Header-Byte != 0x60 - verworfen (evtl. anderer Nachrichtentyp, z.B. 'ENEA'-Lichtsteuerung auf demselben Kanal)
I (300513) PL225: ADC DC-Bias ~ 1777 / 4095 (Soll: nahe 2048) | AC-Spitzenwert: 0.0227 | Datenverlust-Drift: 0.003s (Soll: nahe 0 und stabil)
W (304183) PL225: Dekodierfehler: PL225: Header-Byte != 0x60 - verworfen (evtl. anderer Nachrichtentyp, z.B. 'ENEA'-Lichtsteuerung auf demselben Kanal)
I (310523) PL225: ADC DC-Bias ~ 1778 / 4095 (Soll: nahe 2048) | AC-Spitzenwert: 0.0237 | Datenverlust-Drift: 0.001s (Soll: nahe 0 und stabil)
W (319183) PL225: Dekodierfehler: PL225: Header-Byte != 0x60 - verworfen (evtl. anderer Nachrichtentyp, z.B. 'ENEA'-Lichtsteuerung auf demselben Kanal)
I (320523) PL225: ADC DC-Bias ~ 1778 / 4095 (Soll: nahe 2048) | AC-Spitzenwert: 0.0216 | Datenverlust-Drift: -0.001s (Soll: nahe 0 und stabil)
[PL225] Zeitpunkt dekodiert: 2026-09-05 06:23:39 UTC (Wochentag 6) TZ=+2h Status='Normalbetrieb'
I (330543) PL225: ADC DC-Bias ~ 1777 / 4095 (Soll: nahe 2048) | AC-Spitzenwert: 0.0218 | Datenverlust-Drift: 0.003s (Soll: nahe 0 und stabil)
W (340173) PL225: Dekodierfehler: PL225: Header-Byte != 0x60 - verworfen (evtl. anderer Nachrichtentyp, z.B. 'ENEA'-Lichtsteuerung auf demselben Kanal)
I (340543) PL225: ADC DC-Bias ~ 1777 / 4095 (Soll: nahe 2048) | AC-Spitzenwert: 0.0216 | Datenverlust-Drift: -0.001s (Soll: nahe 0 und stabil)
[PL225] Zeitpunkt dekodiert: 2026-09-05 06:24:00 UTC (Wochentag 6) TZ=+2h Status='Normalbetrieb'
[PL225] Zeitpunkt dekodiert: 2026-09-05 06:24:03 UTC (Wochentag 6) TZ=+2h Status='Normalbetrieb'
```

Dekodierfehler (unvollständige/nicht valide Rahmen) werden als Warnung
(`ESP_LOGW`) ausgegeben; das ist im Dauerbetrieb normal (Frames kommen alle
paar Sekunden, nicht jeder ist fehlerfrei/vollständig empfangbar).

## Bekannte Einschränkungen

- Wie im PC-Original: der genaue Zeitbezug eines Frames ist laut PA3FWM
  senderseitig auf 100-200ms genau - für eine Wanduhr unproblematisch, für
  einen NTP-Server ungeeignet.
- Reiner AM-Envelope-Empfang funktioniert grundsätzlich nicht (siehe oben) -
  das ist keine Einschränkung dieser Portierung, sondern Signaltheorie.

