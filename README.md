🇬🇧 English | [🇩🇪 Deutsch](README.de.md)

# PL225 / PCSK225 (e-CzasPL) Decoder for the Original ESP32

Test port of the PL225 decoder from [Timesignal_decoder](https://github.com/radiolab81/Timesignal_decoder) to the
original ESP32 (Xtensa, ESP-IDF 6.1). The actual decoder/DSP logic
(`main/pl225/*`) is carried over **unchanged** from the PC project (plain
standard C++17, no platform dependencies) — only the audio input chain
(ADC instead of a WAV file) and the output (serial console instead of
stdout) have been ported.

## Why ADC and not "just AM"?

PL225 is a **phase-modulation** scheme, not on-off keying. Conventional
envelope AM reception irrecoverably destroys the phase information (see
the header comment in `main/pl225/pl225_decoder.hpp`). An **SSB
receiver** is therefore required (e.g. reception on 224 kHz USB, or an
SI4732 in SSB mode), whose audio/headphone output presents the 225 kHz
carrier as a stable tone at the configured BFO frequency (1000 Hz by
default here). This tone still contains the phase modulation in full —
which is exactly what `MonoToIqDownconverter` (likewise carried over
unchanged from the original project) exploits, recovering a complex
baseband IQ signal via a software BFO mix, which is then fed to
`Pl225Decoder::processBlock()` exactly as in the PC project.

## Hardware

- **Original ESP32** (Xtensa ESP32; the ESP32-S2/S3/C3 have different ADC
  peripherals and have not been tested with this code).

- Audio input: **GPIO34 (ADC1, channel 6)**.

- The ESP32's ADC can only measure 0..~3.1 V (unipolar), while the
  receiver's audio signal is bipolar. A simple **AC coupling + bias
  network ("bias tee")** is therefore required.

  Set the receiver volume so the tone does not overdrive the ADC — the
  firmware prints the current DC average every 10 s on the console
  (target range: near 2048 of 4095), allowing the circuit/drive level to
  be monitored.

## Building and flashing (ESP-IDF 6.1)

```
. $IDF_PATH/export.sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Configuration

In `main/main.cpp`:

- `kAdcChannel` / GPIO: which ADC1 pin is used.
- `kAdcSampleRateHz`: audio sampling rate (default 20 kHz, comfortably
  sufficient for the 50-bit/s PL225 rate).
- `kBfoFrequencyHz`: must match the BFO/reception frequency actually set
  on the receiver (default 1000 Hz, as in the PC project's
  `225kHz_IQ.wav`/README example).
- `kLowpassCutoffHz`: low-pass cutoff frequency after the software BFO
  mixing stage (default 300 Hz, as in the original).

## Output

Successfully decoded time telegrams appear on the serial console (UART0,
115200 baud), e.g.:

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

Decode errors (incomplete/invalid frames) are logged as warnings
(`ESP_LOGW`); this is normal during continuous operation (frames arrive
every few seconds, and not every one is received error-free/complete).

## Known limitations

- As in the original PC project: according to PA3FWM, the exact time
  reference of a frame is accurate to only 100–200 ms even at the
  transmitter — unproblematic for a wall clock, unsuitable for an NTP
  server.
- Pure AM envelope reception fundamentally does not work (see above) —
  this is not a limitation of this port, but a consequence of signal
  theory.
