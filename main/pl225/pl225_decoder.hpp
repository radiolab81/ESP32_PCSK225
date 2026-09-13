// pl225_decoder.hpp
//
// Decoder fuer das polnische Zeitzeichensystem "e-CzasPL" (PL225), das seit
// Ende 2023 als Phasenmodulation dem 225 kHz Mittelwellensender "Program
// Pierwszy Polskiego Radia" (Sender Solec Kujawski, 1 MW) ueberlagert wird.
//
// Quellen (siehe README fuer Details/Links):
//   - PA3FWM: "New timecode on Poland's 225 kHz signal"
//     (Dokumentation der Protokoll-Unklarheiten/Fehler, Reverse-Engineering
//     der genauen Bitreihenfolge, Skalierung, Scrambling-Interpretation)
//   - SP5WWP (e-Czas) und SP6HFE (e-CzasPL): offene Referenzimplementierungen
//     in C/C++, die CRC8- und Reed-Solomon-Parameter experimentell ermittelt
//     haben (in der offiziellen polnischen Spezifikation fehlen diese Angaben
//     komplett). Die Reed-Solomon-Fehlerkorrektur in reed_solomon_gf16.hpp
//     ist eine EIGENSTAENDIGE Implementierung (klassischer Berlekamp-Massey-
//     Algorithmus, freie/oeffentliche Mathematik) und steht wie der Rest
//     dieses Projekts unter einer freien Lizenz - siehe dortige Kommentare
//     zur Kompatibilitaet mit den realen Feldparametern.
//   - Eigene Verifikation gegen die beiliegenden Testaufnahmen (siehe
//     test/test_pl225_synthetic.cpp und README).
//
// -------------------------------------------------------------------------
// WARUM NUR IQ UND NICHT (reines) AM FUNKTIONIERT
// -------------------------------------------------------------------------
// Anders als DCF77/MSF/JJY ist PL225 KEINE Amplitudentastung, sondern eine
// reine Phasenmodulation des Traegers: 1-Bit = normale Traegerphase,
// 0-Bit = Phase um 45 Grad nachgezogen (linear eingeschwenkt). Ein echter
// Huellkurven-("Envelope"-)AM-Demodulator berechnet mathematisch exakt
// |A*e^{i*phi}| = A - das Ergebnis haengt beweisbar NICHT von der Phase phi
// ab. Die Phaseninformation ist in einer reinen Envelope-AM-Aufnahme also
// nicht "schwer", sondern GRUNDSAETZLICH UNMOEGLICH herauszufiltern, auch
// nicht mit einer Costas-Loop: eine Costas-Loop braucht ein tatsaechlich
// noch schwingendes Traegersignal zum Einrasten, keinen bereits auf die
// Hüllkurve reduzierten Skalarwert. Das im Rahmen dieser Erweiterung
// gelieferte "225kHz_AM.wav" wurde exakt so verifiziert (Spektrum entspricht
// dem normalen Rundfunkprogramm, keine stabile Traegerlinie) - siehe
// README, Abschnitt "Fehleranalyse AM vs. IQ".
//
// Die beigelegte IQ-Aufnahme ("225kHz_IQ.wav") enthaelt dagegen den
// vollstaendigen komplexen Traeger (I/Q, Traeger nahe 0 Hz da der
// Empfaenger exakt auf 225.000 kHz abgestimmt war) und wurde erfolgreich
// bis zum fertig dekodierten Zeitstempel verifiziert.
//
// -------------------------------------------------------------------------
// SIGNAL- UND RAHMENSTRUKTUR (empirisch verifiziert + Referenzcode)
// -------------------------------------------------------------------------
//   - Bitrate: 50 Bit/s (20ms/Bit), konstant (Quelle: SP5WWP e-Czas README:
//     "10 samples per symbol, that is 500 samples per second" bei deren
//     intern verwendeter 500Hz-Verarbeitungsrate -> 500/10 = 50 Bit/s).
//   - Bit=1: Phase 0 Grad (Referenzphase). Bit=0: Phase -45 Grad.
//   - Rahmen: 96 Bit (12 Byte), MSB-first (PA3FWM-Erratum: die
//     Spezifikation selbst suggeriert LSB-first, tatsaechlich gesendet
//     wird aber MSB-first):
//       Byte 0-1  : Synchronisationswort 0x5555 (alternierend, 16 Bit)
//       Byte 2    : Header/Rahmentyp, 0x60 fuer ein Zeit-Telegramm
//       Byte 3 (3 MSB) : statisches Praefix 0b101
//       Byte 3 (5 LSB) .. Byte 7 (MSB) : 37 Bit Nutzdaten S0..SK1, davon:
//           - 30 Bit Zeitstempel: Sekunden seit 01.01.2000 UTC, aber nur
//             JEDE DRITTE Sekunde gezaehlt (Wert*3 ergibt echte Sekunden;
//             Schaltsekunden werden dabei NICHT mitgezaehlt lt. PA3FWM)
//           - 2 Bit TZ0/TZ1: Zeitzonen-Offset (0/1/2/3 -> +0/+2/+1/+3h;
//             die Reihenfolge 0x01->+2h ist ein weiteres von PA3FWM
//             dokumentiertes Kuriosum der Spezifikation)
//           - 1 Bit TZC: Ankuendigung Zeitzonenwechsel
//           - 1 Bit LS: Schaltsekunde angekuendigt
//           - 1 Bit LSS: Vorzeichen der Schaltsekunde (1=positiv/einfuegen)
//           - 2 Bit SK0/SK1: Sender-Wartungsstatus
//       Byte 8-10 : Reed-Solomon(15,9) Fehlerkorrektur ueber GF(16)
//                   (4-Bit-Symbole, bis zu 3 Symbole korrigierbar),
//                   deckt S0..SK1 ab (nicht deckend: das SK1-Bit selbst,
//                   da RS(15,9) nur 15*4=60 von 61 benoetigten Bits hat -
//                   ein von PA3FWM dokumentierter Spezifikationsmangel)
//       Byte 11   : CRC8 (Polynom 0x07, Startwert 0x00) ueber die
//                   VERSCHLUESSELTEN ("gescrambelten") Byte 3-7 - nicht
//                   ueber die entschluesselten Werte (SP6HFE-Erkenntnis)
//   - Scrambling: Byte 3 (5 LSB) .. Byte 7 werden mit dem 37-Bit-Wort
//     0x0A47554D2B (MSB-first) EXOR-verknuepft.
//   - Das S0..SK1-Datenfeld selbst ist NICHT durchgehend byte-aligned
//     (beginnt bei Bit 5 von Byte 3) - die RS-Codewort-Extraktion muss das
//     beruecksichtigen, siehe extractRsCodeword()/updateFrameFromCodeword().
//
// Bekannte Einschraenkung: der genaue Zeitpunkt, auf den sich ein Frame
// bezieht, ist laut PA3FWM selbst beim Sender ungenau (100-200ms Jitter) -
// fuer eine Wanduhr unproblematisch, fuer einen NTP-Server nicht geeignet.

#pragma once

#include <reed_solomon_gf16.hpp>

#include <array>
#include <complex>
#include <cstdint>
#include <deque>
#include <vector>

#include "time_signal_decoder.hpp"

namespace pl225 {

class Pl225Decoder {
public:
    Pl225Decoder(timesignal::IDecodedTimeSink& sink, unsigned sampleRate);

    // Verarbeitet einen Block komplexer IQ-Samples.
    void processBlock(const std::vector<std::complex<float>>& samples, uint64_t blockEndMs);

    std::string name() const { return "PL225"; }

private:
    static constexpr double kBitRateHz = 50.0;          // siehe Header-Kommentar
    static constexpr double kLoopBandwidthHz = 0.05;    // PLL-Schleifenbandbreite (sehr schmal)
    static constexpr double kLoopDamping = 0.707;        // kritische Daempfung
    static constexpr uint8_t kSyncWordHi = 0x55;
    static constexpr uint8_t kSyncWordLo = 0x55;
    static constexpr uint8_t kFrameHeaderByte = 0x60;
    static constexpr uint8_t kTimeMessagePrefix = 0b101;
    static constexpr uint8_t kCrc8Polynomial = 0x07;
    static constexpr uint8_t kCrc8Init = 0x00;
    static constexpr int kFrameBits = 96;
    static constexpr int kFrameBytes = 12;

    using RS = pl225rs::ReedSolomon15_9; // RS(15,9), 4-Bit-Symbole, 3 korrigierbare Symbole

    timesignal::IDecodedTimeSink& sink_;
    unsigned sampleRate_;
    double samplesPerBit_;

    // --- Phasennachfuehrung: echte Typ-2-PLL / Costas-Loop-Prinzip ---
    // (Phasenfehler + Frequenz-Integrator, damit auch eine KONSTANTE
    // Frequenzabweichung zwischen echtem Traeger und BFO/NCO - beim realen
    // SSB-Empfang typischerweise einige Zehntel Hz - im eingeschwungenen
    // Zustand FEHLERFREI ausgeregelt wird. Eine reine Tiefpass-Basislinie
    // (1. Ordnung) kann eine Rampe grundsaetzlich nur mit bleibendem
    // Schleppfehler verfolgen - das reicht fuer die 45-Grad-Modulation
    // nicht aus, siehe README/Fehleranalyse.)
    bool havePrevRawPhase_ = false;
    double prevRawPhase_ = 0.0;
    double unwrappedPhase_ = 0.0;
    bool baselineInitialized_ = false;
    double baseline_ = 0.0;      // NCO-Phasenschaetzung (Loop-Ausgang)
    double freqEstimate_ = 0.0;  // NCO-Frequenzschaetzung (Integratorzustand), rad/Sample
    double loopKp_;              // proportionaler Loop-Filter-Koeffizient
    double loopKi_;              // integraler Loop-Filter-Koeffizient (Frequenznachfuehrung)

    // --- Bit-Puffer (detrendete Phase in Grad, ring-artig verwaltet) ---
    std::deque<double> phaseHistoryDeg_;
    uint64_t totalSamplesProcessed_ = 0;
    uint64_t phaseHistoryStartSampleIndex_ = 0; // Sample-Index des ersten Elements in phaseHistoryDeg_
    uint64_t nextSearchFromSampleIndex_ = 0;    // ab hier nach neuen Frames suchen

    RS rs_{};

    void appendSample(std::complex<float> sample);
    void trimHistory();
    void searchAndDecodeFrames();

    // Extrahiert ein klassifiziertes Bit (0/1), zentriert um bitCenterSampleIndex
    // (absoluter Sample-Index seit Streambeginn), gegen einen explizit
    // uebergebenen Schwellenwert (siehe computeAdaptiveThreshold() - der
    // tatsaechlich erreichte Phasenhub haengt vom Empfangspfad/Filter ab
    // und ist nicht immer exakt 45 Grad, ein fester Schwellenwert waere
    // nicht robust genug). Gibt -1 zurueck wenn Index ausserhalb des
    // aktuell gepufferten Bereichs liegt.
    int classifyBitAt(uint64_t bitCenterSampleIndex, double thresholdDeg) const;

    // Bestimmt einen adaptiven Schwellenwert (Mitte zwischen Minimum und
    // Maximum) ueber den angegebenen Sample-Bereich der Phasenhistorie -
    // analog zum adaptiven Schwellenwert-Prinzip der amplitudenbasierten
    // Decoder (DCF77/MSF/JJY), nur eben auf Phasenwerte angewendet.
    double computeAdaptiveThreshold(uint64_t fromSampleIndex, uint64_t toSampleIndex) const;

    // Versucht ab syncStartSampleIndex (Beginn des 16-Bit Sync-Worts) einen
    // vollstaendigen 96-Bit-Rahmen zu lesen und zu dekodieren. Bei Erfolg
    // wird sink_ benachrichtigt. Gibt true zurueck wenn genug Daten fuer den
    // VERSUCH vorhanden waren (unabhaengig vom Dekodiererfolg) - false wenn
    // die Daten noch nicht (komplett) vorliegen (spaeter erneut versuchen).
    bool tryDecodeFrameAt(uint64_t syncStartSampleIndex, double thresholdDeg);

    static RS::Codeword extractRsCodeword(const std::array<uint8_t, 12>& frame);
    static void updateFrameFromCodeword(std::array<uint8_t, 12>& frame, const RS::Codeword& codeword);

    static bool validateCrc(const std::array<uint8_t, 12>& frame);
    static bool correctSk1WithCrc(std::array<uint8_t, 12>& frame);

    static void descramble(std::array<uint8_t, 12>& frame);

    // Baut aus dem fertig korrigierten/entschluesselten Rahmen ein
    // timesignal::DecodedTime (inkl. Umrechnung Unix-Zeitstempel -> Datum).
    static timesignal::DecodedTime extractTimeData(const std::array<uint8_t, 12>& frame);
};

} // namespace pl225
