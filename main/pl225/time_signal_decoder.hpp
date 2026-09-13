// time_signal_decoder.hpp
//
// Generisches Interface für Langwellen-Zeitzeichensender-Decoder.
// Jeder konkrete Decoder (DCF77, MSF, JJY, WWVB, ...) bekommt vom
// ToneEnvelopeDetector einzelne "Sekundenimpulse" (Low-Phasen-Dauer)
// gemeldet und wertet daraus sein jeweiliges Protokoll aus.
//
// Dadurch ist der komplette Audio-/Spektrum-/Envelope-Pfad unabhängig
// vom konkreten Sender: man tauscht nur den Decoder aus bzw. betreibt
// mehrere Decoder parallel an unterschiedlichen Zielfrequenzen/Ports.

#pragma once

#include <cstdint>
#include <string>

namespace timesignal {

// Ein vom ToneEnvelopeDetector erkanntes Ereignis: EINE einzelne
// Trägerabsenkung ("Dip") - fallende Flanke bis nachfolgende steigende
// Flanke. Manche Protokolle senden pro Sekunde genau einen Dip (DCF77),
// andere teilweise zwei kurze Dips innerhalb einer Sekunde (MSF-Doppelpuls
// fuer "Bit A=0, Bit B=1": 100ms ab/100ms an/100ms ab). Der Decoder erhält
// daher rohe Dip-Ereignisse und gruppiert sie selbst zu Sekunden/Bits -
// das hält den Envelope-Detector protokollunabhängig.
struct CarrierDipEvent {
    uint64_t dipStartMs;          // Systemzeit der fallenden Flanke (Absenkung beginnt)
    uint64_t dipEndMs;            // Systemzeit der steigenden Flanke (Trägerrückkehr)
    double   durationMs;          // dipEndMs - dipStartMs
    double   sinceLastDipStartMs; // dipStartMs - Start des vorherigen Dips (0 beim allerersten Dip)
};

// Ergebnis einer vollständig dekodierten Zeittelegramm-Minute.
// Nicht alle Felder werden von jedem Sender genutzt (year_2digits etc.),
// daher "valid"-Flags pro logischer Gruppe.
struct DecodedTime {
    bool     timeValid   = false;
    bool     dateValid   = false;

    int      second = -1;
    int      minute = -1;
    int      hour   = -1;
    int      day    = -1;   // Tag des Monats
    int      weekday = -1;  // 1=Montag .. 7=Sonntag (ISO)
    int      month  = -1;
    int      year2  = -1;   // zweistellige Jahreszahl (z.B. 26 für 2026)

    bool     dst    = false; // Sommerzeit aktiv (MESZ/BST/...)
    bool     leapSecondAnnounced = false;
    bool     antennaChangeAnnounced = false; // DCF77 "Rufbit"
    bool     dstChangeAnnounced = false;     // MSF Bit 53B: Sommerzeitwechsel in <=61 Minuten
    int      dut1DeciSeconds = 0;            // MSF: UT1-UTC in 0.1s-Schritten (0 falls nicht zutreffend/DCF77)
    int      dayOfYear = -1;                 // JJY: Tag des Jahres (1-366), -1 falls nicht ermittelt/protokolliert
    bool     leapSecondIsRemoval = false;     // JJY/PL225: bei leapSecondAnnounced=true - true=Sekunde wird ENTFERNT statt eingefuegt
    int      tzOffsetHours = 0;               // PL225: Zeitzonen-Offset des Senderstandorts zu UTC in Stunden (0-3)
    std::string transmitterStateText;         // PL225: Klartext-Status des Senders (Normalbetrieb/Wartung angekuendigt)

    std::string sourceTag; // z.B. "DCF77", "MSF", "JJY", "PL225"
};

// Callback-Interface, über das der Decoder fertige Minuten/Fehler meldet.
class IDecodedTimeSink {
public:
    virtual ~IDecodedTimeSink() = default;
    virtual void onDecodedTime(const DecodedTime& t) = 0;
    virtual void onDecodeError(const std::string& reason) = 0;
    // Für Diagnose/Debug: rohes Bit wurde erkannt (0/1) inkl. Bitindex.
    virtual void onBitDecoded(int /*bitIndex*/, int /*bitValue*/) {}
};

// Basisklasse für alle Zeitzeichen-Protokoll-Decoder.
class ITimeSignalDecoder {
public:
    explicit ITimeSignalDecoder(IDecodedTimeSink& sink) : sink_(sink) {}
    virtual ~ITimeSignalDecoder() = default;

    // Wird für jede erkannte Trägerabsenkung (ein "Dip", siehe oben) vom
    // ToneEnvelopeDetector aufgerufen. Der Decoder entscheidet selbst,
    // wie viele Dips zu einer Sekunde/einem Bit gehören.
    virtual void onCarrierDip(const CarrierDipEvent& ev) = 0;

    // Menschlich lesbarer Name des Protokolls, z.B. "DCF77".
    virtual std::string name() const = 0;

protected:
    IDecodedTimeSink& sink_;
};

} // namespace timesignal
