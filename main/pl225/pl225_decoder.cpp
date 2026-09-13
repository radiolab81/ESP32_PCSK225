// pl225_decoder.cpp
#include "pl225_decoder.hpp"
#include "crc8.hpp"

#include <cmath>
#include <cstring>
#include <algorithm>

namespace pl225 {

namespace {

// Klassische Tageszahl-Kalenderumrechnung (Howard Hinnant, "days_from_civil"
// bzw. hier die Umkehrung "civil_from_days" - gemeinfrei/public domain,
// weit verbreiteter Standardalgorithmus, siehe
// http://howardhinnant.github.io/date_algorithms.html).
// Wandelt Tage seit 1970-01-01 in (Jahr, Monat, Tag) um, proleptischer
// gregorianischer Kalender.
void civilFromDays(long z, int& year, unsigned& month, unsigned& day) {
    z += 719468;
    const long era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);            // [0, 146096]
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365; // [0, 399]
    const long y = static_cast<long>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    const unsigned mp = (5 * doy + 2) / 153;                      // [0, 11]
    day = doy - (153 * mp + 2) / 5 + 1;                           // [1, 31]
    month = mp + (mp < 10 ? 3 : -9);                              // [1, 12]
    year = static_cast<int>(y + (month <= 2 ? 1 : 0));
}

} // namespace

Pl225Decoder::Pl225Decoder(timesignal::IDecodedTimeSink& sink, unsigned sampleRate)
    : sink_(sink), sampleRate_(sampleRate) {
    samplesPerBit_ = static_cast<double>(sampleRate_) / kBitRateHz;

    // Standard-Entwurfsformeln fuer einen digitalen Typ-2 PI-Loop-Filter
    // (siehe z.B. F. Harris, "Multirate Signal Processing", oder jede
    // Costas-/PLL-Referenz): theta = normierte Schleifenbandbreite,
    // daraus Kp (proportional) und Ki (integral, = Frequenznachfuehrung).
    const double theta = kLoopBandwidthHz / static_cast<double>(sampleRate_);
    const double d = 1.0 + 2.0 * kLoopDamping * theta + theta * theta;
    loopKp_ = (4.0 * kLoopDamping * theta) / d;
    loopKi_ = (4.0 * theta * theta) / d;
}

void Pl225Decoder::appendSample(std::complex<float> sample) {
    // 1. Rohphase + fortlaufendes Unwrapping (Standardtrick: Differenz zur
    //    letzten Rohphase auf [-pi,pi] normieren und aufaddieren, statt der
    //    Rohphase selbst - vermeidet Sprünge bei +-180 Grad).
    double rawPhase = std::atan2(static_cast<double>(sample.imag()), static_cast<double>(sample.real()));

    if (!havePrevRawPhase_) {
        unwrappedPhase_ = rawPhase;
        havePrevRawPhase_ = true;
    } else {
        double delta = rawPhase - prevRawPhase_;
        while (delta > M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;
        unwrappedPhase_ += delta;
    }
    prevRawPhase_ = rawPhase;

    // 2. Typ-2 PLL / Costas-Loop: NCO-Phasenschaetzung (baseline_) UND
    //    NCO-Frequenzschaetzung (freqEstimate_) werden aus dem Phasenfehler
    //    nachgefuehrt. Der Integralanteil (freqEstimate_) ist entscheidend:
    //    er sorgt dafuer, dass eine KONSTANTE Frequenzabweichung zwischen
    //    echtem Traeger und BFO/NCO im eingeschwungenen Zustand komplett
    //    ausgeregelt wird (bleibender Fehler = 0), waehrend die sehr
    //    schmale Schleifenbandbreite (kLoopBandwidthHz) verhindert, dass
    //    die schnellen 20ms-Bitwechsel selbst mit ausgeregelt (und damit
    //    "weggefiltert") werden.
    if (!baselineInitialized_) {
        baseline_ = unwrappedPhase_;
        baselineInitialized_ = true;
    } else {
        double phaseError = unwrappedPhase_ - baseline_;
        freqEstimate_ += loopKi_ * phaseError;
        baseline_ += freqEstimate_ + loopKp_ * phaseError;
    }

    double detrendedRad = unwrappedPhase_ - baseline_;
    double detrendedDeg = detrendedRad * 180.0 / M_PI;

    phaseHistoryDeg_.push_back(detrendedDeg);
    totalSamplesProcessed_++;
}

void Pl225Decoder::trimHistory() {
    // Halte den Puffer auf ca. 3 Sekunden begrenzt (96 Bit * 20ms = 1.92s
    // fuer einen kompletten Rahmen + Suchvorlauf), alles vor
    // nextSearchFromSampleIndex_ wird nicht mehr gebraucht.
    const uint64_t keepFromIndex =
        (nextSearchFromSampleIndex_ > sampleRate_) ? (nextSearchFromSampleIndex_ - sampleRate_) : 0;

    while (phaseHistoryStartSampleIndex_ < keepFromIndex && !phaseHistoryDeg_.empty()) {
        phaseHistoryDeg_.pop_front();
        phaseHistoryStartSampleIndex_++;
    }
}

int Pl225Decoder::classifyBitAt(uint64_t bitCenterSampleIndex, double thresholdDeg) const {
    if (bitCenterSampleIndex < phaseHistoryStartSampleIndex_) return -1;
    uint64_t offset = bitCenterSampleIndex - phaseHistoryStartSampleIndex_;
    if (offset >= phaseHistoryDeg_.size()) return -1;

    double v = phaseHistoryDeg_[offset];
    return (v > thresholdDeg) ? 1 : 0;
}

double Pl225Decoder::computeAdaptiveThreshold(uint64_t fromSampleIndex, uint64_t toSampleIndex) const {
    // Minimum/Maximum ueber den angegebenen Bereich - wie bei den
    // amplitudenbasierten Decodern (DCF77/MSF/JJY), nur auf Phasenwerte
    // statt Pegelwerte angewendet. Der tatsaechliche Phasenhub nach dem
    // Tiefpassfilter des Mono-Downconverters ist typischerweise kleiner
    // als die theoretischen 45 Grad (Filterdaempfung schneller Uebergaenge),
    // ein fester Schwellenwert waere daher nicht robust.
    if (fromSampleIndex < phaseHistoryStartSampleIndex_) fromSampleIndex = phaseHistoryStartSampleIndex_;
    uint64_t fromOffset = fromSampleIndex - phaseHistoryStartSampleIndex_;
    uint64_t toOffset = std::min<uint64_t>(toSampleIndex - phaseHistoryStartSampleIndex_,
                                            phaseHistoryDeg_.size());

    double lo = 1e18, hi = -1e18;
    for (uint64_t i = fromOffset; i < toOffset; ++i) {
        double v = phaseHistoryDeg_[i];
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    if (hi <= lo) return 0.0;
    return (lo + hi) / 2.0;
}

void Pl225Decoder::processBlock(const std::vector<std::complex<float>>& samples, uint64_t /*blockEndMs*/) {
    for (const auto& s : samples) {
        appendSample(s);
    }
    searchAndDecodeFrames();
    trimHistory();
}

void Pl225Decoder::searchAndDecodeFrames() {
    // Wie viele Samples werden fuer einen kompletten 96-Bit-Rahmen ab einem
    // Sync-Kandidaten benoetigt?
    const uint64_t samplesForFrame = static_cast<uint64_t>(kFrameBits * samplesPerBit_) + 1;

    // Solange genug Daten im Puffer sind, um ab der naechsten Suchposition
    // einen vollstaendigen Rahmen zu pruefen, weiter nach dem 16-Bit
    // Sync-Wort (0101010101010101 bzw. dessen Komplement) suchen.
    while (true) {
        uint64_t latestAvailable = phaseHistoryStartSampleIndex_ + phaseHistoryDeg_.size();
        if (nextSearchFromSampleIndex_ + samplesForFrame > latestAvailable) {
            break; // noch nicht genug Daten fuer einen weiteren Versuch
        }

        // Adaptiven Schwellenwert ueber den kompletten Kandidaten-Rahmen
        // (Sync-Wort + Nutzdaten) EINMALIG bestimmen und fuer die gesamte
        // Extraktion dieses Versuchs konsistent verwenden.
        double thresholdDeg = computeAdaptiveThreshold(nextSearchFromSampleIndex_,
                                                         nextSearchFromSampleIndex_ + samplesForFrame);

        // Sync-Bits ab der aktuellen Suchposition klassifizieren (16 Bit).
        int bits[16];
        bool allValid = true;
        for (int i = 0; i < 16; ++i) {
            uint64_t center = nextSearchFromSampleIndex_ +
                               static_cast<uint64_t>(i * samplesPerBit_) +
                               static_cast<uint64_t>(samplesPerBit_ / 2.0);
            int b = classifyBitAt(center, thresholdDeg);
            if (b < 0) { allValid = false; break; }
            bits[i] = b;
        }

        if (allValid) {
            bool alternatingOk = true;
            for (int i = 1; i < 16 && alternatingOk; ++i) {
                if (bits[i] == bits[i - 1]) alternatingOk = false;
            }
            // Muss mit Bit=0 beginnen (0x5555 = 0101...; siehe PA3FWM:
            // "Each time frame starts with an indication of the bit value 0").
            if (alternatingOk && bits[0] == 0) {
                if (tryDecodeFrameAt(nextSearchFromSampleIndex_, thresholdDeg)) {
                    // Erfolgreich (versucht) - hinter den kompletten Rahmen springen,
                    // um den gleichen Sync nicht nochmal zu treffen.
                    nextSearchFromSampleIndex_ += samplesForFrame;
                    continue;
                }
            }
        }

        // Kein Treffer an dieser Position - eine Bitperiode weiterruecken
        // (kein Sample-fuer-Sample-Scan noetig, das Sync-Wort ist 16 Bit
        // lang und Fehlalarme durch Ein-Sample-Versatz sind durch die
        // Bit-Mitten-Abtastung sehr unwahrscheinlich; im Zweifel wird der
        // naechste Rahmen ohnehin gefunden).
        nextSearchFromSampleIndex_ += static_cast<uint64_t>(samplesPerBit_ / 4.0);
    }
}

bool Pl225Decoder::tryDecodeFrameAt(uint64_t syncStartSampleIndex, double thresholdDeg) {
    std::array<uint8_t, 12> frame{};

    for (int byteIdx = 0; byteIdx < kFrameBytes; ++byteIdx) {
        uint8_t byteVal = 0;
        for (int bitIdx = 0; bitIdx < 8; ++bitIdx) {
            int globalBit = byteIdx * 8 + bitIdx;
            uint64_t center = syncStartSampleIndex +
                               static_cast<uint64_t>(globalBit * samplesPerBit_) +
                               static_cast<uint64_t>(samplesPerBit_ / 2.0);
            int b = classifyBitAt(center, thresholdDeg);
            if (b < 0) return false; // Daten (noch) nicht vollstaendig verfuegbar
            byteVal = static_cast<uint8_t>((byteVal << 1) | b);
        }
        frame[byteIdx] = byteVal;
    }

    // --- Statische Felder pruefen ---
    if (frame[0] != kSyncWordHi || frame[1] != kSyncWordLo) {
        sink_.onDecodeError("PL225: Sync-Wort nicht bestaetigt - verworfen");
        return true; // Versuch war moeglich, nur eben kein gueltiger Rahmen
    }
    if (frame[2] != kFrameHeaderByte) {
        sink_.onDecodeError("PL225: Header-Byte != 0x60 - verworfen (evtl. anderer Nachrichtentyp, "
                             "z.B. 'ENEA'-Lichtsteuerung auf demselben Kanal)");
        return true;
    }
    if ((frame[3] >> 5) != kTimeMessagePrefix) {
        sink_.onDecodeError("PL225: Zeitnachricht-Praefix (0b101) nicht bestaetigt - verworfen");
        return true;
    }

    // --- Reed-Solomon FEC ---
    RS::Codeword codeword = extractRsCodeword(frame);
    bool rsFailed = rs_.recoverCodeword(codeword);
    if (rsFailed) {
        sink_.onDecodeError("PL225: Reed-Solomon Fehlerkorrektur fehlgeschlagen - Rahmen verworfen");
        return true;
    }
    updateFrameFromCodeword(frame, codeword);

    // --- CRC8 + SK1-Korrektur (CRC wirkt auf die VERSCHLUESSELTEN Bytes 3-7) ---
    if (!correctSk1WithCrc(frame)) {
        sink_.onDecodeError("PL225: CRC8-Pruefung fehlgeschlagen - Rahmen verworfen");
        return true;
    }

    // --- Entschluesseln + Felder extrahieren ---
    descramble(frame);
    timesignal::DecodedTime result = extractTimeData(frame);
    sink_.onDecodedTime(result);

    return true;
}

Pl225Decoder::RS::Codeword Pl225Decoder::extractRsCodeword(const std::array<uint8_t, 12>& frame) {
    // Das 37-Bit Datenfeld S0..SK1 (nicht byte-aligned, beginnt bei Byte3
    // Bit 4) wird in 9 4-Bit-Symbole aufgeteilt, gefolgt von 6 weiteren
    // 4-Bit-Symbolen aus den (byte-alignierten) ECC-Bytes 8-10 - exakt wie
    // von SP6HFE reverse-engineered (siehe Header-Kommentar/README).
    RS::Codeword codeword{};
    uint8_t codewordIndex = 0;

    for (uint8_t frameByteNo = 3; frameByteNo < 8; ++frameByteNo) {
        if (frameByteNo != 3) {
            codeword[codewordIndex++] += static_cast<uint16_t>((frame[frameByteNo] >> 5) & 0x07);
        }
        codeword[codewordIndex++] = static_cast<uint16_t>((frame[frameByteNo] >> 1) & 0x0F);
        if (frameByteNo != 7) {
            codeword[codewordIndex] = static_cast<uint16_t>((frame[frameByteNo] & 0x01) << 3);
        }
    }
    for (uint8_t frameByteNo = 8; frameByteNo < 11; ++frameByteNo) {
        codeword[codewordIndex++] = static_cast<uint16_t>((frame[frameByteNo] >> 4) & 0x0F);
        codeword[codewordIndex++] = static_cast<uint16_t>(frame[frameByteNo] & 0x0F);
    }
    return codeword;
}

void Pl225Decoder::updateFrameFromCodeword(std::array<uint8_t, 12>& frame, const RS::Codeword& codeword) {
    uint8_t codewordIndex = 0;
    for (uint8_t frameByteNo = 3; frameByteNo < 8; ++frameByteNo) {
        uint8_t updated = 0;
        if (frameByteNo == 3) {
            updated = frame[frameByteNo] & 0xE0; // 3 MSb (Praefix 101) unveraendert lassen
        } else {
            updated = static_cast<uint8_t>((codeword[codewordIndex++] & 0x07) << 5);
        }
        updated |= static_cast<uint8_t>(codeword[codewordIndex++] << 1);
        if (frameByteNo == 7) {
            updated |= (frame[frameByteNo] & 0x01); // SK1 bleibt unveraendert (nicht RS-geschuetzt)
        } else {
            updated |= static_cast<uint8_t>((codeword[codewordIndex] & 0x08) >> 3);
        }
        frame[frameByteNo] = updated;
    }
    for (uint8_t frameByteNo = 8; frameByteNo < 11; ++frameByteNo) {
        frame[frameByteNo] = static_cast<uint8_t>(codeword[codewordIndex++] << 4);
        frame[frameByteNo] |= static_cast<uint8_t>(codeword[codewordIndex++]);
    }
}

bool Pl225Decoder::validateCrc(const std::array<uint8_t, 12>& frame) {
    Crc8 crc(kCrc8Polynomial, kCrc8Init);
    for (int i = 3; i < 8; ++i) {
        crc.update(frame[i]);
    }
    return crc.get() == frame[11];
}

bool Pl225Decoder::correctSk1WithCrc(std::array<uint8_t, 12>& frame) {
    // SK1 (Bit 0 von Byte 7) ist das einzige Datenbit, das NICHT vom
    // Reed-Solomon-Code abgedeckt wird (siehe Header-Kommentar). Nach
    // erfolgreicher RS-Korrektur bleibt es daher u.U. fehlerhaft; die CRC
    // erlaubt es, seinen korrekten Wert durch Ausprobieren (0/1) zu finden.
    if (validateCrc(frame)) return true;

    frame[7] ^= 0x01;
    if (validateCrc(frame)) return true;

    frame[7] ^= 0x01; // zuruecksetzen
    return false;
}

void Pl225Decoder::descramble(std::array<uint8_t, 12>& frame) {
    static constexpr std::array<uint8_t, 5> scramblingWord{0x0A, 0x47, 0x55, 0x4D, 0x2B};
    for (size_t i = 0; i < scramblingWord.size(); ++i) {
        frame[3 + i] ^= scramblingWord[i];
    }
}

timesignal::DecodedTime Pl225Decoder::extractTimeData(const std::array<uint8_t, 12>& frame) {
    timesignal::DecodedTime result;
    result.sourceTag = "PL225";

    uint32_t units3s = frame[3] & 0x1F;
    units3s = (units3s << 8) + frame[4];
    units3s = (units3s << 8) + frame[5];
    units3s = (units3s << 8) + frame[6];
    units3s = (units3s << 1) + ((frame[7] & 0x80) ? 1U : 0U);

    // Sekunden seit 01.01.2000 UTC (siehe PA3FWM: schliesst Schaltsekunden aus)
    static constexpr uint32_t secondsYear2000To1970 = 946684800U;
    uint32_t utcSeconds2000 = units3s * 3U;
    uint32_t unixTimestamp = utcSeconds2000 + secondsYear2000To1970;

    long daysSinceEpoch = static_cast<long>(unixTimestamp / 86400U);
    uint32_t secondsOfDay = unixTimestamp % 86400U;

    int year;
    unsigned month, day;
    civilFromDays(daysSinceEpoch, year, month, day);

    result.timeValid = true;
    result.dateValid = true;
    result.year2 = year % 100;
    result.month = static_cast<int>(month);
    result.day = static_cast<int>(day);
    result.hour = static_cast<int>(secondsOfDay / 3600U);
    result.minute = static_cast<int>((secondsOfDay / 60U) % 60U);
    result.second = static_cast<int>(secondsOfDay % 60U);

    // ISO-Wochentag (1=Montag..7=Sonntag, konsistent mit der DCF77-Ausgabe):
    // 01.01.1970 (Tag 0) war ein Donnerstag (ISO 4).
    long isoWeekday = ((daysSinceEpoch % 7) + 7 + 3) % 7 + 1;
    result.weekday = static_cast<int>(isoWeekday);

    const uint8_t tzBits = (frame[7] >> 5) & 0x03;
    switch (tzBits) {
        case 0x01: result.tzOffsetHours = 2; break;
        case 0x02: result.tzOffsetHours = 1; break;
        case 0x03: result.tzOffsetHours = 3; break;
        default:   result.tzOffsetHours = 0; break;
    }
    result.dst = (result.tzOffsetHours == 2); // in Polen entspricht +2h MESZ/CEST

    result.dstChangeAnnounced = ((frame[7] >> 2) & 0x01) != 0;   // TZC
    result.leapSecondAnnounced = ((frame[7] >> 4) & 0x01) != 0;  // LS
    bool leapSecondPositive = ((frame[7] >> 3) & 0x01) != 0;     // LSS
    result.leapSecondIsRemoval = result.leapSecondAnnounced && !leapSecondPositive;

    const uint8_t transmitterState = frame[7] & 0x03;
    switch (transmitterState) {
        case 0x01: result.transmitterStateText = "Wartung angekuendigt (1 Woche)"; break;
        case 0x02: result.transmitterStateText = "Wartung angekuendigt (1 Tag)"; break;
        case 0x03: result.transmitterStateText = "Wartung angekuendigt (>1 Woche)"; break;
        default:   result.transmitterStateText = "Normalbetrieb"; break;
    }

    return result;
}

} // namespace pl225
