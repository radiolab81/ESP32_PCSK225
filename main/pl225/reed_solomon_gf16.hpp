// reed_solomon_gf16.hpp
//
// Eigenstaendige Reed-Solomon(15,9)-Implementierung ueber GF(16), geschrieben
// als GPLv3-freier Ersatz fuer external/ReedSolomon (siehe README-Abschnitt
// "Lizenzwechsel"). Implementiert den klassischen, seit den 1960er/70er
// Jahren oeffentlich dokumentierten Berlekamp-Massey-Dekodieralgorithmus
// (Chien-Suche fuer Fehlerpositionen, Forney-Formel fuer Fehlerwerte) - der
// Algorithmus selbst ist frei verfuegbare Mathematik/Lehrbuchwissen (siehe
// z.B. Lin & Costello, "Error Control Coding"), keine Ableitung von
// SP6HFEs oder Simon Rockliffs Code.
//
// -------------------------------------------------------------------------
// Kompatibilitaet zum echten PL225-Signal:
// -------------------------------------------------------------------------
// Damit reale, ueber Funk empfangene Rahmen weiterhin korrigierbar bleiben,
// MUESSEN die Feldparameter (primitives Polynom, Generatorwurzeln,
// Symbolreihenfolge) exakt mit denen uebereinstimmen, die der polnische
// Sender tatsaechlich verwendet. Diese Parameter sind Teil der (informellen)
// Protokollspezifikation, nicht urheberrechtlich schuetzbarer Code - sie
// wurden anhand der oeffentlich dokumentierten Parameter der Referenz-
// dekoder ermittelt und hier unabhaengig neu implementiert:
//   - GF(16), primitives Polynom p(x) = x^4 + x + 1  (Standardwahl, auch
//     z.B. bei QR-Codes und DVB verwendet - keine PL225-Spezifik)
//   - Generatorpolynom g(x) = Produkt (x + alpha^i), i=1..6 (6 Nullstellen,
//     entspricht 3 korrigierbaren Symbolen: fecSize = 2*3 = 6)
//   - Systematische Kodierung: c(x) = data(x)*x^6 + rest(x), wobei die 9
//     Datensymbole die Codewortpositionen 0..8 (aufsteigend) und die 6
//     Pruefsymbole die Positionen 9..14 belegen - siehe extractRsCodeword()/
//     updateFrameFromCodeword() in pl225_decoder.cpp, die diese Reihenfolge
//     direkt aus den empfangenen Bits befuellen.
// Erfolgreich gegen echte Aufnahmen verifiziert (siehe README).

#pragma once

#include <array>
#include <cstdint>
#include <cstddef>

namespace pl225rs {

class ReedSolomon15_9 {
public:
    static constexpr int kSymbolBits = 4;
    static constexpr int kFieldSize = 15;   // 2^4 - 1
    static constexpr int kFecSize = 6;      // 2 * korrigierbare Symbole (3)
    static constexpr int kDataSize = kFieldSize - kFecSize; // 9
    static constexpr int kCorrectableSymbols = kFecSize / 2;

    using Codeword = std::array<uint16_t, kFieldSize>;
    using Message = std::array<uint16_t, kDataSize>;

    ReedSolomon15_9() {
        buildGaloisField();
        buildGeneratorPolynomial();
    }

    // Erzeugt ein vollstaendiges, gueltiges Codewort aus einer Nachricht
    // (Datenpositionen 0..8, Pruefsymbole 9..14 werden berechnet).
    Codeword generateCodeword(const Message& message) const {
        Codeword codeword{};
        for (int i = 0; i < kDataSize; ++i) codeword[i] = message[i];

        // Dividend = data(x)*x^6 : Grad (6+i) traegt den Koeffizienten data[i].
        std::array<uint8_t, kFieldSize> dividend{};
        for (int i = 0; i < kDataSize; ++i) dividend[kFecSize + i] = static_cast<uint8_t>(message[i]);

        auto remainder = polyModulo(dividend);
        for (int k = 0; k < kFecSize; ++k) codeword[kDataSize + k] = remainder[k];
        return codeword;
    }

    // Versucht, bis zu kCorrectableSymbols fehlerhafte Symbole im Codewort
    // zu korrigieren. Gibt true zurueck, wenn die Fehlerkorrektur
    // fehlgeschlagen ist (Codewort NICHT veraendert), false bei Erfolg
    // (Codewort ggf. korrigiert - inkl. dem Fall "keine Fehler vorhanden").
    bool recoverCodeword(Codeword& codeword) const {
        // Empfangenes Polynom R(x): Position 9..14 -> Grad 0..5 (Pruefsymbole),
        // Position 0..8 -> Grad 6..14 (Datensymbole) - siehe Klassenkommentar.
        std::array<uint8_t, kFieldSize> recd{};
        for (int k = 0; k < kFecSize; ++k) recd[k] = static_cast<uint8_t>(codeword[kDataSize + k]);
        for (int i = 0; i < kDataSize; ++i) recd[kFecSize + i] = static_cast<uint8_t>(codeword[i]);

        // --- Syndrome S_1..S_(2t) = R(alpha^j) ---
        std::array<uint8_t, kFecSize> syndrome{};
        bool allZero = true;
        for (int j = 1; j <= kFecSize; ++j) {
            uint8_t s = evalPoly(recd, expOf(j));
            syndrome[j - 1] = s;
            if (s != 0) allZero = false;
        }
        if (allZero) return false; // keine Fehler - nichts zu tun, Erfolg

        // --- Berlekamp-Massey: Fehlerlokator-Polynom sigma(x) bestimmen ---
        // Kanonische Formulierung (Massey 1969): sigma=C(x), Hilfspolynom B(x),
        // L=aktueller Grad/Laenge des Schieberegisters, b=letzte von Null
        // verschiedene Diskrepanz, m=Schritte seit deren Auftreten.
        std::array<uint8_t, kFecSize + 1> sigma{};
        std::array<uint8_t, kFecSize + 1> B{};
        sigma[0] = 1;
        B[0] = 1;
        int L = 0;
        uint8_t b = 1;
        int m = 1;

        for (int n = 0; n < kFecSize; ++n) {
            uint8_t delta = syndrome[n];
            for (int i = 1; i <= L; ++i) {
                delta ^= gfMul(sigma[i], syndrome[n - i]);
            }

            if (delta == 0) {
                m++;
            } else {
                std::array<uint8_t, kFecSize + 1> T = sigma;
                uint8_t factor = gfMul(delta, gfInverse(b));
                for (int i = 0; i <= kFecSize - m; ++i) {
                    sigma[i + m] ^= gfMul(factor, B[i]);
                }
                if (2 * L <= n) {
                    L = n + 1 - L;
                    B = T;
                    b = delta;
                    m = 1;
                } else {
                    m++;
                }
            }
        }
        int sigmaDegree = L;

        if (sigmaDegree > kCorrectableSymbols) return true; // zu viele Fehler

        // --- Chien-Suche: Nullstellen von sigma(x) unter x=alpha^{-i}, i=0..14 ---
        std::array<int, kCorrectableSymbols> errorPositions{};
        int errorsFound = 0;
        for (int i = 0; i < kFieldSize && errorsFound <= kCorrectableSymbols; ++i) {
            uint8_t x = expOf(kFieldSize - i); // alpha^{-i} = alpha^{(15-i) mod 15}
            uint8_t val = 0;
            for (int k = sigmaDegree; k >= 0; --k) {
                val = gfMul(val, x) ^ sigma[k];
            }
            if (val == 0) {
                if (errorsFound < kCorrectableSymbols) errorPositions[errorsFound] = i;
                errorsFound++;
            }
        }
        if (errorsFound != sigmaDegree) return true; // Lokator-Grad passt nicht zur Anzahl gefundener Nullstellen

        // --- Fehler-Auswerter-Polynom omega(x) = [S(x)*sigma(x)] mod x^(2t) ---
        std::array<uint8_t, kFecSize> omega{};
        for (int i = 0; i < kFecSize; ++i) {
            uint8_t acc = 0;
            for (int j = 0; j <= i && j <= sigmaDegree; ++j) {
                acc ^= gfMul(sigma[j], syndrome[i - j]);
            }
            omega[i] = acc;
        }

        // --- Forney-Formel: Fehlerwert an Position i = Omega(beta) / sigma'(beta),
        //     ausgewertet an der tatsaechlichen Nullstelle beta=alpha^{-pos} von
        //     sigma(x) (der bei der Chien-Suche gefundene Wert) - NICHT an deren
        //     Inversem! (In GF(2^m) entfaellt das Minuszeichen der Lehrbuchformel,
        //     da Addition=Subtraktion=XOR.)
        for (int e = 0; e < errorsFound; ++e) {
            int pos = errorPositions[e];
            uint8_t beta = expOf(kFieldSize - pos); // dieselbe Nullstelle wie in der Chien-Suche

            uint8_t omegaVal = 0;
            for (int k = kFecSize - 1; k >= 0; --k) {
                omegaVal = gfMul(omegaVal, beta) ^ omega[k];
            }
            // Formale Ableitung sigma'(x): nur die Terme mit ungeradem Grad
            // bleiben (Char. 2 -> gerade Potenzen fallen weg, Faktor wird 1).
            uint8_t sigmaDerivVal = 0;
            for (int k = 1; k <= sigmaDegree; k += 2) {
                uint8_t term = sigma[k];
                for (int p = 0; p < k - 1; ++p) term = gfMul(term, beta);
                sigmaDerivVal ^= term;
            }
            if (sigmaDerivVal == 0) return true; // sollte nicht vorkommen - defensiv abbrechen

            uint8_t errorMagnitude = gfMul(omegaVal, gfInverse(sigmaDerivVal));
            recd[pos] ^= errorMagnitude;
        }

        // Korrigierte Symbole zurueck ins Codewort schreiben.
        for (int k = 0; k < kFecSize; ++k) codeword[kDataSize + k] = recd[k];
        for (int i = 0; i < kDataSize; ++i) codeword[i] = recd[kFecSize + i];
        return false;
    }

private:
    std::array<uint8_t, kFieldSize> expTable_{}; // expTable_[i] = alpha^i, i=0..14
    std::array<int8_t, kFieldSize + 1> logTable_{}; // logTable_[x] = i sodass alpha^i = x (x=1..15)

    static constexpr uint16_t kPrimitivePoly = 0b10011; // x^4 + x + 1

    void buildGaloisField() {
        uint16_t reg = 1;
        for (int i = 0; i < kFieldSize; ++i) {
            expTable_[i] = static_cast<uint8_t>(reg);
            logTable_[reg] = static_cast<int8_t>(i);
            reg <<= 1;
            if (reg & (1 << kSymbolBits)) reg ^= kPrimitivePoly;
        }
    }

    uint8_t expOf(int i) const {
        int m = ((i % kFieldSize) + kFieldSize) % kFieldSize;
        return expTable_[m];
    }

    uint8_t gfMul(uint8_t a, uint8_t b) const {
        if (a == 0 || b == 0) return 0;
        return expOf(logTable_[a] + logTable_[b]);
    }

    uint8_t gfInverse(uint8_t a) const {
        // a * a^{-1} = alpha^0 = 1  ->  a^{-1} = alpha^{-log(a)}
        return expOf(-logTable_[a]);
    }

    // Wertet ein Polynom (Koeffizienten aufsteigend nach Grad, Index=Grad)
    // an der Stelle x via Horner-Schema aus.
    uint8_t evalPoly(const std::array<uint8_t, kFieldSize>& coeffsAscending, uint8_t x) const {
        uint8_t result = 0;
        for (int deg = kFieldSize - 1; deg >= 0; --deg) {
            result = gfMul(result, x) ^ coeffsAscending[deg];
        }
        return result;
    }

    std::array<uint8_t, kFecSize + 1> generator_{}; // aufsteigend, Grad 0..kFecSize, generator_[kFecSize]=1

    void buildGeneratorPolynomial() {
        std::array<uint8_t, kFecSize + 1> g{};
        g[0] = 1; // g(x) = 1
        int degree = 0;
        for (int i = 1; i <= kFecSize; ++i) {
            uint8_t root = expOf(i); // alpha^i
            std::array<uint8_t, kFecSize + 1> newG{};
            for (int k = 0; k <= degree; ++k) {
                newG[k] ^= gfMul(g[k], root);
                newG[k + 1] ^= g[k];
            }
            g = newG;
            degree++;
        }
        generator_ = g;
    }

    // Polynomdivision (Koeffizienten aufsteigend, dividend Grad 0..14) durch
    // das (monische) Generatorpolynom; liefert den Rest (Grad 0..kFecSize-1).
    std::array<uint8_t, kFecSize> polyModulo(std::array<uint8_t, kFieldSize> dividend) const {
        for (int i = kFieldSize - 1; i >= kFecSize; --i) {
            uint8_t coeff = dividend[i];
            if (coeff == 0) continue;
            for (int j = 0; j <= kFecSize; ++j) {
                dividend[i - kFecSize + j] ^= gfMul(coeff, generator_[j]);
            }
        }
        std::array<uint8_t, kFecSize> remainder{};
        for (int k = 0; k < kFecSize; ++k) remainder[k] = dividend[k];
        return remainder;
    }
};

} // namespace pl225rs
