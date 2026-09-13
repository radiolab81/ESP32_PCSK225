// crc8.hpp
//
// Minimale, unabhaengige CRC8-Implementierung. Das PL225/e-CzasPL Protokoll
// (siehe pl225_decoder.hpp) verwendet CRC8 mit Polynom 0x07, Startwert 0x00,
// MSB-first, keine Reflektion, kein finales XOR (das ist die "klassische"
// CRC-8, manchmal auch CRC-8/SMBUS genannt). Die genauen Parameter wurden
// von SP5WWP/SP6HFE per Trial-and-Error gegen echte Aufnahmen ermittelt
// (in der offiziellen e-CzasPL Spezifikation fehlen sie, siehe README).

#pragma once

#include <cstdint>
#include <cstddef>

namespace pl225 {

class Crc8 {
public:
    Crc8(uint8_t polynomial, uint8_t initValue) : poly_(polynomial), crc_(initValue) {}

    void update(uint8_t byte) {
        crc_ ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc_ & 0x80) {
                crc_ = static_cast<uint8_t>((crc_ << 1) ^ poly_);
            } else {
                crc_ = static_cast<uint8_t>(crc_ << 1);
            }
        }
    }

    uint8_t get() const { return crc_; }

private:
    uint8_t poly_;
    uint8_t crc_;
};

} // namespace pl225
