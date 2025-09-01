#pragma once
#include <cstdint>
#include <algorithm>
#include <cmath>

namespace conv {
inline std::uint8_t to_u8(int x) {
    return static_cast<std::uint8_t>(std::clamp(x, 0, 255));
}

inline std::uint8_t integerPartToHex(double value) {
    int integerPart = static_cast<int>(std::floor(value));
    return to_u8(integerPart);
}

inline std::uint8_t decimalPartToHex(double value) {
    double frac = value - std::floor(value);
    int scaled = static_cast<int>(std::floor(frac * 100.0));
    return to_u8(scaled);
}

inline std::uint8_t tensOnesDigits(double value) {
    int integerPart = static_cast<int>(std::floor(value));
    int lastTwo = std::abs(integerPart) % 100;
    return to_u8(lastTwo);
}

inline std::uint8_t thousandsHundredsDigits(double value) {
    int integerPart = static_cast<int>(std::floor(std::fabs(value)));
    int highTwo = (integerPart >= 100) ? (integerPart / 100) % 100 : 0;
    return to_u8(highTwo);
}
}
