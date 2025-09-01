// #include "conversions.h"
// #include <algorithm> // std::clamp
// #include <cmath>     // std::floor, std::fabs
// namespace conv {

// // clamp an int to 0..255 and cast
// static inline std::uint8_t to_u8(int x) {
//     return static_cast<std::uint8_t>(std::clamp(x, 0, 255));
// }

// std::uint8_t integerPartToHex(double value) {
//     // handles negatives safely
//     int integerPart = static_cast<int>(std::floor(value));
//     return to_u8(integerPart);
// }

// std::uint8_t decimalPartToHex(double value) {
//     // fractional part in [0, 1)
//     double frac = value - std::floor(value);
//     // scale to 0..99 (truncate)
//     int scaled = static_cast<int>(std::floor(frac * 100.0));
//     return to_u8(scaled);
// }

// std::uint8_t tensOnesDigits(double value) {
//     int integerPart = static_cast<int>(std::floor(value));
//     int lastTwo = std::abs(integerPart) % 100; // handle negatives nicely
//     return to_u8(lastTwo);
// }

// std::uint8_t thousandsHundredsDigits(double value) {
//     // get the two digits in the hundreds/thousands place.
//     // Example: 1234 -> 12, 987 -> 9, 45 -> 0
//     int integerPart = static_cast<int>(std::floor(std::fabs(value)));
//     int highTwo = (integerPart >= 100) ? (integerPart / 100) % 100 : 0;
//     return to_u8(highTwo);
// }

// } // namespace conv