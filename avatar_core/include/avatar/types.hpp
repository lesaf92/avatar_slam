// Shared enumerations with stable integer codes (docs/conventions.md §5).
#pragma once

#include <cstdint>

namespace avatar {

/// Operating domain of an agent. Codes are part of the wire format.
enum class Domain : std::uint8_t { kAerial = 0, kGround = 1, kSurface = 2, kUnderwater = 3 };

/// Physical communication link class.
enum class LinkType : std::uint8_t { kRf = 0, kAcoustic = 1 };

/// Bit flags carried by each landmark record on the wire (wire spec §3).
namespace landmark_flags {
constexpr std::uint8_t kAbove = 1U << 0U;
constexpr std::uint8_t kBelow = 1U << 1U;
constexpr std::uint8_t kCamera = 1U << 2U;
constexpr std::uint8_t kLidar = 1U << 3U;
constexpr std::uint8_t kSonar = 1U << 4U;
constexpr std::uint8_t kValidMask = 0x1FU;
}  // namespace landmark_flags

/// |z| [m] below which a vehicle counts as being at the water surface.
constexpr double kWaterSurfaceTolM = 0.3;

}  // namespace avatar
