/**
 * @file audio_system.hpp
 * @brief Audio configuration, conversion constants, buffers, and mixer state.
 */

#pragma once

#include "arm_math.h"
#include "audio_ring_buffer.hpp"
#include <algorithm>
#include <array>

// ============================================================================
// 0. Audio Math & Conversion Constants
// ============================================================================
namespace Audio {
namespace Math {
/** @brief Mathematical constant pi. */
constexpr float32_t Pi{3.14159265358979323846f};
} // namespace Math

namespace Convert {
/** @brief Convert int16 PCM sample to float in [-1.0, 1.0). */
constexpr float32_t Int16ToFloat{1.0f / 32768.0f};
/** @brief Convert int32 PCM sample to float in [-1.0, 1.0). */
constexpr float32_t Int32ToFloat{1.0f / 2147483648.0f};
/** @brief Convert float in [-1.0, 1.0) to int32 PCM scale. */
constexpr float32_t FloatToInt32{2147483647.0f};
} // namespace Convert
} // namespace Audio

// ============================================================================
// 1. Audio System Configurations
// ============================================================================
namespace AudioConfig {
/** @brief Audio sample rate in Hz. */
constexpr uint32_t SampleRate{48000};
/** @brief Number of channels (stereo). */
constexpr uint32_t Channels{2};

/** @brief Frames processed per 1 ms block. */
constexpr uint32_t FramesPerBlock{(SampleRate) / 1000};
/** @brief Samples in one processing block (frames * channels). */
constexpr uint32_t SamplesPerBlock{FramesPerBlock * Channels};
/** @brief Total DMA samples for double-buffered audio transfer. */
constexpr uint32_t DMABufferSize{SamplesPerBlock * 2};

/** @brief USB audio ring buffer size in samples. */
constexpr uint32_t USBRingBufferSize{2048};
} // namespace AudioConfig

// ============================================================================
// 2. Hardware DMA & USB Buffers (External Declarations)
// ============================================================================
// SAI DMA Buffers
/** @brief Output DMA buffer for monitoring TX stream. */
extern
    __attribute__((section(".sram4"),
                   aligned(4))) std::array<int32_t, AudioConfig::DMABufferSize>
        monitorTxBuffer;

/** @brief Input DMA buffer for ADC RX stream. */
extern
    __attribute__((section(".sram4"),
                   aligned(4))) std::array<int32_t, AudioConfig::DMABufferSize>
        ADCRxBuffer;

/** @brief Front USB RX intermediate buffer aligned for DMA/cache. */
extern
    __attribute__((aligned(32))) std::array<int32_t, AudioConfig::DMABufferSize>
        frontUSBRxBuffer;

/** @brief Rear USB RX intermediate buffer aligned for DMA/cache. */
extern
    __attribute__((aligned(32))) std::array<int32_t, AudioConfig::DMABufferSize>
        rearUSBRxBuffer;

// USB Ring Buffers
/** @brief Main USB receive ring buffer. */
extern AudioRingBuffer<int16_t, AudioConfig::USBRingBufferSize> mainUSBRxBuffer;
/** @brief Main USB transmit ring buffer. */
extern AudioRingBuffer<int16_t, AudioConfig::USBRingBufferSize> mainUSBTxBuffer;

// ============================================================================
// 3. DSP & Mixer Core
// ============================================================================
namespace Mixer {
/** @brief Left channel processing buffer for one audio block. */
extern std::array<float32_t, AudioConfig::FramesPerBlock> chLeft;
/** @brief Right channel processing buffer for one audio block. */
extern std::array<float32_t, AudioConfig::FramesPerBlock> chRight;

/** @brief Runtime mixer gain parameters. */
struct Parameter {
  float32_t gainADCLeft{1.0f};
  float32_t gainADCRight{0.0f};
  float32_t gainMainUSB{0.5f};
  float32_t gainFrontUSB{0.3f};
  float32_t gainRearUSB{0.3f};
  float32_t gainMaster{1.0f};
};

/** @brief Global mixer parameter instance. */
extern Parameter param;

// Inline utility functions
/**
 * @brief Clamp gain value to the valid range [0.0, 1.0].
 * @param value Gain candidate.
 * @return Clamped gain.
 */
inline float32_t clampGain(float32_t value) {
  return std::min(1.0f, std::max(0.0f, value));
}
} // namespace Mixer
