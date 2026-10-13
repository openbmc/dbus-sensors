/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

struct StaleBitInfo
{
    uint16_t offset{0}; // Register offset of the stale byte in virtual EEPROM
    uint8_t bit{0};     // Stale bit index within the stale byte (0 - 7)
};

struct SensorStaleEntry
{
    uint16_t sensorOffset;
    StaleBitInfo staleInfo;
};

class SensorStaleBitTable
{
  public:
    // Pre-defined lookup table sorted strictly by sensorOffset for O(log N)
    // constexpr binary search
    static constexpr auto table = std::to_array<SensorStaleEntry>({
        {0x019, {0x039, 0}}, // HGX_GPU_0_DRAM_0_Temp_0
        {0x01D, {0x039, 1}}, // HGX_GPU_1_DRAM_0_Temp_0
        {0x021, {0x039, 2}}, // HGX_GPU_2_DRAM_0_Temp_0
        {0x025, {0x039, 3}}, // HGX_GPU_3_DRAM_0_Temp_0
        {0x029, {0x039, 4}}, // HGX_GPU_4_DRAM_0_Temp_0
        {0x02D, {0x039, 5}}, // HGX_GPU_5_DRAM_0_Temp_0
        {0x031, {0x039, 6}}, // HGX_GPU_6_DRAM_0_Temp_0
        {0x035, {0x039, 7}}, // HGX_GPU_7_DRAM_0_Temp_0
        {0x04B, {0x06B, 0}}, // HGX_GPU_0_Power_0
        {0x04F, {0x06B, 1}}, // HGX_GPU_1_Power_0
        {0x053, {0x06B, 2}}, // HGX_GPU_2_Power_0
        {0x057, {0x06B, 3}}, // HGX_GPU_3_Power_0
        {0x05B, {0x06B, 4}}, // HGX_GPU_4_Power_0
        {0x05F, {0x06B, 5}}, // HGX_GPU_5_Power_0
        {0x063, {0x06B, 6}}, // HGX_GPU_6_Power_0
        {0x067, {0x06B, 7}}, // HGX_GPU_7_Power_0
        {0x06C, {0x08C, 0}}, // HGX_GPU_0_TEMP_1
        {0x070, {0x08C, 1}}, // HGX_GPU_1_TEMP_1
        {0x074, {0x08C, 2}}, // HGX_GPU_2_TEMP_1
        {0x078, {0x08C, 3}}, // HGX_GPU_3_TEMP_1
        {0x07C, {0x08C, 4}}, // HGX_GPU_4_TEMP_1
        {0x080, {0x08C, 5}}, // HGX_GPU_5_TEMP_1
        {0x084, {0x08C, 6}}, // HGX_GPU_6_TEMP_1
        {0x088, {0x08C, 7}}, // HGX_GPU_7_TEMP_1
        {0x08D, {0x0CD, 0}}, // HGX_GPU_0_Energy_0
        {0x095, {0x0CD, 1}}, // HGX_GPU_1_Energy_0
        {0x09D, {0x0CD, 2}}, // HGX_GPU_2_Energy_0
        {0x0A5, {0x0CD, 3}}, // HGX_GPU_3_Energy_0
        {0x0AD, {0x0CD, 4}}, // HGX_GPU_4_Energy_0
        {0x0B5, {0x0CD, 5}}, // HGX_GPU_5_Energy_0
        {0x0BD, {0x0CD, 6}}, // HGX_GPU_6_Energy_0
        {0x0C5, {0x0CD, 7}}, // HGX_GPU_7_Energy_0
        {0x0CE, {0x0EE, 0}}, // HGX_GPU_0_DRAM_0_Power_0
        {0x0D2, {0x0EE, 1}}, // HGX_GPU_1_DRAM_0_Power_0
        {0x0D6, {0x0EE, 2}}, // HGX_GPU_2_DRAM_0_Power_0
        {0x0DA, {0x0EE, 3}}, // HGX_GPU_3_DRAM_0_Power_0
        {0x0DE, {0x0EE, 4}}, // HGX_GPU_4_DRAM_0_Power_0
        {0x0E2, {0x0EE, 5}}, // HGX_GPU_5_DRAM_0_Power_0
        {0x0E6, {0x0EE, 6}}, // HGX_GPU_6_DRAM_0_Power_0
        {0x0EA, {0x0EE, 7}}, // HGX_GPU_7_DRAM_0_Power_0
        {0x120, {0x100, 0}}, // HGX_GPU_0_TEMP_0
        {0x124, {0x100, 1}}, // HGX_GPU_1_TEMP_0
        {0x128, {0x100, 2}}, // HGX_GPU_2_TEMP_0
        {0x12C, {0x100, 3}}, // HGX_GPU_3_TEMP_0
        {0x130, {0x100, 4}}, // HGX_GPU_4_TEMP_0
        {0x134, {0x100, 5}}, // HGX_GPU_5_TEMP_0
        {0x138, {0x100, 6}}, // HGX_GPU_6_TEMP_0
        {0x13C, {0x100, 7}}, // HGX_GPU_7_TEMP_0
        {0x140, {0x101, 0}}, // HGX_ProcessorModule_0_Inlet_Temp_0
        {0x144, {0x101, 1}}, // HGX_ProcessorModule_0_Inlet_Temp_1
        {0x148, {0x101, 2}}, // HGX_ProcessorModule_0_Exhaust_Temp_0
        {0x150, {0x101, 4}}, // HGX_ProcessorModule_1_Inlet_Temp_0
        {0x154, {0x101, 5}}, // HGX_ProcessorModule_1_Inlet_Temp_1
        {0x158, {0x101, 6}}, // HGX_ProcessorModule_1_Exhaust_Temp_0
        {0x280, {0x200, 0}}, // ProcessorModule_0_CPU_0_Energy_0
        {0x288, {0x200, 1}}, // ProcessorModule_0_CPU_0_Power_0
        {0x290, {0x200, 2}}, // ProcessorModule_0_Vreg_0_CpuPower_0
        {0x298, {0x200, 3}}, // ProcessorModule_0_Vreg_0_SocPower_0
        {0x2A0, {0x200, 4}}, // ProcessorModule_0_CPU_0_TempAvg_0
        {0x2A8, {0x200, 5}}, // ProcessorModule_0_CPU_0_TempLimit_0
        {0x300, {0x210, 0}}, // ProcessorModule_0_CPU_1_Energy_0
        {0x308, {0x210, 1}}, // ProcessorModule_0_CPU_1_Power_0
        {0x310, {0x210, 2}}, // ProcessorModule_0_Vreg_1_CpuPower_0
        {0x318, {0x210, 3}}, // ProcessorModule_0_Vreg_1_SocPower_0
        {0x320, {0x210, 4}}, // ProcessorModule_0_CPU_1_TempAvg_0
        {0x328, {0x210, 5}}, // ProcessorModule_0_CPU_1_TempLimit_0
        {0x380, {0x220, 0}}, // ProcessorModule_1_CPU_0_Energy_0
        {0x388, {0x220, 1}}, // ProcessorModule_1_CPU_0_Power_0
        {0x390, {0x220, 2}}, // ProcessorModule_1_Vreg_0_CpuPower_0
        {0x398, {0x220, 3}}, // ProcessorModule_1_Vreg_0_SocPower_0
        {0x3A0, {0x220, 4}}, // ProcessorModule_1_CPU_0_TempAvg_0
        {0x3A8, {0x220, 5}}, // ProcessorModule_1_CPU_0_TempLimit_0
        {0x400, {0x230, 0}}, // ProcessorModule_1_CPU_1_Energy_0
        {0x408, {0x230, 1}}, // ProcessorModule_1_CPU_1_Power_0
        {0x410, {0x230, 2}}, // ProcessorModule_1_Vreg_1_CpuPower_0
        {0x418, {0x230, 3}}, // ProcessorModule_1_Vreg_1_SocPower_0
        {0x420, {0x230, 4}}, // ProcessorModule_1_CPU_1_TempAvg_0
        {0x428, {0x230, 5}}  // ProcessorModule_1_CPU_1_TempLimit_0
    });

    /**
     * @brief Lookup the corresponding stale bit offset and bit number based on
     * sensor offset.
     * @param sensorOffset Offset of the sensor in virtual EEPROM.
     * @return StaleBitInfo if the sensor has an associated stale bit, or
     * std::nullopt if NA.
     */
    static constexpr std::optional<StaleBitInfo> getStaleBitInfo(
        uint16_t sensorOffset) noexcept
    {
        auto it =
            std::lower_bound(table.begin(), table.end(), sensorOffset,
                             [](const SensorStaleEntry& entry, uint16_t off) {
                                 return entry.sensorOffset < off;
                             });

        if (it != table.end() && it->sensorOffset == sensorOffset)
        {
            return it->staleInfo;
        }
        return std::nullopt;
    }

    /**
     * @brief Check if a sensor offset has an associated stale bit defined.
     */
    static constexpr bool hasStaleBit(uint16_t sensorOffset) noexcept
    {
        return getStaleBitInfo(sensorOffset).has_value();
    }
};
