/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

class SmbpbiStoragePool
{
  public:
    SmbpbiStoragePool();
    ~SmbpbiStoragePool();

    SmbpbiStoragePool(const SmbpbiStoragePool&) = delete;
    SmbpbiStoragePool& operator=(const SmbpbiStoragePool&) = delete;
    SmbpbiStoragePool(SmbpbiStoragePool&&) = delete;
    SmbpbiStoragePool& operator=(SmbpbiStoragePool&&) = delete;

    void registerSensor(uint8_t busId, uint8_t addr, uint16_t offset,
                        size_t length, size_t pollRate,
                        uint16_t staleOffset = 0, uint8_t staleBit = 0);
    void unregisterSensor(uint8_t busId, uint8_t addr, uint16_t offset,
                          size_t length);
    int readData(uint8_t busId, uint8_t addr, uint16_t offset, uint8_t* reading,
                 size_t length, uint16_t staleOffset, uint8_t staleBit,
                 bool& isStale);

  private:
    struct DeviceData
    {
        uint16_t baseOffset{0};
        std::vector<uint8_t> buffer;
        bool valid{false};
    };

    struct SensorTarget
    {
        uint16_t offset{0};
        size_t length{0};
        size_t pollRate{1};
        uint16_t staleOffset{0};
        uint8_t staleBit{0};
    };

    struct DeviceConfig
    {
        uint8_t busId{0};
        uint8_t addr{0};
        uint16_t minOffset{0};
        uint16_t maxEndOffset{0};
        bool hasSensors{false};
        std::vector<SensorTarget> sensors;
    };

    mutable std::mutex mutex;
    std::condition_variable cv;
    bool stopPolling{false};
    bool hasNewRegistration{false};
    size_t minPollRate{1};

    std::map<std::pair<uint8_t, uint8_t>, DeviceConfig> deviceConfigs;
    std::map<std::pair<uint8_t, uint8_t>, DeviceData> deviceDataPool;
    std::unordered_map<uint8_t, int> busFds;

    std::thread pollingThread;

    void recalculateDeviceConfig(DeviceConfig& config);
    void recalculateMinPollRate();
    void pollingWorker();
    bool executeI2CRead(uint8_t busId, uint8_t addr, uint16_t offset,
                        bool twoByteOffset, uint8_t* buffer, size_t length);
    int getBusFd(uint8_t busId);
};

SmbpbiStoragePool& getStoragePool();
