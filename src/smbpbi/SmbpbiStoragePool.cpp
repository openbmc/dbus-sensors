/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "SmbpbiStoragePool.hpp"

#include <fcntl.h>
#include <linux/i2c.h>
#include <unistd.h>

#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <format>
#include <limits>
#include <string>

extern "C"
{
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
}

SmbpbiStoragePool::SmbpbiStoragePool()
{
    pollingThread = std::thread(&SmbpbiStoragePool::pollingWorker, this);
}

SmbpbiStoragePool::~SmbpbiStoragePool()
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        stopPolling = true;
    }
    cv.notify_all();
    if (pollingThread.joinable())
    {
        pollingThread.join();
    }
    for (auto& [busId, fd] : busFds)
    {
        if (fd >= 0)
        {
            ::close(fd);
        }
    }
    busFds.clear();
}

void SmbpbiStoragePool::recalculateDeviceConfig(DeviceConfig& config)
{
    if (config.sensors.empty())
    {
        config.hasSensors = false;
        config.minOffset = 0;
        config.maxEndOffset = 0;
        return;
    }

    config.hasSensors = true;
    uint16_t minOff = std::numeric_limits<uint16_t>::max();
    uint16_t maxEnd = 0;
    for (const auto& s : config.sensors)
    {
        minOff = std::min(minOff, s.offset);
        maxEnd = std::max<uint16_t>(maxEnd,
                                    static_cast<uint16_t>(s.offset + s.length));

        if (s.staleOffset != 0)
        {
            minOff = std::min(minOff, s.staleOffset);
            maxEnd = std::max<uint16_t>(
                maxEnd, static_cast<uint16_t>(s.staleOffset + 1));
        }
    }
    config.minOffset = minOff;
    config.maxEndOffset = maxEnd;
}

void SmbpbiStoragePool::recalculateMinPollRate()
{
    size_t minRate = std::numeric_limits<size_t>::max();
    for (const auto& [key, cfg] : deviceConfigs)
    {
        for (const auto& s : cfg.sensors)
        {
            if (s.pollRate > 0)
            {
                minRate = std::min(minRate, s.pollRate);
            }
        }
    }
    minPollRate = (minRate == std::numeric_limits<size_t>::max()) ? 1 : minRate;
}

void SmbpbiStoragePool::registerSensor(
    uint8_t busId, uint8_t addr, uint16_t offset, size_t length,
    size_t pollRate, uint16_t staleOffset, uint8_t staleBit)
{
    std::lock_guard<std::mutex> lock(mutex);
    auto& config = deviceConfigs[{busId, addr}];
    config.busId = busId;
    config.addr = addr;
    config.sensors.push_back({offset, length, pollRate, staleOffset, staleBit});

    recalculateDeviceConfig(config);
    recalculateMinPollRate();

    hasNewRegistration = true;
    cv.notify_all();
}

void SmbpbiStoragePool::unregisterSensor(uint8_t busId, uint8_t addr,
                                         uint16_t offset, size_t length)
{
    std::lock_guard<std::mutex> lock(mutex);
    auto it = deviceConfigs.find({busId, addr});
    if (it == deviceConfigs.end())
    {
        return;
    }

    std::erase_if(it->second.sensors, [offset, length](const auto& s) {
        return s.offset == offset && s.length == length;
    });

    recalculateDeviceConfig(it->second);
    recalculateMinPollRate();
}

int SmbpbiStoragePool::readData(
    uint8_t busId, uint8_t addr, uint16_t offset, uint8_t* reading,
    size_t length, uint16_t staleOffset, uint8_t staleBit, bool& isStale)
{
    isStale = false;
    if (reading == nullptr || length == 0)
    {
        return -EINVAL;
    }
    std::lock_guard<std::mutex> lock(mutex);
    auto it = deviceDataPool.find({busId, addr});
    if (it == deviceDataPool.end() || !it->second.valid)
    {
        return -EAGAIN;
    }
    const auto& data = it->second;
    if (offset < data.baseOffset ||
        (static_cast<size_t>(offset) + length) >
            (static_cast<size_t>(data.baseOffset) + data.buffer.size()))
    {
        lg2::error(
            "Requested range [{OFF}, {END}) out of polled buffer [{BASE}, {MAX})",
            "OFF", offset, "END", offset + length, "BASE", data.baseOffset,
            "MAX", data.baseOffset + data.buffer.size());
        return -EINVAL;
    }
    size_t relOffset = offset - data.baseOffset;
    std::memcpy(reading, data.buffer.data() + relOffset, length);

    // Atomic stale bit evaluation from the exact same buffer snapshot
    if (staleOffset != 0)
    {
        if (staleOffset >= data.baseOffset &&
            staleOffset < (data.baseOffset + data.buffer.size()))
        {
            size_t staleRelOffset = staleOffset - data.baseOffset;
            uint8_t staleByte = data.buffer[staleRelOffset];
            isStale = (staleByte & (1U << staleBit)) != 0;
            if (isStale)
            {
                std::string hexReading;
                for (size_t i = 0; i < length; ++i)
                {
                    if (i > 0)
                    {
                        hexReading += ' ';
                    }
                    hexReading += std::format("{:02x}", reading[i]);
                }
                lg2::info(
                    "Stale bit asserted on bus {BUS} addr {ADDR}: offset {OFF} (len {LEN}), stale {STALEOFF}[bit {BIT}] (stale byte={STALEBYTE}), reading: [ {DATA} ]",
                    "BUS", busId, "ADDR", lg2::hex, addr, "OFF", lg2::hex,
                    offset, "LEN", length, "STALEOFF", lg2::hex, staleOffset,
                    "BIT", staleBit, "STALEBYTE", lg2::hex,
                    static_cast<int>(staleByte), "DATA", hexReading);
            }
        }
        else
        {
            lg2::error(
                "Stale offset {STALEOFF} out of polled buffer [{BASE}, {MAX})",
                "STALEOFF", staleOffset, "BASE", data.baseOffset, "MAX",
                data.baseOffset + data.buffer.size());
            return -EINVAL;
        }
    }
    return 0;
}

int SmbpbiStoragePool::getBusFd(uint8_t busId)
{
    auto it = busFds.find(busId);
    if (it != busFds.end() && it->second >= 0)
    {
        return it->second;
    }

    std::string path = "/dev/i2c-" + std::to_string(busId);
    // NOLINTNEXTLINE(android-cloexec-open)
    int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        lg2::error("unable to open i2c device on bus {BUS} err={ERR}", "BUS",
                   busId, "ERR", errno);
        return -1;
    }

    unsigned long funcs = 0;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        lg2::error("I2C_FUNCS not supported on bus {BUS}", "BUS", busId);
        ::close(fd);
        return -1;
    }

    busFds[busId] = fd;
    return fd;
}

bool SmbpbiStoragePool::executeI2CRead(uint8_t busId, uint8_t addr,
                                       uint16_t offset, bool twoByteOffset,
                                       uint8_t* buffer, size_t length)
{
    if (buffer == nullptr || length == 0 || length > 0xFFFF)
    {
        return false;
    }

    int fd = getBusFd(busId);
    if (fd < 0)
    {
        return false;
    }

    struct i2c_rdwr_ioctl_data args = {nullptr, 0};
    std::array<struct i2c_msg, 2> msgs = {
        {{0, 0, 0, nullptr}, {0, 0, 0, nullptr}}};
    std::array<uint8_t, 2> cmd{};

    args.msgs = msgs.data();
    args.nmsgs = static_cast<__u32>(msgs.size());

    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].buf = cmd.data();
    if (twoByteOffset)
    {
        msgs[0].len = 2;
        msgs[0].buf[0] = static_cast<uint8_t>(offset >> 8);
        msgs[0].buf[1] = static_cast<uint8_t>(offset & 0xFF);
    }
    else
    {
        msgs[0].len = 1;
        msgs[0].buf[0] = static_cast<uint8_t>(offset & 0xFF);
    }

    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = static_cast<__u16>(length);
    msgs[1].buf = buffer;

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    int ret = ioctl(fd, I2C_RDWR, &args);
    if (ret < 0)
    {
        lg2::error(
            "I2C_RDWR failed on bus {BUS} addr {ADDR} offset {OFF}: {ERR}",
            "BUS", busId, "ADDR", addr, "OFF", offset, "ERR", errno);
        if (errno == EBADF || errno == ENXIO)
        {
            ::close(fd);
            busFds.erase(busId);
        }
        return false;
    }
    return true;
}

void SmbpbiStoragePool::pollingWorker()
{
    while (true)
    {
        struct PollTask
        {
            uint8_t busId{0};
            uint8_t addr{0};
            uint16_t minOffset{0};
            size_t length{0};
            bool twoByteOffset{false};
        };
        std::vector<PollTask> tasks;
        size_t sleepSeconds = 1;

        {
            std::unique_lock<std::mutex> lock(mutex);
            if (stopPolling)
            {
                break;
            }
            sleepSeconds = minPollRate;
            for (const auto& [key, config] : deviceConfigs)
            {
                if (config.hasSensors && config.maxEndOffset > config.minOffset)
                {
                    PollTask task;
                    task.busId = config.busId;
                    task.addr = config.addr;
                    task.minOffset = config.minOffset;
                    task.length = config.maxEndOffset - config.minOffset;
                    task.twoByteOffset =
                        (config.minOffset > 255 || config.maxEndOffset > 255);
                    tasks.push_back(task);
                }
            }
        }

        for (const auto& task : tasks)
        {
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (stopPolling)
                {
                    break;
                }
            }

            std::vector<uint8_t> buffer(task.length, 0);
            bool success =
                executeI2CRead(task.busId, task.addr, task.minOffset,
                               task.twoByteOffset, buffer.data(), task.length);

            {
                std::lock_guard<std::mutex> lock(mutex);
                auto& data = deviceDataPool[{task.busId, task.addr}];
                data.baseOffset = task.minOffset;
                if (success)
                {
                    data.buffer = std::move(buffer);
                    data.valid = true;
                }
                else
                {
                    data.valid = false;
                }
            }
        }

        {
            std::unique_lock<std::mutex> lock(mutex);
            if (stopPolling)
            {
                break;
            }
            cv.wait_for(lock, std::chrono::seconds(sleepSeconds),
                        [&]() { return stopPolling || hasNewRegistration; });
            hasNewRegistration = false;
        }
    }
}

SmbpbiStoragePool& getStoragePool()
{
    static SmbpbiStoragePool pool;
    return pool;
}
