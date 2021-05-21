/*
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "NVMeContext.hpp"

#include "NVMeDevice.hpp"

#include <crc32c.h>
#include <libmctp-smbus.h>
#include <libmctp.h>

#include <boost/container/flat_map.hpp>

static constexpr bool debug = false;

static void rxMessage(uint8_t eid, void* data, void* msg, size_t len);

namespace nvmeMCTP
{
struct mctp_binding_smbus* smbus = mctp_smbus_init();
struct mctp* mctp = mctp_init();

static boost::container::flat_map<int, int> inFds;
static boost::container::flat_map<int, int> outFds;

int getInFd(int rootBus)
{
    auto findBus = inFds.find(rootBus);
    if (findBus != inFds.end())
    {
        return findBus->second;
    }
    int fd = mctp_smbus_open_in_bus(smbus, rootBus);
    if (fd < 0)
    {
        std::cerr << "Error opening IN Bus " << rootBus << "\n";
    }
    inFds[rootBus] = fd;
    return fd;
}

int getOutFd(int bus)
{
    auto findBus = outFds.find(bus);
    if (findBus != outFds.end())
    {
        return findBus->second;
    }
    int fd = mctp_smbus_open_out_bus(smbus, bus);
    if (fd < 0)
    {
        std::cerr << "Error opening Out Bus " << bus << "\n";
    }
    outFds[bus] = fd;
    return fd;
}

// we don't close the outFd as multiple sensors could be sharing the fd, we need
// to close the inFd as it can only be used on 1 socket at a time
void closeInFd(int rootBus)
{
    auto findFd = inFds.find(rootBus);
    if (findFd == inFds.end())
    {
        return;
    }
    close(findFd->second);
    inFds.erase(rootBus);
}

int getRootBus(int inFd)
{
    // we assume that we won't have too many FDs, so looping is OK
    for (const auto [root, fd] : inFds)
    {
        if (fd == inFd)
        {
            return root;
        }
    }

    return -1;
}

void init()
{
    if (mctp == nullptr || smbus == nullptr)
    {
        throw std::runtime_error("Unable to init mctp");
    }
    mctp_smbus_register_bus(smbus, nvmeMCTP::mctp, 0);
    mctp_set_rx_all(mctp, rxMessage, nullptr);
}

} // namespace nvmeMCTP

static void rxMessage(uint8_t eid, void*, void* msg, size_t len)
{
    int inFd = mctp_smbus_get_in_fd(nvmeMCTP::smbus);
    int rootBus = nvmeMCTP::getRootBus(inFd);

    NVMEMap& nvmeMap = getNVMEMap();
    auto findMap = nvmeMap.find(rootBus);
    if (findMap == nvmeMap.end())
    {
        std::cerr << "Unable to lookup root bus " << rootBus << "\n";
        return;
    }

    if (debug)
    {
        std::cout << "Eid from the received messaged: " << eid << "\n";
    }

    findMap->second->processResponse(msg, len);
}

static int verifyIntegrity(uint8_t* msg, size_t len)
{
    uint32_t msgIntegrity = {0};
    if (len < NVME_MI_MSG_RESPONSE_HEADER_SIZE + sizeof(msgIntegrity))
    {
        std::cerr << "Not enough bytes for nvme header and trailer\n";
        return -1;
    }

    msgIntegrity = (msg[len - 4]) + (msg[len - 3] << 8) + (msg[len - 2] << 16) +
                   (msg[len - 1] << 24);

    uint32_t calculateIntegrity = crc32c(msg, len - sizeof(msgIntegrity));
    if (msgIntegrity != calculateIntegrity)
    {
        std::cerr << "CRC mismatch. Got=" << msgIntegrity
                  << " Expected=" << calculateIntegrity << "\n";
        return -1;
    }
    return 0;
}

static double getTemperatureReading(int8_t reading)
{

    if (reading == static_cast<int8_t>(0x80) ||
        reading == static_cast<int8_t>(0x81))
    {
        // 0x80 = No temperature data or temperature data is more the 5 s
        // old 0x81 = Temperature sensor failure
        return std::numeric_limits<double>::quiet_NaN();
    }

    return reading;
}

void NVMeContext::processResponse(void* msg, size_t len)
{
    struct nvme_mi_msg_response_header header
    {};

    if (msg == nullptr)
    {
        std::cerr << "Bad message received\n";
        return;
    }

    if (len <= 0)
    {
        std::cerr << "Received message not long enough\n";
        return;
    }

    uint8_t* messageData = static_cast<uint8_t*>(msg);

    if ((*messageData & NVME_MI_MESSAGE_TYPE_MASK) != NVME_MI_MESSAGE_TYPE)
    {
        std::cerr << "Got unknown type message_type="
                  << (*messageData & NVME_MI_MESSAGE_TYPE_MASK) << "\n";
        return;
    }

    if (len < NVME_MI_MSG_RESPONSE_HEADER_SIZE + sizeof(uint32_t))
    {
        std::cerr << "Not enough bytes for NVMe header and trailer\n";
        return;
    }

    if (verifyIntegrity(messageData, len) != 0)
    {
        std::cerr << "Verification of message integrity failed\n";
        return;
    }

    header.message_type = messageData[0];
    header.flags = messageData[1];
    header.status = messageData[4];

    if (header.status == NVME_MI_HDR_STATUS_MORE_PROCESSING_REQUIRED)
    {
        return;
    }

    if (header.status != NVME_MI_HDR_STATUS_SUCCESS)
    {
        std::cerr << "Command failed with status= " << header.status << "\n";
        return;
    }

    messageData += NVME_MI_MSG_RESPONSE_HEADER_SIZE;
    size_t messageLength =
        len - NVME_MI_MSG_RESPONSE_HEADER_SIZE - sizeof(uint32_t);
    if (((header.flags >> NVME_MI_HDR_FLAG_MSG_TYPE_SHIFT) &
         NVME_MI_HDR_FLAG_MSG_TYPE_MASK) != NVME_MI_HDR_MESSAGE_TYPE_MI_COMMAND)
    {
        std::cerr << "Not MI type comamnd\n";
        return;
    }

    if (messageLength < NVME_MI_HEALTH_STATUS_POLL_MSG_MIN)
    {
        std::cerr << "Got improperly sized health status poll\n";
        return;
    }

    std::shared_ptr<NVMeSensor> sensorInfo = sensors.front();
    if (debug)
    {
        std::cout << "Temperature Reading: "
                  << getTemperatureReading(messageData[5])
                  << " Celsius for device " << sensorInfo->name << "\n";
    }

    double value = getTemperatureReading(messageData[5]);
    if (!std::isfinite(value))
    {
        sensorInfo->markAvailable(false);
        sensorInfo->incrementError();
    }
    else
    {
        sensorInfo->updateValue(value);
    }

    if (debug)
    {
        std::cout << "Cancelling the timer now\n";
    }

    // move to back of scan queue
    sensors.pop_front();
    sensors.emplace_back(sensorInfo);

    mctpResponseTimer.cancel();
}

static int nvmeMessageTransmit(mctp& mctp, nvme_mi_msg_request& req)
{
    std::array<uint8_t, NVME_MI_MSG_BUFFER_SIZE> messageBuf = {};

    req.header.flags |= NVME_MI_HDR_MESSAGE_TYPE_MI_COMMAND
                        << NVME_MI_HDR_FLAG_MSG_TYPE_SHIFT;
    req.header.message_type =
        NVME_MI_MESSAGE_TYPE | NVME_MI_MCTP_INTEGRITY_CHECK;

    uint32_t integrity = 0;
    size_t msgSize = NVME_MI_MSG_REQUEST_HEADER_SIZE + req.request_data_len +
                     sizeof(integrity);

    if (sizeof(messageBuf) < msgSize)
    {
        return EXIT_FAILURE;
    }

    messageBuf[0] = req.header.message_type;
    messageBuf[1] = req.header.flags;
    // Reserved bytes 2-3

    messageBuf[4] = req.header.opcode;
    // reserved bytes 5-7
    messageBuf[8] = req.header.dword0 & 0xff;
    messageBuf[9] = (req.header.dword0 >> 8) & 0xff;
    messageBuf[10] = (req.header.dword0 >> 16) & 0xff;
    messageBuf[11] = (req.header.dword0 >> 24) & 0xff;

    messageBuf[12] = req.header.dword1 & 0xff;
    messageBuf[13] = (req.header.dword1 >> 8) & 0xff;
    messageBuf[14] = (req.header.dword1 >> 16) & 0xff;
    messageBuf[15] = (req.header.dword1 >> 24) & 0xff;

    std::copy_n(req.request_data, req.request_data_len,
                messageBuf.data() +
                    static_cast<uint8_t>(NVME_MI_MSG_REQUEST_HEADER_SIZE));

    msgSize = NVME_MI_MSG_REQUEST_HEADER_SIZE + req.request_data_len;
    integrity = crc32c(messageBuf.data(),
                       NVME_MI_MSG_REQUEST_HEADER_SIZE + req.request_data_len);
    messageBuf[msgSize] = integrity & 0xff;
    messageBuf[msgSize + 1] = (integrity >> 8) & 0xff;
    messageBuf[msgSize + 2] = (integrity >> 16) & 0xff;
    messageBuf[msgSize + 3] = (integrity >> 24) & 0xff;
    msgSize += sizeof(integrity);

    return mctp_message_tx(&mctp, 0, messageBuf.data(), msgSize);
}

void NVMeContext::readResponse()
{
    nvmeSlaveSocket.async_wait(
        boost::asio::ip::tcp::socket::wait_error,
        [this](const boost::system::error_code errorCode) {
            if (errorCode)
            {
                return;
            }

            mctp_smbus_set_in_fd(nvmeMCTP::smbus, nvmeMCTP::getInFd(rootBus));

            // through libmctp this will invoke rxMessage
            mctp_smbus_read(nvmeMCTP::smbus);
        });
}

void NVMeContext::readAndProcessNVMeSensor()
{
    struct nvme_mi_msg_request requestMsg = {};
    requestMsg.header.opcode = NVME_MI_OPCODE_HEALTH_STATUS_POLL;
    requestMsg.header.dword0 = 0;
    requestMsg.header.dword1 = 0;

    int mctpResponseTimeout = 1;

    if (sensors.empty())
    {
        return;
    }

    std::shared_ptr<NVMeSensor>& sensor = sensors.front();

    // setup the timeout timer
    mctpResponseTimer.expires_from_now(
        boost::posix_time::seconds(mctpResponseTimeout));

    mctpResponseTimer.async_wait(
        [sensor, this](const boost::system::error_code errorCode) {
            if (errorCode)
            {
                // timer cancelled successfully
                return;
            }

            sensor->incrementError();

            // cycle it back
            sensors.pop_front();
            sensors.emplace_back(sensor);

            nvmeSlaveSocket.cancel();
        });

    readResponse();

    if (debug)
    {
        std::cout << "Sending message to read data from Drive on bus: "
                  << sensor->bus << " , rootBus: " << rootBus
                  << " device: " << sensor->name << "\n";
    }

    mctp_smbus_set_out_fd(nvmeMCTP::smbus, nvmeMCTP::getOutFd(sensor->bus));
    int rc = nvmeMessageTransmit(*nvmeMCTP::mctp, requestMsg);

    if (rc != 0)
    {
        std::cerr << "Error sending request message to NVMe device\n";
    }
}

NVMeContext::NVMeContext(boost::asio::io_service& io, int rootBus) :
    scanTimer(io), rootBus(rootBus), mctpResponseTimer(io), nvmeSlaveSocket(io)
{
    nvmeSlaveSocket.assign(boost::asio::ip::tcp::v4(),
                           nvmeMCTP::getInFd(rootBus));
}

void NVMeContext::pollNVMeDevices()
{
    scanTimer.expires_from_now(boost::posix_time::seconds(1));
    scanTimer.async_wait(
        [self{shared_from_this()}](const boost::system::error_code errorCode) {
            if (errorCode == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }
            else if (errorCode)
            {
                std::cerr << "Error:" << errorCode.message() << "\n";
                return;
            }
            else
            {
                self->readAndProcessNVMeSensor();
            }

            self->pollNVMeDevices();
        });
}

void NVMeContext::close()
{
    scanTimer.cancel();
    mctpResponseTimer.cancel();
    nvmeSlaveSocket.cancel();
    nvmeMCTP::closeInFd(rootBus);
}

NVMeContext::~NVMeContext()
{
    close();
}
