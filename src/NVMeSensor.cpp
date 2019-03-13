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

#include "NVMeSensor.hpp"

#include "NVMeDevice.hpp"

#include <unistd.h>

#include <iostream>
#include <limits>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>

int getDeviceInfoToQuery()
{
    static int lastQueriedDeviceIndex = -1;
    if (!nvmeDeviceList.empty())
    {
        lastQueriedDeviceIndex++;

        if (lastQueriedDeviceIndex >= nvmeDeviceList.size())
        {
            lastQueriedDeviceIndex = 0;
        }
    }
    else
    {
        lastQueriedDeviceIndex = -1;
    }

    return lastQueriedDeviceIndex;
}

void readResponse(struct NVMeContext& nvmeDevice,
                  boost::asio::deadline_timer& mctpResponseTimer)
{
    nvmeDevice.nvmeSlaveSocket->async_wait(
        boost::asio::ip::tcp::socket::wait_error,
        [&mctpResponseTimer,
         &nvmeDevice](const boost::system::error_code& errorCode) {
            if (errorCode)
            {
                return;
            }

            int rc = mctp_smbus_read(nvmeDevice.smbus.get());

            if (rc == MCTP_READ_RETRY)
            {
                if (DEBUG)
                {
                    std::cout
                        << "Initiating a re-read for bus: " << nvmeDevice.bus
                        << " , rootBus: " << nvmeDevice.rootBus << "\n";
                }
                readResponse(nvmeDevice, mctpResponseTimer);
            }
            else
            {
                if (DEBUG)
                {
                    std::cout << "Cancelling the timer now\n";
                }

                mctpResponseTimer.cancel();
            }
        });
}

void readAndProcessNVMeSensor(boost::asio::deadline_timer& mctpResponseTimer,
                              int index)
{
    struct nvme_mi_msg_request requestMsg = {};
    requestMsg.header.opcode = NVME_MI_OPCODE_HEALTH_STATUS_POLL;
    requestMsg.header.dword0 = 0;
    requestMsg.header.dword1 = 0;

    int mctpResponseTimeout = 1;

    NVMeContext& nvmeDevice = *nvmeDeviceList[index].second;
    nvmeDevice.sindex = index;

    mctpResponseTimer.expires_from_now(
        boost::posix_time::seconds(mctpResponseTimeout));
    mctpResponseTimer.async_wait(
        [&nvmeDevice](const boost::system::error_code& errorCode) {
            if (errorCode != boost::asio::error::operation_aborted)
            {
                nvmeDevice.nvmeSlaveSocket->cancel();
            }
            else if (errorCode)
            {
                return;
            }
        });

    readResponse(nvmeDevice, mctpResponseTimer);

    if (DEBUG)
    {
        std::cout << "Sending message to read data from Drive on bus: "
                  << nvmeDevice.bus << " , rootBus: " << nvmeDevice.rootBus
                  << " device index:" << nvmeDevice.sindex << "\n";
    }

    int rc = nvmeMessageTransmit(nvmeDevice.mctp, nvmeDevice.eid, &requestMsg);

    if (rc != 0)
    {
        std::cerr << "Error sending request message to NVMe device\n";
    }
}

void pollNVMeDevices(boost::asio::io_service& io,
                     boost::asio::deadline_timer& scanTimer,
                     boost::asio::deadline_timer& mctpResponseTimer)
{
    scanTimer.expires_from_now(boost::posix_time::seconds(1));
    scanTimer.async_wait([&io, &scanTimer, &mctpResponseTimer](
                             const boost::system::error_code& errorCode) {
        if (errorCode)
        {
            std::cerr << "Error:" << errorCode.message() << "\n";
        }
        else
        {
            int index = getDeviceInfoToQuery();
            if (index >= 0)
            {
                readAndProcessNVMeSensor(mctpResponseTimer, index);
            }
        }

        pollNVMeDevices(io, scanTimer, mctpResponseTimer);
    });
}

int nvmeMessageTransmit(struct mctp* mctp, uint8_t eid,
                        struct nvme_mi_msg_request* req)
{
    uint8_t messageBuf[NVME_MI_MSG_BUFFER_SIZE] = {};

    if (req == nullptr || mctp == nullptr)
    {
        std::cerr << "Bad request\n";
        return -1;
    }

    req->header.flags |= NVME_MI_HDR_MESSAGE_TYPE_MI_COMMAND
                         << NVME_MI_HDR_FLAG_MSG_TYPE_SHIFT;
    req->header.message_type =
        NVME_MI_MESSAGE_TYPE | NVME_MI_MCTP_INTEGRITY_CHECK;

    size_t msgSize;
    uint32_t integrity;
    msgSize = NVME_MI_MSG_REQUEST_HEADER_SIZE + req->request_data_len +
              sizeof(integrity);

    if (sizeof(messageBuf) < msgSize)
    {
        return EXIT_FAILURE;
    }

    messageBuf[0] = req->header.message_type;
    messageBuf[1] = req->header.flags;
    // Reserved bytes 2-3

    messageBuf[4] = req->header.opcode;
    // reserved bytes 5-7
    messageBuf[8] = req->header.dword0 & 0xff;
    messageBuf[9] = (req->header.dword0 >> 8) & 0xff;
    messageBuf[10] = (req->header.dword0 >> 16) & 0xff;
    messageBuf[11] = (req->header.dword0 >> 24) & 0xff;

    messageBuf[12] = req->header.dword1 & 0xff;
    messageBuf[13] = (req->header.dword1 >> 8) & 0xff;
    messageBuf[14] = (req->header.dword1 >> 16) & 0xff;
    messageBuf[15] = (req->header.dword1 >> 24) & 0xff;

    std::copy(req->request_data, req->request_data + req->request_data_len,
              messageBuf +
                  static_cast<uint8_t>(NVME_MI_MSG_REQUEST_HEADER_SIZE));

    msgSize = NVME_MI_MSG_REQUEST_HEADER_SIZE + req->request_data_len;
    integrity = crc32c(messageBuf,
                       NVME_MI_MSG_REQUEST_HEADER_SIZE + req->request_data_len);
    messageBuf[msgSize] = integrity & 0xff;
    messageBuf[msgSize + 1] = (integrity >> 8) & 0xff;
    messageBuf[msgSize + 2] = (integrity >> 16) & 0xff;
    messageBuf[msgSize + 3] = (integrity >> 24) & 0xff;
    msgSize += sizeof(integrity);

    return mctp_message_tx(mctp, eid, messageBuf, msgSize);
}

int verifyIntegrity(uint8_t* msg, size_t len)
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

static double getTemperatureReading(double tempByte)
{
    double readingValue = maxReading;

    if (tempByte <= maxReading || tempByte >= minReading)
    {
        readingValue = tempByte;
    }
    else if (tempByte == 0x80 || tempByte == 0x81)
    {
        // 0x80 = No temperature data or temperature data is more the 5 s old
        // 0x81 = Temperature sensor failure
        readingValue = maxReading;
    }

    return readingValue;
}

void NVMeContext::rxMessage(uint8_t eid, void* data, void* msg, size_t len)
{
    struct nvme_mi_msg_response_header header
    {
    };

    if (data == nullptr)
    {
        std::cout << "Index not set for the NVMe drive to be updated \n";
        return;
    }

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

    if (DEBUG)
    {
        std::cout << "Eid from the received messaged: " << eid << "\n";
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

    int* index = static_cast<int*>(data);

    if (*index >= 0 && *index <= nvmeDeviceList.size())
    {
        NVMeSensor& sensorInfo = nvmeDeviceList[*index].first;
        if (DEBUG)
        {
            std::cout << "Temperature Reading: "
                      << getTemperatureReading(messageData[5])
                      << " Celsius for device at index: " << *index << "\n";
        }

        sensorInfo.updateNVMeReading(getTemperatureReading(messageData[5]));
    }
}

NVMeContext::NVMeContext(boost::asio::io_service& io, int bus, int rootBus) :
    bus(bus), rootBus(rootBus)
{
    eid = 0;
    struct mctp_binding_smbus* smbusTmp = mctp_smbus_init();

    smbus = std::unique_ptr<struct mctp_binding_smbus,
                            std::function<void(mctp_binding_smbus*)>>(
        smbusTmp, [](mctp_binding_smbus*) {
            std::cerr << "Delete libmctp raw ptr\n";
            // Call libmctp free() here
        });

    smbusTmp = nullptr;

    mctp = mctp_init();

    int r = mctp_smbus_open_bus(smbus.get(), bus, rootBus);
    if (r < 0)
    {
        std::cerr << "Error while opening the bus\n";
        return;
    }

    mctp_smbus_register_bus(smbus.get(), mctp, eid);

    mctp_set_rx_all(mctp, rxMessage, &sindex);

    nvmeSlaveSocket = std::make_shared<boost::asio::ip::tcp::socket>(io);

    nvmeSlaveSocket->assign(boost::asio::ip::tcp::v4(),
                            mctp_smbus_get_in_fd(smbus.get()));
}

NVMeSensor::NVMeSensor(sdbusplus::asio::object_server& objectServer,
                       std::shared_ptr<sdbusplus::asio::connection>& conn,
                       const std::string& sensorName,
                       std::vector<thresholds::Threshold>&& _thresholds,
                       const std::string& sensorConfiguration, size_t bus) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(_thresholds), sensorConfiguration,
           "xyz.openbmc_project.Configuration.NVMe", maxReading, minReading),
    bus(bus)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    setInitialProperties(conn);
    // setup match
    setupPowerMatch(conn);
}

NVMeSensor::~NVMeSensor()
{
}

void NVMeSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

void NVMeSensor::updateNVMeReading(const double& value)
{
    updateValue(value);
}
