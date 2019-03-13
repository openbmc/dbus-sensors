/*
// Copyright (c) 2017 Intel Corporation
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

#include <unistd.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <limits>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>

static struct nvme_mi_msg_request requestMsg = {0};

void readAndProcessNVMeSensor(NVMeSensor& sensorInfo,
                              struct NVMeContext& nvmeDevice,
                              boost::asio::io_service& io,
                              boost::asio::ip::tcp::socket& nvmeSlaveSocket,
                              boost::asio::deadline_timer& tcpResponseTimer)
{
    requestMsg = {0};
    requestMsg.header.opcode = NVME_MI_OPCODE_HEALTH_STATUS_POLL;
    requestMsg.header.dword0 = 0;
    requestMsg.header.dword1 = 0;

    tcpResponseTimer.expires_from_now(boost::posix_time::seconds(1));
    tcpResponseTimer.async_wait(
        [&nvmeSlaveSocket](const boost::system::error_code& errorCode) {
            if (errorCode != boost::asio::error::operation_aborted)
            {
                nvmeSlaveSocket.cancel();
            }
        });

    nvmeSlaveSocket.async_wait(boost::asio::ip::tcp::socket::wait_error,
                               [&tcpResponseTimer, &nvmeDevice](
                                   const boost::system::error_code& errorCode) {
                                   if (errorCode)
                                   {
                                       return;
                                   }

                                   auto rc = mctp_smbus_read(nvmeDevice.smbus);

                                   tcpResponseTimer.cancel();
                               });

    int rc = nvmeMessageTransmit(nvmeDevice.mctp, nvmeDevice.eid, &requestMsg);
}

int nvmeMessageTransmit(struct mctp* mctp, uint8_t eid,
                        struct nvme_mi_msg_request* req)
{
    uint8_t messageBuf[256] = {0};
    size_t msgSize;
    uint32_t integrity;

    if (req == NULL)
    {
        std::cerr << "Bad request\n";
        return -1;
    }

    req->header.flags |= NVME_MI_HDR_MESSAGE_TYPE_MI_COMMAND
                         << NVME_MI_HDR_FLAG_MSG_TYPE_SHIFT;
    req->header.message_type =
        NVME_MI_MESSAGE_TYPE | NVME_MI_MCTP_INTEGRITY_CHECK;

    msgSize = NVME_MI_MSG_REQUEST_HEADER_SIZE + req->request_data_len +
              sizeof(integrity);

    if (sizeof(messageBuf) < msgSize)
    {
        return EXIT_FAILURE;
    }

    messageBuf[0] = req->header.message_type;
    messageBuf[1] = req->header.flags;
    // Reserved bits 2-3

    messageBuf[4] = req->header.opcode;
    // reserved bits 5-7
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

    auto rc = mctp_message_tx(mctp, eid, messageBuf, msgSize);

    return rc;
}

static int verifyIntegrity(uint8_t* msg, size_t len)
{
    uint32_t calculateIntegrity;
    uint32_t msgIntegrity;

    if (len < NVME_MI_MSG_RESPONSE_HEADER_SIZE + sizeof(msgIntegrity))
    {
        std::cerr << "Not enough bytes for nvme header and trailer\n";
        return -1;
    }

    msgIntegrity = (msg[len - 4]) + (msg[len - 3] << 8) + (msg[len - 2] << 16) +
                   (msg[len - 1] << 24);

    calculateIntegrity = crc32c(msg, len - sizeof(msgIntegrity));
    if (msgIntegrity != calculateIntegrity)
    {
        std::cerr << "CRC mismatch. Got=" << msgIntegrity
                  << " Expected=" << calculateIntegrity << "\n";
        return -1;
    }
    return 0;
}

static int8_t getTemperatureReading(uint8_t temp_byte)
{
    int8_t readingValue = 0xFF;

    if (temp_byte <= 0x7E || temp_byte > 0xC5)
    {
        readingValue = (int8_t)temp_byte;
    }
    else if (temp_byte == 0x7F)
    {
        std::cerr << "Temperature 127 Celsius or higher"
                  << "\n";
        readingValue = (int8_t)temp_byte;
    }
    else if (temp_byte == 0x80)
    {
        readingValue = 0xFF;
    }
    else if (temp_byte == 0x81)
    {
        std::cerr << "Temperature sensor failure"
                  << "\n";
        readingValue = 0xFF;
    }
    else if (temp_byte == 0xC4)
    {
        std::cerr << "Temperature is -60 Celsius or lower"
                  << "\n";
        readingValue = 0xFF;
    }
    return readingValue;
}

void NVMeContext::rxMessage(uint8_t _eid, void* data, void* msg, size_t len)
{
    struct nvme_mi_msg_response_header header
    {
    };
    uint8_t* messageData{nullptr};
    size_t messageLength{};

    int count{0};

    if (msg == NULL)
    {
        std::cerr << "Bad message received\n";
        return;
    }

    if (len < 1)
    {
        std::cerr << "Received message not long enough\n";
        return;
    }

    messageData = (uint8_t*)msg;

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
    messageLength = len - NVME_MI_MSG_RESPONSE_HEADER_SIZE - sizeof(uint32_t);
    if (((header.flags >> NVME_MI_HDR_FLAG_MSG_TYPE_SHIFT) &
         NVME_MI_HDR_FLAG_MSG_TYPE_MASK) != NVME_MI_HDR_MESSAGE_TYPE_MI_COMMAND)
    {
        std::cerr << "Not MI type comamnd\n";
        return;
    }

    if (messageLength < 8)
    {
        std::cerr << "Got improperly sized health status poll\n";
        return;
    }

    NVMeSensor& sensorInfo = *deviceRootBusMap.find(lastQueriedDevice.first)
                                  ->second.find(lastQueriedDevice.second)
                                  ->second.first;

    sensorInfo.updateNVMeReading(getTemperatureReading(messageData[5]));
}

NVMeContext::NVMeContext(boost::asio::io_service& io, int _bus, int _rootBus) :
    bus(_bus)
{
    eid = 0;
    smbus = mctp_smbus_init();

    mctp = mctp_init();

    int r = mctp_smbus_open_bus(smbus, bus, _rootBus);
    if (r < 0)
    {
        std::cerr << "Error while opening the bus\n";
        return;
    }

    mctp_smbus_register_bus(smbus, mctp, eid);

    mctp_set_rx_all(mctp, rxMessage, NULL);
}

NVMeContext::~NVMeContext()
{
}

NVMeSensor::NVMeSensor(const std::string& path,
                       sdbusplus::asio::object_server& objectServer,
                       std::shared_ptr<sdbusplus::asio::connection>& conn,
                       boost::asio::io_service& io,
                       const std::string& sensorName,
                       std::vector<thresholds::Threshold>&& _thresholds,
                       const std::string& sensorConfiguration, size_t bus) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"), path,
           std::move(_thresholds), sensorConfiguration,
           "xyz.openbmc_project.Configuration.NVMe", maxReading, minReading),
    objServer(objectServer), inputDev(io, open(path.c_str(), O_RDONLY)),
    waitTimer(io), errCount(0), thresholdTimer(io, this), bus(bus),
    nvmeMasterSocket(io)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");
    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }

    setInitialProperties(conn);
    // setup match
    setupPowerMatch(conn);
}

NVMeSensor::~NVMeSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    objServer.remove_interface(sensorInterface);
}

void NVMeSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

void NVMeSensor::updateNVMeReading(const double& value)
{
    updateValue(value);
}
