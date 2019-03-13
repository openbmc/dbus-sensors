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

#include <unistd.h>

#include <NVMeSensor.hpp>
// #include <boost/algorithm/string/predicate.hpp>
// #include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <limits>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>

void readAndProcessNVMeSensor(NVMeSensor& sensorInfo,
                              struct nvmeContext& nvmeDevice,
                              boost::asio::io_service& io,
                              boost::asio::ip::tcp::socket& nvmeSlaveSocket,
                              boost::asio::deadline_timer& tcpResponseTimer)
{
    struct nvme_mi_msg_request requestMsg = {0};
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
    uint8_t message_buf[256] = {0};
    size_t msg_size;
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

    msg_size = NVME_MI_MSG_REQUEST_HEADER_SIZE + req->request_data_len +
               sizeof(integrity);

    if (sizeof(message_buf) < msg_size)
    {
        return EXIT_FAILURE;
    }

    message_buf[0] = req->header.message_type;
    message_buf[1] = req->header.flags;
    // Reserved bits 2-3

    message_buf[4] = req->header.opcode;
    // reserved bits 5-7
    message_buf[8] = req->header.dword0 & 0xff;
    message_buf[9] = (req->header.dword0 >> 8) & 0xff;
    message_buf[10] = (req->header.dword0 >> 16) & 0xff;
    message_buf[11] = (req->header.dword0 >> 24) & 0xff;

    message_buf[12] = req->header.dword1 & 0xff;
    message_buf[13] = (req->header.dword1 >> 8) & 0xff;
    message_buf[14] = (req->header.dword1 >> 16) & 0xff;
    message_buf[15] = (req->header.dword1 >> 24) & 0xff;

    memcpy(message_buf + NVME_MI_MSG_REQUEST_HEADER_SIZE, req->request_data,
           req->request_data_len);
    msg_size = NVME_MI_MSG_REQUEST_HEADER_SIZE + req->request_data_len;
    integrity = crc32c(message_buf,
                       NVME_MI_MSG_REQUEST_HEADER_SIZE + req->request_data_len);
    message_buf[msg_size] = integrity & 0xff;
    message_buf[msg_size + 1] = (integrity >> 8) & 0xff;
    message_buf[msg_size + 2] = (integrity >> 16) & 0xff;
    message_buf[msg_size + 3] = (integrity >> 24) & 0xff;
    msg_size += sizeof(integrity);

    auto rc = mctp_message_tx(mctp, eid, message_buf, msg_size);

    return rc;
}

static int verifyIntegrity(uint8_t* msg, size_t len)
{
    uint32_t calc_integrity;
    uint32_t msg_integrity;

    if (len < NVME_MI_MSG_RESPONSE_HEADER_SIZE + sizeof(msg_integrity))
    {
        std::cerr << "Not enough bytes for nvme header and trailer\n";
        return -1;
    }

    msg_integrity = (msg[len - 4]) + (msg[len - 3] << 8) +
                    (msg[len - 2] << 16) + (msg[len - 1] << 24);

    calc_integrity = crc32c(msg, len - sizeof(msg_integrity));
    if (msg_integrity != calc_integrity)
    {
        std::cerr << "CRC mismatch. Got=" << msg_integrity
                  << " Expected=" << calc_integrity << "\n";
        return -1;
    }
    return 0;
}

static int8_t getTemperatureReading(uint8_t temp_byte)
{
    int8_t readingValue = 0xFF;

    if (temp_byte <= 0x7E || temp_byte > 0xC5)
    {
        std::cerr << "Temperature " << static_cast<int>(temp_byte)
                  << " Celsius\n";
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
        std::cerr << "No temperature data available"
                  << "\n";
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

void nvmeContext::rxMessage(uint8_t _eid, void* data, void* msg, size_t len)
{
    struct nvme_mi_msg_response_header header;
    uint8_t* message_data;
    size_t message_len;

    int count;

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

    message_data = (uint8_t*)msg;

    if ((*message_data & NVME_MI_MESSAGE_TYPE_MASK) != NVME_MI_MESSAGE_TYPE)
    {
        std::cerr << "Got unknown type message_type="
                  << (*message_data & NVME_MI_MESSAGE_TYPE_MASK) << "\n";
        return;
    }

    if (len < NVME_MI_MSG_RESPONSE_HEADER_SIZE + sizeof(uint32_t))
    {
        std::cerr << "Not enough bytes for NVMe header and trailer\n";
        return;
    }

    if (verifyIntegrity(message_data, len) != 0)
    {
        std::cerr << "Verification of message integrity failed\n";
        return;
    }

    header.message_type = message_data[0];
    header.flags = message_data[1];
    header.status = message_data[4];

    if (header.status == NVME_MI_HDR_STATUS_MORE_PROCESSING_REQUIRED)
    {
        return;
    }

    if (header.status != NVME_MI_HDR_STATUS_SUCCESS)
    {
        std::cerr << "Command failed with status= " << header.status << "\n";
        return;
    }

    message_data += NVME_MI_MSG_RESPONSE_HEADER_SIZE;
    message_len = len - NVME_MI_MSG_RESPONSE_HEADER_SIZE - sizeof(uint32_t);
    if (((header.flags >> NVME_MI_HDR_FLAG_MSG_TYPE_SHIFT) &
         NVME_MI_HDR_FLAG_MSG_TYPE_MASK) != NVME_MI_HDR_MESSAGE_TYPE_MI_COMMAND)
    {
        std::cerr << "Not MI type comamnd\n";
        return;
    }

    if (message_len < 8)
    {
        std::cerr << "Got improperly sized health status poll\n";
        return;
    }

    NVMeSensor& sensorInfo = *deviceRootBusMap.find(lastQueriedDevice.first)
                                  ->second.find(lastQueriedDevice.second)
                                  ->second.first;

    sensorInfo.updateNVMeReading(getTemperatureReading(message_data[5]));
}