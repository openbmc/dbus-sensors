#include "Inventory.hpp"
#include <phosphor-logging/lg2.hpp>

constexpr int maxAttempts = 3;

Inventory::Inventory(const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     sdbusplus::asio::object_server& objectServer,
                     const std::string& inventoryName,
                     mctp::MctpRequester& mctpRequester,
                     uint8_t eid)
    : path("/xyz/openbmc_project/inventory/" + inventoryName), mctpRequester(mctpRequester), eid(eid)
{
    (void)conn;
    assetIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Decorator.Asset");
    assetIface->register_property("PartNumber", std::string{});
    assetIface->register_property("SerialNumber", std::string{});
    assetIface->initialize();
}

void Inventory::update()
{
    // Start the chain by requesting the first property
    requestPartNumber();
}

void Inventory::requestPartNumber()
{
    int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(gpu::InventoryPropertyId::DEVICE_PART_NUMBER), requestBuffer);
    if (rc != 0)
    {
        lg2::error("Failed to encode device part number request: rc={RC}", "RC", rc);
        // Optionally, continue to next property even if this fails
        requestSerialNumber();
        return;
    }
    mctpRequester.sendRecvMsg(
        eid, requestBuffer, responseBuffer,
        [this](int sendRecvMsgResult) { handlePartNumberResponse(sendRecvMsgResult); });
}

void Inventory::handlePartNumberResponse(int sendRecvMsgResult)
{
    static int attempts = 0;
    bool success = false;

    if (sendRecvMsgResult == 0)
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        gpu::InventoryInfo info;
        int rc = gpu::decodeGetInventoryInformationResponse(
            responseBuffer, cc, reasonCode, gpu::InventoryPropertyId::DEVICE_PART_NUMBER, info);
        if (rc == 0 && cc == ocp::accelerator_management::CompletionCode::SUCCESS &&
            std::holds_alternative<std::string>(info))
        {
            std::string partNumber = std::get<std::string>(info);
            if (assetIface)
            {
                assetIface->set_property("PartNumber", partNumber);
            }
            success = true;
        }
    }

    if (!success)
    {
        attempts++;
        lg2::error("Failed to update PartNumber (attempt {ATTEMPT}/{MAX})", "ATTEMPT", attempts, "MAX", maxAttempts);
        if (attempts < maxAttempts)
        {
            requestPartNumber();
        }
        else
        {
            attempts = 0;
        }
    }
    else
    {
        attempts = 0;
    }
}

void Inventory::requestSerialNumber()
{
    int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(gpu::InventoryPropertyId::SERIAL_NUMBER), requestBuffer);
    if (rc != 0)
    {
        lg2::error("Failed to encode device serial number request: rc={RC}", "RC", rc);
        return;
    }
    mctpRequester.sendRecvMsg(
        eid, requestBuffer, responseBuffer,
        [this](int sendRecvMsgResult) { handleSerialNumberResponse(sendRecvMsgResult); });
}

void Inventory::handleSerialNumberResponse(int sendRecvMsgResult)
{
    static int attempts = 0;
    bool success = false;

    if (sendRecvMsgResult == 0)
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        gpu::InventoryInfo info;
        int rc = gpu::decodeGetInventoryInformationResponse(
            responseBuffer, cc, reasonCode, gpu::InventoryPropertyId::SERIAL_NUMBER, info);
        if (rc == 0 && cc == ocp::accelerator_management::CompletionCode::SUCCESS &&
            std::holds_alternative<std::string>(info))
        {
            std::string serialNumber = std::get<std::string>(info);
            if (assetIface)
            {
                assetIface->set_property("SerialNumber", serialNumber);
            }
            success = true;
        }
    }

    if (!success)
    {
        attempts++;
        lg2::error("Failed to update SerialNumber (attempt {ATTEMPT}/{MAX})", "ATTEMPT", attempts, "MAX", maxAttempts);
        if (attempts < maxAttempts)
        {
            requestSerialNumber();
        }
        else
        {
            attempts = 0;
        }
    }
    else
    {
        attempts = 0;
    }
} 