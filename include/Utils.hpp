#pragma once
#include <boost/container/flat_map.hpp>
#include <experimental/filesystem>
#include <iostream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/message/types.hpp>

const constexpr char* jsonStore = "/var/configuration/flattened.json";
const constexpr char* inventoryPath = "/xyz/openbmc_project/inventory";
const constexpr char* entityManagerName = "xyz.openbmc_project.EntityManager";
const std::regex illegalDbusRegex("[^A-Za-z0-9_]");

using BasicVariantType =
    sdbusplus::message::variant<std::string, int64_t, uint64_t, double, int32_t,
                                uint32_t, int16_t, uint16_t, uint8_t, bool>;

using ManagedObjectType = boost::container::flat_map<
    sdbusplus::message::object_path,
    boost::container::flat_map<
        std::string,
        boost::container::flat_map<std::string, BasicVariantType>>>;
using SensorData = boost::container::flat_map<
    std::string, boost::container::flat_map<std::string, BasicVariantType>>;

bool findFiles(const std::experimental::filesystem::path dirPath,
               const std::string& matchString,
               std::vector<std::experimental::filesystem::path>& foundPaths,
               unsigned int symlinkDepth = 1);
bool isPowerOn(const std::shared_ptr<sdbusplus::asio::connection>& conn);
bool getSensorConfiguration(
    const std::string& type,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    ManagedObjectType& resp, bool useCache = false);