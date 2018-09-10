#pragma once
#include <boost/container/flat_map.hpp>
#include <experimental/filesystem>
#include <iostream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/message/types.hpp>

const constexpr char* JSON_STORE = "/var/configuration/flattened.json";
const constexpr char* INVENTORY_PATH = "/xyz/openbmc_project/inventory";
const constexpr char* ENTITY_MANAGER_NAME = "xyz.openbmc_project.EntityManager";
const std::regex ILLEGAL_DBUS_REGEX("[^A-Za-z0-9_]");

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

bool find_files(const std::experimental::filesystem::path dir_path,
                const std::string& match_string,
                std::vector<std::experimental::filesystem::path>& found_paths,
                unsigned int symlink_depth = 1);
bool isPowerOn(const std::shared_ptr<sdbusplus::asio::connection>& conn);
bool getSensorConfiguration(
    const std::string& type,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    ManagedObjectType& resp, bool useCache = false);