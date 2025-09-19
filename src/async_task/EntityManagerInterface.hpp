#pragma once

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <functional>
#include <string>
#include <vector>

namespace entity_manager
{

class EntityManagerInterface
{
  public:
    using Callback_t = std::function<void(
        const sdbusplus::message::object_path&, const std::string&)>;
    using interface_list_t = std::vector<std::string>;
    static constexpr auto serviceName = "xyz.openbmc_project.EntityManager";

    EntityManagerInterface() = delete;

    EntityManagerInterface(sdbusplus::async::context& ctx,
                           const interface_list_t& interfaceNames,
                           Callback_t addedCallback,
                           Callback_t removedCallback);

    /** Get the inventory info from Entity Manager */
    auto handleInventoryGet() -> sdbusplus::async::task<>;

  private:
    /** @brief Handle async inventory add from Entity Manager */
    auto handleInventoryAdded() -> sdbusplus::async::task<>;

    /** @brief Handle async inventory remove from Entity Manager */
    auto handleInventoryRemoved() -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;
    interface_list_t interfaceNames;
    Callback_t addedCallback;
    Callback_t removedCallback;
};

} // namespace entity_manager
