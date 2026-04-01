#pragma once

#include "BaseValve.hpp"
#include "ValveEvents.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Configuration/AnalogValve/FeedbackADC/client.hpp>
#include <xyz/openbmc_project/Configuration/AnalogValve/SetPointDAC/client.hpp>
#include <xyz/openbmc_project/Configuration/AnalogValve/client.hpp>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace valve
{

using AnalogConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::AnalogValve<>;
using DACConfigIntf = sdbusplus::client::xyz::openbmc_project::configuration::
    analog_valve::SetPointDAC<>;
using ADCConfigIntf = sdbusplus::client::xyz::openbmc_project::configuration::
    analog_valve::FeedbackADC<>;

namespace config
{

struct DACConfig
{
    uint64_t bus = 0;
    uint64_t address = 0;
    uint64_t index = 0;
    double offset = 0.0;
    double scaleFactor = 1.0;
};

struct ADCConfig
{
    uint64_t index = 0;
    double scaleFactor = 1.0;
};

struct AnalogConfig : public BaseConfig
{
    double maxFlowRate = 0.0;
    std::vector<double> setPointVoltageRange;
    double tolerance = 0.0;
    double openThreshold = 0.0;
    DACConfig dac;
    ADCConfig adc;
};

/** @brief Get the analog valve configuration from the Entity Manager */
auto getAnalogConfig(sdbusplus::async::context& ctx,
                     sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<std::optional<AnalogConfig>>;

} // namespace config

class AnalogValve : public BaseValve
{
  public:
    explicit AnalogValve(sdbusplus::async::context& ctx,
                         sdbusplus::message::object_path& objectPath,
                         Events& events, const LocalConfig& localConfig,
                         const config::AnalogConfig& config);

  protected:
    auto getState() const -> State override;
    auto setState(State state) -> bool override;

  private:
    /** @brief Periodically read ADC feedback and update valve position */
    auto monitorFeedbackAsync() -> sdbusplus::async::task<>;

    /** @brief Handle valve state change based on feedback voltage */
    auto handleStateChange(double voltage) -> sdbusplus::async::task<>;

    /** @brief Check if feedback voltage is within tolerance of setpoint */
    auto checkSetPointTolerance(double voltage) -> sdbusplus::async::task<>;

    /** @brief Write a voltage to the DAC device
     *  @param voltage - voltage to write
     *  @return true on success
     */
    auto writeDACVoltage(double voltage) -> bool;

    /** @brief Read the feedback voltage from the ADC device
     *  @return voltage read, or nullopt on failure
     */
    auto readADCVoltage() const -> std::optional<double>;

    /** @brief Convert a voltage to a valve position percentage
     *  @param voltage - the voltage to convert
     *  @return percentage (0-100)
     */
    auto voltageToPercent(double voltage) const -> double;

    /** @brief Find the sysfs path for the DAC IIO device */
    auto findDACSysfsPath() -> std::optional<std::string>;

    /** @brief Find the sysfs path for the ADC IIO channel */
    auto findADCSysfsPath() -> std::optional<std::string>;

    config::AnalogConfig analogConfig;
    std::string dacSysfsPath;
    std::string adcSysfsPath;
    double currentSetPointVoltage = 0.0;
    bool isOpen = false;
    std::chrono::steady_clock::time_point lastSetPointChange;
};

} // namespace valve
