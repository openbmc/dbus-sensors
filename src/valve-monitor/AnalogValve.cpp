#include "AnalogValve.hpp"

#include "BaseValve.hpp"
#include "EntityManagerInterface.hpp"
#include "LocalConfig.hpp"
#include "SystemdInterface.hpp"
#include "ValveEvents.hpp"

#include <glob.h>

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace valve
{

PHOSPHOR_LOG2_USING;

namespace config
{

auto getAnalogConfig(sdbusplus::async::context& ctx,
                     sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<std::optional<AnalogConfig>>
{
    AnalogConfig config = {};

    // Fetch main AnalogValve properties
    try
    {
        auto properties =
            co_await AnalogConfigIntf(ctx)
                .service(entity_manager::EntityManagerInterface::serviceName)
                .path(objectPath.str)
                .properties();

        config.name = properties.name;
        config.maxFlowRate = properties.max_flow_rate;
        config.setPointVoltageRange = properties.set_point_voltage_range;
        config.tolerance = properties.tolerance;
        config.openThreshold = properties.open_threshold;
    }
    catch (const std::exception& e)
    {
        error("Failed to get AnalogValve properties for {PATH}: {ERR}", "PATH",
              objectPath.str, "ERR", e);
        co_return std::nullopt;
    }

    if (config.setPointVoltageRange.size() != 2)
    {
        error("Invalid SetPointVoltageRange size for {NAME}", "NAME",
              config.name);
        co_return std::nullopt;
    }

    // Fetch SetPointDAC sub-interface properties
    try
    {
        auto dacProperties =
            co_await DACConfigIntf(ctx)
                .service(entity_manager::EntityManagerInterface::serviceName)
                .path(objectPath.str)
                .properties();

        config.dac.bus = dacProperties.bus;
        config.dac.address = dacProperties.address;
        config.dac.index = dacProperties.index;
        config.dac.offset = dacProperties.offset;
        config.dac.scaleFactor = dacProperties.scale_factor;
    }
    catch (const std::exception& e)
    {
        error("Failed to get SetPointDAC properties for {PATH}: {ERR}", "PATH",
              objectPath.str, "ERR", e);
        co_return std::nullopt;
    }

    // Fetch FeedbackADC sub-interface properties
    try
    {
        auto adcProperties =
            co_await ADCConfigIntf(ctx)
                .service(entity_manager::EntityManagerInterface::serviceName)
                .path(objectPath.str)
                .properties();

        config.adc.index = adcProperties.index;
        config.adc.scaleFactor = adcProperties.scale_factor;
    }
    catch (const std::exception& e)
    {
        error("Failed to get FeedbackADC properties for {PATH}: {ERR}", "PATH",
              objectPath.str, "ERR", e);
        co_return std::nullopt;
    }

    info("AnalogValve config: {NAME} maxFlowRate={FLOW_RATE} "
         "voltageRange=[{V_MIN},{V_MAX}] tolerance={TOL} "
         "openThreshold={OPEN_THR} "
         "DAC(bus={BUS},addr={ADDR},idx={IDX},offset={OFFSET},scale={SCALE}) "
         "ADC(idx={ADC_IDX},scale={ADC_SCALE})",
         "NAME", config.name, "FLOW_RATE", config.maxFlowRate, "V_MIN",
         config.setPointVoltageRange[0], "V_MAX",
         config.setPointVoltageRange[1], "TOL", config.tolerance, "OPEN_THR",
         config.openThreshold, "BUS", config.dac.bus, "ADDR",
         config.dac.address, "IDX", config.dac.index, "OFFSET",
         config.dac.offset, "SCALE", config.dac.scaleFactor, "ADC_IDX",
         config.adc.index, "ADC_SCALE", config.adc.scaleFactor);

    co_return config;
}

} // namespace config

namespace fs = std::filesystem;

static constexpr auto pollIntervalMs = 1000;

// iio-hwmon sysfs nodes (in*_input) report values in millivolts
static constexpr double hwmonMillivoltScale = 1000.0;

AnalogValve::AnalogValve(sdbusplus::async::context& ctx,
                         sdbusplus::message::object_path& objectPath,
                         Events& events, const LocalConfig& localConfig,
                         const config::AnalogConfig& config) :
    BaseValve(ctx, objectPath, events, localConfig, config),
    analogConfig(config)
{
    auto dacPath = findDACSysfsPath();
    if (dacPath.has_value())
    {
        dacSysfsPath = dacPath.value();
        info("DAC sysfs path for {VALVE}: {PATH}", "VALVE", baseConfig.name,
             "PATH", dacSysfsPath);
    }
    else
    {
        error("Failed to find DAC sysfs path for {VALVE}", "VALVE",
              baseConfig.name);
    }

    auto adcPath = findADCSysfsPath();
    if (adcPath.has_value())
    {
        adcSysfsPath = adcPath.value();
        info("ADC sysfs path for {VALVE}: {PATH}", "VALVE", baseConfig.name,
             "PATH", adcSysfsPath);
    }
    else
    {
        error("Failed to find ADC sysfs path for {VALVE}", "VALVE",
              baseConfig.name);
    }

    ctx.spawn(monitorFeedbackAsync());
}

auto AnalogValve::getState() const -> State
{
    debug("Getting {VALVE} state", "VALVE", baseConfig.name);
    return isOpen ? State::Open : State::Close;
}

auto AnalogValve::setState(State state) -> bool
{
    debug("Setting {VALVE} to {STATE}", "VALVE", baseConfig.name, "STATE",
          convertStateToString(state));

    if ((!isOpen && state == State::Close) || (isOpen && state == State::Open))
    {
        info("Ignoring, as new state {STATE} matches the current state",
             "STATE", convertStateToString(state));
        return true;
    }

    double targetVoltage = analogConfig.setPointVoltageRange[0];

    if (state == State::Open)
    {
        double flowRate = localConfig.getFlowRate(baseConfig.name);
        double minV = analogConfig.setPointVoltageRange[0];
        double maxV = analogConfig.setPointVoltageRange[1];

        // Map flow rate (0 - maxFlowRate) to voltage range (minV - maxV)
        targetVoltage = minV +
                        (flowRate / analogConfig.maxFlowRate) * (maxV - minV);
        targetVoltage = std::clamp(targetVoltage, minV, maxV);

        info(
            "Mapped flow rate {RATE} / {MAX_RATE} LPM to voltage {VOLTAGE} for {VALVE}",
            "RATE", flowRate, "MAX_RATE", analogConfig.maxFlowRate, "VOLTAGE",
            targetVoltage, "VALVE", baseConfig.name);
    }

    if (!writeDACVoltage(targetVoltage))
    {
        error("Failed to set {VALVE} to {STATE}", "VALVE", baseConfig.name,
              "STATE", convertStateToString(state));
        return false;
    }

    currentSetPointVoltage = targetVoltage;
    lastSetPointChange = std::chrono::steady_clock::now();
    info("Successfully set {VALVE} to {STATE}", "VALVE", baseConfig.name,
         "STATE", convertStateToString(state));
    return true;
}

auto AnalogValve::handleStateChange(double voltage) -> sdbusplus::async::task<>
{
    bool wasOpen = isOpen;

    // Apply hysteresis on the open transition to prevent oscillation
    // when voltage fluctuates near the threshold.
    // Operating range is 2-10V, so voltage won't drop below 2V.
    double hysteresis =
        analogConfig.openThreshold * (analogConfig.tolerance / 100.0);
    if (isOpen)
    {
        // Valve is currently open. Close when voltage drops to or
        // below the threshold. No hysteresis needed here as the
        // valve is essentially shut at this voltage.
        isOpen = (voltage > analogConfig.openThreshold);
    }
    else
    {
        // Valve is currently closed. Only mark as open when voltage
        // rises above (threshold + hysteresis) to confirm the valve
        // is truly opening and not just noise around the threshold.
        isOpen = (voltage > analogConfig.openThreshold + hysteresis);
    }

    if (wasOpen == isOpen)
    {
        co_return;
    }

    co_await events.generateValveEvent(inventoryPath, isOpen);

    /** @brief Valve state to systemd target service map */
    static constexpr std::array<std::pair<State, std::string_view>, 2>
        valveActionTargets = {{
            {State::Open, "xyz.openbmc_project.valve.open@"},
            {State::Close, "xyz.openbmc_project.valve.close@"},
        }};

    auto newState = isOpen ? State::Open : State::Close;
    for (const auto& [state, serviceSuffix] : valveActionTargets)
    {
        if (state == newState)
        {
            auto target = std::string(serviceSuffix) + baseConfig.name +
                          ".service";
            debug("Starting systemd target {TARGET}", "TARGET", target);
            co_await systemd::SystemdInterface::startUnit(ctx, target);
            break;
        }
    }
}

auto AnalogValve::checkSetPointTolerance(double voltage)
    -> sdbusplus::async::task<>
{
    // Skip if no setpoint has been written yet
    if (lastSetPointChange == std::chrono::steady_clock::time_point{})
    {
        co_return;
    }

    // Skip during the settling period after a set-point change,
    // as the valve takes time to physically reach the target position.
    static constexpr auto settlingPeriod = std::chrono::seconds(90);
    auto elapsed = std::chrono::steady_clock::now() - lastSetPointChange;

    if (elapsed < settlingPeriod)
    {
        co_return;
    }

    double percentError = std::abs(voltage - currentSetPointVoltage) /
                          std::abs(currentSetPointVoltage);
    bool outOfTolerance = percentError > analogConfig.tolerance / 100.0;

    debug(
        "Valve tolerance: voltage={VOLTAGE}, setpoint={SETPOINT}, "
        "percentError={PERCENT_ERROR}%, percentTolerance={PERCENT_TOLERANCE}%",
        "VOLTAGE", voltage, "SETPOINT", currentSetPointVoltage, "PERCENT_ERROR",
        percentError * 100.0, "PERCENT_TOLERANCE", analogConfig.tolerance);

    co_await events.handleValveSetPointWarning(inventoryPath, outOfTolerance);
}

auto AnalogValve::monitorFeedbackAsync() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        co_await sdbusplus::async::sleep_for(
            ctx, std::chrono::milliseconds(pollIntervalMs));

        auto voltage = readADCVoltage();
        if (!voltage.has_value())
        {
            continue;
        }

        co_await handleStateChange(voltage.value());

        auto percent = voltageToPercent(voltage.value());
        auto newValue = static_cast<int>(std::round(percent));
        newValue = std::clamp(newValue, 0, 100);

        if (newValue != value())
        {
            debug("Updating valve {VALVE} to {VALUE}%", "VALVE",
                  baseConfig.name, "VALUE", newValue);
            value(newValue);
        }

        co_await checkSetPointTolerance(voltage.value());
    }
}

auto AnalogValve::writeDACVoltage(double voltage) -> bool
{
    if (dacSysfsPath.empty())
    {
        error("DAC sysfs path not available for {VALVE}", "VALVE",
              baseConfig.name);
        return false;
    }

    // Convert voltage to raw DAC value: raw = (voltage - offset) / scaleFactor
    auto rawValue = static_cast<int>(std::round(
        (voltage - analogConfig.dac.offset) / analogConfig.dac.scaleFactor));

    try
    {
        std::ofstream dacFile(dacSysfsPath);
        if (!dacFile.is_open())
        {
            error("Failed to open DAC sysfs {PATH}", "PATH", dacSysfsPath);
            return false;
        }
        dacFile << rawValue;
        dacFile.flush();

        if (dacFile.fail())
        {
            error("Failed to write to DAC sysfs {PATH}", "PATH", dacSysfsPath);
            return false;
        }

        debug("Wrote DAC raw={RAW} (voltage={VOLTAGE}) for {VALVE}", "RAW",
              rawValue, "VOLTAGE", voltage, "VALVE", baseConfig.name);
        return true;
    }
    catch (const std::exception& e)
    {
        error("Failed to write DAC for {VALVE}: {ERR}", "VALVE",
              baseConfig.name, "ERR", e);
        return false;
    }
}

auto AnalogValve::readADCVoltage() const -> std::optional<double>
{
    if (adcSysfsPath.empty())
    {
        error("ADC sysfs path not available for {VALVE}", "VALVE",
              baseConfig.name);
        return std::nullopt;
    }

    try
    {
        std::ifstream adcFile(adcSysfsPath);
        if (!adcFile.is_open())
        {
            error("Failed to open ADC sysfs {PATH}", "PATH", adcSysfsPath);
            return std::nullopt;
        }

        int rawValue = 0;
        adcFile >> rawValue;

        if (adcFile.fail())
        {
            error("Failed to read ADC sysfs {PATH}", "PATH", adcSysfsPath);
            return std::nullopt;
        }

        // Convert millivolt reading to voltage, then apply EM scale factor
        double voltage =
            (rawValue / hwmonMillivoltScale) / analogConfig.adc.scaleFactor;

        debug("Read ADC raw={RAW} (voltage={VOLTAGE}) for {VALVE}", "RAW",
              rawValue, "VOLTAGE", voltage, "VALVE", baseConfig.name);

        return voltage;
    }
    catch (const std::exception& e)
    {
        error("Failed to read ADC for {VALVE}: {ERR}", "VALVE", baseConfig.name,
              "ERR", e);
        return std::nullopt;
    }
}

auto AnalogValve::voltageToPercent(double voltage) const -> double
{
    double minV = analogConfig.setPointVoltageRange[0];
    double maxV = analogConfig.setPointVoltageRange[1];

    if (maxV <= minV)
    {
        return 0.0;
    }

    return ((voltage - minV) / (maxV - minV)) * 100.0;
}

auto AnalogValve::findDACSysfsPath() -> std::optional<std::string>
{
    // DAC is an I2C device, look for its IIO device under
    // /sys/bus/i2c/devices/<bus>-<addr>/iio:device*/out_voltage<index>_raw
    auto i2cDevPath =
        std::format("/sys/bus/i2c/devices/{}-{:04x}", analogConfig.dac.bus,
                    analogConfig.dac.address);

    if (!fs::exists(i2cDevPath))
    {
        error("I2C device path {PATH} does not exist", "PATH", i2cDevPath);
        return std::nullopt;
    }

    for (const auto& entry : fs::directory_iterator(i2cDevPath))
    {
        auto dirName = entry.path().filename().string();
        if (dirName.starts_with("iio:device"))
        {
            auto outPath =
                std::format("{}/out_voltage{}_raw", entry.path().string(),
                            analogConfig.dac.index);
            if (fs::exists(outPath))
            {
                return outPath;
            }
        }
    }

    error("Failed to find IIO DAC device at {PATH}", "PATH", i2cDevPath);
    return std::nullopt;
}

auto AnalogValve::findADCSysfsPath() -> std::optional<std::string>
{
    std::string pattern = "/sys/class/hwmon/hwmon*/in" +
                          std::to_string(analogConfig.adc.index) + "_input";
    glob_t result{};
    if (glob(pattern.c_str(), GLOB_NOSORT, nullptr, &result) != 0 ||
        result.gl_pathc == 0)
    {
        globfree(&result);
        error("Failed to find IIO ADC channel for index {IDX}", "IDX",
              analogConfig.adc.index);
        return std::nullopt;
    }

    std::string path = result.gl_pathv[0];
    globfree(&result);

    std::ifstream ifs(path);
    std::string val;
    if (std::getline(ifs, val) && !val.empty())
    {
        debug("ADC iio-hwmon sysfs node probe OK: path={PATH}", "PATH", path);
        return fs::canonical(path).string();
    }

    error("Failed to read ADC iio-hwmon sysfs node: {PATH}", "PATH", path);
    return std::nullopt;
}

} // namespace valve
