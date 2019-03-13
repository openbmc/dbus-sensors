#pragma once

#include "NVMeSensor.hpp"

extern std::vector<
    std::pair<std::unique_ptr<NVMeSensor>, std::shared_ptr<struct NVMeContext>>>
    nvmeDeviceList;
