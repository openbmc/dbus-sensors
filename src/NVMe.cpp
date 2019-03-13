#include "NVMeDeviceList.hpp"

std::vector<
    std::pair<std::unique_ptr<NVMeSensor>, std::shared_ptr<struct NVMeContext>>>
    nvmeDeviceList = {};