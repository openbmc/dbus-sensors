#ifdef ADC
#include "adc/ADCSensor.hpp"
#endif
#ifdef CABLEMONITOR
#include "cable-monitor/CableMonitor.hpp"
#endif
#ifdef EXITAIRTEMP
#include "exit-air/ExitAirTempSensor.hpp"
#endif
#ifdef EXTERNAL
#include "external/ExternalSensor.hpp"
#endif
#ifdef FAN
#include "fan/FanSensor.hpp"
#endif
#ifdef HWMONTEMP
#include "hwmon-temp/HwmonTempSensor.hpp"
#endif
#ifdef INTELCPU
#include "intel-cpu/IntelCPUSensor.hpp"
#endif
#ifdef INTRUSION
#include "intrusion/ChassisIntrusionSensor.hpp"
#endif
#ifdef IPMB
#include "ipmb/IpmbSensor.hpp"
#endif
#ifdef LEAKDETECTOR
#include "leakdetector/LeakDetectionManager.hpp"
#endif
#ifdef MCTP
#include "mctp/MCTPReactor.hpp"
#endif
#ifdef MCU
#include "mcu/MCUTempSensor.hpp"
#endif
#ifdef NVDIAGPU
#include "nvidia-gpu/NvidiaGpuSensor.hpp"
#endif
#ifdef NVME
#include "nvme/NVMeSensor.hpp"
#endif
#ifdef PSU
#include "psu/PSUSensor.hpp"
#endif
#ifdef SMBPBI
#include "smbpbi/SmbpbiSensor.hpp"
#endif

#include <phosphor-logging/lg2.hpp>

#include <string>

int main(int argc, char* argv[])
{
    if (argc < 1)
    {
        lg2::error("No program name provided");
        return 1;
    }
    std::string programName = argv[0];
#ifdef ADC
    if (programName.ends_with("adcsensor"))
    {
        return ADCSensorMain();
    }
#endif
#ifdef CABLEMONITOR
    if (programName.ends_with("cablemonitor"))
    {
        return cable::CableMonitorMain();
    }
#endif
#ifdef EXITAIRTEMP
    if (programName.ends_with("exitairtempsensor"))
    {
        return ExitAirTempSensorMain();
    }
#endif
#ifdef INTELCPU
    if (programName.ends_with("intelcpusensor"))
    {
        return IntelCPUSensorMain();
    }
#endif
#ifdef EXTERNAL
    if (programName.ends_with("externalsensor"))
    {
        return ExternalSensorMain();
    }
#endif
#ifdef FAN
    if (programName.ends_with("fansensor"))
    {
        return FanSensorMain();
    }
#endif
#ifdef HWMONTEMP
    if (programName.ends_with("hwmontempsensor"))
    {
        return HwmonTempSensorMain();
    }
#endif
#ifdef INTRUSION
    if (programName.ends_with("intrusionsensor"))
    {
        return IntrusionSensorMain();
    }
#endif
#ifdef IPMB
    if (programName.ends_with("ipmbsensor"))
    {
        return IpmbSensorMain();
    }
#endif
#ifdef LEAKDETECTOR
    if (programName.ends_with("leakdetector"))
    {
        return leak::LeakDetectionManagerMain();
    }
#endif
#ifdef MCTP
    if (programName.ends_with("mctpreactor"))
    {
        return MCTPReactorMain();
    }
#endif
#ifdef MCU
    if (programName.ends_with("mcutempsensor"))
    {
        return MCUTempSensorMain();
    }
#endif
#ifdef NVDIAGPU
    if (programName.ends_with("nvidiagpusensor"))
    {
        return NvidiaGpuSensorMain();
    }
#endif
#ifdef NVME
    if (programName.ends_with("nvmesensor"))
    {
        return NVMeSensorMain();
    }
#endif
#ifdef PSU
    if (programName.ends_with("psusensor"))
    {
        return PSUSensorMain();
    }
#endif
#ifdef SMBPBI
    if (programName.ends_with("smbpbisensor"))
    {
        return SmbpbiSensorMain();
    }
#endif
    lg2::error("Program not supported");
    return 1;
}
