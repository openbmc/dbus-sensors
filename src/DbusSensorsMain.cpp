
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
        return adcSensorMain();
    }
#endif
#ifdef CABLEMONITOR
    if (programName.ends_with("cablemonitor"))
    {
        return cable::cableMonitorMain();
    }
#endif
#ifdef EXITAIRTEMP
    if (programName.ends_with("exitairtempsensor"))
    {
        return exitAirTempSensorMain();
    }
#endif
#ifdef INTELCPU
    if (programName.ends_with("intelcpusensor"))
    {
        return intelCPUSensorMain();
    }
#endif
#ifdef EXTERNAL
    if (programName.ends_with("externalsensor"))
    {
        return externalSensorMain();
    }
#endif
#ifdef FAN
    if (programName.ends_with("fansensor"))
    {
        return fanSensorMain();
    }
#endif
#ifdef HWMONTEMP
    if (programName.ends_with("hwmontempsensor"))
    {
        return hwmonTempSensorMain();
    }
#endif
#ifdef INTRUSION
    if (programName.ends_with("intrusionsensor"))
    {
        return intrusionSensorMain();
    }
#endif
#ifdef IPMB
    if (programName.ends_with("ipmbsensor"))
    {
        return ipmbSensorMain();
    }
#endif
#ifdef LEAKDETECTOR
    if (programName.ends_with("leakdetector"))
    {
        return leak::leakDetectionManagerMain();
    }
#endif
#ifdef MCTP
    if (programName.ends_with("mctpreactor"))
    {
        return mctpReactorMain();
    }
#endif
#ifdef MCU
    if (programName.ends_with("mcutempsensor"))
    {
        return mcuTempSensorMain();
    }
#endif
#ifdef NVDIAGPU
    if (programName.ends_with("nvidiagpusensor"))
    {
        return nvidiaGpuSensorMain();
    }
#endif
#ifdef NVME
    if (programName.ends_with("nvmesensor"))
    {
        return nvmeSensorMain();
    }
#endif
#ifdef PSU
    if (programName.ends_with("psusensor"))
    {
        return psuSensorMain();
    }
#endif
#ifdef SMBPBI
    if (programName.ends_with("smbpbisensor"))
    {
        return smbpbiSensorMain();
    }
#endif

    return 0;
}
