#pragma once

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace phosphor
{
namespace smbus
{

class Smbus
{
  public:
    Smbus(){};

    int open_i2c_dev(int i2cbus, char* filename, size_t size, int quiet);

    int smbusInit(int smbus_num);

    void smbusClose(int smbus_num);

    int SendSmbusRWBlockCmdRAW(int smbus_num, int8_t device_addr,
                               uint8_t* tx_data, uint8_t tx_len,
                               uint8_t* rsp_data);

};

} // namespace smbus

namespace nvme
{

class Nvme
{

    /**
     * Structure for keeping nvme configure data required by nvme monitoring
     */
    struct NVMeConfig
    {
        std::string index;
        uint8_t busID;
        std::string faultLedGroupPath;
        uint8_t presentPin;
        uint8_t pwrGoodPin;
        std::string locateLedControllerBusName;
        std::string locateLedControllerPath;
        std::string locateLedGroupPath;
        int8_t criticalHigh;
        int8_t criticalLow;
        int8_t maxValue;
        int8_t minValue;
        int8_t warningHigh;
        int8_t warningLow;
    };

    /**
     * Structure for keeping nvme data required by nvme monitoring
     */
	struct NVMeData
	{
		bool present;              /* Whether or not the nvme is present  */
		std::string vendor;        /* The nvme manufacturer  */
		std::string serialNumber;  /* The nvme serial number  */
		std::string smartWarnings; /* Indicates smart warnings for the state  */
		std::string statusFlags;   /* Indicates the status of the drives  */
		std::string
			driveLifeUsed;  /* A vendor specific estimate of the percentage  */
		int8_t sensorValue; /* Sensor value, if sensor value didn't be
                               update, means sensor failure, default set to
                               129(0x81) accroding to NVMe-MI SPEC*/
	};

    /** @brief Setup polling timer in a sd event loop and attach to D-Bus
     *         event loop.
     */
    void run();
    /** @brief Set up initial configuration value of NVMe */
    void init();
    /** @brief Monitor NVMe drives every one second  */
    void read();

    void createNVMeInventory();

    /** @brief read and update NVME data to dbus */
    void readNvmeData(NVMeConfig& config);
};
} // namespace nvme

} // namespace phosphor
