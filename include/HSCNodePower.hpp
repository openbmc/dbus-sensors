#pragma once

#include <Thresholds.hpp>
#include <Utils.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

static constexpr uint8_t PMBUS_REVISION = 0x98;
static constexpr uint8_t MP5920_PMBUS_PWD = 0xe1;
static constexpr uint8_t MP5920_BLACKBOX_PWD = 0xf0;
static constexpr uint8_t MP5920_NVM_PWD = 0xfd;
static constexpr uint8_t MP5920_AVGTIME = 0xe2;
static constexpr uint8_t MP5920_OPERATION = 0x80;
static constexpr uint8_t PMBUS_PWD_HIGH = 0x82;
static constexpr uint8_t PMBUS_PWD_LOW = 0xc2;
static constexpr uint8_t BLACKBOX_PWD_HIGH = 0xbb;
static constexpr uint8_t BLACKBOX_PWD_LOW = 0x40;
static constexpr uint8_t NVM_PWD_HIGH = 0xc4;
static constexpr uint8_t NVM_PWD_LOW = 0xc4;
static constexpr uint8_t OPERATION_ON = 0x01;
static constexpr uint8_t PMB_CMD_READ_EIN = 0x86;

static constexpr int MP5920_VOLTAGE_m = 2450;
static constexpr int MP5920_VOLTAGE_r = 1;
static constexpr int MP5920_VOLTAGE_b = 0;

static constexpr int MP5920_CURRENT_m = 944;
static constexpr int MP5920_CURRENT_r = 2;
static constexpr int MP5920_CURRENT_b = 0;

static constexpr int MP5920_PWR_m = 9034;
static constexpr int MP5920_PWR_b = 0;
static constexpr int MP5920_PWR_r = 3;

// PMBus unit Information
struct SHSCDevInfo
{
    uint8_t u8LogicalBus;
    uint8_t u8Addr;
    uint8_t u8HSCNum;
    uint32_t u32Flags;
    uint32_t u32CmdFlags;
    uint16_t u16PrevStatusValue;
    uint16_t u16StatusValue;
    uint16_t u16StatusReadMask;
    uint8_t u8StartupCount;

    uint64_t u64CurrentEnergyCount; // Direct Value
    uint64_t u64PrevEnergyCount;    // Direct Value
    uint64_t u64CurrentRolloverCount;
    uint64_t u64PrevRolloverCount;
    uint64_t u64CurrentSampleCount;
    uint64_t u64PrevSampleCount;
    uint64_t u64AveragePower; // Real-world value
};

enum HSCType
{
    MP5920
};

struct HSC_Coefficient
{
    int current_m;
    int current_b;
    int current_r;
    int voltage_m;
    int voltage_b;
    int voltage_r;
    int power_m;
    int power_b;
    int power_r;
};

class HSCNodePower
{
  public:
    HSCNodePower(boost::asio::io_service& io,
                 sdbusplus::asio::object_server& objectServer,
                 std::shared_ptr<sdbusplus::asio::connection>& conn);

    ~HSCNodePower();

    void start(HSCType type, int busId, int slaveAddr);

    int get_average_power(SHSCDevInfo*);
    void initCoefficient();
    int pingHSCDevice();
    void configHSCDevice();
    void initMP5920();

    void pollHSCChipRegs();
    void updateValue(uint64_t newValue);

    int getHSCRegsInfoByte(uint8_t regs, uint8_t* pu8data);
    int getHSCRegsInfoBytes(uint8_t regs, uint8_t length, uint8_t* pu8data);
    int setHSCRegsInfoBytes(uint8_t regs, uint8_t length, uint8_t* pu8data);
    int setHSCRegsInfoByte(uint8_t regs, uint8_t length, uint8_t pu8data);

  private:
    std::shared_ptr<sdbusplus::asio::connection> mDbusConn;

    int mBusId;
    int mSlaveAddr;
    HSCType mType;
    uint64_t mValue;

    boost::asio::deadline_timer mPollTimer;
    sdbusplus::asio::object_server& mObjServer;
    std::shared_ptr<sdbusplus::asio::dbus_interface> mItemIface;
    std::shared_ptr<sdbusplus::asio::connection> mConn;

    HSC_Coefficient mHSCCoefficient;
    SHSCDevInfo g_sHSCDevInfo;
};