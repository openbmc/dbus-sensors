#pragma once

#include <Thresholds.hpp>
#include <Utils.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

static constexpr uint8_t pmbusRevision = 0x98;
static constexpr uint8_t mp5920PmbusPwd = 0xe1;
static constexpr uint8_t mp5920BlackboxPwd = 0xf0;
static constexpr uint8_t mp5920NvmPwd = 0xfd;
static constexpr uint8_t mp5920Avgtime = 0xe2;
static constexpr uint8_t mp5920Operation = 0x80;
static constexpr uint8_t pmbusPwdHigh = 0x82;
static constexpr uint8_t pmbusPwdLow = 0xc2;
static constexpr uint8_t blackboxPwdHigh = 0xbb;
static constexpr uint8_t blackboxPwdLow = 0x40;
static constexpr uint8_t nvmPwdHigh = 0xc4;
static constexpr uint8_t nvmPwdLow = 0xc4;
static constexpr uint8_t operationOn = 0x01;
static constexpr uint8_t pmbCmdReadEin = 0x86;

static constexpr int mp5920VoltageM = 2450;
static constexpr int mp5920VoltageR = 1;
static constexpr int mp5920VoltageB = 0;

static constexpr int mp5920CurrentM = 944;
static constexpr int mp5920CurrentR = 2;
static constexpr int mp5920CurrentB = 0;

static constexpr int mp5920PwrM = 9034;
static constexpr int mp5920PwrB = 0;
static constexpr int mp5920PwrR = 3;

// PMBus unit Information
struct SHSCDevInfo {
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

enum HSCType { MP5920 };

struct HSC_Coefficient {
  int currentM;
  int currentB;
  int currentR;
  int voltageM;
  int voltageB;
  int voltageR;
  int powerM;
  int powerB;
  int powerR;
};

class HSCNodePowerSensor : public Sensor {
public:
  HSCNodePowerSensor(std::shared_ptr<sdbusplus::asio::connection> &conn,
                     boost::asio::io_service &io, const std::string &sensorName,
                     const std::string &sensorConfiguration,
                     sdbusplus::asio::object_server &objectServer,
                     std::vector<thresholds::Threshold> &&thresholds,
                     uint8_t busId, uint8_t hscAddress, HSCType type);

  ~HSCNodePowerSensor();

  void start(HSCType type, int busId, int slaveAddr);
  void checkThresholds(void);

  int getAveragePower(SHSCDevInfo *);
  void initCoefficient();
  int pingHSCDevice();
  void configHSCDevice();
  void initMP5920();

  void pollHSCChipRegs();
  void updateValue(uint64_t newValue);

  int getHSCRegsInfoByte(uint8_t regs, int8_t *pu8data);
  int getHSCRegsInfoBytes(uint8_t regs, uint8_t length, int8_t *pu8data);
  int setHSCRegsInfoBytes(uint8_t regs, uint8_t length, uint8_t *pu8data);
  int setHSCRegsInfoByte(uint8_t regs, uint8_t pu8data);

private:
  std::shared_ptr<sdbusplus::asio::connection> mDbusConn;

  int mBusId;
  int mSlaveAddr;
  HSCType mType;
  uint64_t mValue;

  boost::asio::deadline_timer mPollTimer;
  sdbusplus::asio::object_server &mObjServer;
  std::shared_ptr<sdbusplus::asio::dbus_interface> mItemIface;
  std::shared_ptr<sdbusplus::asio::connection> mConn;

  HSC_Coefficient mHSCCoefficient;
  SHSCDevInfo mHSCDevInfo;
};