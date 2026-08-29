#include "ChassisIntrusionSensor.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <new>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{

class TestPathsOfDevice : public testing::Test
{
  public:
    std::filesystem::path testDir;
    std::filesystem::path aspeedAlarm;
    std::filesystem::path clockAlarm;
    std::filesystem::path namelessAlarm;

    TestPathsOfDevice()
    {
        auto dir = std::to_array("./testIntrusionXXXXXX");
        const char* made = mkdtemp(dir.data());
        if (made == nullptr)
        {
            throw std::bad_alloc();
        }
        testDir = made;

        aspeedAlarm = makeDevice("hwmon0", "aspeed_chassis");
        clockAlarm = makeDevice("hwmon5", "nct3018y");
        namelessAlarm = makeDevice("hwmon9", std::nullopt);
    }

    ~TestPathsOfDevice() override
    {
        std::filesystem::remove_all(testDir);
    }

    TestPathsOfDevice(const TestPathsOfDevice&) = delete;
    TestPathsOfDevice(TestPathsOfDevice&&) = delete;
    TestPathsOfDevice& operator=(const TestPathsOfDevice&) = delete;
    TestPathsOfDevice& operator=(TestPathsOfDevice&&) = delete;

  private:
    std::filesystem::path makeDevice(
        const std::string& directory,
        std::optional<std::string> deviceName) const
    {
        std::filesystem::path device = testDir / directory;
        std::filesystem::create_directory(device);

        if (deviceName)
        {
            std::ofstream nameFile{device / "name"};
            nameFile << *deviceName << "\n";
        }

        std::filesystem::path alarm = device / "intrusion0_alarm";
        std::ofstream alarmFile{alarm};

        return alarm;
    }
};

TEST_F(TestPathsOfDevice, KeepsOnlyThePathsOfTheNamedDevice)
{
    std::vector<std::filesystem::path> found{aspeedAlarm, clockAlarm};

    EXPECT_EQ(pathsOfDevice(found, "nct3018y"),
              std::vector<std::filesystem::path>{clockAlarm});
    EXPECT_EQ(pathsOfDevice(found, "aspeed_chassis"),
              std::vector<std::filesystem::path>{aspeedAlarm});
}

TEST_F(TestPathsOfDevice, KeepsNothingWhenNoDeviceCarriesTheName)
{
    std::vector<std::filesystem::path> found{aspeedAlarm, clockAlarm};

    EXPECT_TRUE(pathsOfDevice(found, "no_such_device").empty());
}

TEST_F(TestPathsOfDevice, DropsADeviceThatStatesNoName)
{
    std::vector<std::filesystem::path> found{namelessAlarm};

    EXPECT_TRUE(pathsOfDevice(found, "aspeed_chassis").empty());
}

} // namespace
