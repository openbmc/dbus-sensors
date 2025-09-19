#include "utils/Utils.hpp"

#include <phosphor-logging/lg2.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <new>
#include <string>
#include <vector>

#include <gtest/gtest.h>

class TestUtils : public testing::Test
{
  public:
    std::string testDir;
    std::filesystem::path hwmonDir;
    std::filesystem::path peciDir;
    TestUtils()
    {
        // Create test environment
        auto dir = std::to_array("./testDirXXXXXX");
        testDir = mkdtemp(dir.data());

        if (testDir.empty())
        {
            throw std::bad_alloc();
        }
        hwmonDir = std::filesystem::path(testDir) / "hwmon";
        std::filesystem::create_directory(hwmonDir);
        auto hwmon10 = hwmonDir / "hwmon10";
        std::filesystem::create_directory(hwmonDir / "hwmon10");
        {
            std::ofstream temp1Input{hwmon10 / "temp1_input"};
            std::ofstream temp1Min{hwmon10 / "temp1_min"};
            std::ofstream temp1Max{hwmon10 / "temp1_max"};
            std::ofstream temp2Input{hwmon10 / "temp2_input"};
        }
        createPECIDir();
    }

    ~TestUtils() override
    {
        std::filesystem::remove_all(testDir);
    }

    TestUtils(const TestUtils&) = delete;
    TestUtils(TestUtils&&) = delete;
    TestUtils& operator=(const TestUtils&) = delete;
    TestUtils& operator=(TestUtils&&) = delete;

    void createPECIDir()
    {
        peciDir = std::filesystem::path(testDir) / "peci";
        auto peci0 = peciDir /
                     "peci-0/device/0-30/peci-cputemp.0/hwmon/hwmon25";
        std::filesystem::create_directories(peci0);
        {
            std::ofstream temp0Input{peci0 / "temp0_input"};
            std::ofstream temp1Input{peci0 / "temp1_input"};
            std::ofstream temp2Input{peci0 / "temp2_input"};
            std::ofstream name{peci0 / "name"};
        }
        auto devDir = peciDir / "peci-0/peci_dev/peci-0";
        std::filesystem::create_directories(devDir);
        std::filesystem::create_directory_symlink("../../../peci-0",
                                                  devDir / "device");
        std::filesystem::create_directory_symlink("device/0-30",
                                                  peciDir / "peci-0/0-30");

        // Let's keep this for debugging purpose
        for (auto p = std::filesystem::recursive_directory_iterator(
                 peciDir,
                 std::filesystem::directory_options::follow_directory_symlink);
             p != std::filesystem::recursive_directory_iterator(); ++p)
        {
            std::string path = p->path().string();
            lg2::info("{PATH}", "PATH", path);
            if (p.depth() >= 6)
            {
                p.disable_recursion_pending();
            }
        }
    }
};

TEST_F(TestUtils, findFiles_non_exist)
{
    std::vector<std::filesystem::path> foundPaths;
    auto ret = findFiles("non-exist", "", foundPaths);

    EXPECT_FALSE(ret);
    EXPECT_TRUE(foundPaths.empty());
}

TEST_F(TestUtils, findFiles_in_hwmon_no_match)
{
    std::vector<std::filesystem::path> foundPaths;
    auto ret = findFiles(hwmonDir, R"(in\d+_input)", foundPaths);

    EXPECT_TRUE(ret);
    EXPECT_EQ(foundPaths.size(), 0U);
}

TEST_F(TestUtils, findFiles_in_hwmon_match)
{
    std::vector<std::filesystem::path> foundPaths;
    auto ret = findFiles(hwmonDir, R"(temp\d+_input)", foundPaths);

    EXPECT_TRUE(ret);
    EXPECT_EQ(foundPaths.size(), 2U);
}

TEST_F(TestUtils, findFiles_in_peci_no_match)
{
    std::vector<std::filesystem::path> foundPaths;
    auto ret =
        findFiles(peciDir, R"(peci-\d+/\d+-.+/peci-.+/hwmon/hwmon\d+/aaa$)",
                  foundPaths, 6);

    EXPECT_TRUE(ret);
    EXPECT_TRUE(foundPaths.empty());
}

TEST_F(TestUtils, findFiles_in_peci_match)
{
    std::vector<std::filesystem::path> foundPaths;
    auto ret =
        findFiles(peciDir, R"(peci-\d+/\d+-.+/peci-.+/hwmon/hwmon\d+/name$)",
                  foundPaths, 6);
    EXPECT_TRUE(ret);
    EXPECT_EQ(foundPaths.size(), 1U);

    foundPaths.clear();

    ret = findFiles(peciDir,
                    R"(peci-\d+/\d+-.+/peci-.+/hwmon/hwmon\d+/temp\d+_input)",
                    foundPaths, 6);
    EXPECT_TRUE(ret);
    EXPECT_EQ(foundPaths.size(), 3U);
}

TEST_F(TestUtils, findFiles_hwmonPath_end_with_slash)
{
    std::string p = hwmonDir.string() + "/";
    std::vector<std::filesystem::path> foundPaths;
    auto ret = findFiles(p, R"(temp\d+_input)", foundPaths);

    EXPECT_TRUE(ret);
    EXPECT_EQ(foundPaths.size(), 2U);
}

TEST_F(TestUtils, findFiles_peciPath_end_with_slash)
{
    std::string p = peciDir.string() + "/";
    std::vector<std::filesystem::path> foundPaths;
    auto ret =
        findFiles(p, R"(peci-\d+/\d+-.+/peci-.+/hwmon/hwmon\d+/temp\d+_input)",
                  foundPaths, 6);

    EXPECT_TRUE(ret);
    EXPECT_EQ(foundPaths.size(), 3U);
}

TEST_F(TestUtils, findFiles_in_sub_peci_match)
{
    std::vector<std::filesystem::path> foundPaths;
    auto ret =
        findFiles(peciDir / "peci-0", R"(\d+-.+/peci-.+/hwmon/hwmon\d+/name$)",
                  foundPaths, 5);
    EXPECT_TRUE(ret);
    EXPECT_EQ(foundPaths.size(), 1U);

    foundPaths.clear();

    ret = findFiles(peciDir / "peci-0",
                    R"(\d+-.+/peci-.+/hwmon/hwmon\d+/temp\d+_input)",
                    foundPaths, 5);
    EXPECT_TRUE(ret);
    EXPECT_EQ(foundPaths.size(), 3U);
}

TEST(GetDeviceBusAddrTest, DevNameInvalid)
{
    size_t bus = 0;
    size_t addr = 0;
    std::string devName;

    auto ret = getDeviceBusAddr(devName, bus, addr);
    EXPECT_FALSE(ret);

    devName = "NoHyphen";
    ret = getDeviceBusAddr(devName, bus, addr);
    EXPECT_FALSE(ret);

    devName = "pwm-fan";
    ret = getDeviceBusAddr(devName, bus, addr);
    EXPECT_FALSE(ret);
}

TEST(GetDeviceBusAddrTest, BusInvalid)
{
    size_t bus = 0;
    size_t addr = 0;
    std::string devName = "FF-00FF";

    auto ret = getDeviceBusAddr(devName, bus, addr);
    EXPECT_FALSE(ret);
}

TEST(GetDeviceBusAddrTest, AddrInvalid)
{
    size_t bus = 0;
    size_t addr = 0;
    std::string devName = "12-fan";

    auto ret = getDeviceBusAddr(devName, bus, addr);
    EXPECT_FALSE(ret);
}

TEST(GetDeviceBusAddrTest, I3CBusAddrValid)
{
    uint64_t bus = 0;
    uint64_t provisionedId = 0;
    std::string devName = "0-22400000001";

    auto ret = getDeviceBusAddr(devName, bus, provisionedId);
    EXPECT_TRUE(ret);
    EXPECT_EQ(bus, 0U);
    EXPECT_EQ(provisionedId, 0x22400000001U);
}

TEST(GetDeviceBusAddrTest, AllValid)
{
    size_t bus = 0;
    size_t addr = 0;
    std::string devName = "12-00af";

    auto ret = getDeviceBusAddr(devName, bus, addr);
    EXPECT_TRUE(ret);
    EXPECT_EQ(bus, 12U);
    EXPECT_EQ(addr, 0xafU);
}
