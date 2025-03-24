#include <gtest/gtest.h>
#include "modbus_test_utils.h"
#include "../src/byd_battery_model.h"

using namespace byd;
using namespace testutils;

namespace
{
constexpr auto VALUE1 = 1.1f;
constexpr auto VALUE2 = 22.22f;
constexpr auto VALUE3 = 333.333f;
constexpr auto VALUE4 = 0.4444f;

} // namespace

class BydBatteryModelTest : public ::testing::Test
{
protected:
    BatteryModel m_bat;
};

TEST_F(BydBatteryModelTest, Constructor_InitializedRegisters)
{
    std::vector<uint8_t> regRaw(100);
    // Reg100
    ASSERT_EQ(m_bat.Read(100, 2, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualString(&regRaw[0], "SI"));
    ASSERT_TRUE(IsEqualUint16(&regRaw[2], 1));

    ASSERT_EQ(m_bat.Read(102, 16, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualString(&regRaw[0], "BYD"));

    ASSERT_EQ(m_bat.Read(118, 16, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualString(&regRaw[0], "BYD Battery-Box Premium HV"));

    ASSERT_EQ(m_bat.Read(134, 8, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualString(&regRaw[0], "5.0"));

    ASSERT_EQ(m_bat.Read(142, 8, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualString(&regRaw[0], "3.16"));

    ASSERT_EQ(m_bat.Read(150, 16, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualString(&regRaw[0], "2025-9876"));

    ASSERT_EQ(m_bat.Read(166, 2, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualUint16(&regRaw[0], 1));
    ASSERT_TRUE(IsEqualUint16(&regRaw[2], 0));

    // Reg200: check only battery independant registers
    ASSERT_EQ(m_bat.Read(208, 1, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualUint16(&regRaw[0], 10));
    ASSERT_EQ(m_bat.Read(210, 1, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualUint16(&regRaw[0], 10));

    // Reg300: check only initial states
    ASSERT_EQ(m_bat.Read(300, 1, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualUint16(&regRaw[0], 1));
    ASSERT_EQ(m_bat.Read(302, 1, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualUint16(&regRaw[0], 128));

    // Reg400: check only initial states
    ASSERT_EQ(m_bat.Read(401, 1, &regRaw[0]), modb::ResponseError::None);
    ASSERT_TRUE(IsEqualUint16(&regRaw[0], 0xff00));

    // Rest is all 0x0000
}

TEST_F(BydBatteryModelTest, Read_InvalidAddress_ReturnError)
{
    std::vector<uint8_t> regRaw(4);
    // Reg100
    ASSERT_EQ(m_bat.Read(REG100_OFFSET - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG100_OFFSET + REG100_COUNT - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    // Reg200
    ASSERT_EQ(m_bat.Read(REG200_OFFSET - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG200_OFFSET + REG200_COUNT - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    // Reg300
    ASSERT_EQ(m_bat.Read(REG300_OFFSET - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG300_OFFSET + REG300_COUNT - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    // Reg400
    ASSERT_EQ(m_bat.Read(REG400_OFFSET - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG400_OFFSET + REG400_COUNT - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    // Reg1000
    ASSERT_EQ(m_bat.Read(REG1000_OFFSET - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG1000_OFFSET + REG1000_COUNT - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    // Reg12288
    ASSERT_EQ(m_bat.Read(REG12288_OFFSET - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG12288_OFFSET + REG12288_COUNT - 1, 2, &regRaw[0]), modb::ResponseError::IllegalAddress);
}

TEST_F(BydBatteryModelTest, Read_InvalidCount_ReturnError)
{
    std::vector<uint8_t> regRaw(300);
    ASSERT_EQ(m_bat.Read(REG100_OFFSET, REG100_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG200_OFFSET, REG200_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG300_OFFSET, REG300_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG400_OFFSET, REG400_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG1000_OFFSET, REG1000_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG12288_OFFSET, REG12288_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
}

TEST_F(BydBatteryModelTest, Read_FullSubRange_ReturnOk)
{
    std::vector<uint8_t> regRaw(REG12288_COUNT * 2);
    ASSERT_EQ(m_bat.Read(REG100_OFFSET, REG100_COUNT, &regRaw[0]), modb::ResponseError::None);
    ASSERT_EQ(m_bat.Read(REG200_OFFSET, REG200_COUNT, &regRaw[0]), modb::ResponseError::None);
    ASSERT_EQ(m_bat.Read(REG300_OFFSET, REG300_COUNT, &regRaw[0]), modb::ResponseError::None);
    ASSERT_EQ(m_bat.Read(REG400_OFFSET, REG400_COUNT, &regRaw[0]), modb::ResponseError::None);
    ASSERT_EQ(m_bat.Read(REG1000_OFFSET, REG1000_COUNT, &regRaw[0]), modb::ResponseError::None);
    ASSERT_EQ(m_bat.Read(REG12288_OFFSET, REG12288_COUNT, &regRaw[0]), modb::ResponseError::None);
}

TEST_F(BydBatteryModelTest, Read_MoreThanFullSubRange_ReturnError)
{
    std::vector<uint8_t> regRaw((REG12288_COUNT + 1) * 2);
    ASSERT_EQ(m_bat.Read(REG100_OFFSET, REG100_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG200_OFFSET, REG200_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG300_OFFSET, REG300_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG400_OFFSET, REG400_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG1000_OFFSET, REG1000_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
    ASSERT_EQ(m_bat.Read(REG12288_OFFSET, REG12288_COUNT + 1, &regRaw[0]), modb::ResponseError::IllegalAddress);
}

TEST_F(BydBatteryModelTest, Write_ValidAddressReadOnly_ReturnError)
{
    std::vector<uint8_t> regRaw(2, 0x42);
    // Reg100
    ASSERT_EQ(m_bat.Write(REG100_OFFSET, 1, &regRaw[0]), modb::ResponseError::IllegalFunction);
    ASSERT_EQ(m_bat.Write(REG100_OFFSET + REG100_COUNT - 1, 1, &regRaw[0]), modb::ResponseError::IllegalFunction);
    // Reg200
    ASSERT_EQ(m_bat.Write(REG200_OFFSET, 1, &regRaw[0]), modb::ResponseError::IllegalFunction);
    ASSERT_EQ(m_bat.Write(REG200_OFFSET + REG200_COUNT - 1, 1, &regRaw[0]), modb::ResponseError::IllegalFunction);
    // Reg300
    ASSERT_EQ(m_bat.Write(REG300_OFFSET, 1, &regRaw[0]), modb::ResponseError::IllegalFunction);
    ASSERT_EQ(m_bat.Write(REG300_OFFSET + REG300_COUNT - 1, 1, &regRaw[0]), modb::ResponseError::IllegalFunction);
}

TEST_F(BydBatteryModelTest, Write_ValidAddressReadWrite_ReturnOk)
{
    std::vector<uint8_t> regWrite(REG12288_COUNT * 2, 0x42);
    std::vector<uint8_t> regRead(REG12288_COUNT * 2, 0x00);

    ASSERT_EQ(m_bat.Write(REG400_OFFSET, REG400_COUNT, &regWrite[0]), modb::ResponseError::None);
    ASSERT_EQ(m_bat.Read(REG400_OFFSET, REG400_COUNT, &regRead[0]), modb::ResponseError::None);
    ASSERT_TRUE(std::equal(regRead.begin(), regRead.begin() + REG400_COUNT, regWrite.begin()));
    std::fill(regRead.begin(), regRead.end(), 0x00);

    ASSERT_EQ(m_bat.Write(REG1000_OFFSET, REG1000_COUNT, &regWrite[0]), modb::ResponseError::None);
    ASSERT_EQ(m_bat.Read(REG1000_OFFSET, REG1000_COUNT, &regRead[0]), modb::ResponseError::None);
    ASSERT_TRUE(std::equal(regRead.begin(), regRead.begin() + REG1000_COUNT, regWrite.begin()));
    std::fill(regRead.begin(), regRead.end(), 0x00);

    ASSERT_EQ(m_bat.Write(REG12288_OFFSET, REG12288_COUNT, &regWrite[0]), modb::ResponseError::None);
    ASSERT_EQ(m_bat.Read(REG12288_OFFSET, REG12288_COUNT, &regRead[0]), modb::ResponseError::None);
    ASSERT_TRUE(std::equal(regRead.begin(), regRead.begin() + REG12288_COUNT, regWrite.begin()));
}
