#include <cstdint>
/*
 * Data types as per specification
 */

enum DataType
{
    NullData = 0x00,
    Boolean = 0x03,
    BitString = 0x04,
    DoubleLong = 0x05,
    DoubleLongUnsigned = 0x06,
    OctetString = 0x09,
    VisibleString = 0x0A,
    Utf8String = 0x0C,
    BinaryCodedDecimal = 0x0D,
    Integer = 0x0F,
    Long = 0x10,
    Unsigned = 0x11,
    LongUnsigned = 0x12,
    Long64 = 0x14,
    Long64Unsigned = 0x15,
    Enum = 0x16,
    Float32 = 0x17,
    Float64 = 0x18,
    DateTime = 0x19,
    Date = 0x1A,
    Time = 0x1B,

    Array = 0x01,
    Structure = 0x02,
    CompactArray = 0x13
};

enum Medium
{
    Abstract = 0x00,
    Electricity = 0x01,
    Heat = 0x06,
    Gas = 0x07,
    Water = 0x08
};

enum CodeType : uint16_t
{
    Unknown = 0x0000,
    Timestamp = 0x0001,
    SerialNumber = 0x0160,
    DeviceName = 0x002A,
    VoltageL1 = 0x0720,
    VoltageL2 = 0x0734,
    VoltageL3 = 0x0748,
    CurrentL1 = 0x071F,
    CurrentL2 = 0x0733,
    CurrentL3 = 0x0747,
    ActivePowerPlus = 0x0701,
    ActivePowerMinus = 0x0702,
    ActiveEnergyPlus = 0x0801,
    ActiveEnergyMinus = 0x0802,
    ReactiveEnergyPlus = 0x0803,
    ReactiveEnergyMinus = 0x0804
};

enum Accuracy
{
    SingleDigit = 0xFF,
    DoubleDigit = 0xFE
};

/*
 * Data structure
 */

static const int DECODER_START_OFFSET = 20; // Offset for start of OBIS decoding, skip header, timestamp and break block

static const int OBIS_TYPE_OFFSET = 0;
static const int OBIS_LENGTH_OFFSET = 1;

static const int OBIS_CODE_OFFSET = 2;

static const int OBIS_A = 0;
static const int OBIS_B = 1;
static const int OBIS_C = 2;
static const int OBIS_D = 3;
static const int OBIS_E = 4;
static const int OBIS_F = 5;
