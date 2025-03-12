#pragma once

#include <stdint.h>

#define CHAR2UINT16(c1, c2) uint16_t(uint8_t(c1) << 8) | uint8_t(c2)

template <typename T>
T Convert2BigEndian(T n)
{
    T m;
    for (size_t i = 0; i < sizeof(T); i++)
    {
        reinterpret_cast<uint8_t*>(&m)[i] = reinterpret_cast<uint8_t*>(&n)[sizeof(T) - 1 - i];
    }
    return m;
}
