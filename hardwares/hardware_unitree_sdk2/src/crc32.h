#ifndef STAND_GO2_CRC32_H
#define STAND_GO2_CRC32_H

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

static inline uint32_t crc32_core(const uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;

    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1u << 31;
        data = ptr[i];

        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000u)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
            {
                CRC32 ^= dwPolynomial;
            }

            xbit >>= 1;
        }
    }

    return CRC32;
}

#ifdef __cplusplus
}
#endif

#endif // STAND_GO2_CRC32_H