#pragma once

#if defined(MCUDRV_C28X)

#include <emblib/array.hpp>

typedef emb::array<uint8_t, 8> can_payload;
typedef uint32_t can_id;

#else

#include <array>
#include <cstdint>

using can_payload = std::array<uint8_t, 8>;
using can_id = uint32_t;

struct can_frame {
    can_id id;
    uint8_t len;
    can_payload payload;
};

#endif
