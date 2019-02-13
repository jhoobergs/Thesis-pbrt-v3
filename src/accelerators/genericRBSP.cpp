//
// Created by jesse on 11.11.18.
//

#include <core/stats.h>
#include <core/progressreporter.h>
#include "paramset.h"
#include "accelerators/genericRBSP.h"
#include <fstream>
#include <shapes/triangle.h>
#include <random>
#include <cmath>
#include <limits.h>
#include <string>
#include <vector>

#define clz(x) __builtin_clz(x)
namespace pbrt {

    inline uint32_t log2_fast(const uint32_t x) {
        return sizeof(uint32_t) * CHAR_BIT - clz(x - 1);
    }

    inline const uint32_t getBitOffset(const uint32_t M) {
        return log2_fast(M + 1);
        //return (uint32_t) std::ceil(std::log2(M + 1));
    }

    inline uint32_t getBitMask(const uint32_t M) {
        return ((uint32_t) 1 << getBitOffset(M)) - 1;
    }

}  // namespace pbrt