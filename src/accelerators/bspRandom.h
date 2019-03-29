//
// Created by jesse on 29.03.19.
//

#ifndef PBRT_V3_BSPRANDOM_H
#define PBRT_V3_BSPRANDOM_H

#include "bspNodeBased.h"

namespace pbrt {
    class BSPRandom : public BSPNodeBased {
    public:

        BSPRandom(std::vector<std::shared_ptr<Primitive>> p,
                  uint32_t isectCost = 80, uint32_t traversalCost = 1,
                  Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3);

    protected:
        std::vector<Vector3f> calculateDirections(std::mt19937 &gen, const uint32_t K,
                                                  const std::vector<std::shared_ptr<Primitive>> &primitives,
                                                  const uint32_t *primNums, const uint32_t np) override;
    };

    std::shared_ptr<BSPRandom> CreateBSPRandomTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}


#endif //PBRT_V3_BSPRANDOM_H
