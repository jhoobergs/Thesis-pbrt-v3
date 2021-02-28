//
// Created by jesse on 28.03.19.
//

#ifndef PBRT_V3_BSPARBITRARY_H
#define PBRT_V3_BSPARBITRARY_H

#include "bspNodeBased.h"

namespace pbrt {
    class BSPArbitrary : public BSPNodeBased {
    public:

        BSPArbitrary(std::vector<std::shared_ptr<Primitive>> p,
                     uint32_t isectCost = 80, uint32_t traversalCost = 1,
                     Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3);

    protected:
        std::vector<Vector3f> calculateDirections(std::mt19937 &gen, uint32_t K,
                                                  const std::vector<std::shared_ptr<Primitive>> &primitives,
                                                  const uint32_t *primNums, uint32_t np) override;
    };

    std::shared_ptr<BSPArbitrary> CreateBSPArbitraryTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);

}

#endif //PBRT_V3_BSPARBITRARY_H