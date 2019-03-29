//
// Created by jesse on 29.03.19.
//

#ifndef PBRT_V3_BSPNODEBASEDFASTKD_H
#define PBRT_V3_BSPNODEBASEDFASTKD_H

#include "BSPKd.h"
#include <random>

namespace pbrt {
    class BSPNodeBasedFastKd: public BSPKd {
    public:

        BSPNodeBasedFastKd(std::vector<std::shared_ptr<Primitive>> p,
                           uint32_t isectCost = 80, uint32_t traversalCost = 5,
                           Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3, uint32_t kdTravCost=1);

    protected:
        void buildTree() override;

        virtual std::vector<Vector3f> calculateDirections(std::mt19937 &gen, uint32_t K,
                                                          const std::vector<std::shared_ptr<Primitive>> &primitives,
                                                          const uint32_t *primNums, uint32_t np) = 0;

        uint32_t K;
    };
}


#endif //PBRT_V3_BSPNODEBASEDFASTKD_H
