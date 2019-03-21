//
// Created by jesse on 20.03.19.
//

#ifndef PBRT_V3_BSPCLUSTERFASEKD_H
#define PBRT_V3_BSPCLUSTERFASEKD_H

#include "BSPKd.h"

namespace pbrt {
    class BSPClusterFastKd: public BSPKd {
    public:

        BSPClusterFastKd(std::vector<std::shared_ptr<Primitive>> p,
                         uint32_t isectCost = 80, uint32_t traversalCost = 5,
                         Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3, uint32_t kdTravCost=1);

    protected:
        void buildTree() override;

    private:
        uint32_t K;
    };

    std::shared_ptr<BSPClusterFastKd> CreateBSPClusterFastKdTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}


#endif //PBRT_V3_BSPCLUSTERFASEKD_H
