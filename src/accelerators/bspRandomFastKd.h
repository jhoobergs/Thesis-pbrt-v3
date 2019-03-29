//
// Created by jesse on 28.03.19.
//

#ifndef PBRT_V3_BSPRANDOMFASTKD_H
#define PBRT_V3_BSPRANDOMFASTKD_H

#include "BSPKd.h"

namespace pbrt {
    class BSPRandomFastKd: public BSPKd {
    public:

        BSPRandomFastKd(std::vector<std::shared_ptr<Primitive>> p,
                         uint32_t isectCost = 80, uint32_t traversalCost = 5,
                         Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3, uint32_t kdTravCost=1);

    protected:
        void buildTree() override;

    private:
        uint32_t K;
    };

    std::shared_ptr<BSPRandomFastKd> CreateBSPRandomFastKdTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}



#endif //PBRT_V3_BSPRANDOMFASTKD_H
