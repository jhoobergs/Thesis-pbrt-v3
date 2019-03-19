//
// Created by jesse on 17.03.19.
//

#ifndef PBRT_V3_BSPPAPERKD_H
#define PBRT_V3_BSPPAPERKD_H

#include "BSPKd.h"

namespace pbrt {


    class BSPPaperKd : public BSPKd {
    public:

        BSPPaperKd(std::vector<std::shared_ptr<Primitive>> p,
                   uint32_t isectCost = 80, uint32_t traversalCost = 5,
                   Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3, uint32_t kdTraversalCost=1);

    protected:
        void buildTree() override;

    };

    std::shared_ptr<BSPPaperKd> CreateBSPPaperKdTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}


#endif //PBRT_V3_BSPPAPERKD_H
