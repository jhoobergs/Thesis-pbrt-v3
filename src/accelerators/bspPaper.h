//
// Created by jesse on 13.02.19.
//

#ifndef PBRT_V3_BSPPAPER_H
#define PBRT_V3_BSPPAPER_H

#include "BSP.h"

namespace pbrt {
    class BSPPaper : public BSP {
    public:

        BSPPaper(std::vector<std::shared_ptr<Primitive>> p,
                 uint32_t isectCost = 80, uint32_t traversalCost = 1,
                 Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3);

    protected:
        void buildTree() override;

    };

    std::shared_ptr<BSPPaper> CreateBSPPaperTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}


#endif //PBRT_V3_BSPPAPER_H
