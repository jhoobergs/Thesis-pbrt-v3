//
// Created by jesse on 13.02.19.
//

#ifndef PBRT_V3_RBSPCLUSTER_H
#define PBRT_V3_RBSPCLUSTER_H

#include "BSP.h"

namespace pbrt {
    class BSPCluster : public BSP {
    public:

        BSPCluster(std::vector<std::shared_ptr<Primitive>> p,
                   uint32_t isectCost = 80, uint32_t traversalCost = 1,
                   Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3);

    protected:
        void buildTree() override;

    private:
        uint32_t K;
    };

    std::shared_ptr<BSPCluster> CreateBSPClusterTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}


#endif //PBRT_V3_RBSPCLUSTER_H
