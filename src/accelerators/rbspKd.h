//
// Created by jesse on 22.03.19.
//

#ifndef PBRT_V3_RBSPKD_H
#define PBRT_V3_RBSPKD_H

#include <core/geometry.h>
#include <algorithm>
#include "genericBSP.h"
#include "kDOPMesh.h"

namespace pbrt {
    struct RBSPKdNode;

    class RBSPKd : public GenericBSP<RBSPKdNode> {
    public:

        RBSPKd(std::vector<std::shared_ptr<Primitive>> p,
             uint32_t isectCost = 80, uint32_t traversalCost = 5,
             Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3,
             Float splitAlpha = 90, uint32_t alphaType = 0, uint32_t axisSelectionType = 0,
             uint32_t axisSelectionAmount = -1, uint32_t kdTraversalCost=1);

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const override;

        bool IntersectP(const Ray &ray) const override;

        void printNodes(std::ofstream &os) const override;

    protected:
        void buildTree() override;
        const uint32_t kdTraversalCost;
    };

    std::shared_ptr<RBSPKd> CreateRBSPKdTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);

}

#endif //PBRT_V3_RBSPKD_H
