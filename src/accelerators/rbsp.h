//
// Created by jesse on 11.11.18.
//

#ifndef PBRT_V3_RBSP_H
#define PBRT_V3_RBSP_H

#include <core/geometry.h>
#include "kdtreeaccel.h"

namespace pbrt {
    struct RBSPNode;
    class RBSP  : public Aggregate {
    public:

        // KdTreeAccel Public Methods
        RBSP(std::vector<std::shared_ptr<Primitive>> p,
             uint32_t isectCost = 80, uint32_t traversalCost = 1,
                Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1);

        Bounds3f WorldBound() const { return bounds; }

        ~RBSP();

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;

        bool IntersectP(const Ray &ray) const;

    private:
        // RBSP Private Methods
        void buildTree(BoundsMf &rootNodeMBounds,
                       const std::vector<BoundsMf> &allPrimBounds, uint32_t M, uint32_t maxDepth);

        // KdTreeAccel Private Data
        const uint32_t isectCost, traversalCost, maxPrims;
        const Float emptyBonus;
        std::vector<std::shared_ptr<Primitive>> primitives;
        std::vector<uint32_t> primitiveIndices;
        RBSPNode *nodes;
        uint32_t nAllocedNodes, nextFreeNode;
        Bounds3f bounds;
    };

    struct RBSPBuildNode{
        RBSPBuildNode(uint32_t depth, uint32_t nPrimitives, uint32_t badRefines, BoundsMf nodeBounds, uint32_t *primNums, uint32_t parentNum = -1)
                : depth(depth),
                  nPrimitives(nPrimitives), badRefines(badRefines), nodeBounds(std::move(nodeBounds)),
                  primNums(primNums), parentNum(parentNum) {}

        uint32_t depth;
        uint32_t nPrimitives;
        uint32_t badRefines;
        BoundsMf nodeBounds;
        uint32_t *primNums;
        uint32_t parentNum;
    };

    struct RBSPToDo {
        const RBSPNode *node;
        Float tMin, tMax;
    };

    std::shared_ptr<RBSP> CreateRBSPTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);

}
#endif //PBRT_V3_RBSP_H
