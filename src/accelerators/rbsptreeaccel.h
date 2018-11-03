#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_ACCELERATORS_RBSPTREEACCEL_H
#define PBRT_ACCELERATORS_RBSPTREEACCEL_H

// accelerators/rbsptreeaccel.h*
#include "pbrt.h"
#include "primitive.h"

namespace pbrt {

    // RBSPTreeAccel Declarations
    struct RBSPAccelNode;
    struct BoundEdge;
    struct EdgeSoup;
    struct EdgeSoupEdge;
    class RBSPTreeAccel : public Aggregate {
    public:
        // RBSPTreeAccel Public Methods
        RBSPTreeAccel(std::vector<std::shared_ptr<Primitive>> p,
                    int isectCost = 80, int traversalCost = 1,
                    Float emptyBonus = 0.5, int maxPrims = 1, int maxDepth = -1);
        Bounds3f WorldBound() const { return bounds; }
        ~RBSPTreeAccel();
        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;
        bool IntersectP(const Ray &ray) const;
        static const Vector3f directions[];

    private:
        // RBSPTreeAccel Private Method
        void buildTree(int nodeNum, const EdgeSoupEdge *edgeSoupEdges,
                       const std::vector<Bounds3f> &allPrimBounds, int *primNums,
                       int nPrimitives, int depth,
                       const std::unique_ptr<BoundEdge[]> edges[3], int *prims0,
                       int *prims1, int badRefines = 0);

        // RBSPTreeAccel Private Data
        const int isectCost, traversalCost, maxPrims;
        const Float emptyBonus;
        std::vector<std::shared_ptr<Primitive>> primitives;
        std::vector<int> primitiveIndices;
        RBSPAccelNode *nodes;
        int nAllocedNodes, nextFreeNode;
        Bounds3f bounds;
    };

    struct RBSPToDo {
        const RBSPAccelNode *node;
        Float tMin, tMax;
    };

    std::shared_ptr<RBSPTreeAccel> CreateRBSPTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);

}  // namespace pbrt

#endif  // PBRT_ACCELERATORS_RBSPTREEACCEL_H