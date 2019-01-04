
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_ACCELERATORS_KDTREEACCEL_H
#define PBRT_ACCELERATORS_KDTREEACCEL_H

// accelerators/kdtreeaccel.h*
#include "pbrt.h"
#include "primitive.h"

namespace pbrt {

    // KdTreeAccel Declarations
    struct KdAccelNode;


    enum class EdgeType {
        Start, End
    };

    struct BoundEdge {
        // BoundEdge Public Methods
        BoundEdge() {}

        BoundEdge(Float t, uint32_t primNum, bool starting) : t(t), primNum(primNum) {
            type = starting ? EdgeType::Start : EdgeType::End;
        }

        Float t;
        uint32_t primNum;
        EdgeType type;
    };

    class KdTreeAccel : public Aggregate {
    public:

        // KdTreeAccel Public Methods
        KdTreeAccel(std::vector<std::shared_ptr<Primitive>> p,
                    uint32_t isectCost = 80, uint32_t traversalCost = 1,
                    Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, Float splitAlpha = 90,
                    uint32_t alphaType = 0,
                    uint32_t axisSelectionType = 0, uint32_t axisSelectionAmount = -1);

        Bounds3f WorldBound() const { return bounds; }

        ~KdTreeAccel();

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;

        bool IntersectP(const Ray &ray) const;

        friend std::ofstream &operator<<(std::ofstream &os, const KdTreeAccel &kdTreeAccel);

    private:
        // KdTreeAccel Private Methods
        void buildTree(Bounds3f &rootNodeBounds,
                       const std::vector<Bounds3f> &allPrimBounds, uint32_t maxDepth, Float splitAlpha,
                       uint32_t alphaType,
                       uint32_t axisSelectionType, uint32_t axisSelectionAmount);

        // KdTreeAccel Private Data
        const uint32_t isectCost, traversalCost, maxPrims;
        const Float emptyBonus;
        std::vector<std::shared_ptr<Primitive>> primitives;
        std::vector<uint32_t> primitiveIndices;
        KdAccelNode *nodes;
        uint32_t nAllocedNodes, nextFreeNode;
        Bounds3f bounds;
    };

    struct KdBuildNode {
        KdBuildNode(uint32_t depth, uint32_t nPrimitives, uint32_t badRefines, Bounds3f nodeBounds, uint32_t *primNums,
                    uint32_t parentNum = -1)
                : depth(depth),
                  nPrimitives(nPrimitives), badRefines(badRefines), nodeBounds(std::move(nodeBounds)),
                  primNums(primNums), parentNum(parentNum) {}

        uint32_t depth;
        uint32_t nPrimitives;
        uint32_t badRefines;
        Bounds3f nodeBounds;
        uint32_t *primNums;
        uint32_t parentNum;
    };

    struct KdToDo {
        const KdAccelNode *node;
        Float tMin, tMax;
    };

    std::shared_ptr<KdTreeAccel> CreateKdTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);

}  // namespace pbrt

#endif  // PBRT_ACCELERATORS_KDTREEACCEL_H
