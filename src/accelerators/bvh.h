
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

#ifndef PBRT_ACCELERATORS_BVH_H
#define PBRT_ACCELERATORS_BVH_H

// accelerators/bvh.h*
#include "pbrt.h"
#include "primitive.h"
#include <atomic>

namespace pbrt {
    struct BVHBuildNode;
    struct BVHBuildToDo;
    struct BVHBuildCentroid;

    // BVHAccel Forward Declarations
    struct BVHPrimitiveInfo;
    struct LinearBVHNode;

    // BVHAccel Declarations
    class BVHAccel : public Aggregate {
    public:

        // BVHAccel Public Methods
        BVHAccel(std::vector<std::shared_ptr<Primitive>> p, int isectCost = 8, int traversalCost = 1,
                 int maxPrimsInNode = 1);

        Bounds3f WorldBound() const;

        ~BVHAccel();

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;

        bool IntersectP(const Ray &ray) const;

        std::pair<uint32_t, uint32_t> getAmountToLeftAndRight(const Plane &p) const;

        void getPrimnumsToLeftAndRight(const Plane &p, std::vector<uint32_t> &left, std::vector<uint32_t> &right) const;


            private:
        // BVHAccel Private Methods
        BVHBuildNode *iterativeBuild(MemoryArena &arena, int *totalNodes,
                                     std::vector<std::shared_ptr<Primitive>> &orderedPrims);

        int flattenBVHTree(BVHBuildNode *node, int *offset);

        // BVHAccel Private Data
        const int maxPrimsInNode;
        const int isectCost;
        const int traversalCost;
        std::vector<std::shared_ptr<Primitive>> primitives;
        std::vector<uint32_t> primNumMapping;
        LinearBVHNode *nodes = nullptr;
    };

    std::shared_ptr<BVHAccel> CreateBVHAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);

}  // namespace pbrt

#endif  // PBRT_ACCELERATORS_BVH_H
