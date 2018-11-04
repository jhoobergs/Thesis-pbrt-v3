
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

#ifndef PBRT_ACCELERATORS_BVH_OLD_H
#define PBRT_ACCELERATORS_BVH_OLD_H

// accelerators/bvhOld.h*
#include "pbrt.h"
#include "primitive.h"
#include <atomic>

namespace pbrt {
    struct BVHBuildNodeOld;

    // BVHAccelOld Forward Declarations
    struct BVHPrimitiveInfoOld;
    struct MortonPrimitiveOld;
    struct LinearBVHNodeOld;

    // BVHAccelOld Declarations
    class BVHAccelOld : public Aggregate {
    public:
        // BVHAccelOld Public Types
        enum class SplitMethod {
            SAH, HLBVH, Middle, EqualCounts
        };

        // BVHAccelOld Public Methods
        BVHAccelOld(std::vector<std::shared_ptr<Primitive>> p,
                    int maxPrimsInNode = 1,
                    SplitMethod splitMethod = SplitMethod::SAH);

        Bounds3f WorldBound() const;

        ~BVHAccelOld();

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;

        bool IntersectP(const Ray &ray) const;

    private:
        // BVHAccelOld Private Methods
        BVHBuildNodeOld *recursiveBuild(
                MemoryArena &arena, std::vector<BVHPrimitiveInfoOld> &primitiveInfo,
                int start, int end, int *totalNodes,
                std::vector<std::shared_ptr<Primitive>> &orderedPrims);

        BVHBuildNodeOld *HLBVHBuild(
                MemoryArena &arena, const std::vector<BVHPrimitiveInfoOld> &primitiveInfo,
                int *totalNodes,
                std::vector<std::shared_ptr<Primitive>> &orderedPrims) const;

        BVHBuildNodeOld *emitLBVH(
                BVHBuildNodeOld *&buildNodes,
                const std::vector<BVHPrimitiveInfoOld> &primitiveInfo,
                MortonPrimitiveOld *mortonPrims, int nPrimitives, int *totalNodes,
                std::vector<std::shared_ptr<Primitive>> &orderedPrims,
                std::atomic<int> *orderedPrimsOffset, int bitIndex) const;

        BVHBuildNodeOld *buildUpperSAH(MemoryArena &arena,
                                       std::vector<BVHBuildNodeOld *> &treeletRoots,
                                       int start, int end, int *totalNodes) const;

        int flattenBVHTree(BVHBuildNodeOld *node, int *offset);

        // BVHAccelOld Private Data
        const int maxPrimsInNode;
        const SplitMethod splitMethod;
        std::vector<std::shared_ptr<Primitive>> primitives;
        LinearBVHNodeOld *nodes = nullptr;
    };

    std::shared_ptr<BVHAccelOld> CreateBVHAcceleratorOld(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);

}  // namespace pbrt

#endif  // PBRT_ACCELERATORS_BVH_OLD_H
