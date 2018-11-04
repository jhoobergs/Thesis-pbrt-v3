
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


// accelerators/bvh.cpp*
#include "accelerators/bvh.h"
#include "interaction.h"
#include "paramset.h"
#include "stats.h"
#include "parallel.h"
#include <algorithm>

namespace pbrt {

    STAT_MEMORY_COUNTER("Memory/BVH tree", treeBytes);
    STAT_RATIO("BVH/Primitives per leaf node", totalPrimitives, totalLeafNodes);
    STAT_COUNTER("BVH/Interior nodes", interiorNodes);
    STAT_COUNTER("BVH/Leaf nodes", leafNodes);

    // BVHAccel Local Declarations
    struct BVHPrimitiveInfo {
        BVHPrimitiveInfo() {}

        BVHPrimitiveInfo(size_t primitiveNumber, const Bounds3f &bounds)
                : primitiveNumber(primitiveNumber),
                  bounds(bounds),
                  centroid(.5f * bounds.pMin + .5f * bounds.pMax) {}

        size_t primitiveNumber;
        Bounds3f bounds;
        Point3f centroid;
    };

    struct BVHBuildToDo {
        BVHBuildToDo(BVHBuildNode *node, int start, int end, BVHBuildNode *parent) : node(node), start(start), end(end),
                                                                                     parent(parent) {}

        int start;
        int end;
        BVHBuildNode *parent;
        BVHBuildNode *node;
    };

    struct BVHBuildNode {
        // BVHBuildNode Public Methods
        void InitLeaf(int first, int n, const Bounds3f &b) {
            firstPrimOffset = first;
            nPrimitives = n;
            bounds = b;
            children[0] = children[1] = nullptr;
            ++leafNodes;
            ++totalLeafNodes;
            totalPrimitives += n;
        }

        void InitInterior(int axis, BVHBuildNode *c0, BVHBuildNode *c1) {
            children[0] = c0;
            children[1] = c1;
            splitAxis = axis;
            nPrimitives = 0;
            ++interiorNodes;
        }

        void calculateBounds() {
            if (nPrimitives == 0) {
                children[0]->calculateBounds();
                children[1]->calculateBounds();
                bounds = Union(children[0]->bounds, children[1]->bounds);
            }
        }

        Bounds3f bounds;
        BVHBuildNode *children[2];
        int splitAxis, firstPrimOffset, nPrimitives;
    };

    struct LinearBVHNode {
        Bounds3f bounds;
        union {
            int primitivesOffset;   // leaf
            int secondChildOffset;  // interior
        };
        uint16_t nPrimitives;  // 0 -> interior node
        uint8_t axis;          // interior node: xyz
        uint8_t pad[1];        // ensure 32 byte total size
    };

    // BVHAccel Method Definitions
    BVHAccel::BVHAccel(std::vector<std::shared_ptr<Primitive>> p,
                       int maxPrimsInNode)
            : maxPrimsInNode(std::min(255, maxPrimsInNode)),
              primitives(std::move(p)) {
        ProfilePhase _(Prof::AccelConstruction);
        if (primitives.empty()) return;
        // Build BVH from _primitives_

        // Build BVH tree for primitives using _primitiveInfo_
        MemoryArena arena(1024 * 1024);

        int totalNodes = 0;
        std::vector<std::shared_ptr<Primitive>> orderedPrims;
        orderedPrims.reserve(primitives.size());
        BVHBuildNode *root = iterativeBuild(arena, &totalNodes, orderedPrims);
        primitives.swap(orderedPrims);

        // Compute representation of depth-first traversal of BVH tree
        treeBytes += totalNodes * sizeof(LinearBVHNode) + sizeof(*this) +
                     primitives.size() * sizeof(primitives[0]);
        nodes = AllocAligned<LinearBVHNode>(totalNodes);
        int offset = 0;
        flattenBVHTree(root, &offset);
        CHECK_EQ(totalNodes, offset);
    }

    Bounds3f BVHAccel::WorldBound() const {
        return nodes ? nodes[0].bounds : Bounds3f();
    }

    struct BucketInfo {
        int count = 0;
        Bounds3f bounds;
    };

    BVHBuildNode *BVHAccel::iterativeBuild(MemoryArena &arena, int *totalNodes,
                                           std::vector<std::shared_ptr<Primitive>> &orderedPrims) {
        // Initialize _primitiveInfo_ array for primitives
        std::vector<BVHPrimitiveInfo> primitiveInfo(primitives.size());
        for (size_t i = 0; i < primitives.size(); ++i)
            primitiveInfo[i] = {i, primitives[i]->WorldBound()};


        BVHBuildNode *root = arena.Alloc<BVHBuildNode>();
        std::vector<BVHBuildToDo> stack;
        stack.emplace_back(BVHBuildToDo(root, 0, primitives.size(), nullptr));

        while (!stack.empty()) {
            BVHBuildToDo currentBuildNode = stack.back();
            stack.pop_back();
            CHECK_NE(currentBuildNode.start, currentBuildNode.end);

            (*totalNodes)++;
            // Compute bounds of all primitives in BVH node
            Bounds3f bounds;
            for (int i = currentBuildNode.start; i < currentBuildNode.end; ++i)
                bounds = Union(bounds, primitiveInfo[i].bounds);
            int nPrimitives = currentBuildNode.end - currentBuildNode.start;
            if (nPrimitives == 1) {
                // Create leaf _BVHBuildNode_
                int firstPrimOffset = orderedPrims.size();
                for (int i = currentBuildNode.start; i < currentBuildNode.end; ++i) {
                    int primNum = primitiveInfo[i].primitiveNumber;
                    orderedPrims.push_back(primitives[primNum]);
                }
                currentBuildNode.node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
            } else {
                // Compute bound of primitive centroids, choose split dimension _dim_
                Bounds3f centroidBounds;
                for (int i = currentBuildNode.start; i < currentBuildNode.end; ++i)
                    centroidBounds = Union(centroidBounds, primitiveInfo[i].centroid);
                int dim = centroidBounds.MaximumExtent();

                // Partition primitives into two sets and build children
                int mid = (currentBuildNode.start + currentBuildNode.end) / 2;
                if (centroidBounds.pMax[dim] == centroidBounds.pMin[dim]) {
                    // Create leaf _BVHBuildNode_
                    int firstPrimOffset = orderedPrims.size();
                    for (int i = currentBuildNode.start; i < currentBuildNode.end; ++i) {
                        int primNum = primitiveInfo[i].primitiveNumber;
                        orderedPrims.push_back(primitives[primNum]);
                    }
                    currentBuildNode.node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
                } else {

                    // Partition primitives using approximate SAH
                    if (nPrimitives <= 2) {
                        // Partition primitives into equally-sized subsets
                        mid = (currentBuildNode.start + currentBuildNode.end) / 2;
                        std::nth_element(&primitiveInfo[currentBuildNode.start], &primitiveInfo[mid],
                                         &primitiveInfo[currentBuildNode.end - 1] + 1,
                                         [dim](const BVHPrimitiveInfo &a,
                                               const BVHPrimitiveInfo &b) {
                                             return a.centroid[dim] <
                                                    b.centroid[dim];
                                         });
                    } else {
                        // Allocate _BucketInfo_ for SAH partition buckets
                        PBRT_CONSTEXPR int nBuckets = 12;
                        BucketInfo buckets[nBuckets];

                        // Initialize _BucketInfo_ for SAH partition buckets
                        for (int i = currentBuildNode.start; i < currentBuildNode.end; ++i) {
                            int b = nBuckets *
                                    centroidBounds.Offset(
                                            primitiveInfo[i].centroid)[dim];
                            if (b == nBuckets) b = nBuckets - 1;
                            CHECK_GE(b, 0);
                            CHECK_LT(b, nBuckets);
                            buckets[b].count++;
                            buckets[b].bounds =
                                    Union(buckets[b].bounds, primitiveInfo[i].bounds);
                        }

                        // Compute costs for splitting after each bucket
                        Float cost[nBuckets - 1];
                        for (int i = 0; i < nBuckets - 1; ++i) {
                            Bounds3f b0, b1;
                            int count0 = 0, count1 = 0;
                            for (int j = 0; j <= i; ++j) {
                                b0 = Union(b0, buckets[j].bounds);
                                count0 += buckets[j].count;
                            }
                            for (int j = i + 1; j < nBuckets; ++j) {
                                b1 = Union(b1, buckets[j].bounds);
                                count1 += buckets[j].count;
                            }
                            cost[i] = 1 +
                                      (count0 * b0.SurfaceArea() +
                                       count1 * b1.SurfaceArea()) /
                                      bounds.SurfaceArea();
                        }

                        // Find bucket to split at that minimizes SAH metric
                        Float minCost = cost[0];
                        int minCostSplitBucket = 0;
                        for (int i = 1; i < nBuckets - 1; ++i) {
                            if (cost[i] < minCost) {
                                minCost = cost[i];
                                minCostSplitBucket = i;
                            }
                        }

                        // Either create leaf or split primitives at selected SAH
                        // bucket
                        Float leafCost = nPrimitives;
                        if (nPrimitives > maxPrimsInNode || minCost < leafCost) {
                            BVHPrimitiveInfo *pmid = std::partition(
                                    &primitiveInfo[currentBuildNode.start],
                                    &primitiveInfo[currentBuildNode.end - 1] + 1,
                                    [=](const BVHPrimitiveInfo &pi) {
                                        int b = nBuckets *
                                                centroidBounds.Offset(pi.centroid)[dim];
                                        if (b == nBuckets) b = nBuckets - 1;
                                        CHECK_GE(b, 0);
                                        CHECK_LT(b, nBuckets);
                                        return b <= minCostSplitBucket;
                                    });
                            mid = pmid - &primitiveInfo[0];
                        } else {
                            // Create leaf _BVHBuildNode_
                            int firstPrimOffset = orderedPrims.size();
                            for (int i = currentBuildNode.start; i < currentBuildNode.end; ++i) {
                                int primNum = primitiveInfo[i].primitiveNumber;
                                orderedPrims.push_back(primitives[primNum]);
                            }
                            currentBuildNode.node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
                            continue;
                        }
                    }

                    BVHBuildNode *c0 = arena.Alloc<BVHBuildNode>();
                    BVHBuildNode *c1 = arena.Alloc<BVHBuildNode>();

                    currentBuildNode.node->InitInterior(dim, c0, c1);
                    stack.emplace_back(BVHBuildToDo(c0, currentBuildNode.start, mid, currentBuildNode.node));
                    stack.emplace_back(BVHBuildToDo(c1, mid, currentBuildNode.end, currentBuildNode.node));
                }

            }

        }

        primitiveInfo.resize(0);
        LOG(INFO) << StringPrintf("BVH created with %d nodes for %d "
                                  "primitives (%.2f MB), arena allocated %.2f MB",
                                  *totalNodes, (int) primitives.size(),
                                  float((*totalNodes) * sizeof(LinearBVHNode)) /
                                  (1024.f * 1024.f),
                                  float(arena.TotalAllocated()) /
                                  (1024.f * 1024.f));

        root->calculateBounds();

        return root;
    }

    int BVHAccel::flattenBVHTree(BVHBuildNode *node, int *offset) {
        LinearBVHNode *linearNode = &nodes[*offset];
        linearNode->bounds = node->bounds;

        int myOffset = (*offset)++;
        if (node->nPrimitives > 0) {
            CHECK(!node->children[0] && !node->children[1]);
            CHECK_LT(node->nPrimitives, 65536);
            linearNode->primitivesOffset = node->firstPrimOffset;
            linearNode->nPrimitives = node->nPrimitives;
        } else {
            // Create interior flattened BVH node
            linearNode->axis = node->splitAxis;
            linearNode->nPrimitives = 0;
            flattenBVHTree(node->children[0], offset);
            linearNode->secondChildOffset =
                    flattenBVHTree(node->children[1], offset);
        }
        return myOffset;
    }

    BVHAccel::~BVHAccel() { FreeAligned(nodes); }

    bool BVHAccel::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
        if (!nodes) return false;
        ProfilePhase p(Prof::AccelIntersect);
        bool hit = false;
        Vector3f invDir(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
        int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
        // Follow ray through BVH nodes to find primitive intersections
        int toVisitOffset = 0, currentNodeIndex = 0;
        int nodesToVisit[64];
        while (true) {
            const LinearBVHNode *node = &nodes[currentNodeIndex];
            // Check ray against BVH node
            if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
                if (node->nPrimitives > 0) {
                    // Intersect ray with primitives in leaf BVH node
                    for (int i = 0; i < node->nPrimitives; ++i)
                        if (primitives[node->primitivesOffset + i]->Intersect(
                                ray, isect))
                            hit = true;
                    if (toVisitOffset == 0) break;
                    currentNodeIndex = nodesToVisit[--toVisitOffset];
                } else {
                    // Put far BVH node on _nodesToVisit_ stack, advance to near
                    // node
                    if (dirIsNeg[node->axis]) {
                        nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                        currentNodeIndex = node->secondChildOffset;
                    } else {
                        nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                        currentNodeIndex = currentNodeIndex + 1;
                    }
                }
            } else {
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            }
        }
        return hit;
    }

    bool BVHAccel::IntersectP(const Ray &ray) const {
        if (!nodes) return false;
        ProfilePhase p(Prof::AccelIntersectP);
        Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
        int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
        int nodesToVisit[64];
        int toVisitOffset = 0, currentNodeIndex = 0;
        while (true) {
            const LinearBVHNode *node = &nodes[currentNodeIndex];
            if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
                // Process BVH node _node_ for traversal
                if (node->nPrimitives > 0) {
                    for (int i = 0; i < node->nPrimitives; ++i) {
                        if (primitives[node->primitivesOffset + i]->IntersectP(
                                ray)) {
                            return true;
                        }
                    }
                    if (toVisitOffset == 0) break;
                    currentNodeIndex = nodesToVisit[--toVisitOffset];
                } else {
                    if (dirIsNeg[node->axis]) {
                        /// second child first
                        nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                        currentNodeIndex = node->secondChildOffset;
                    } else {
                        nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                        currentNodeIndex = currentNodeIndex + 1;
                    }
                }
            } else {
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            }
        }
        return false;
    }

    std::shared_ptr<BVHAccel> CreateBVHAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        int maxPrimsInNode = ps.FindOneInt("maxnodeprims", 4);
        return std::make_shared<BVHAccel>(std::move(prims), maxPrimsInNode);
    }

}  // namespace pbrt
