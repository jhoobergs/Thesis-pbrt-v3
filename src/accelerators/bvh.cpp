
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

        int depth(){
            if(nPrimitives > 0)
                return 1;
            return 1 + std::max(children[0]->depth(), children[1]->depth());
        }

        Bounds3f bounds;
        BVHBuildNode *children[2];
        int splitAxis, firstPrimOffset, nPrimitives;
    };

    struct BVHBuildCentroid {
        // Centroid Public Methods
        BVHBuildCentroid() {}

        BVHBuildCentroid(Float t, int primOffset, int primNum) : t(t), primOffset(primOffset), primNum(primNum) {}

        Float t;
        int primOffset;
        int primNum;
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
    BVHAccel::BVHAccel(std::vector<std::shared_ptr<Primitive>> p, int isectCost, int traversalCost,
                       int maxPrimsInNode)
            : maxPrimsInNode(std::min(255, maxPrimsInNode)),
              traversalCost(traversalCost),
              isectCost(isectCost),
              primitives(std::move(p)) {
        Warning("%d %d", traversalCost, isectCost);
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

        BVHBuildCentroid centroids[3][primitives.size()];

        BVHBuildNode *root = arena.Alloc<BVHBuildNode>();
        std::vector<BVHBuildToDo> stack;
        stack.emplace_back(BVHBuildToDo(root, 0, primitives.size(), nullptr));

        while (!stack.empty()) {
            BVHBuildToDo currentBuildNode = stack.back();
            stack.pop_back();
            CHECK_NE(currentBuildNode.start, currentBuildNode.end);

            (*totalNodes)++;
            // Compute bounds of all primitives in BVH node
            Bounds3f bounds = Bounds3f();
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
                // Choose split axis position for interior node
                int bestAxis = -1, bestOffset = -1, bestPrimNum = -1;
                Float bestCost = Infinity;
                Float oldCost = isectCost * Float(nPrimitives);
                Float totalSA = bounds.SurfaceArea();
                Float invTotalSA = 1 / totalSA;

                for (int dim = 0; dim < 3; dim++) {
                    for (int i = 0; i < nPrimitives; ++i) {
                        int pn = primitiveInfo[currentBuildNode.start +  i].primitiveNumber;
                        centroids[dim][i] = BVHBuildCentroid(primitiveInfo[currentBuildNode.start + i].centroid[dim], currentBuildNode.start + i, pn);
                    }

                    // Sort _edges_ for _axis_
                    std::sort(&centroids[dim][0], &centroids[dim][nPrimitives],
                              [](const BVHBuildCentroid &e0, const BVHBuildCentroid &e1) -> bool {
                                  if (e0.t == e1.t)
                                      return (int) e0.primNum < (int) e1.primNum;
                                  else
                                      return e0.t < e1.t;
                              });

                    Bounds3f currentRightToLeftBounds;
                    Bounds3f rightToLeftBounds[nPrimitives];
                    for (int i = nPrimitives - 2; i >= 0; i--) {
                        int primOffset = centroids[dim][i].primOffset;
                        currentRightToLeftBounds = Union(currentRightToLeftBounds, primitiveInfo[primOffset].bounds);
                        rightToLeftBounds[i] = currentRightToLeftBounds;
                    }

                    Bounds3f currentLeftToRightBounds;
                    for (int i = 0; i < nPrimitives-1; ++i) {
                        int primOffset = centroids[dim][i].primOffset;
                        currentLeftToRightBounds = Union(currentLeftToRightBounds, primitiveInfo[primOffset].bounds);
                        float cost = traversalCost + isectCost *
                                                     ((i+1) * currentLeftToRightBounds.SurfaceArea() +
                                                      (nPrimitives - i - 1) * rightToLeftBounds[i].SurfaceArea()) *
                                                     invTotalSA;

                        if (cost < bestCost) {
                            bestCost = cost;
                            bestAxis = dim;
                            bestOffset = primOffset;
                            bestPrimNum = centroids[dim][i].primNum;
                        }

                    }

                }

                if (bestAxis != -1 && (bestCost < oldCost || nPrimitives > maxPrimsInNode)) {
                    BVHBuildNode *c0 = arena.Alloc<BVHBuildNode>();
                    BVHBuildNode *c1 = arena.Alloc<BVHBuildNode>();
                    CHECK_EQ(primitiveInfo[bestOffset].primitiveNumber, bestPrimNum);
                    /* BVHPrimitiveInfo *pmid = std::partition(
                            &primitiveInfo[currentBuildNode.start],
                            &primitiveInfo[currentBuildNode.end-1]+1,
                            [=](const BVHPrimitiveInfo &pi) {
                                return pi.centroid[bestAxis] < primitiveInfo[bestOffset].centroid[bestAxis] ||
                                       (pi.centroid[bestAxis] == primitiveInfo[bestOffset].centroid[bestAxis] &&
                                        pi.primitiveNumber <= bestPrimNum);
                            });
                    int mid = pmid - &primitiveInfo[0]; */
                    int mid = currentBuildNode.start;
                    for(int i=currentBuildNode.start;i<currentBuildNode.end;++i) {
                        if(primitiveInfo[i].centroid[bestAxis] < primitiveInfo[bestOffset].centroid[bestAxis] ||
                        (primitiveInfo[i].centroid[bestAxis] == primitiveInfo[bestOffset].centroid[bestAxis] && primitiveInfo[i].primitiveNumber <= bestPrimNum)) {
                            std::swap( primitiveInfo[i], primitiveInfo[mid]);
                            ++mid;
                        }
                    }

                    currentBuildNode.node->InitInterior(bestAxis, c0, c1);
                    stack.emplace_back(BVHBuildToDo(c0, currentBuildNode.start, mid, currentBuildNode.node));
                    stack.emplace_back(BVHBuildToDo(c1, mid, currentBuildNode.end, currentBuildNode.node));
                } else {
                    //Create leaf
                    int firstPrimOffset = orderedPrims.size();
                    for (int i = currentBuildNode.start; i < currentBuildNode.end; ++i) {
                        int primNum = primitiveInfo[i].primitiveNumber;
                        orderedPrims.push_back(primitives[primNum]);
                    }
                    currentBuildNode.node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
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
        Warning("Depth %d", root->depth());

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
        int isectCost = ps.FindOneInt("intersectcost", 8);
        int travCost = ps.FindOneInt("traversalcost", 1);
        return std::make_shared<BVHAccel>(std::move(prims), isectCost, travCost, maxPrimsInNode);
    }

}  // namespace pbrt
