#include <utility>


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
#include "paramset.h"
#include "stats.h"
#include "parallel.h"

namespace pbrt {

    STAT_MEMORY_COUNTER("Memory/BVH tree", treeBytes);
    STAT_RATIO("BVH/Primitives per leaf node", totalPrimitives, totalLeafNodes);
    STAT_COUNTER("BVH/Interior nodes", interiorNodes);
    STAT_COUNTER("BVH/Leaf nodes", leafNodes);
    STAT_COUNTER("BVH node traversals during intersect", nbNodeTraversals);
    STAT_COUNTER("BVH node traversals during intersectP", nbNodeTraversalsP);


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
        void InitLeaf(uint32_t first, uint32_t n, const Bounds3f &b) {
            firstPrimOffset = first;
            nPrimitives = n;
            bounds = b;
            children[0] = children[1] = nullptr;
            leaf = true;
            ++leafNodes;
            ++totalLeafNodes;
            totalPrimitives += n;
        }

        void InitInterior(uint32_t axis, BVHBuildNode *c0, BVHBuildNode *c1, const Bounds3f &b, uint32_t nPrims) {
            children[0] = c0;
            children[1] = c1;
            splitAxis = axis;
            nPrimitives = nPrims;
            bounds = b;
            leaf = false;
            ++interiorNodes;
        }

        int depth() {
            if (leaf)
                return 1;
            return 1 + std::max(children[0]->depth(), children[1]->depth());
        }

        Bounds3f bounds;
        BVHBuildNode *children[2];
        uint32_t splitAxis, firstPrimOffset, nPrimitives;
        bool leaf;
    };

    struct BVHBuildCentroid {
        // Centroid Public Methods
        BVHBuildCentroid() {}

        BVHBuildCentroid(Float t, uint32_t primOffset, uint32_t primNum) : t(t), primOffset(primOffset),
                                                                           primNum(primNum) {}

        Float t;
        uint32_t primOffset;
        uint32_t primNum;
    };

    struct LinearBVHNode {
        uint32_t nPrimitives() const { return nPrims >> 2; }

        uint32_t SplitAxis() const { return axis & 3; }

        bool IsLeaf() const { return (axis & 3) == 3; }

        void InitLeaf(uint32_t nPrimitives, Bounds3f boundsf, int primOffset) {
            axis = 3;
            nPrims |= (nPrimitives << 2);
            primitivesOffset = primOffset;
            bounds = std::move(boundsf);
        }

        void InitInterior(uint32_t nPrimitives, Bounds3f boundsf, uint32_t axs) {
            axis = axs;
            nPrims |= (nPrimitives << 2);
            bounds = std::move(boundsf);
        }

        Bounds3f bounds;
        union {
            int primitivesOffset;   // leaf
            int secondChildOffset;  // interior
        };
        union {
            uint32_t nPrims;  //
            uint32_t axis;          // interior node: xyz
        };
    };

    // BVHAccel Method Definitions
    BVHAccel::BVHAccel(std::vector<std::shared_ptr<Primitive>> p, int isectCost, int traversalCost,
                       int maxPrimsInNode)
            : maxPrimsInNode(std::min(255, maxPrimsInNode)),
              traversalCost(traversalCost),
              isectCost(isectCost),
              primitives(std::move(p)) {
        ProfilePhase _(Prof::AccelConstruction);
        if (primitives.empty()) return;
        // Build BVH from _primitives_

        // Build BVH tree for primitives using _primitiveInfo_
        MemoryArena arena(1024 * 1024);
        int totalNodes = 0;
        std::vector<std::shared_ptr<Primitive>> orderedPrims;
        orderedPrims.reserve(primitives.size());
        primNumMapping.reserve(primitives.size());
        for (const auto &prim: primitives)
            primNumMapping.emplace_back(0);

        BVHBuildNode *root = iterativeBuild(arena, &totalNodes, orderedPrims);

        for(int i = 0; i < primNumMapping.size(); ++i){
            for(int j = i+1; j < primNumMapping.size(); ++j){
                CHECK_NE(primNumMapping[i], primNumMapping[j]);
            }
        }

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

        std::unique_ptr<BVHBuildCentroid[]> centroids[3];
        for (uint32_t i = 0; i < 3; ++i)
            centroids[i].reset(new BVHBuildCentroid[primitives.size()]);
        std::unique_ptr<Bounds3f[]> rightToLeftBounds(
                new Bounds3f[primitives.size()]); // rightToLeftBounds[i] equals the bounds of all triangle to the right of triangle i, including triangle i
        std::unique_ptr<Bounds3f[]> leftToRightBounds(
                new Bounds3f[primitives.size()]); // leftToRightBounds[i] equals the bounds of all triangle to the left of triangle i, including triangle i

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

            uint32_t nPrimitives = currentBuildNode.end - currentBuildNode.start;
            if (nPrimitives == 1) {
                // Create leaf _BVHBuildNode_
                uint32_t firstPrimOffset = orderedPrims.size();
                uint32_t primNum = primitiveInfo[currentBuildNode.start].primitiveNumber;
                orderedPrims.push_back(primitives[primNum]);
                primNumMapping[firstPrimOffset] = primNum;
                currentBuildNode.node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
            } else {
                // Choose split axis position for interior node
                uint32_t bestAxis = -1, bestOffset = -1, bestPrimNum = -1;
                Bounds3f bestBounds;
                Float bestCost = Infinity;
                Float oldCost = isectCost * Float(nPrimitives);
                Float totalSA = bounds.SurfaceArea();
                Float invTotalSA = 1 / totalSA;

                for (uint32_t dim = 0; dim < 3; dim++) {
                    for (uint32_t i = 0; i < nPrimitives; ++i) {
                        uint32_t pn = primitiveInfo[currentBuildNode.start + i].primitiveNumber;
                        centroids[dim][i] = BVHBuildCentroid(primitiveInfo[currentBuildNode.start + i].centroid[dim],
                                                             currentBuildNode.start + i, pn);
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
                    for (int i = nPrimitives - 1; i >= 0; i--) {
                        int primOffset = centroids[dim][i].primOffset;
                        currentRightToLeftBounds = Union(currentRightToLeftBounds, primitiveInfo[primOffset].bounds);
                        rightToLeftBounds[i] = currentRightToLeftBounds;
                    }

                    Bounds3f currentLeftToRightBounds;
                    for (int i = 0; i < nPrimitives - 1; ++i) {
                        int primOffset = centroids[dim][i].primOffset;
                        currentLeftToRightBounds = Union(currentLeftToRightBounds, primitiveInfo[primOffset].bounds);
                        leftToRightBounds[i] = currentLeftToRightBounds;
                    }


                    for (int i = 0; i < nPrimitives - 1; ++i) {
                        int primOffset = centroids[dim][i].primOffset;
                        float cost = traversalCost + isectCost *
                                                     ((i + 1) * leftToRightBounds[i].SurfaceArea() +
                                                      (nPrimitives - i - 1) * rightToLeftBounds[i + 1].SurfaceArea()) *
                                                     invTotalSA;

                        if (cost < bestCost) {
                            bestCost = cost;
                            bestAxis = dim;
                            bestOffset = primOffset;
                            bestPrimNum = centroids[dim][i].primNum;
                            bestBounds = rightToLeftBounds[0];
                        }
                    }
                }

                if (bestAxis != -1 && (bestCost < oldCost || nPrimitives > maxPrimsInNode)) {
                    auto *c0 = arena.Alloc<BVHBuildNode>();
                    auto *c1 = arena.Alloc<BVHBuildNode>();
                    const BVHPrimitiveInfo bestPrimitive = primitiveInfo[bestOffset];
                    const float bestCentroid = bestPrimitive.centroid[bestAxis];
                    CHECK_EQ(bestPrimitive.primitiveNumber, bestPrimNum);
                    BVHPrimitiveInfo *pmid = std::partition(
                            &primitiveInfo[currentBuildNode.start],
                            &primitiveInfo[currentBuildNode.end - 1] + 1,
                            [&](const BVHPrimitiveInfo &pi) {
                                return pi.centroid[bestAxis] < bestCentroid ||
                                       (pi.centroid[bestAxis] == bestCentroid &&
                                        pi.primitiveNumber <= bestPrimNum);
                            });
                    uint32_t mid = pmid - &primitiveInfo[0];

                    currentBuildNode.node->InitInterior(bestAxis, c0, c1, bestBounds, nPrimitives);
                    stack.emplace_back(BVHBuildToDo(c0, currentBuildNode.start, mid, currentBuildNode.node));
                    stack.emplace_back(BVHBuildToDo(c1, mid, currentBuildNode.end, currentBuildNode.node));
                } else {
                    //Create leaf
                    uint32_t firstPrimOffset = orderedPrims.size();
                    for (int i = currentBuildNode.start; i < currentBuildNode.end; ++i) {
                        uint32_t primNum = primitiveInfo[i].primitiveNumber;
                        orderedPrims.push_back(primitives[primNum]);
                        primNumMapping[orderedPrims.size() - 1] = primNum;
                    }
                    currentBuildNode.node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
                }
            }
        }
        LOG(INFO) << StringPrintf("BVH created with %d nodes for %d "
                                  "primitives (%.2f MB), arena allocated %.2f MB",
                                  *totalNodes, (int) primitives.size(),
                                  float((*totalNodes) * sizeof(LinearBVHNode)) /
                                  (1024.f * 1024.f),
                                  float(arena.TotalAllocated()) /
                                  (1024.f * 1024.f));

        //Warning("BVH Depth %d", root->depth());
        CHECK_EQ(root->nPrimitives, primitives.size());
        return root;
    }

    int BVHAccel::flattenBVHTree(BVHBuildNode *node, int *offset) {
        LinearBVHNode *linearNode = &nodes[*offset];
        int myOffset = (*offset)++;
        if (node->leaf) {
            CHECK(!node->children[0] && !node->children[1]);
            CHECK_LT(node->nPrimitives, 65536); // Not really needed anymore
            linearNode->InitLeaf(node->nPrimitives, node->bounds, node->firstPrimOffset);
        } else {
            // Create interior flattened BVH node
            linearNode->InitInterior(node->nPrimitives, node->bounds, node->splitAxis);
            flattenBVHTree(node->children[0], offset);
            linearNode->secondChildOffset = flattenBVHTree(node->children[1], offset);
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
            nbNodeTraversals++;
            ray.stats.bvhTreeNodeTraversals++;
            // Check ray against BVH node
            if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
                if (node->IsLeaf()) {
                    // Intersect ray with primitives in leaf BVH node
                    for (int i = 0; i < node->nPrimitives(); ++i)
                        if (primitives[node->primitivesOffset + i]->Intersect(
                                ray, isect))
                            hit = true;
                    if (toVisitOffset == 0) break;
                    currentNodeIndex = nodesToVisit[--toVisitOffset];
                } else {
                    // Put far BVH node on _nodesToVisit_ stack, advance to near
                    // node
                    if (dirIsNeg[node->SplitAxis()]) {
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
            nbNodeTraversalsP++;
            ray.stats.bvhTreeNodeTraversalsP++;
            if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
                // Process BVH node _node_ for traversal
                if (node->IsLeaf()) {
                    for (int i = 0; i < node->nPrimitives(); ++i) {
                        if (primitives[node->primitivesOffset + i]->IntersectP(
                                ray)) {
                            return true;
                        }
                    }
                    if (toVisitOffset == 0) break;
                    currentNodeIndex = nodesToVisit[--toVisitOffset];
                } else {
                    if (dirIsNeg[node->SplitAxis()]) {
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

    std::pair<uint32_t, uint32_t> BVHAccel:: getAmountToLeftAndRight(const Plane &p) const {
        uint32_t left = 0, right = 0;
        uint32_t currentNodeIndex = 0;
        LinearBVHNode *node;
        std::vector<uint32_t> stack;
        stack.emplace_back(0);
        while (!stack.empty()) {
            currentNodeIndex = stack.back();
            stack.pop_back();
            node = &nodes[currentNodeIndex];
            Float maxDiff = node->bounds.Diagonal().Length() / 2;
            Point3f center = node->bounds.pMin + node->bounds.Diagonal() / 2;
            Float centerProjection = Dot(p.axis, center);
            if (centerProjection + maxDiff < p.t) {
                left += node->nPrimitives();
            } else if (centerProjection - maxDiff > p.t) {
                right += node->nPrimitives();
            } else if (node->IsLeaf()) {
                uint32_t oldL = left, oldR = right;
                for (int i = 0; i < node->nPrimitives(); ++i) {
                    Boundsf bounds = primitives[node->primitivesOffset + i]->getBounds(p.axis);
                    if (bounds.min <= p.t)
                        left += 1;
                    if (bounds.max >= p.t)
                        right += 1;
                    //Warning("%f %f %f", bounds.min, bounds.max, p.t);
                    CHECK(oldL + oldR + i+1 <= left + right);
                }
            } else {
                stack.emplace_back(currentNodeIndex + 1);
                stack.emplace_back(node->secondChildOffset);
            }
        }
        //Warning("%d %d %d", left, right, primitives.size());
        CHECK(left + right >= primitives.size());
        return std::make_pair(left, right);
    }

    void BVHAccel::getPrimnumsToLeftAndRight(const Plane &p, std::vector<uint32_t> &left, std::vector<uint32_t> &right) const {
        std::pair<uint32_t, uint8_t> currentNodeData = std::make_pair(0, 0);
        LinearBVHNode *node;
        std::vector<std::pair<uint32_t, uint8_t>> stack; // (nodeIndex, state: 0=unknown, 1=shouldGoLeft, 2=shouldGoRight)
        stack.emplace_back(currentNodeData);
        while (!stack.empty()) {
            currentNodeData = stack.back();
            stack.pop_back();
            node = &nodes[currentNodeData.first];
            if (currentNodeData.second == 0) {
                Float maxDiff = node->bounds.Diagonal().Length() / 2;
                Point3f center = node->bounds.pMin + node->bounds.Diagonal() / 2;
                float centerProjection = Dot(p.axis, center);
                if (centerProjection + maxDiff < p.t) {
                    if (node->IsLeaf()) {
                        for (int i = 0; i < node->nPrimitives(); ++i)
                            left.emplace_back(primNumMapping[node->primitivesOffset + i]);
                    } else {
                        stack.emplace_back(std::make_pair(currentNodeData.first + 1, 1));
                        stack.emplace_back(std::make_pair(node->secondChildOffset, 1));
                    }
                } else if (centerProjection - maxDiff > p.t) {
                    if (node->IsLeaf()) {
                        for (int i = 0; i < node->nPrimitives(); ++i)
                            right.emplace_back(primNumMapping[node->primitivesOffset + i]);
                    } else {
                        stack.emplace_back(std::make_pair(currentNodeData.first + 1, 2));
                        stack.emplace_back(std::make_pair(node->secondChildOffset, 2));
                    }
                } else if (node->IsLeaf()) {
                    for (int i = 0; i < node->nPrimitives(); ++i) {
                        Boundsf bounds = primitives[node->primitivesOffset + i]->getBounds(p.axis);
                        if (bounds.min <= p.t)
                            left.emplace_back(primNumMapping[node->primitivesOffset + i]);
                        if (bounds.max >= p.t)
                            right.emplace_back(primNumMapping[node->primitivesOffset + i]);
                    }
                } else {
                    stack.emplace_back(std::make_pair(currentNodeData.first + 1, 0));
                    stack.emplace_back(std::make_pair(node->secondChildOffset, 0));
                }
            } else if (node->IsLeaf()) {
                if (currentNodeData.second == 1)
                    for (int i = 0; i < node->nPrimitives(); ++i)
                        left.emplace_back(primNumMapping[node->primitivesOffset + i]);

                else if (currentNodeData.second == 2)
                    for (int i = 0; i < node->nPrimitives(); ++i)
                        right.emplace_back(primNumMapping[node->primitivesOffset + i]);
            } else {
                stack.emplace_back(std::make_pair(currentNodeData.first + 1, currentNodeData.second));
                stack.emplace_back(std::make_pair(node->secondChildOffset, currentNodeData.second));
            }
        }
    }

    std::shared_ptr<BVHAccel> CreateBVHAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        int maxPrimsInNode = ps.FindOneInt("maxnodeprims", 4);
        int isectCost = ps.FindOneInt("intersectcost", 8);
        int travCost = ps.FindOneInt("traversalcost", 1);
        return std::make_shared<BVHAccel>(std::move(prims), isectCost, travCost, maxPrimsInNode);
    }

}  // namespace pbrt
