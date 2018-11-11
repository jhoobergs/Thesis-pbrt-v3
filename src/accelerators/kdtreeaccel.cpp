
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


// accelerators/kdtreeaccel.cpp*
#include "accelerators/kdtreeaccel.h"
#include "paramset.h"
#include "interaction.h"
#include "stats.h"
#include <algorithm>

namespace pbrt {

    STAT_COUNTER("Accelerator/Kd-tree node traversals during intersect", nbNodeTraversals);
    STAT_COUNTER("Accelerator/Kd-tree node traversals during intersectP", nbNodeTraversalsP);

    // KdTreeAccel Local Declarations
    struct KdAccelNode {
        // KdAccelNode Methods
        void InitLeaf(int *primNums, int np, std::vector<int> *primitiveIndices);

        void InitInterior(int axis, Float s) {
            split = s;
            flags = axis;
        }

        void setAboveChild(int ac) {
            aboveChild |= (ac << 2);
        }

        Float SplitPos() const { return split; }

        int nPrimitives() const { return nPrims >> 2; }

        int SplitAxis() const { return flags & 3; }

        bool IsLeaf() const { return (flags & 3) == 3; }

        int AboveChild() const { return aboveChild >> 2; }

        uint32_t depth(KdAccelNode *nodes, int id=0) {
            if(IsLeaf())
                return 0;
            else
                return 1 + std::max(nodes[AboveChild()].depth(nodes, AboveChild()), nodes[id+1].depth(nodes, id+1));
        }

        union {
            Float split;                 // Interior
            int onePrimitive;            // Leaf
            int primitiveIndicesOffset;  // Leaf
        };

    private:
        union {
            int flags;       // Both
            int nPrims;      // Leaf
            int aboveChild;  // Interior
        };
    };

    enum class EdgeType {
        Start, End
    };

    struct BoundEdge {
        // BoundEdge Public Methods
        BoundEdge() {}

        BoundEdge(Float t, int primNum, bool starting) : t(t), primNum(primNum) {
            type = starting ? EdgeType::Start : EdgeType::End;
        }

        Float t;
        int primNum;
        EdgeType type;
    };

    struct KdBuildNode {
        KdBuildNode(int depth, int nPrimitives, int badRefines, Bounds3f nodeBounds, int *primNums, int parentNum = -1)
                : depth(depth),
                  nPrimitives(nPrimitives), badRefines(badRefines), nodeBounds(std::move(nodeBounds)),
                  primNums(primNums), parentNum(parentNum) {}

        int depth;
        int nPrimitives;
        int badRefines;
        Bounds3f nodeBounds;
        int *primNums;
        int parentNum = -1;
    };

    // KdTreeAccel Method Definitions
    KdTreeAccel::KdTreeAccel(std::vector<std::shared_ptr<Primitive>> p,
                             int isectCost, int traversalCost, Float emptyBonus,
                             int maxPrims, int maxDepth)
            : isectCost(isectCost),
              traversalCost(traversalCost),
              maxPrims(maxPrims),
              emptyBonus(emptyBonus),
              primitives(std::move(p)) {
        // Build kd-tree for accelerator
        ProfilePhase _(Prof::AccelConstruction);
        nextFreeNode = nAllocedNodes = 0;
        if (maxDepth <= 0)
            maxDepth = std::round(8 + 1.3f * Log2Int(int64_t(primitives.size())));

        // Compute bounds for kd-tree construction
        std::vector<Bounds3f> primBounds;
        primBounds.reserve(primitives.size());
        for (const std::shared_ptr<Primitive> &prim : primitives) {
            Bounds3f b = prim->WorldBound();
            bounds = Union(bounds, b);
            primBounds.push_back(b);
        }

        // Start recursive construction of kd-tree
        buildTree(bounds, primBounds, maxDepth);
    }

    // TODO uint32 -> u32 type
    void KdAccelNode::InitLeaf(int *primNums, int np,
                               std::vector<int> *primitiveIndices) {
        flags = 3;
        nPrims |= (np << 2u);
        // Store primitive ids for leaf node
        if (np == 0)
            onePrimitive = 0;
        else if (np == 1)
            onePrimitive = primNums[0];
        else {
            primitiveIndicesOffset = primitiveIndices->size();
            for (int i = 0; i < np; ++i) primitiveIndices->push_back(primNums[i]);
        }
    }

    KdTreeAccel::~KdTreeAccel() { FreeAligned(nodes); }

    void KdTreeAccel::buildTree(Bounds3f &rootNodeBounds,
                                const std::vector<Bounds3f> &allPrimBounds,
                                int maxDepth) {
        int nodeNum = 0;
        // Allocate working memory for kd-tree construction
        std::unique_ptr<BoundEdge[]> edges[3];
        for (int i = 0; i < 3; ++i)
            edges[i].reset(new BoundEdge[2 * primitives.size()]);
        std::unique_ptr<int> prims_p(
                new int[(maxDepth + 1) * primitives.size()]);
        int *prims = prims_p.get();
        // Initialize _primNums_ for kd-tree construction
        for (size_t i = 0; i < primitives.size(); ++i) {
            prims[i] = i;
        }
        int maxPrimsOffset = 0;

        std::vector<KdBuildNode> stack;
        stack.emplace_back(KdBuildNode(maxDepth, (int) primitives.size(), 0, rootNodeBounds, prims));

        while (!stack.empty()) {

            KdBuildNode currentBuildNode = stack.back();
            stack.pop_back();
            CHECK_EQ(nodeNum, nextFreeNode);

            if (currentBuildNode.parentNum != -1)
                nodes[currentBuildNode.parentNum].setAboveChild(nodeNum);

            maxPrimsOffset = std::max(maxPrimsOffset, (int) (currentBuildNode.primNums - prims));

            // Get next free node from _nodes_ array
            if (nextFreeNode == nAllocedNodes) {
                int nNewAllocNodes = std::max(2 * nAllocedNodes, 512);
                KdAccelNode *n = AllocAligned<KdAccelNode>(nNewAllocNodes);
                if (nAllocedNodes > 0) {
                    memcpy(n, nodes, nAllocedNodes * sizeof(KdAccelNode));
                    FreeAligned(nodes);
                }
                nodes = n;
                nAllocedNodes = nNewAllocNodes;
            }
            ++nextFreeNode;

            // Initialize leaf node if termination criteria met
            if (currentBuildNode.nPrimitives <= maxPrims || currentBuildNode.depth == 0) {
                nodes[nodeNum++].InitLeaf(currentBuildNode.primNums, currentBuildNode.nPrimitives, &primitiveIndices);
                continue;
            }

            // Initialize interior node and continue recursion

            // Choose split axis position for interior node
            int bestAxis = -1, bestOffset = -1;
            Float bestCost = Infinity;
            Float oldCost = isectCost * Float(currentBuildNode.nPrimitives);
            Float totalSA = currentBuildNode.nodeBounds.SurfaceArea();
            const Float invTotalSA = 1 / totalSA;
            const Vector3f d = currentBuildNode.nodeBounds.pMax - currentBuildNode.nodeBounds.pMin;

            for (int axis = 0; axis < 3; ++axis) {
                // Initialize edges for _axis_
                for (int i = 0; i < currentBuildNode.nPrimitives; ++i) {
                    const int pn = currentBuildNode.primNums[i];
                    const Bounds3f &bounds = allPrimBounds[pn];
                    edges[axis][2 * i] = BoundEdge(bounds.pMin[axis], pn, true);
                    edges[axis][2 * i + 1] = BoundEdge(bounds.pMax[axis], pn, false);
                }

                // Sort _edges_ for _axis_
                std::sort(&edges[axis][0], &edges[axis][2 * currentBuildNode.nPrimitives],
                          [](const BoundEdge &e0, const BoundEdge &e1) -> bool {
                              if (e0.t == e1.t)
                                  return (int) e0.type < (int) e1.type;
                              else
                                  return e0.t < e1.t;
                          });

                // Compute cost of all splits for _axis_ to find best
                int nBelow = 0, nAbove = currentBuildNode.nPrimitives;
                for (int i = 0; i < 2 * currentBuildNode.nPrimitives; ++i) {
                    if (edges[axis][i].type == EdgeType::End) --nAbove;
                    const Float edgeT = edges[axis][i].t;

                    if (edgeT > currentBuildNode.nodeBounds.pMin[axis] &&
                        edgeT < currentBuildNode.nodeBounds.pMax[axis]) {
                        // Compute cost for split at _i_th edge

                        // Compute child surface areas for split at _edgeT_
                        const int otherAxis0 = (axis + 1) % 3, otherAxis1 = (axis + 2) % 3;
                        const Float belowSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                                             (edgeT - currentBuildNode.nodeBounds.pMin[axis]) *
                                             (d[otherAxis0] + d[otherAxis1]));
                        const Float aboveSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                                             (currentBuildNode.nodeBounds.pMax[axis] - edgeT) *
                                             (d[otherAxis0] + d[otherAxis1]));
                        const Float pBelow = belowSA * invTotalSA;
                        const Float pAbove = aboveSA * invTotalSA;
                        const Float eb = (nAbove == 0 || nBelow == 0) ? emptyBonus : 0;
                        const Float cost =
                                traversalCost +
                                isectCost * (1 - eb) * (pBelow * nBelow + pAbove * nAbove);

                        // Update best split if this is lowest cost so far
                        if (cost < bestCost) {
                            bestCost = cost;
                            bestAxis = axis;
                            bestOffset = i;
                        }
                    }
                    if (edges[axis][i].type == EdgeType::Start) ++nBelow;
                }
                CHECK(nBelow == currentBuildNode.nPrimitives && nAbove == 0);
            }

            // Create leaf if no good splits were found
            if (bestCost > oldCost) ++currentBuildNode.badRefines;
            if ((bestCost > 4 * oldCost && currentBuildNode.nPrimitives < 16) || bestAxis == -1 ||
                currentBuildNode.badRefines == 3) {
                nodes[nodeNum++].InitLeaf(currentBuildNode.primNums, currentBuildNode.nPrimitives, &primitiveIndices);
                continue;
            }

            // Classify primitives with respect to split
            int n0 = 0, n1 = 0;
            int *prims1 = currentBuildNode.primNums; // prims1 needs to be put in de array first, so it isn't overriden by child 0
            for (int i = bestOffset + 1; i < 2 * currentBuildNode.nPrimitives; ++i)
                if (edges[bestAxis][i].type == EdgeType::End)
                    prims1[n1++] = edges[bestAxis][i].primNum;

            int *prims0 = prims1 + n1;
            for (int i = 0; i < bestOffset; ++i)
                if (edges[bestAxis][i].type == EdgeType::Start)
                    prims0[n0++] = edges[bestAxis][i].primNum;

            // Add child nodes to stack
            const Float tSplit = edges[bestAxis][bestOffset].t;
            Bounds3f bounds0 = currentBuildNode.nodeBounds, bounds1 = currentBuildNode.nodeBounds;
            bounds0.pMax[bestAxis] = bounds1.pMin[bestAxis] = tSplit;

            nodes[nodeNum].InitInterior(bestAxis, tSplit);

            stack.emplace_back(
                    KdBuildNode(currentBuildNode.depth - 1, n1, currentBuildNode.badRefines, bounds1, prims1, nodeNum));
            stack.emplace_back(
                    KdBuildNode(currentBuildNode.depth - 1, n0, currentBuildNode.badRefines, bounds0, prims0));
            ++nodeNum;
        }

        Warning("Depth %d", nodes[0].depth(nodes, 0));
    }

    bool KdTreeAccel::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
        ProfilePhase p(Prof::AccelIntersect);
        // Compute initial parametric range of ray inside kd-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse kd-tree for ray
        Vector3f invDir(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
        PBRT_CONSTEXPR int maxTodo = 64;
        KdToDo todo[maxTodo];
        int todoPos = 0;

        // Traverse kd-tree nodes in order for ray
        bool hit = false;
        const KdAccelNode *node = &nodes[0];
        while (node != nullptr) {
            // Bail out if we found a hit closer than the current node
            if (ray.tMax < tMin) break;
            nbNodeTraversals++;
            ray.stats.kdTreeNodeTraversals++;
            if (!node->IsLeaf()) {
                // Process kd-tree interior node

                // Compute parametric distance along ray to split plane
                int axis = node->SplitAxis();
                Float tPlane = (node->SplitPos() - ray.o[axis]) * invDir[axis];

                // Get node children pointers for ray
                const KdAccelNode *firstChild, *secondChild;
                int belowFirst =
                        (ray.o[axis] < node->SplitPos()) ||
                        (ray.o[axis] == node->SplitPos() && ray.d[axis] <= 0);
                if (belowFirst) {
                    firstChild = node + 1;
                    secondChild = &nodes[node->AboveChild()];
                } else {
                    firstChild = &nodes[node->AboveChild()];
                    secondChild = node + 1;
                }

                // Advance to next child node, possibly enqueue other child
                if (tPlane > tMax || tPlane <= 0)
                    node = firstChild;
                else if (tPlane < tMin)
                    node = secondChild;
                else {
                    // Enqueue _secondChild_ in todo list
                    todo[todoPos].node = secondChild;
                    todo[todoPos].tMin = tPlane;
                    todo[todoPos].tMax = tMax;
                    ++todoPos;
                    node = firstChild;
                    tMax = tPlane;
                }
            } else {
                // Check for intersections inside leaf node
                int nPrimitives = node->nPrimitives();
                if (nPrimitives == 1) {
                    const std::shared_ptr<Primitive> &p =
                            primitives[node->onePrimitive];
                    // Check one primitive inside leaf node
                    if (p->Intersect(ray, isect)) hit = true;
                } else {
                    for (int i = 0; i < nPrimitives; ++i) {
                        int index =
                                primitiveIndices[node->primitiveIndicesOffset + i];
                        const std::shared_ptr<Primitive> &p = primitives[index];
                        // Check one primitive inside leaf node
                        if (p->Intersect(ray, isect)) hit = true;
                    }
                }

                // Grab next node to process from todo list
                if (todoPos > 0) {
                    --todoPos;
                    node = todo[todoPos].node;
                    tMin = todo[todoPos].tMin;
                    tMax = todo[todoPos].tMax;
                } else
                    break;
            }
        }
        return hit;
    }

    bool KdTreeAccel::IntersectP(const Ray &ray) const {
        ProfilePhase p(Prof::AccelIntersectP);
        // Compute initial parametric range of ray inside kd-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse kd-tree for ray
        Vector3f invDir(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
        PBRT_CONSTEXPR int maxTodo = 64;
        KdToDo todo[maxTodo];
        int todoPos = 0;
        const KdAccelNode *node = &nodes[0];
        while (node != nullptr) {
            nbNodeTraversalsP++;
            ray.stats.kdTreeNodeTraversalsP++;
            if (node->IsLeaf()) {
                // Check for shadow ray intersections inside leaf node
                int nPrimitives = node->nPrimitives();
                if (nPrimitives == 1) {
                    const std::shared_ptr<Primitive> &p =
                            primitives[node->onePrimitive];
                    if (p->IntersectP(ray)) {
                        return true;
                    }
                } else {
                    for (int i = 0; i < nPrimitives; ++i) {
                        int primitiveIndex =
                                primitiveIndices[node->primitiveIndicesOffset + i];
                        const std::shared_ptr<Primitive> &prim =
                                primitives[primitiveIndex];
                        if (prim->IntersectP(ray)) {
                            return true;
                        }
                    }
                }

                // Grab next node to process from todo list
                if (todoPos > 0) {
                    --todoPos;
                    node = todo[todoPos].node;
                    tMin = todo[todoPos].tMin;
                    tMax = todo[todoPos].tMax;
                } else
                    break;
            } else {
                // Process kd-tree interior node

                // Compute parametric distance along ray to split plane
                int axis = node->SplitAxis();
                Float tPlane = (node->SplitPos() - ray.o[axis]) * invDir[axis];

                // Get node children pointers for ray
                const KdAccelNode *firstChild, *secondChild;
                int belowFirst =
                        (ray.o[axis] < node->SplitPos()) ||
                        (ray.o[axis] == node->SplitPos() && ray.d[axis] <= 0);
                if (belowFirst) {
                    firstChild = node + 1;
                    secondChild = &nodes[node->AboveChild()];
                } else {
                    firstChild = &nodes[node->AboveChild()];
                    secondChild = node + 1;
                }

                // Advance to next child node, possibly enqueue other child
                if (tPlane > tMax || tPlane <= 0)
                    node = firstChild;
                else if (tPlane < tMin)
                    node = secondChild;
                else {
                    // Enqueue _secondChild_ in todo list
                    todo[todoPos].node = secondChild;
                    todo[todoPos].tMin = tPlane;
                    todo[todoPos].tMax = tMax;
                    ++todoPos;
                    node = firstChild;
                    tMax = tPlane;
                }
            }
        }
        return false;
    }

    std::shared_ptr<KdTreeAccel> CreateKdTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        int isectCost = ps.FindOneInt("intersectcost", 80);
        int travCost = ps.FindOneInt("traversalcost", 1);
        Float emptyBonus = ps.FindOneFloat("emptybonus", 0.5f);
        int maxPrims = ps.FindOneInt("maxprims", 1);
        int maxDepth = ps.FindOneInt("maxdepth", -1);

        return std::make_shared<KdTreeAccel>(std::move(prims), isectCost, travCost, emptyBonus,
                                             maxPrims, maxDepth);
    }

}  // namespace pbrt
