#include <random>


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
#include "stats.h"
#include <fstream>
#include <core/progressreporter.h>

namespace pbrt {

    STAT_COUNTER("Accelerator/Kd-tree node traversals during intersect", nbNodeTraversals);
    STAT_COUNTER("Accelerator/Kd-tree node traversals during intersectP", nbNodeTraversalsP);
    STAT_COUNTER("Accelerator/Kd-tree nodes", nbNodes);
    STAT_COUNTER_DOUBLE("Accelerator/Kd-tree param:maxdepth", statParamMaxDepth);

    STAT_COUNTER("Accelerator/Kd-tree build: splitTests", statNbSplitTests);
    STAT_COUNTER("Accelerator/Kd-tree param:intersectioncost", statParamIntersectCost);
    STAT_COUNTER("Accelerator/Kd-tree param:axisSelectionType", statParamAxisSelectionType);
    STAT_COUNTER("Accelerator/Kd-tree param:axisSelectionAmount", statParamAxisSelectionAmount);
    STAT_COUNTER_DOUBLE("Accelerator/Kd-tree param:emptybonus", statParamEmptyBonus);

    STAT_COUNTER("Accelerator/Kd-tree param:traversalcost", statParamTraversalCost);
    STAT_COUNTER("Accelerator/Kd-tree param:maxprims", statParamMaxPrims);
    STAT_COUNTER_DOUBLE("Accelerator/Kd-tree param:splitalpha", statParamSplitAlpha);
    STAT_COUNTER_DOUBLE("Accelerator/Kd-tree param:alphatype", statParamAlphaType);
    STAT_COUNTER_DOUBLE("Accelerator/Kd-tree SA-cost", totalSACost);
    STAT_COUNTER_DOUBLE("Accelerator/Kd-tree Depth", statDepth);

    // KdTreeAccel Local Declarations
    struct KdAccelNode {
        // KdAccelNode Methods
        void InitLeaf(uint32_t *primNums, uint32_t np, std::vector<uint32_t> *primitiveIndices);

        void InitInterior(uint32_t axis, Float s) {
            split = s;
            flags = axis;
        }

        void setAboveChild(uint32_t ac) {
            aboveChild |= (ac << 2u);
        }

        Float SplitPos() const { return split; }

        uint32_t nPrimitives() const { return nPrims >> 2u; }

        uint32_t SplitAxis() const { return flags & 3u; }

        bool IsLeaf() const { return (flags & 3u) == 3u; }

        uint32_t AboveChild() const { return aboveChild >> 2u; }

        uint32_t depth(KdAccelNode *nodes, int id = 0) {
            if (IsLeaf())
                return 0;
            else
                return 1 + std::max(nodes[AboveChild()].depth(nodes, AboveChild()), nodes[id + 1].depth(nodes, id + 1));
        }

        std::string toString(const std::vector<uint32_t> &primitiveIndices) {
            std::stringstream ss;
            if (IsLeaf()) {
                uint32_t np = nPrimitives();
                ss << "L " << np;
                if (np == 1) {
                    ss << " " << onePrimitive;
                } else {
                    for (int i = 0; i < np; i++) {
                        uint32_t primitiveIndex =
                                primitiveIndices[primitiveIndicesOffset + i];
                        ss << " " << primitiveIndex;
                    }
                }
            } else {
                ss << "I " << SplitAxis() << " " << SplitPos() << " " << AboveChild();
            }


            return ss.str();
        }

        union {
            Float split;                 // Interior
            uint32_t onePrimitive;            // Leaf
            uint32_t primitiveIndicesOffset;  // Leaf
        };

    private:
        union {
            uint32_t flags;       // Both
            uint32_t nPrims;      // Leaf
            uint32_t aboveChild;  // Interior
        };
    };

    // KdTreeAccel Method Definitions
    KdTreeAccel::KdTreeAccel(std::vector<std::shared_ptr<Primitive>> p,
                             uint32_t isectCost, uint32_t traversalCost, Float emptyBonus,
                             uint32_t maxPrims, uint32_t maxDepth, Float splitAlpha, uint32_t alphaType,
                             uint32_t axisSelectionType, uint32_t axisSelectionAmount)
            : GenericBSP(std::move(p), isectCost, traversalCost, emptyBonus, maxPrims, maxDepth, 3u, splitAlpha, alphaType, axisSelectionType, axisSelectionAmount) {
        ProfilePhase _(Prof::AccelConstruction);

        statParamMaxDepth = maxDepth;
        statParamEmptyBonus = emptyBonus;
        statParamIntersectCost = isectCost;
        statParamTraversalCost = traversalCost;
        statParamMaxPrims = maxPrims;
        statNbSplitTests = 0;
        statParamSplitAlpha = splitAlpha;
        statParamAlphaType = alphaType;
        statParamAxisSelectionType = axisSelectionType;
        statParamAxisSelectionAmount = axisSelectionAmount;

        directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
        directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
        directions.emplace_back(Vector3f(0.0, 0.0, 1.0));

        // Start recursive construction of kd-tree
        buildTree();
    }

    void KdAccelNode::InitLeaf(uint32_t *primNums, uint32_t np,
                               std::vector<uint32_t> *primitiveIndices) {
        flags = 3u;
        nPrims |= (np << 2u);
        // Store primitive ids for leaf node
        if (np == 0)
            onePrimitive = 0;
        else if (np == 1)
            onePrimitive = primNums[0];
        else {
            primitiveIndicesOffset = primitiveIndices->size();
            for (uint32_t i = 0; i < np; ++i) primitiveIndices->push_back(primNums[i]);
        }
    }

    void KdTreeAccel::printNodes(std::ofstream &os) const {
        for (int i = 0; i < nextFreeNode; i++) {
            os << nodes[i].toString(primitiveIndices) << std::endl;
        }
    }

    void KdTreeAccel::buildTree() {
        // Compute bounds for kd-tree construction
        std::vector<Bounds3f> allPrimBounds;
        allPrimBounds.reserve(primitives.size());
        for (const std::shared_ptr<Primitive> &prim : primitives) {
            Bounds3f b = prim->WorldBound();
            bounds = Union(bounds, b);
            allPrimBounds.push_back(b);
        }

        ProgressReporter reporter(2 * primitives.size() * maxDepth - 1, "Building");

        uint32_t nodeNum = 0;
        // Allocate working memory for kd-tree construction
        std::unique_ptr<BoundEdge[]> edges[3];
        for (auto &edge : edges)
            edge.reset(new BoundEdge[2 * primitives.size()]);
        std::unique_ptr<uint32_t[]> prims(
                new uint32_t[(maxDepth + 1) * primitives.size()]); // TODO: vector ?
        // Initialize _primNums_ for kd-tree construction
        for (uint32_t i = 0; i < primitives.size(); ++i) {
            prims[i] = i;
        }
        // Precalculate normals
        std::vector<std::vector<Float>> directionDotNormals; // for each direction a vector with for each primitive the dot product between it's normal and the direction
        std::vector<std::vector<bool>> angleMatrix; // for each direction a vector with for each primitive whether it should be checked
        std::vector<uint32_t> closestDirection; // for each primitive, the index of the closest direction

        auto splitAlphaCos0 = (Float) std::abs(std::cos(splitAlpha * M_PI / 180));
        auto splitAlphaCos1 = (Float) std::abs(std::cos((90 - splitAlpha) * M_PI / 180));
        for (uint32_t axis = 0; axis < 3; ++axis) {
            std::vector<Float> dotNormals;
            directionDotNormals.emplace_back(dotNormals);
            std::vector<bool> angleVector;
            angleMatrix.emplace_back(angleVector);
        }
        for (uint32_t i = 0; i < primitives.size(); ++i) {
            Normal3f n = primitives[i]->Normal();
            uint32_t best = 0;
            directionDotNormals[0].emplace_back(std::abs(n.x));
            directionDotNormals[1].emplace_back(std::abs(n.y));
            directionDotNormals[2].emplace_back(std::abs(n.z));

            for (uint32_t axis = 1; axis < 3; ++axis)
                if (directionDotNormals[axis][i] > directionDotNormals[best][i])
                    best = axis;

            closestDirection.emplace_back(best);

            if (alphaType == 1)
                for (uint32_t axis = 0; axis < 3; ++axis)
                    angleMatrix[axis].emplace_back(directionDotNormals[axis][i] >= splitAlphaCos0);
            else if (alphaType == 2)
                for (uint32_t axis = 0; axis < 3; ++axis)
                    angleMatrix[axis].emplace_back(directionDotNormals[axis][i] <= splitAlphaCos1);
            else if (alphaType == 3)
                for (uint32_t axis = 0; axis < 3; ++axis)
                    angleMatrix[axis].emplace_back(directionDotNormals[axis][i] <= splitAlphaCos1 or
                                                   directionDotNormals[axis][i] >= splitAlphaCos0);
            else if (alphaType == 0)
                for (uint32_t axis = 0; axis < 3; ++axis)
                    angleMatrix[axis].emplace_back(true);
        }


        uint32_t maxPrimsOffset = 0;
        double currentSACost = 0;


        std::vector<KdBuildNode> stack;
        stack.emplace_back(KdBuildNode(maxDepth, (uint32_t) primitives.size(), 0, bounds, &prims[0]));

        while (!stack.empty()) {
            reporter.Update();
            KdBuildNode currentBuildNode = stack.back();
            stack.pop_back();
            CHECK_EQ(nodeNum, nextFreeNode);

            if (currentBuildNode.parentNum != -1)
                nodes[currentBuildNode.parentNum].setAboveChild(nodeNum);

            maxPrimsOffset = std::max(maxPrimsOffset, (uint32_t) (currentBuildNode.primNums - &prims[0]));

            // Get next free node from _nodes_ array
            if (nextFreeNode == nAllocedNodes) {
                uint32_t nNewAllocNodes = std::max(2u * nAllocedNodes, 512u);
                auto *n = AllocAligned<KdAccelNode>(nNewAllocNodes);
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
                currentSACost += currentBuildNode.nPrimitives * isectCost * currentBuildNode.nodeBounds.SurfaceArea();
                nodes[nodeNum++].InitLeaf(currentBuildNode.primNums, currentBuildNode.nPrimitives, &primitiveIndices);
                continue;
            }

            // Choose split axis position for interior node
            uint32_t bestAxis = -1, bestOffset = -1;
            Float bestCost = Infinity;
            Float oldCost = isectCost * Float(currentBuildNode.nPrimitives);
            Float totalSA = currentBuildNode.nodeBounds.SurfaceArea();
            const Float invTotalSA = 1 / totalSA;
            const Vector3f d = currentBuildNode.nodeBounds.pMax - currentBuildNode.nodeBounds.pMin;

            std::vector<uint32_t> directionsToUse;
            if (axisSelectionType == 0) {
                for (uint32_t axis = 0; axis < 3u; ++axis)
                    directionsToUse.emplace_back(axis);
                std::shuffle(directionsToUse.begin(), directionsToUse.end(), std::mt19937(std::random_device()()));
                directionsToUse.erase(directionsToUse.begin() + axisSelectionAmount, directionsToUse.end());
            } else if (axisSelectionType == 1) {
                std::vector<std::pair<Float, uint32_t>> means;
                for (uint32_t axis = 0; axis < 3u; ++axis) {
                    Float sum = 0;
                    for (uint32_t i = 0; i < currentBuildNode.nPrimitives; ++i) {
                        sum += directionDotNormals[axis][currentBuildNode.primNums[i]];
                    }
                    means.emplace_back(std::make_pair(sum / currentBuildNode.nPrimitives, axis));
                }
                std::sort(means.begin(), means.end());
                for (uint32_t axis =0; axis < axisSelectionAmount; ++axis) {
                    directionsToUse.emplace_back(means[means.size() - 1 - axis].second);
                }
            } else if (axisSelectionType == 2) {
                std::vector<std::pair<Float , uint32_t>> amounts;
                for (uint32_t axis = 0; axis < 3u; ++axis) {
                    amounts.emplace_back(std::make_pair(0, axis));
                }
                for (uint32_t i = 0; i < currentBuildNode.nPrimitives; ++i) {
                   amounts[closestDirection[i]].first += 1;
                }

                std::sort(amounts.begin(), amounts.end());
                for (uint32_t axis =0; axis < axisSelectionAmount; ++axis) {
                    directionsToUse.emplace_back(amounts[amounts.size() - 1 - axis].second);
                }
            } else if (axisSelectionType == 3) {
                Float f = currentBuildNode.depth * 1.0f / maxDepth;
                if(f < 0.2){
                    for (uint32_t axis = 0; axis < 3u; ++axis)
                        directionsToUse.emplace_back(axis);
                }
                else{
                    std::vector<std::pair<Float , uint32_t>> amounts;
                    for (uint32_t axis = 0; axis < 3u; ++axis) {
                        amounts.emplace_back(std::make_pair(0, axis));
                    }
                    for (uint32_t i = 0; i < currentBuildNode.nPrimitives; ++i) {
                        amounts[closestDirection[i]].first += 1;
                    }

                    std::sort(amounts.begin(), amounts.end());
                    uint32_t after = 0, before = 0;
                    if(f <= 0.6){
                        after = std::max((axisSelectionAmount > 1) ? 1u : 0u, (uint32_t) std::floor(f * axisSelectionAmount));
                        before = axisSelectionAmount - after;
                        CHECK(before >= after);
                    }
                    else {
                        before = std::max((axisSelectionAmount > 1) ? 1u : 0u, (uint32_t) std::floor((1-f) * axisSelectionAmount));
                        after = axisSelectionAmount - before;
                        CHECK(after >= before);
                    }

                    for (uint32_t axis =0; axis < before; ++axis) {
                        directionsToUse.emplace_back(amounts[amounts.size() - 1 - axis].second);
                    }
                    for (uint32_t axis =0; axis < after; ++axis) {
                        directionsToUse.emplace_back(amounts[axis].second);
                    }
                }
            }



            for (auto axis: directionsToUse) {
                // Initialize edges for _axis_
                for (uint32_t i = 0; i < currentBuildNode.nPrimitives; ++i) {
                    const uint32_t pn = currentBuildNode.primNums[i];
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
                uint32_t nBelow = 0, nAbove = currentBuildNode.nPrimitives;
                for (uint32_t i = 0; i < 2 * currentBuildNode.nPrimitives; ++i) {
                    if (edges[axis][i].type == EdgeType::End) --nAbove;
                    const Float edgeT = edges[axis][i].t;

                    if (edgeT > currentBuildNode.nodeBounds.pMin[axis] &&
                        edgeT < currentBuildNode.nodeBounds.pMax[axis] && angleMatrix[axis][edges[axis][i].primNum]) {
                        statNbSplitTests += 1;
                        // Compute cost for split at _i_th edge

                        // Compute child surface areas for split at _edgeT_
                        const uint32_t otherAxis0 = (axis + 1) % 3, otherAxis1 = (axis + 2) % 3;
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
                currentSACost += currentBuildNode.nPrimitives * isectCost * currentBuildNode.nodeBounds.SurfaceArea();
                nodes[nodeNum++].InitLeaf(currentBuildNode.primNums, currentBuildNode.nPrimitives, &primitiveIndices);
                continue;
            }

            // Classify primitives with respect to split
            uint32_t n0 = 0, n1 = 0;
            uint32_t *prims1 = currentBuildNode.primNums; // prims1 needs to be put in de array first, so it isn't overriden by child 0
            for (uint32_t i = bestOffset + 1; i < 2 * currentBuildNode.nPrimitives; ++i)
                if (edges[bestAxis][i].type == EdgeType::End)
                    prims1[n1++] = edges[bestAxis][i].primNum;

            uint32_t *prims0 = prims1 + n1;
            for (uint32_t i = 0; i < bestOffset; ++i)
                if (edges[bestAxis][i].type == EdgeType::Start)
                    prims0[n0++] = edges[bestAxis][i].primNum;

            // Add child nodes to stack
            const Float tSplit = edges[bestAxis][bestOffset].t;
            Bounds3f bounds0 = currentBuildNode.nodeBounds, bounds1 = currentBuildNode.nodeBounds;
            bounds0.pMax[bestAxis] = bounds1.pMin[bestAxis] = tSplit;

            currentSACost += traversalCost * currentBuildNode.nodeBounds.SurfaceArea();
            nodes[nodeNum].InitInterior(bestAxis, tSplit);

            stack.emplace_back(
                    KdBuildNode(currentBuildNode.depth - 1, n1, currentBuildNode.badRefines, bounds1, prims1, nodeNum));
            stack.emplace_back(
                    KdBuildNode(currentBuildNode.depth - 1, n0, currentBuildNode.badRefines, bounds0, prims0));
            ++nodeNum;
        }

        reporter.Done();
        statDepth = nodes[0].depth(nodes, 0);
        nbNodes = nodeNum;
        totalSACost = currentSACost / bounds.SurfaceArea();
        Warning("%f", totalSACost);
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
        PBRT_CONSTEXPR uint32_t maxTodo = 64u;
        BSPToDo<KdAccelNode> todo[maxTodo];
        uint32_t todoPos = 0;

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
                uint32_t axis = node->SplitAxis();
                Float tPlane = planeDistance(node->SplitPos(), ray, invDir, axis);

                // Get node children pointers for ray
                const KdAccelNode *firstChild, *secondChild;
                bool belowFirst =
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
                uint32_t nPrimitives = node->nPrimitives();
                if (nPrimitives == 1) {
                    const std::shared_ptr<Primitive> &p =
                            primitives[node->onePrimitive];
                    // Check one primitive inside leaf node
                    if (p->Intersect(ray, isect)) hit = true;
                } else {
                    for (uint32_t i = 0; i < nPrimitives; ++i) {
                        uint32_t index =
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
        PBRT_CONSTEXPR uint32_t maxTodo = 64;
        BSPToDo<KdAccelNode> todo[maxTodo];
        uint32_t todoPos = 0;
        const KdAccelNode *node = &nodes[0];
        while (node != nullptr) {
            nbNodeTraversalsP++;
            ray.stats.kdTreeNodeTraversalsP++;
            if (node->IsLeaf()) {
                // Check for shadow ray intersections inside leaf node
                uint32_t nPrimitives = node->nPrimitives();
                if (nPrimitives == 1) {
                    const std::shared_ptr<Primitive> &p =
                            primitives[node->onePrimitive];
                    if (p->IntersectP(ray)) {
                        return true;
                    }
                } else {
                    for (uint32_t i = 0; i < nPrimitives; ++i) {
                        uint32_t primitiveIndex =
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
                } else {
                    break;
                }
            } else {
                // Process kd-tree interior node

                // Compute parametric distance along ray to split plane
                uint32_t axis = node->SplitAxis();
                Float tPlane = planeDistance(node->SplitPos(), ray, invDir, axis);

                // Get node children pointers for ray
                const KdAccelNode *firstChild, *secondChild;
                bool belowFirst =
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
        uint32_t isectCost = (uint32_t) ps.FindOneInt("intersectcost", 80);
        uint32_t travCost = (uint32_t) ps.FindOneInt("traversalcost", 1);
        Float emptyBonus = ps.FindOneFloat("emptybonus", 0);
        uint32_t maxPrims = (uint32_t) ps.FindOneInt("maxprims", 1);
        uint32_t maxDepth = (uint32_t) ps.FindOneInt("maxdepth", -1);
        Float splitAlpha = ps.FindOneFloat("splitalpha", 0);
        uint32_t alphaType = (uint32_t) ps.FindOneInt("alphatype",
                                                      0); // 1 is between 0 and alpha, 2 between 90 and 90-alpha and 3 between 0 and alpha or 90 and 90 - alpha


        uint32_t axisSelectionType = (uint32_t) ps.FindOneInt("axisselectiontype",
                                                              0); // 0 is random, 1 is mean, 2 is simpleCluster
        uint32_t axisSelectionAmount = (uint32_t) ps.FindOneInt("axisselectionamount", 3);
        if (axisSelectionAmount > 3)
            axisSelectionAmount = 3;

        return std::make_shared<KdTreeAccel>(std::move(prims), isectCost, travCost, emptyBonus,
                                             maxPrims, maxDepth, splitAlpha, alphaType, axisSelectionType,
                                             axisSelectionAmount);
    }

}  // namespace pbrt