//
// Created by jesse on 11.11.18.
//

#include <core/stats.h>
#include "paramset.h"
#include "accelerators/rbsp.h"
#include "accelerators/kdtreeaccel.h"

namespace pbrt {

    STAT_COUNTER("Accelerator/RBSP-tree node traversals during intersect", nbNodeTraversals);
    STAT_COUNTER("Accelerator/RBSP-tree node traversals during intersectP", nbNodeTraversalsP);

    struct RBSPNode {
        // RBSPNode Methods
        void InitLeaf(uint32_t *primNums, uint32_t np, std::vector<uint32_t> *primitiveIndices);

        void InitInterior(uint32_t axis, Float s) {
            split = s;
            flags = axis;
        }

        void setAboveChild(uint32_t ac) {
            aboveChild |= (ac << 2u);
        }

        Float SplitPos() const { return split; }

        //TODO change amount of bit shifts etc
        uint32_t nPrimitives() const { return nPrims >> 2u; }

        uint32_t SplitAxis() const { return flags & 3u; }

        bool IsLeaf() const { return (flags & 3u) == 3u; }

        uint32_t AboveChild() const { return aboveChild >> 2u; }

        uint32_t depth(RBSPNode *nodes, int id = 0) {
            if (IsLeaf())
                return 0;
            else
                return 1 + std::max(nodes[AboveChild()].depth(nodes, AboveChild()), nodes[id + 1].depth(nodes, id + 1));
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

    // TODO uint32 -> u32 type
    void RBSPNode::InitLeaf(uint32_t *primNums, uint32_t np,
                            std::vector<uint32_t> *primitiveIndices) {
        flags = 3u; // TODO: change this as function of M
        nPrims |= (np << 2u);
        // Store primitive ids for leaf node
        if (np == 0)
            onePrimitive = 0;
        else if (np == 1u)
            onePrimitive = primNums[0];
        else {
            primitiveIndicesOffset = primitiveIndices->size();
            for (uint32_t i = 0; i < np; ++i) primitiveIndices->push_back(primNums[i]);
        }
    }

    RBSP::~RBSP() { FreeAligned(nodes); }

    RBSP::RBSP(std::vector<std::shared_ptr<pbrt::Primitive>> p, uint32_t isectCost, uint32_t traversalCost,
               pbrt::Float emptyBonus, uint32_t maxPrims, uint32_t maxDepth)
            : isectCost(isectCost),
              traversalCost(traversalCost),
              maxPrims(maxPrims),
              emptyBonus(emptyBonus),
              primitives(std::move(p)) {
        ProfilePhase _(Prof::AccelConstruction);
        if (maxDepth == -1)
            maxDepth = (uint32_t) std::round(8 + 1.3f * Log2Int(int64_t(primitives.size())));
        const uint32_t M = 3;
        std::vector<Vector3f> directions;
        directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
        directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
        directions.emplace_back(Vector3f(0.0, 0.0, 1.0));

        pbrt::BoundsMf rootNodeMBounds;
        for (auto &d: directions) {
            rootNodeMBounds.emplace_back(Boundsf());
        }

        // Compute bounds for kd-tree construction
        std::vector<BoundsMf> primBounds;
        primBounds.reserve(primitives.size());
        for (const std::shared_ptr<Primitive> &prim : primitives) {
            Bounds3f b = prim->WorldBound();
            bounds = Union(bounds, b);
            pbrt::BoundsMf mBounds;
            for (size_t i = 0; i < directions.size(); ++i) {
                auto &d= directions[i];
                Boundsf b = prim->getBounds(d);
                mBounds.emplace_back(b);
                Union(rootNodeMBounds[i], b);
            }
            primBounds.emplace_back(mBounds);
        }

        // Start recursive construction of RBSP-tree
        buildTree(rootNodeMBounds, primBounds, M, maxDepth);
    }

    void
    RBSP::buildTree(pbrt::BoundsMf &rootNodeMBounds, const std::vector<BoundsMf> &allPrimBounds, uint32_t M,
                    uint32_t maxDepth) {
        uint32_t nodeNum = 0;
        // Allocate working memory for kd-tree construction
        std::vector<std::unique_ptr<BoundEdge[]>> edges;
        for (uint32_t i = 0; i < M; ++i)
            edges[i].reset(new BoundEdge[2 * primitives.size()]);
        std::unique_ptr<uint32_t> prims_p(
                new uint32_t[(maxDepth + 1) * primitives.size()]);
        uint32_t *prims = prims_p.get();
        // Initialize _primNums_ for kd-tree construction
        for (uint32_t i = 0; i < primitives.size(); ++i) {
            prims[i] = i;
        }
        uint32_t maxPrimsOffset = 0;

        std::vector<RBSPBuildNode> stack;
        stack.emplace_back(RBSPBuildNode(maxDepth, (uint32_t) primitives.size(), 0u, rootNodeMBounds, prims));

        while (!stack.empty()) {

            RBSPBuildNode currentBuildNode = stack.back();
            stack.pop_back();
            CHECK_EQ(nodeNum, nextFreeNode);

            if (currentBuildNode.parentNum != -1)
                nodes[currentBuildNode.parentNum].setAboveChild(nodeNum);

            maxPrimsOffset = std::max(maxPrimsOffset, (uint32_t) (currentBuildNode.primNums - prims));

            // Get next free node from _nodes_ array
            if (nextFreeNode == nAllocedNodes) {
                uint32_t nNewAllocNodes = std::max(2u * nAllocedNodes, 512u);
                auto *n = AllocAligned<RBSPNode>(nNewAllocNodes);
                if (nAllocedNodes > 0) {
                    memcpy(n, nodes, nAllocedNodes * sizeof(RBSPNode));
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

            // Choose split axis position for interior node
            uint32_t bestD = -1, bestOffset = -1;
            Float bestCost = Infinity;
            Float oldCost = isectCost * Float(currentBuildNode.nPrimitives);
            Float totalSA = currentBuildNode.nodeBounds.SurfaceArea();
            const Float invTotalSA = 1 / totalSA;

            for (uint32_t i = 0; i < currentBuildNode.nPrimitives; ++i) {
                const uint32_t pn = currentBuildNode.primNums[i];
                const std::vector<Boundsf> &bounds = allPrimBounds[pn];

                for (uint32_t d = 0; d < M; ++d) {
                    edges[d][2 * i] = BoundEdge(bounds[d].min, pn, true);
                    edges[d][2 * i + 1] = BoundEdge(bounds[d].max, pn, false);
                }
            }

            for (uint32_t d = 0; d < M; ++d) {
                // Sort _edges_ for _axis_
                std::sort(&edges[d][0], &edges[d][2 * currentBuildNode.nPrimitives],
                          [](const BoundEdge &e0, const BoundEdge &e1) -> bool {
                              if (e0.t == e1.t)
                                  return (int) e0.type < (int) e1.type;
                              else
                                  return e0.t < e1.t;
                          });

                // Compute cost of all splits for _axis_ to find best
                uint32_t nBelow = 0, nAbove = currentBuildNode.nPrimitives;
                for (uint32_t i = 0; i < 2 * currentBuildNode.nPrimitives; ++i) {
                    if (edges[d][i].type == EdgeType::End) --nAbove;
                    const Float edgeT = edges[d][i].t;

                    if (edgeT > currentBuildNode.nodeBounds[d].min &&
                        edgeT < currentBuildNode.nodeBounds[d].max) {
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
                            bestD = d;
                            bestOffset = i;
                        }
                    }
                    if (edges[d][i].type == EdgeType::Start) ++nBelow;
                }
                CHECK(nBelow == currentBuildNode.nPrimitives && nAbove == 0);
            }

            // Create leaf if no good splits were found
            if (bestCost > oldCost) ++currentBuildNode.badRefines;
            if ((bestCost > 4 * oldCost && currentBuildNode.nPrimitives < 16) || bestD == -1 ||
                currentBuildNode.badRefines == 3) {
                nodes[nodeNum++].InitLeaf(currentBuildNode.primNums, currentBuildNode.nPrimitives, &primitiveIndices);
                continue;
            }

            // Classify primitives with respect to split
            uint32_t n0 = 0, n1 = 0;
            uint32_t *prims1 = currentBuildNode.primNums; // prims1 needs to be put in de array first, so it isn't overriden by child 0
            for (uint32_t i = bestOffset + 1; i < 2 * currentBuildNode.nPrimitives; ++i)
                if (edges[bestD][i].type == EdgeType::End)
                    prims1[n1++] = edges[bestD][i].primNum;

            uint32_t *prims0 = prims1 + n1;
            for (uint32_t i = 0; i < bestOffset; ++i)
                if (edges[bestD][i].type == EdgeType::Start)
                    prims0[n0++] = edges[bestD][i].primNum;

            // Add child nodes to stack
            const Float tSplit = edges[bestD][bestOffset].t;
            BoundsMf bounds0 = currentBuildNode.nodeBounds, bounds1 = currentBuildNode.nodeBounds;
            bounds0[bestD].max = bounds1[bestD].min = tSplit;

            nodes[nodeNum].InitInterior(bestD, tSplit);

            stack.emplace_back(
                    RBSPBuildNode(currentBuildNode.depth - 1, n1, currentBuildNode.badRefines, bounds1, prims1, nodeNum));
            stack.emplace_back(
                    RBSPBuildNode(currentBuildNode.depth - 1, n0, currentBuildNode.badRefines, bounds0, prims0));
            ++nodeNum;
        }

        Warning("Depth %d", nodes[0].depth(nodes, 0));
    }

    bool RBSP::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
        ProfilePhase p(Prof::AccelIntersect);
        return true
    }

    bool RBSP::IntersectP(const Ray &ray) const {
        ProfilePhase p(Prof::AccelIntersectP);

        return false;
    }

    std::shared_ptr<RBSP> CreateRBSPTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        uint32_t isectCost = (uint32_t) ps.FindOneInt("intersectcost", 80);
        uint32_t travCost = (uint32_t) ps.FindOneInt("traversalcost", 1);
        Float emptyBonus = ps.FindOneFloat("emptybonus", 0.5f);
        uint32_t maxPrims = (uint32_t) ps.FindOneInt("maxprims", 1);
        uint32_t maxDepth = (uint32_t) ps.FindOneInt("maxdepth", -1);

        return std::make_shared<RBSP>(std::move(prims), isectCost, travCost, emptyBonus,
                                      maxPrims, maxDepth);
    }

}  // namespace pbrt