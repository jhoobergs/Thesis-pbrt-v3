//
// Created by jesse on 13.02.19.
//

#include <core/stats.h>
#include <core/progressreporter.h>
#include <bits/random.h>
#include "paramset.h"
#include "bspPaper.h"
#include <set>
#include "bvh.h"

namespace pbrt {

    STAT_COUNTER("Accelerator/RBSP-tree nodes", nbNodes);
    STAT_COUNTER("Accelerator/RBSP-tree Kd-nodes", nbKdNodes);
    STAT_COUNTER("Accelerator/RBSP-tree BSP-nodes", nbBSPNodes);
    STAT_COUNTER("Accelerator/RBSP-tree build: splitTests", statNbSplitTests);
    STAT_COUNTER("Accelerator/RBSP-tree param:directions", statParamnbDirections);
    STAT_COUNTER("Accelerator/RBSP-tree param:intersectioncost", statParamIntersectCost);

    STAT_COUNTER("Accelerator/RBSP-tree param:traversalcost", statParamTraversalCost);
    STAT_COUNTER("Accelerator/RBSP-tree param:maxprims", statParamMaxPrims);
    STAT_COUNTER_DOUBLE("Accelerator/RBSP-tree param:emptybonus", statParamEmptyBonus);
    STAT_COUNTER_DOUBLE("Accelerator/RBSP-tree param:maxdepth", statParamMaxDepth);
    STAT_COUNTER_DOUBLE("Accelerator/RBSP-tree SA-cost", totalSACost);
    STAT_COUNTER_DOUBLE("Accelerator/RBSP-tree Depth", statDepth);

    BSPPaper::BSPPaper(std::vector<std::shared_ptr<pbrt::Primitive>> p, uint32_t isectCost,
                       uint32_t traversalCost,
                       Float emptyBonus, uint32_t maxPrims, uint32_t maxDepth, uint32_t nbDirections)
            : BSP(std::move(p), isectCost, traversalCost, emptyBonus, maxPrims, maxDepth, nbDirections,
                         0, 0, 0, 0) {
        ProfilePhase _(Prof::AccelConstruction);

        statParamnbDirections = nbDirections;
        statParamMaxDepth = maxDepth;
        statParamEmptyBonus = emptyBonus;
        statParamIntersectCost = isectCost;
        statParamTraversalCost = traversalCost;
        statParamMaxPrims = maxPrims;
        statNbSplitTests = 0;

        // Start recursive construction of RBSP-tree
        buildTree();
    }

    void BSPPaper::buildTree() {
        //Initialize
        // Compute bounds for rbsp-tree construction: CANNOT PRECOMPUTE ALLPRIMBOUNDS
        for (const std::shared_ptr<Primitive> &prim : primitives) {
            bounds = Union(bounds, prim->WorldBound());
        }

        KDOPMeshWithDirections kDOPMesh;
        bounds.toKDOPMesh(kDOPMesh, kDOPMesh.directions);

        std::vector<Vector3f> kdDirections;
        kdDirections.emplace_back(1, 0, 0);
        kdDirections.emplace_back(0, 1, 0);
        kdDirections.emplace_back(0, 0, 1);

        // Building
        ProgressReporter reporter(2 * primitives.size() * maxDepth - 1, "Building");

        uint32_t nodeNum = 0;
        // Allocate working memory for rbsp-tree construction
        std::vector<std::unique_ptr<BoundEdge[]>> edges;
        for (uint32_t i = 0; i < 3; ++i)
            edges.emplace_back(std::unique_ptr<BoundEdge[]>(new BoundEdge[2 * primitives.size()]));
        std::unique_ptr<uint32_t[]> prims(
                new uint32_t[(maxDepth + 1) * primitives.size()]); // TODO: use vector ?
        // Initialize _primNums_ for rbsp-tree construction
        for (uint32_t i = 0; i < primitives.size(); ++i) {
            prims[i] = i;
        }

        uint32_t maxPrimsOffset = 0;
        double currentSACost = 0;

        std::vector<BSPBuildNode> stack;
        stack.emplace_back(maxDepth, (uint32_t) primitives.size(), 0u, kDOPMesh,
                           kDOPMesh.SurfaceArea(), &prims[0]);
        // Warning("Building RBSP: Lets loop");
        while (!stack.empty()) { // || nodeNum > 235000 || nodeNum > 1400000
            if (nodeNum % 10000 == 0) // 100000
                Warning("Nodenum %d", nodeNum);
            reporter.Update();
            BSPBuildNode currentBuildNode = stack.back();
            stack.pop_back();
            CHECK_EQ(nodeNum, nextFreeNode);

            if (currentBuildNode.parentNum != -1)
                nodes[currentBuildNode.parentNum].setAboveChild(nodeNum);

            maxPrimsOffset = std::max(maxPrimsOffset, (uint32_t) (currentBuildNode.primNums - &prims[0]));

            // Get next free node from _nodes_ array
            if (nextFreeNode == nAllocedNodes) {
                uint32_t nNewAllocNodes = std::max(2u * nAllocedNodes, 512u);
                auto *n = AllocAligned<BSPNode>(nNewAllocNodes);
                if (nAllocedNodes > 0) {
                    memcpy(n, nodes, nAllocedNodes * sizeof(BSPNode));
                    FreeAligned(nodes);
                }
                nodes = n;
                nAllocedNodes = nNewAllocNodes;
            }
            ++nextFreeNode;

            // Initialize leaf node if termination criteria met
            if (currentBuildNode.nPrimitives <= maxPrims || currentBuildNode.depth == 0) {
                currentSACost += currentBuildNode.nPrimitives * isectCost * currentBuildNode.kdopMeshArea;
                nodes[nodeNum++].InitLeaf(currentBuildNode.primNums, currentBuildNode.nPrimitives,
                                          &primitiveIndices);
                continue;
            }

            // Choose split axis position for interior node
            uint32_t bestK = -1, bestOffset = -1;
            Float bestSplitT = 0;
            Vector3f bestSplitAxis = Vector3f();
            std::pair<KDOPMeshWithDirections, KDOPMeshWithDirections> bestSplittedKDOPs;
            std::pair<Float, Float> bestSplittedKDOPAreas = std::make_pair(0, 0);
            Float bestCost = Infinity;
            Float oldCost = isectCost * Float(currentBuildNode.nPrimitives);
            const Float invTotalSA = 1 / currentBuildNode.kdopMeshArea;
            std::pair<KDOPMeshWithDirections, KDOPMeshWithDirections> splittedKDOPs;

            // Sweep for kd directions
            for (uint32_t k = 0; k < 3; ++k) {
                auto d = kdDirections[k];

                Boundsf directionBounds = Boundsf();
                for (auto &edge: currentBuildNode.kDOPMesh.edges) {
                    Boundsf b = edge.getBounds(d);
                    directionBounds = Union(directionBounds, b);
                }

                // Sort _edges_ for _axis_
                for (uint32_t i = 0; i < currentBuildNode.nPrimitives; ++i) {
                    const uint32_t pn = currentBuildNode.primNums[i];
                    const Boundsf &bounds = primitives[pn]->getBounds(d);
                    edges[k][2 * i] = BoundEdge(bounds.min, pn, true);
                    edges[k][2 * i + 1] = BoundEdge(bounds.max, pn, false);

                }
                std::sort(&edges[k][0], &edges[k][2 * currentBuildNode.nPrimitives],
                          [](const BoundEdge &e0, const BoundEdge &e1) -> bool {
                              if (e0.t == e1.t)
                                  return (int) e0.type < (int) e1.type;
                              else
                                  return e0.t < e1.t;
                          });
                // Compute cost of all splits for _axis_ to find best
                uint32_t nBelow = 0, nAbove = currentBuildNode.nPrimitives;
                for (uint32_t i = 0; i < 2 * currentBuildNode.nPrimitives; ++i) {
                    if (edges[k][i].type == EdgeType::End) --nAbove;
                    const Float edgeT = edges[k][i].t;

                    if (edgeT > directionBounds.min &&
                        edgeT < directionBounds.max) {
                        statNbSplitTests += 1;
                        // Compute cost for split at _i_th edge
                        // Compute child surface areas for split at _edgeT_
                        splittedKDOPs = currentBuildNode.kDOPMesh.cut(
                                edgeT, d);
                        const Float areaBelow = splittedKDOPs.first.SurfaceArea();
                        const Float areaAbove = splittedKDOPs.second.SurfaceArea();
                        const Float pBelow = areaBelow * invTotalSA;
                        const Float pAbove = areaAbove * invTotalSA;

                        const Float eb = (nAbove == 0 || nBelow == 0) ? emptyBonus : 0;
                        const Float cost =
                                traversalCost +
                                isectCost * (1 - eb) * (pBelow * nBelow + pAbove * nAbove);

                        // Update best split if this is lowest cost so far
                        if (cost < bestCost) {
                            bestCost = cost;
                            bestSplitT = edgeT;
                            bestSplitAxis = d;
                            bestK = k;
                            bestOffset = i;
                            bestSplittedKDOPs = splittedKDOPs;
                            bestSplittedKDOPAreas.first = areaBelow;
                            bestSplittedKDOPAreas.second = areaAbove;
                        }
                    }
                    if (edges[k][i].type == EdgeType::Start) ++nBelow;
                }
                CHECK(nBelow == currentBuildNode.nPrimitives && nAbove == 0);
            }
            // Other directions for each triangle
            std::vector<std::shared_ptr<Primitive>> currentPrimitives;
            for (uint32_t i = 0; i < currentBuildNode.nPrimitives; ++i) {
                const uint32_t pn = currentBuildNode.primNums[i];
                currentPrimitives.emplace_back(primitives[pn]);
            }
            BVHAccel bvh = BVHAccel(currentPrimitives, 4, 8, 1);
            for (uint32_t i = 0; i < currentBuildNode.nPrimitives; ++i) {
                const uint32_t pn = currentBuildNode.primNums[i];
                std::vector<Plane> planes = primitives[pn]->getBSPPaperPlanes();
                for (const auto &plane: planes) {
                    Boundsf directionBounds = Boundsf();
                    for (auto &edge: currentBuildNode.kDOPMesh.edges) {
                        Vector3f p = plane.axis;
                        Boundsf b = edge.getBounds(p);
                        directionBounds = Union(directionBounds, b);
                    }

                    if (plane.t > directionBounds.min &&
                        plane.t < directionBounds.max) {
                        splittedKDOPs = currentBuildNode.kDOPMesh.cut(
                                plane.t, plane.axis);
                        const Float areaBelow = splittedKDOPs.first.SurfaceArea();
                        const Float areaAbove = splittedKDOPs.second.SurfaceArea();
                        const Float pBelow = areaBelow * invTotalSA;
                        const Float pAbove = areaAbove * invTotalSA;
                        auto lr = bvh.getAmountToLeftAndRight(plane);

                        const Float eb = (lr.second == 0 || lr.first == 0) ? emptyBonus : 0;
                        const Float cost =
                                traversalCost +
                                isectCost * (1 - eb) * (pBelow * lr.first + pAbove * lr.second);
                        // Update best split if this is lowest cost so far
                        if (cost < bestCost) {
                            bestCost = cost;
                            bestK = 33; // random number
                            bestSplitT = plane.t;
                            bestSplitAxis = plane.axis;
                            bestSplittedKDOPs = splittedKDOPs;
                            bestSplittedKDOPAreas.first = areaBelow;
                            bestSplittedKDOPAreas.second = areaAbove;
                        }
                    }
                }
            }


            // Create leaf if no good splits were found
            if (bestCost > oldCost) ++currentBuildNode.badRefines;
            if ((bestCost > 4 * oldCost && currentBuildNode.nPrimitives < 16) || bestK == -1 ||
                currentBuildNode.badRefines == 3) {
                currentSACost += currentBuildNode.nPrimitives * isectCost * currentBuildNode.kdopMeshArea;
                nodes[nodeNum++].InitLeaf(currentBuildNode.primNums, currentBuildNode.nPrimitives,
                                          &primitiveIndices);
                continue;
            }

            // Classify primitives with respect to split
            uint32_t n0 = 0, n1 = 0;
            uint32_t *prims1, *prims0;
            prims1 = currentBuildNode.primNums; // prims1 needs to be put in de array first, so it isn't overriden by child 0
            if (bestK != 33) {
                nbKdNodes++;
                if (nbKdNodes % 10000 == 0)
                    Warning("KD nodes %d", nbKdNodes);
                for (uint32_t i = bestOffset + 1; i < 2 * currentBuildNode.nPrimitives; ++i)
                    if (edges[bestK][i].type == EdgeType::End)
                        prims1[n1++] = edges[bestK][i].primNum;

                prims0 = prims1 + n1;
                for (uint32_t i = 0; i < bestOffset; ++i)
                    if (edges[bestK][i].type == EdgeType::Start)
                        prims0[n0++] = edges[bestK][i].primNum;
            } else {
                nbBSPNodes++;
                if (nbBSPNodes % 10000 == 0)
                    Warning("BSP nodes %d", nbBSPNodes);
                std::vector<uint32_t> left, right;
                bvh.getPrimnumsToLeftAndRight(Plane(bestSplitT, bestSplitAxis), left, right);
                // Map zero based primNums in BVH to currentBuildNode based primNums
                std::transform(left.begin(), left.end(), left.begin(),
                               [&currentBuildNode](
                                       uint32_t index) -> uint32_t { return currentBuildNode.primNums[index]; });
                std::transform(right.begin(), right.end(), right.begin(),
                               [&currentBuildNode](
                                       uint32_t index) -> uint32_t { return currentBuildNode.primNums[index]; });


                for (uint32_t primNum: right) {
                    prims1[n1++] = primNum;
                }
                prims0 = prims1 + n1;
                for (uint32_t primNum: left) {
                    prims0[n0++] = primNum;
                }
            }
            // Add child nodes to stack
            currentSACost += traversalCost * currentBuildNode.kdopMeshArea;
            nodes[nodeNum].InitInterior(bestSplitAxis, bestSplitT);
            stack.emplace_back(
                    currentBuildNode.depth - 1, n1, currentBuildNode.badRefines,
                    bestSplittedKDOPs.second, bestSplittedKDOPAreas.second, prims1, nodeNum);
            stack.emplace_back(
                    currentBuildNode.depth - 1, n0, currentBuildNode.badRefines,
                    bestSplittedKDOPs.first, bestSplittedKDOPAreas.first, prims0);


            ++nodeNum;
        }
        reporter.Done();
        statDepth = nodes[0].depth(nodes, 0);
        nbNodes = nodeNum;
        totalSACost = currentSACost / bounds.SurfaceArea();
    }

    std::shared_ptr<BSPPaper> CreateBSPPaperTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        uint32_t isectCost = (uint32_t) ps.FindOneInt("intersectcost", 80);
        uint32_t travCost = (uint32_t) ps.FindOneInt("traversalcost", 5);
        Float emptyBonus = ps.FindOneFloat("emptybonus", 0);
        uint32_t maxPrims = (uint32_t) ps.FindOneInt("maxprims", 1);
        uint32_t maxDepth = (uint32_t) ps.FindOneInt("maxdepth", -1);
        uint32_t nbDirections = (uint32_t) ps.FindOneInt("nbDirections", 3);

        return std::make_shared<BSPPaper>(std::move(prims), isectCost, travCost, emptyBonus,
                                          maxPrims, maxDepth, nbDirections);
    }
} // namespace pbrt