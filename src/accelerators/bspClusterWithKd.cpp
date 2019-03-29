//
// Created by jesse on 18.03.19.
//

#include "bspClusterWithKd.h"
#include <core/stats.h>
#include <core/progressreporter.h>
#include <bits/random.h>
#include "paramset.h"
#include <set>
#include <random>
#include "clustering.h"

namespace pbrt {

     BSPClusterWithKd::BSPClusterWithKd(std::vector<std::shared_ptr<pbrt::Primitive>> p, uint32_t isectCost,
                           uint32_t traversalCost,
                           Float emptyBonus, uint32_t maxPrims, uint32_t maxDepth, uint32_t nbDirections)
            : BSP(std::move(p), isectCost, traversalCost, emptyBonus, maxPrims, maxDepth, nbDirections,
                  0, 0, 0, 0) {
        ProfilePhase _(Prof::AccelConstruction);

        statParamnbDirections = nbDirections;

        K = nbDirections;

        // Start recursive construction of RBSP-tree
        buildTree();
    }

    void BSPClusterWithKd::buildTree() {
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

        //Initialize
        // Compute bounds for rbsp-tree construction: CANNOT PRECOMPUTE ALLPRIMBOUNDS
        for (const std::shared_ptr<Primitive> &prim : primitives) {
            bounds = Union(bounds, prim->WorldBound());
        }

        KDOPMeshWithDirections kDOPMesh;
        bounds.toKDOPMesh(kDOPMesh, kDOPMesh.directions);

        // Building
        ProgressReporter reporter(2 * primitives.size() * maxDepth - 1, "Building");

        uint32_t nodeNum = 0;
        // Allocate working memory for rbsp-tree construction
        std::vector<std::unique_ptr<BoundEdge[]>> edges;
        for (uint32_t i = 0; i < K; ++i)
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
                treeSetAboveChild(&nodes[currentBuildNode.parentNum], (nodeNum));

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
                treeInitLeaf(&nodes[nodeNum++], currentBuildNode.primNums, currentBuildNode.nPrimitives,
                             &primitiveIndices);
                /*nodes[nodeNum++].InitLeaf(currentBuildNode.primNums, currentBuildNode.nPrimitives,
                                          &primitiveIndices);*/
                continue;
            }

            // Choose split axis position for interior node
            uint32_t bestK = -1, bestOffset = -1;
            std::pair<KDOPMeshWithDirections, KDOPMeshWithDirections> bestSplittedKDOPs;
            std::pair<Float, Float> bestSplittedKDOPAreas = std::make_pair(0, 0);
            Float bestCost = Infinity;
            Float oldCost = isectCost * Float(currentBuildNode.nPrimitives);
            // Float totalSA = currentBuildNode.kDOPMesh.SurfaceArea(directions);
            const Float invTotalSA = 1 / currentBuildNode.kdopMeshArea;
            std::pair<KDOPMeshWithDirections, KDOPMeshWithDirections> splittedKDOPs;

            const uint32_t Kmeans = K - 3;
            std::vector<Vector3f> clusterMeans;
            clusterMeans.emplace_back(1,0,0);
            clusterMeans.emplace_back(0,1,0);
            clusterMeans.emplace_back(0,0,1);
            if(Kmeans > 0) {
                auto generatatedClusterMeans = calculateClusterMeans(gen, Kmeans, primitives, currentBuildNode.primNums,
                                                          currentBuildNode.nPrimitives);
                clusterMeans.insert(clusterMeans.end(), generatatedClusterMeans.begin(), generatatedClusterMeans.end());
            }

            for (uint32_t k = 0; k < clusterMeans.size(); ++k) {
                auto d = clusterMeans[k];

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
                        ++statNbSplitTests;
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

            // Create leaf if no good splits were found
            if (bestCost > oldCost) ++currentBuildNode.badRefines;
            if ((bestCost > 4 * oldCost && currentBuildNode.nPrimitives < 16) || bestK == -1 ||
                currentBuildNode.badRefines == 3) {
                currentSACost += currentBuildNode.nPrimitives * isectCost * currentBuildNode.kdopMeshArea;
                treeInitLeaf(&nodes[nodeNum++], currentBuildNode.primNums, currentBuildNode.nPrimitives,
                             &primitiveIndices);
                /*nodes[nodeNum++].InitLeaf(currentBuildNode.primNums, currentBuildNode.nPrimitives,
                                          &primitiveIndices);*/
                continue;
            }

            // Classify primitives with respect to split
            uint32_t n0 = 0, n1 = 0;
            uint32_t *prims1 = currentBuildNode.primNums; // prims1 needs to be put in de array first, so it isn't overriden by child 0
            for (uint32_t i = bestOffset + 1; i < 2 * currentBuildNode.nPrimitives; ++i)
                if (edges[bestK][i].type == EdgeType::End)
                    prims1[n1++] = edges[bestK][i].primNum;

            uint32_t *prims0 = prims1 + n1;
            for (uint32_t i = 0; i < bestOffset; ++i)
                if (edges[bestK][i].type == EdgeType::Start)
                    prims0[n0++] = edges[bestK][i].primNum;

            // Add child nodes to stack
            const Float tSplit = edges[bestK][bestOffset].t;

            if(bestK < 3)
                ++nbKdNodes;
            else
                ++nbBSPNodes;

            currentSACost += traversalCost * currentBuildNode.kdopMeshArea;
            treeInitInterior(&nodes[nodeNum], clusterMeans[bestK], tSplit);

            stack.emplace_back(
                    currentBuildNode.depth - 1, n1, currentBuildNode.badRefines,
                    bestSplittedKDOPs.second, bestSplittedKDOPAreas.second, prims1, nodeNum);
            stack.emplace_back(
                    currentBuildNode.depth - 1, n0, currentBuildNode.badRefines,
                    bestSplittedKDOPs.first, bestSplittedKDOPAreas.first, prims0);

            ++nodeNum;
        }
        reporter.Done();
        Warning("Done building");
        statDepth = treeDepth(&nodes[0], nodes, 0);
        Warning("END Done building");

        nbNodes = nodeNum;
        totalSACost = currentSACost / bounds.SurfaceArea();
    }

    std::shared_ptr<BSPClusterWithKd> CreateBSPClusterWithKdTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        uint32_t isectCost = (uint32_t) ps.FindOneInt("intersectcost", 80);
        uint32_t travCost = (uint32_t) ps.FindOneInt("traversalcost", 5);
        Float emptyBonus = ps.FindOneFloat("emptybonus", 0);
        uint32_t maxPrims = (uint32_t) ps.FindOneInt("maxprims", 1);
        uint32_t maxDepth = (uint32_t) ps.FindOneInt("maxdepth", -1);
        uint32_t nbDirections = (uint32_t) ps.FindOneInt("nbDirections", 3);

        return std::make_shared<BSPClusterWithKd>(std::move(prims), isectCost, travCost, emptyBonus,
                                            maxPrims, maxDepth, nbDirections);
    }
} // namespace pbrt