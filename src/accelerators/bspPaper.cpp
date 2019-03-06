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

    STAT_COUNTER("Accelerator/RBSP-tree node traversals during intersect", nbNodeTraversals);
    STAT_COUNTER("Accelerator/RBSP-tree node traversals during intersectP", nbNodeTraversalsP);
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

    struct BSPPaperNode {
        // BSPNode Methods
        void InitLeaf(uint32_t *primNums, uint32_t np, std::vector<uint32_t> *primitiveIndices);

        void InitInterior(const Vector3f &axis, Float s) {
            split = s;
            splitAxis = axis;
            flags = 0;
        }

        void setAboveChild(uint32_t ac) {
            aboveChild |= (ac << 1);
        }

        Float SplitPos() const { return split; }

        uint32_t nPrimitives() const { return nPrims >> 1; }

        Vector3f SplitAxis() const { return splitAxis; }

        bool IsLeaf() const { return (flags & 1) == 1; }

        uint32_t AboveChild() const { return aboveChild >> 1; }

        uint32_t depth(BSPPaperNode *nodes, int id = 0) {
            if (IsLeaf())
                return 0;
            else
                return 1 + std::max(nodes[AboveChild()].depth(nodes, AboveChild()),
                                    nodes[id + 1].depth(nodes, id + 1));
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
                ss << "I " << SplitAxis().x << " " << SplitAxis().y << " " << SplitAxis().z << " " << SplitPos() << " "
                   << AboveChild();
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
        Vector3f splitAxis;
    };

    void BSPPaperNode::InitLeaf(uint32_t *primNums, uint32_t np,
                                std::vector<uint32_t> *primitiveIndices) {
        flags = 1;
        nPrims |= (np << 1);
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

    BSPPaper::BSPPaper(std::vector<std::shared_ptr<pbrt::Primitive>> p, uint32_t isectCost,
                       uint32_t traversalCost,
                       Float emptyBonus, uint32_t maxPrims, uint32_t maxDepth, uint32_t nbDirections)
            : GenericBSP(std::move(p), isectCost, traversalCost, emptyBonus, maxPrims, maxDepth, nbDirections,
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

    void BSPPaper::printNodes(std::ofstream &os) const {
        for (int i = 0; i < nextFreeNode; i++) {
            os << nodes[i].toString(primitiveIndices) << std::endl;
        }
    }

    void BSPPaper::buildTree() {
        //Initialize
        // Compute bounds for rbsp-tree construction: CANNOT PRECOMPUTE ALLPRIMBOUNDS
        for (const std::shared_ptr<Primitive> &prim : primitives) {
            bounds = Union(bounds, prim->WorldBound());
        }

        KDOPMeshCluster kDOPMesh;
        Point3f v1 = Point3f(bounds.pMin);
        Point3f v2 = Point3f(bounds.pMin.x, bounds.pMin.y, bounds.pMax.z);
        Point3f v3 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMin.z);
        Point3f v4 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMin.z);
        Point3f v5 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMax.z);
        Point3f v6 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMax.z);
        Point3f v7 = Point3f(bounds.pMax.x, bounds.pMax.y, bounds.pMin.z);
        Point3f v8 = Point3f(bounds.pMax);

        kDOPMesh.addEdge(KDOPEdge(v1, v2, 1, 3));
        kDOPMesh.addEdge(KDOPEdge(v1, v3, 1, 5));
        kDOPMesh.addEdge(KDOPEdge(v1, v4, 3, 5));
        kDOPMesh.addEdge(KDOPEdge(v2, v5, 1, 4));
        kDOPMesh.addEdge(KDOPEdge(v2, v6, 3, 4));
        kDOPMesh.addEdge(KDOPEdge(v3, v5, 1, 2));
        kDOPMesh.addEdge(KDOPEdge(v3, v7, 2, 5));
        kDOPMesh.addEdge(KDOPEdge(v4, v6, 0, 3));
        kDOPMesh.addEdge(KDOPEdge(v4, v7, 0, 5));
        kDOPMesh.addEdge(KDOPEdge(v5, v8, 2, 4));
        kDOPMesh.addEdge(KDOPEdge(v6, v8, 0, 4));
        kDOPMesh.addEdge(KDOPEdge(v7, v8, 0, 2));
        kDOPMesh.directions.emplace_back(1, 0, 0);
        kDOPMesh.directions.emplace_back(0, 1, 0);
        kDOPMesh.directions.emplace_back(0, 0, 1);

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

        std::vector<BSPPaperBuildNode> stack;
        stack.emplace_back(maxDepth, (uint32_t) primitives.size(), 0u, kDOPMesh,
                           kDOPMesh.SurfaceArea(), &prims[0]);
        // Warning("Building RBSP: Lets loop");
        while (!stack.empty()) { // || nodeNum > 235000 || nodeNum > 1400000
            if (nodeNum % 10000 == 0) // 100000
                Warning("Nodenum %d", nodeNum);
            reporter.Update();
            BSPPaperBuildNode currentBuildNode = stack.back();
            stack.pop_back();
            CHECK_EQ(nodeNum, nextFreeNode);

            if (currentBuildNode.parentNum != -1)
                nodes[currentBuildNode.parentNum].setAboveChild(nodeNum);

            maxPrimsOffset = std::max(maxPrimsOffset, (uint32_t) (currentBuildNode.primNums - &prims[0]));

            // Get next free node from _nodes_ array
            if (nextFreeNode == nAllocedNodes) {
                uint32_t nNewAllocNodes = std::max(2u * nAllocedNodes, 512u);
                auto *n = AllocAligned<BSPPaperNode>(nNewAllocNodes);
                if (nAllocedNodes > 0) {
                    memcpy(n, nodes, nAllocedNodes * sizeof(BSPPaperNode));
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
            std::pair<KDOPMeshCluster, KDOPMeshCluster> bestSplittedKDOPs;
            std::pair<Float, Float> bestSplittedKDOPAreas = std::make_pair(0, 0);
            Float bestCost = Infinity;
            Float oldCost = isectCost * Float(currentBuildNode.nPrimitives);
            const Float invTotalSA = 1 / currentBuildNode.kdopMeshArea;
            std::pair<KDOPMeshCluster, KDOPMeshCluster> splittedKDOPs;

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

    bool BSPPaper::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
        ProfilePhase p(Prof::AccelIntersect);
        // Compute initial parametric range of ray inside rbsp-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse rbsp-tree for ray
        PBRT_CONSTEXPR uint32_t maxTodo = 64u;
        BSPToDo<BSPPaperNode> todo[maxTodo];
        uint32_t todoPos = 0;

        // Traverse rbsp-tree nodes in order for ray
        bool hit = false;
        const BSPPaperNode *node = &nodes[0];
        while (node != nullptr) {
            // Bail out if we found a hit closer than the current node
            if (ray.tMax < tMin) break;
            nbNodeTraversals++;
            ray.stats.rBSPTreeNodeTraversals++;
            if (!node->IsLeaf()) {
                // Process rbsp-tree interior node

                // Compute parametric distance along ray to split plane
                const Vector3f axis = node->SplitAxis();
                Float projectedO, inverseProjectedD;
                const Float tPlane = planeDistance(axis, node->SplitPos(), ray, projectedO,
                                                   inverseProjectedD);

                // Get node children pointers for ray
                const BSPPaperNode *firstChild, *secondChild;
                const bool belowFirst =
                        (projectedO < node->SplitPos()) ||
                        (projectedO == node->SplitPos() && inverseProjectedD <= 0);
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
                const uint32_t nPrimitives = node->nPrimitives();
                if (nPrimitives == 1) {
                    const std::shared_ptr<Primitive> &p =
                            primitives[node->onePrimitive];
                    // Check one primitive inside leaf node
                    if (p->Intersect(ray, isect)) hit = true;
                } else {
                    for (uint32_t i = 0; i < nPrimitives; ++i) {
                        const uint32_t index =
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

    bool BSPPaper::IntersectP(const Ray &ray) const {
        ProfilePhase p(Prof::AccelIntersectP);
        // Compute initial parametric range of ray inside rbsp-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse rbsp-tree for ray
        PBRT_CONSTEXPR uint32_t maxTodo = 64;
        BSPToDo<BSPPaperNode> todo[maxTodo];
        uint32_t todoPos = 0;
        const BSPPaperNode *node = &nodes[0];
        while (node != nullptr) {
            nbNodeTraversalsP++;
            ray.stats.rBSPTreeNodeTraversalsP++;
            if (node->IsLeaf()) {
                // Check for shadow ray intersections inside leaf node
                const uint32_t nPrimitives = node->nPrimitives();
                if (nPrimitives == 1) {
                    const std::shared_ptr<Primitive> &p =
                            primitives[node->onePrimitive];
                    if (p->IntersectP(ray)) {
                        return true;
                    }
                } else {
                    for (uint32_t i = 0; i < nPrimitives; ++i) {
                        const uint32_t primitiveIndex =
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
                // Process rbsp-tree interior node

                // Compute parametric distance along ray to split plane
                const Vector3f axis = node->SplitAxis();
                Float projectedO, inverseProjectedD;
                const Float tPlane = planeDistance(axis, node->SplitPos(), ray, projectedO,
                                                   inverseProjectedD);

                // Get node children pointers for ray
                const BSPPaperNode *firstChild, *secondChild;
                const bool belowFirst =
                        (projectedO < node->SplitPos()) ||
                        (projectedO == node->SplitPos() && inverseProjectedD <= 0);
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

    std::shared_ptr<BSPPaper> CreateBSPPaperTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        uint32_t isectCost = (uint32_t) ps.FindOneInt("intersectcost", 80);
        uint32_t travCost = (uint32_t) ps.FindOneInt("traversalcost", 5);
        Float emptyBonus = ps.FindOneFloat("emptybonus", 0);
        uint32_t maxPrims = (uint32_t) ps.FindOneInt("maxprims", 1);
        uint32_t maxDepth = (uint32_t) ps.FindOneInt("maxdepth", -1);
        uint32_t nbDirections = (uint32_t) ps.FindOneInt("nbDirections", 3); //TODO: this is K?

        return std::make_shared<BSPPaper>(std::move(prims), isectCost, travCost, emptyBonus,
                                          maxPrims, maxDepth, nbDirections);
    }
} // namespace pbrt