//
// Created by jesse on 11.11.18.
//

#include <core/stats.h>
#include <core/progressreporter.h>
#include "paramset.h"
#include "accelerators/rbsp.h"
#include <shapes/triangle.h>
#include <random>

namespace pbrt {

    STAT_COUNTER_DOUBLE("Accelerator/RBSP-tree param:splitalpha", statParamSplitAlpha);
    STAT_COUNTER_DOUBLE("Accelerator/RBSP-tree param:alphatype", statParamAlphaType);
    STAT_COUNTER("Accelerator/RBSP-tree node traversals during intersect", nbNodeTraversals);
    STAT_COUNTER("Accelerator/RBSP-tree node traversals during intersectP", nbNodeTraversalsP);
    STAT_COUNTER("Accelerator/RBSP-tree nodes", nbNodes);
    STAT_COUNTER("Accelerator/RBSP-tree build: splitTests", statNbSplitTests);
    STAT_COUNTER("Accelerator/RBSP-tree param:directions", statParamnbDirections);
    STAT_COUNTER("Accelerator/RBSP-tree param:intersectioncost", statParamIntersectCost);
    STAT_COUNTER("Accelerator/RBSP-tree param:axisSelectionType", statParamAxisSelectionType);
    STAT_COUNTER("Accelerator/RBSP-tree param:axisSelectionAmount", statParamAxisSelectionAmount);

    STAT_COUNTER("Accelerator/RBSP-tree param:traversalcost", statParamTraversalCost);
    STAT_COUNTER("Accelerator/RBSP-tree param:maxprims", statParamMaxPrims);
    STAT_COUNTER_DOUBLE("Accelerator/RBSP-tree param:emptybonus", statParamEmptyBonus);
    STAT_COUNTER_DOUBLE("Accelerator/RBSP-tree param:maxdepth", statParamMaxDepth);
    STAT_COUNTER_DOUBLE("Accelerator/RBSP-tree SA-cost", totalSACost);
    STAT_COUNTER_DOUBLE("Accelerator/RBSP-tree Depth", statDepth);

    struct RBSPNode {
        // BSPNode Methods
        void InitLeaf(uint32_t M, uint32_t *primNums, uint32_t np, std::vector<uint32_t> *primitiveIndices);

        void InitInterior(uint32_t axis, Float s) {
            split = s;
            flags = axis;
        }

        void setAboveChild(uint32_t M, uint32_t ac) {
            aboveChild |= (ac << getBitOffset(M));
        }

        Float SplitPos() const { return split; }

        uint32_t nPrimitives(uint32_t M) const { return nPrims >> getBitOffset(M); }

        uint32_t SplitAxis(uint32_t M) const { return flags & getBitMask(M); }

        bool IsLeaf(uint32_t M) const { return (flags & getBitMask(M)) == M; }

        uint32_t AboveChild(uint32_t M) const { return aboveChild >> getBitOffset(M); }

        uint32_t depth(uint32_t M, RBSPNode *nodes, int id = 0) {
            if (IsLeaf(M))
                return 0;
            else
                return 1 + std::max(nodes[AboveChild(M)].depth(M, nodes, AboveChild(M)),
                                    nodes[id + 1].depth(M, nodes, id + 1));
        }

        std::string toString(uint32_t M, const std::vector<uint32_t> &primitiveIndices) {
            std::stringstream ss;
            if (IsLeaf(M)) {
                uint32_t np = nPrimitives(M);
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
                ss << "I " << SplitAxis(M) << " " << SplitPos() << " " << AboveChild(M);
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

    void RBSPNode::InitLeaf(uint32_t M, uint32_t *primNums, uint32_t np,
                            std::vector<uint32_t> *primitiveIndices) {
        flags = M;
        nPrims |= (np << getBitOffset(M));
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

    RBSP::RBSP(std::vector<std::shared_ptr<pbrt::Primitive>> p, uint32_t isectCost, uint32_t traversalCost,
               Float emptyBonus, uint32_t maxPrims, uint32_t maxDepth, uint32_t nbDirections, Float splitAlpha,
               uint32_t alphaType,
               uint32_t axisSelectionType, uint32_t axisSelectionAmount)
            : GenericBSP(std::move(p), isectCost, traversalCost, emptyBonus, maxPrims, maxDepth, nbDirections,
                         splitAlpha, alphaType, axisSelectionType, axisSelectionAmount) {
        ProfilePhase _(Prof::AccelConstruction);

        statParamnbDirections = nbDirections;
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

        directions = getDirections(nbDirections);

        // Start recursive construction of RBSP-tree
        buildTree();
    }

    void RBSP::printNodes(std::ofstream &os) const {
        for (int i = 0; i < nextFreeNode; i++) {
            os << nodes[i].toString(directions.size(), primitiveIndices) << std::endl;
        }
    }


    void RBSP::buildTree() {
        //Initialize
        pbrt::BoundsMf rootNodeMBounds;
        for (auto &d: directions) {
            rootNodeMBounds.emplace_back(Boundsf());
        }

        // Compute bounds for rbsp-tree construction
        std::vector<BoundsMf> allPrimBounds;
        allPrimBounds.reserve(primitives.size());
        for (const std::shared_ptr<Primitive> &prim : primitives) {
            bounds = Union(bounds, prim->WorldBound());
            pbrt::BoundsMf mBounds;
            for (size_t i = 0; i < directions.size(); ++i) {
                auto &d = directions[i];
                Boundsf b = prim->getBounds(d);
                mBounds.emplace_back(b);
                rootNodeMBounds[i] = Union(rootNodeMBounds[i], b);
            }
            allPrimBounds.emplace_back(mBounds);
        }

        KDOPMesh kDOPMesh;
        std::vector<Vector3f> directionsOfKdop;
        bounds.toKDOPMesh(kDOPMesh, directionsOfKdop);

        // Building
        ProgressReporter reporter(2 * primitives.size() * maxDepth - 1, "Building");

        auto M = (uint32_t) directions.size();
        uint32_t nodeNum = 0;
        // Allocate working memory for rbsp-tree construction
        std::vector<std::unique_ptr<BoundEdge[]>> edges;
        for (uint32_t i = 0; i < M; ++i)
            edges.emplace_back(std::unique_ptr<BoundEdge[]>(new BoundEdge[2 * primitives.size()]));
        std::unique_ptr<uint32_t[]> prims(
                new uint32_t[(maxDepth + 1) * primitives.size()]); // TODO: use vector ?
        // Initialize _primNums_ for rbsp-tree construction
        for (uint32_t i = 0; i < primitives.size(); ++i) {
            prims[i] = i;
        }


        uint32_t maxPrimsOffset = 0;
        double currentSACost = 0;

        std::vector<RBSPBuildNode> stack;
        stack.emplace_back(maxDepth, (uint32_t) primitives.size(), 0u, rootNodeMBounds, kDOPMesh,
                           kDOPMesh.SurfaceArea(directions), &prims[0]);
        // Warning("Building RBSP: Lets loop");
        while (!stack.empty()) { // || nodeNum > 235000 || nodeNum > 1400000
            if (nodeNum % 100000 == 0)
                Warning("Nodenum %d", nodeNum);
            reporter.Update();
            RBSPBuildNode currentBuildNode = stack.back();
            stack.pop_back();
            CHECK_EQ(nodeNum, nextFreeNode);

            if (currentBuildNode.parentNum != -1)
                nodes[currentBuildNode.parentNum].setAboveChild(M, nodeNum);

            maxPrimsOffset = std::max(maxPrimsOffset, (uint32_t) (currentBuildNode.primNums - &prims[0]));

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
                currentSACost += currentBuildNode.nPrimitives * isectCost * currentBuildNode.kdopMeshArea;
                nodes[nodeNum++].InitLeaf(M, currentBuildNode.primNums, currentBuildNode.nPrimitives,
                                          &primitiveIndices);
                continue;
            }


            // Choose split axis position for interior node
            uint32_t bestD = -1, bestOffset = -1;
            std::pair<KDOPMesh, KDOPMesh> bestSplittedKDOPs;
            std::pair<Float, Float> bestSplittedKDOPAreas = std::make_pair(0, 0);
            Float bestCost = Infinity;
            Float oldCost = isectCost * Float(currentBuildNode.nPrimitives);
            // Float totalSA = currentBuildNode.kDOPMesh.SurfaceArea(directions);
            const Float invTotalSA = 1 / currentBuildNode.kdopMeshArea;
            std::pair<KDOPMesh, KDOPMesh> splittedKDOPs;

            const uint32_t depth = maxDepth - currentBuildNode.depth;


            for (uint32_t d = 0; d < statParamnbDirections; ++d) {
                //for (uint32_t d = 0; d < M; ++d) {
                /*if(nodeNum == 31 || nodeNum == 308)
                    Warning("Val %d", d);*/
                // Sort _edges_ for _axis_
                for (uint32_t i = 0; i < currentBuildNode.nPrimitives; ++i) {
                    const uint32_t pn = currentBuildNode.primNums[i];
                    const std::vector<Boundsf> &bounds = allPrimBounds[pn];
                    edges[d][2 * i] = BoundEdge(bounds[d].min, pn, true);
                    edges[d][2 * i + 1] = BoundEdge(bounds[d].max, pn, false);
                }
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
                        statNbSplitTests += 1;
                        // Compute cost for split at _i_th edge

                        // Compute child surface areas for split at _edgeT_
                        //if(nodeNum == 31)
                        //    Warning("cutting");
                        splittedKDOPs = currentBuildNode.kDOPMesh.cut(
                                (uint32_t) directions.size(), edgeT, directions[d], d);
                        /* if(nodeNum == 235771)
                            Warning("Cutted %d into %d and %d", i, splittedKDOPs.first.edges.size(), splittedKDOPs.second.edges.size());
                        if(nodeNum == 235771){
                            Warning("Current with t %f, M %d and directions %d = [%f,%f,%f]", edgeT, M, d, directions[d].x, directions[d].y, directions[d].z);
                            for(auto &edge : currentBuildNode.kDOPMesh.edges){
                                Warning("Point3f v%d = Point3f(%f,%f,%f)", edge.v1, edge.v1->x, edge.v1->y, edge.v1->z);
                                Warning("Point3f v%d = Point3f(%f,%f,%f)", edge.v2, edge.v2->x, edge.v2->y, edge.v2->z);
                                Warning("kDOPMesh.addEdge(KDOPEdge(&v%d, &v%d, %d, %d));", edge.v1, edge.v2, edge.faceId1, edge.faceId2);
                            }
                            Warning("First");
                            for(auto &edge : splittedKDOPs.first.edges){
                                Warning("[[%f,%f,%f],[%f,%f,%f]]", edge.v1->x, edge.v1->y, edge.v1->z, edge.v2->x, edge.v2->y, edge.v2->z);
                            }
                            Warning("Second");
                            for(auto &edge : splittedKDOPs.second.edges){
                                Warning("[[%f,%f,%f],[%f,%f,%f]]", edge.v1->x, edge.v1->y, edge.v1->z, edge.v2->x, edge.v2->y, edge.v2->z);
                            }
                        } */
                        const Float areaBelow = splittedKDOPs.first.SurfaceArea(directions);
                        const Float areaAbove = splittedKDOPs.second.SurfaceArea(directions);
                        const Float pBelow = areaBelow * invTotalSA;
                        const Float pAbove = areaAbove * invTotalSA;
                        //if(nodeNum == 31)
                        //    Warning("Calculated SA %d", i);
                        const Float eb = (nAbove == 0 || nBelow == 0) ? emptyBonus : 0;
                        const Float cost =
                                traversalCost +
                                isectCost * (1 - eb) * (pBelow * nBelow + pAbove * nAbove);

                        // Update best split if this is lowest cost so far
                        if (cost < bestCost) {
                            bestCost = cost;
                            bestD = d;
                            bestOffset = i;
                            bestSplittedKDOPs = splittedKDOPs;
                            bestSplittedKDOPAreas.first = areaBelow;
                            bestSplittedKDOPAreas.second = areaAbove;
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
                currentSACost += currentBuildNode.nPrimitives * isectCost * currentBuildNode.kdopMeshArea;
                nodes[nodeNum++].InitLeaf(M, currentBuildNode.primNums, currentBuildNode.nPrimitives,
                                          &primitiveIndices);
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
            //BoundsMf bounds0 = currentBuildNode.nodeBounds, bounds1 = currentBuildNode.nodeBounds;
            //bounds0[bestD].max = bounds1[bestD].min = tSplit;
            BoundsMf bounds0, bounds1;
            for (auto &d: directions) { // TODO ? memory ok ?
                bounds0.emplace_back(Boundsf());
                bounds1.emplace_back(Boundsf());
            }
            for (size_t d = 0; d < M; ++d) {
                for (auto &edge: bestSplittedKDOPs.first.edges) {
                    Boundsf b = edge.getBounds(directions[d]);
                    bounds0[d] = Union(bounds0[d], b);
                }
            }

            for (size_t d = 0; d < M; ++d) {
                for (auto &edge: bestSplittedKDOPs.second.edges) {
                    Boundsf b = edge.getBounds(directions[d]);
                    bounds1[d] = Union(bounds1[d], b);
                }
            }

            currentSACost += traversalCost * currentBuildNode.kdopMeshArea;
            nodes[nodeNum].InitInterior(bestD, tSplit);

            //CHECK(bestSplittedKDOPs.first.edges.size() == 12 && bestSplittedKDOPs.second.edges.size() == 12);
            // TODO: wat by splitten volgens vlak ?
            // TODO: plotten via python als dingen vreemd zijn
            //Float newSASUM = bestSplittedKDOPs.first.SurfaceArea(directions) + bestSplittedKDOPs.second.SurfaceArea(directions);
            //Float oldSASUM = currentBuildNode.kDOPMesh.SurfaceArea(directions)
            //                 + 2 * currentBuildNode.nodeBounds[(bestD + 1) % 3].Diagonal() * currentBuildNode.nodeBounds[(bestD + 2) % 3].Diagonal();
            //Float SASUMDIFF = std::abs(newSASUM - oldSASUM);
            //Float relativeSASUMDIFF = SASUMDIFF / oldSASUM;
            //Warning("Diff %f and rel %f", SASUMDIFF, relativeSASUMDIFF);
            //CHECK(relativeSASUMDIFF < 0.1);
            //if(relativeSASUMDIFF > 0.1)
            //    relativeSASUMDIFF = 4;
            stack.emplace_back(
                    currentBuildNode.depth - 1, n1, currentBuildNode.badRefines, bounds1,
                    bestSplittedKDOPs.second, bestSplittedKDOPAreas.second, prims1, nodeNum);
            stack.emplace_back(
                    currentBuildNode.depth - 1, n0, currentBuildNode.badRefines, bounds0,
                    bestSplittedKDOPs.first, bestSplittedKDOPAreas.first, prims0);

            ++nodeNum;
        }
        reporter.Done();
        statDepth = nodes[0].depth(M, nodes, 0);
        nbNodes = nodeNum;
        totalSACost = currentSACost / bounds.SurfaceArea();
    }

    bool RBSP::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
        ProfilePhase p(Prof::AccelIntersect);
        const auto M = (uint32_t) directions.size();
        // Compute initial parametric range of ray inside rbsp-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse rbsp-tree for ray
        PBRT_CONSTEXPR uint32_t maxTodo = 64u;
        BSPToDo<RBSPNode> todo[maxTodo];
        uint32_t todoPos = 0;

        // Traverse rbsp-tree nodes in order for ray
        bool hit = false;
        const RBSPNode *node = &nodes[0];
        while (node != nullptr) {
            // Bail out if we found a hit closer than the current node
            if (ray.tMax < tMin) break;
            nbNodeTraversals++;
            ray.stats.rBSPTreeNodeTraversals++;
            if (!node->IsLeaf(M)) {
                // Process rbsp-tree interior node

                // Compute parametric distance along ray to split plane
                const uint32_t axis = node->SplitAxis(M);
                Float projectedO, inverseProjectedD;
                const Float tPlane = planeDistance(directions[axis], node->SplitPos(), ray, projectedO,
                                                   inverseProjectedD);

                // Get node children pointers for ray
                const RBSPNode *firstChild, *secondChild;
                const bool belowFirst =
                        (projectedO < node->SplitPos()) ||
                        (projectedO == node->SplitPos() && inverseProjectedD <= 0);
                if (belowFirst) {
                    firstChild = node + 1;
                    secondChild = &nodes[node->AboveChild(M)];
                } else {
                    firstChild = &nodes[node->AboveChild(M)];
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
                const uint32_t nPrimitives = node->nPrimitives(M);
                ray.stats.insertLeafNodeIntersection(nPrimitives);
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

    bool RBSP::IntersectP(const Ray &ray) const {
        ProfilePhase p(Prof::AccelIntersectP);
        const auto M = (uint32_t) directions.size();
        // Compute initial parametric range of ray inside rbsp-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse rbsp-tree for ray
        PBRT_CONSTEXPR uint32_t maxTodo = 64;
        BSPToDo<RBSPNode> todo[maxTodo];
        uint32_t todoPos = 0;
        const RBSPNode *node = &nodes[0];
        while (node != nullptr) {
            nbNodeTraversalsP++;
            ray.stats.rBSPTreeNodeTraversalsP++;
            if (node->IsLeaf(M)) {
                // Check for shadow ray intersections inside leaf node
                const uint32_t nPrimitives = node->nPrimitives(M);
                ray.stats.insertLeafNodeIntersectionP(nPrimitives);
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
                const uint32_t axis = node->SplitAxis(M);
                Float projectedO, inverseProjectedD;
                const Float tPlane = planeDistance(directions[axis], node->SplitPos(), ray, projectedO,
                                                   inverseProjectedD);

                // Get node children pointers for ray
                const RBSPNode *firstChild, *secondChild;
                const bool belowFirst =
                        (projectedO < node->SplitPos()) ||
                        (projectedO == node->SplitPos() && inverseProjectedD <= 0);
                if (belowFirst) {
                    firstChild = node + 1;
                    secondChild = &nodes[node->AboveChild(M)];
                } else {
                    firstChild = &nodes[node->AboveChild(M)];
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

    std::shared_ptr<RBSP> CreateRBSPTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        uint32_t isectCost = (uint32_t) ps.FindOneInt("intersectcost", 80);
        uint32_t travCost = (uint32_t) ps.FindOneInt("traversalcost", 5);
        Float emptyBonus = ps.FindOneFloat("emptybonus", 0);
        uint32_t maxPrims = (uint32_t) ps.FindOneInt("maxprims", 1);
        uint32_t maxDepth = (uint32_t) ps.FindOneInt("maxdepth", -1);
        uint32_t nbDirections = (uint32_t) ps.FindOneInt("nbDirections", 3);
        Float splitAlpha = ps.FindOneFloat("splitalpha", 0);
        uint32_t alphaType = (uint32_t) ps.FindOneInt("alphatype",
                                                      0); // 1 is between 0 and alpha, 2 between 90 and 90-alpha and 3 between 0 and alpha or 90 and 90 - alpha

        uint32_t axisSelectionType = (uint32_t) ps.FindOneInt("axisselectiontype",
                                                              0); // 0 is random, 1 is mean, 2 is simpleCluster
        uint32_t axisSelectionAmount = (uint32_t) ps.FindOneInt("axisselectionamount", nbDirections);
        if (axisSelectionAmount > nbDirections)
            axisSelectionAmount = nbDirections;

        return std::make_shared<RBSP>(std::move(prims), isectCost, travCost, emptyBonus,
                                      maxPrims, maxDepth, nbDirections, splitAlpha, alphaType, axisSelectionType,
                                      axisSelectionAmount);
    }

}  // namespace pbrt