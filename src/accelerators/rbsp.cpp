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
        nextFreeNode = nAllocedNodes = 0;
        if (maxDepth == -1)
            maxDepth = (uint32_t) std::round(8 + 1.3f * Log2Int(int64_t(primitives.size())));

        directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
        directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
        directions.emplace_back(Vector3f(0.0, 0.0, 1.0));

        pbrt::BoundsMf rootNodeMBounds;
        for (auto &d: directions) {
            rootNodeMBounds.emplace_back(Boundsf());
        }

        // Compute bounds for rbsp-tree construction
        std::vector<BoundsMf> primBounds;
        primBounds.reserve(primitives.size());
        for (const std::shared_ptr<Primitive> &prim : primitives) {
            Bounds3f b = prim->WorldBound();
            bounds = Union(bounds, b);
            pbrt::BoundsMf mBounds;
            for (size_t i = 0; i < directions.size(); ++i) {
                auto &d = directions[i];
                Boundsf b = prim->getBounds(d);
                mBounds.emplace_back(b);
                rootNodeMBounds[i] = Union(rootNodeMBounds[i], b);
            }
            primBounds.emplace_back(mBounds);
        }

        KDOPMesh kDOPMesh;
        Point3f v1 = Point3f(bounds.pMin);
        Point3f v2 = Point3f(bounds.pMin.x, bounds.pMin.y, bounds.pMax.z);
        Point3f v3 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMin.z);
        Point3f v4 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMin.z);
        Point3f v5 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMax.z);
        Point3f v6 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMax.z);
        Point3f v7 = Point3f(bounds.pMax.x, bounds.pMax.y, bounds.pMin.z);
        Point3f v8 = Point3f(bounds.pMax);

        kDOPMesh.addEdge(KDOPEdge(&v1, &v2, 1, 3));
        kDOPMesh.addEdge(KDOPEdge(&v1, &v3, 1, 5));
        kDOPMesh.addEdge(KDOPEdge(&v1, &v4, 3, 5));
        kDOPMesh.addEdge(KDOPEdge(&v2, &v5, 1, 4));
        kDOPMesh.addEdge(KDOPEdge(&v2, &v6, 3, 4));
        kDOPMesh.addEdge(KDOPEdge(&v3, &v5, 1, 2));
        kDOPMesh.addEdge(KDOPEdge(&v3, &v7, 2, 5));
        kDOPMesh.addEdge(KDOPEdge(&v4, &v6, 0, 3));
        kDOPMesh.addEdge(KDOPEdge(&v4, &v7, 0, 5));
        kDOPMesh.addEdge(KDOPEdge(&v5, &v8, 2, 4));
        kDOPMesh.addEdge(KDOPEdge(&v6, &v8, 0, 4));
        kDOPMesh.addEdge(KDOPEdge(&v7, &v8, 0, 2));

        // TODO: progressReporter
        // Start recursive construction of RBSP-tree
        buildTree(rootNodeMBounds, kDOPMesh, primBounds, maxDepth);
    }

    void
    RBSP::buildTree(pbrt::BoundsMf &rootNodeMBounds, pbrt::KDOPMesh &kDOPMesh,
                    const std::vector<BoundsMf> &allPrimBounds,
                    uint32_t maxDepth) {
        Warning("Building RBSP");
        uint32_t M = (uint32_t) directions.size();
        uint32_t nodeNum = 0;
        // Allocate working memory for rbsp-tree construction
        std::vector<std::unique_ptr<BoundEdge[]>> edges;
        for (uint32_t i = 0; i < M; ++i)
            edges.emplace_back(std::unique_ptr<BoundEdge[]>(new BoundEdge[2 * primitives.size()]));
        //Warning("Let's create prims");
        std::unique_ptr<uint32_t> prims_p(
                new uint32_t[(maxDepth + 1) * primitives.size()]);
        uint32_t *prims = prims_p.get();
        // Initialize _primNums_ for rbsp-tree construction
        for (uint32_t i = 0; i < primitives.size(); ++i) {
            prims[i] = i;
        }
        uint32_t maxPrimsOffset = 0;

        std::vector<RBSPBuildNode> stack;
        stack.emplace_back(RBSPBuildNode(maxDepth, (uint32_t) primitives.size(), 0u, rootNodeMBounds, kDOPMesh, prims));
        Warning("Building RBSP: Lets loop");
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
            std::pair<KDOPMesh, KDOPMesh> bestSplittedKDOPs;
            Float bestCost = Infinity;
            Float oldCost = isectCost * Float(currentBuildNode.nPrimitives);
            Float totalSA = currentBuildNode.kDOPMesh.SurfaceArea(directions);
            const Float invTotalSA = 1 / totalSA;

            /*for (uint32_t i = 0; i < currentBuildNode.nPrimitives; ++i) {
                const uint32_t pn = currentBuildNode.primNums[i];
                const std::vector<Boundsf> &bounds = allPrimBounds[pn];

                for (uint32_t d = 0; d < M; ++d) {
                    edges[d][2 * i] = BoundEdge(bounds[d].min, pn, true);
                    edges[d][2 * i + 1] = BoundEdge(bounds[d].max, pn, false);
                }
            }*/

            for (uint32_t d = 0; d < M; ++d) {

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
                        // Compute cost for split at _i_th edge

                        // Compute child surface areas for split at _edgeT_
                        std::pair<KDOPMesh, KDOPMesh> splittedKDOPs = currentBuildNode.kDOPMesh.cut(
                                (uint32_t) directions.size(), edgeT, directions[d], d);
                        const Float pBelow = splittedKDOPs.first.SurfaceArea(directions) * invTotalSA;
                        const Float pAbove = splittedKDOPs.second.SurfaceArea(directions) * invTotalSA;
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

            //CHECK(bestSplittedKDOPs.first.edges.size() == 12 && bestSplittedKDOPs.second.edges.size() == 12);
            // TODO: wat by splitten volgens vlak ?
            // TODO: plotten via python als dingen vreemd zijn
            //float newSASUM = bestSplittedKDOPs.first.SurfaceArea(directions) + bestSplittedKDOPs.second.SurfaceArea(directions);
            //float oldSASUM = currentBuildNode.kDOPMesh.SurfaceArea(directions)
            //                 + 2 * currentBuildNode.nodeBounds[(bestD + 1) % 3].Diagonal() * currentBuildNode.nodeBounds[(bestD + 2) % 3].Diagonal();
            //float SASUMDIFF = std::abs(newSASUM - oldSASUM);
            //float relativeSASUMDIFF = SASUMDIFF / oldSASUM;
            //Warning("Diff %f and rel %f", SASUMDIFF, relativeSASUMDIFF);
            //CHECK(relativeSASUMDIFF < 0.1);
            //if(relativeSASUMDIFF > 0.1)
            //    relativeSASUMDIFF = 4;
            stack.emplace_back(
                    RBSPBuildNode(currentBuildNode.depth - 1, n1, currentBuildNode.badRefines, bounds1,
                                  bestSplittedKDOPs.second, prims1, nodeNum));
            stack.emplace_back(
                    RBSPBuildNode(currentBuildNode.depth - 1, n0, currentBuildNode.badRefines, bounds0,
                                  bestSplittedKDOPs.first, prims0));

            ++nodeNum;
        }

        Warning("RBSP Depth %d", nodes[0].depth(nodes, 0));
    }

    bool RBSP::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
        ProfilePhase p(Prof::AccelIntersect);
        // Compute initial parametric range of ray inside rbsp-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }
        // ray.stats.rBSPTreeNodeTraversals = 10000;
        // return false;

        // Prepare to traverse rbsp-tree for ray
        PBRT_CONSTEXPR uint32_t maxTodo = 64u;
        RBSPToDo todo[maxTodo];
        uint32_t todoPos = 0;

        // Traverse rbsp-tree nodes in order for ray
        bool hit = false;
        const RBSPNode *node = &nodes[0];
        while (node != nullptr) {
            // Bail out if we found a hit closer than the current node
            if (ray.tMax < tMin) break;
            nbNodeTraversals++;
            ray.stats.rBSPTreeNodeTraversals++;
            if (!node->IsLeaf()) {
                //Warning("Checking Interior");
                // Process rbsp-tree interior node

                // Compute parametric distance along ray to split plane
                uint32_t axis = node->SplitAxis();

                float projectedO = Dot(directions[axis], ray.o);
                float inverseProjectedD = 1 / Dot(directions[axis], ray.d);
                Float tPlane = (node->SplitPos() - projectedO) * inverseProjectedD;

                // Get node children pointers for ray
                const RBSPNode *firstChild, *secondChild;
                bool belowFirst =
                        (projectedO < node->SplitPos()) ||
                        (projectedO == node->SplitPos() && inverseProjectedD <= 0);
                //Warning("Checking Interior: before if");
                if (belowFirst) {
                    firstChild = node + 1;
                    secondChild = &nodes[node->AboveChild()];
                } else {
                    firstChild = &nodes[node->AboveChild()];
                    secondChild = node + 1;
                }
                //Warning("Checking Interior after if");

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
                //Warning("Checking Interior end");
            } else {
                // Check for intersections inside leaf node
                uint32_t nPrimitives = node->nPrimitives();
                //Warning("Checking Leaf %d", nPrimitives);
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
                //Warning("Checking Leaf finding new");


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
        // Compute initial parametric range of ray inside rbsp-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse rbsp-tree for ray
        PBRT_CONSTEXPR uint32_t maxTodo = 64;
        RBSPToDo todo[maxTodo];
        uint32_t todoPos = 0;
        const RBSPNode *node = &nodes[0];
        while (node != nullptr) {
            nbNodeTraversalsP++;
            ray.stats.rBSPTreeNodeTraversalsP++;
            if (node->IsLeaf()) {
                //Warning("Checking Leaf");
                // Check for shadow ray intersections inside leaf node
                uint32_t nPrimitives = node->nPrimitives();
                //Warning("Checking Leaf %d", nPrimitives);
                if (nPrimitives == 1) {
                    const std::shared_ptr<Primitive> &p =
                            primitives[node->onePrimitive];
                    //Warning("Checking Leaf %d = 1", nPrimitives);
                    if (p->IntersectP(ray)) {
                        //Warning("Checking Leaf %d = 1 hit", nPrimitives);
                        return true;
                    }
                    //Warning("Checking Leaf %d = 1 mis", nPrimitives);
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
                    //Warning("Checking Leaf %d mis, todos %d", nPrimitives, todoPos);
                    --todoPos;
                    node = todo[todoPos].node;
                    tMin = todo[todoPos].tMin;
                    tMax = todo[todoPos].tMax;
                } else {
                    //Warning("breaking Out of Leaf");
                    break;
                }
                //Warning("Out of Leaf");
            } else {
                //Warning("Checking Interior");
                // Process rbsp-tree interior node

                // Compute parametric distance along ray to split plane
                uint32_t axis = node->SplitAxis();
                float projectedO = Dot(directions[axis], ray.o);
                float inverseProjectedD = 1 / Dot(directions[axis], ray.d);
                Float tPlane = (node->SplitPos() - projectedO) * inverseProjectedD;

                // Get node children pointers for ray
                const RBSPNode *firstChild, *secondChild;
                bool belowFirst =
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
                //Warning("Out of Interior");
            }
        }
        return false;
    }

    std::shared_ptr<RBSP> CreateRBSPTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        uint32_t isectCost = (uint32_t) ps.FindOneInt("intersectcost", 80);
        uint32_t travCost = (uint32_t) ps.FindOneInt("traversalcost", 1);
        Float emptyBonus = ps.FindOneFloat("emptybonus", 0);
        uint32_t maxPrims = (uint32_t) ps.FindOneInt("maxprims", 1);
        uint32_t maxDepth = (uint32_t) ps.FindOneInt("maxdepth", -1);

        return std::make_shared<RBSP>(std::move(prims), isectCost, travCost, emptyBonus,
                                      maxPrims, maxDepth);
    }

}  // namespace pbrt