//
// Created by jesse on 07.03.19.
//

#include "BSP.h"
#include <core/stats.h>
#include <core/progressreporter.h>
#include <bits/random.h>
#include "paramset.h"
#include "geometry.h"

namespace pbrt {

    void BSPNode::InitLeaf(uint32_t *primNums, uint32_t np,
                           std::vector<uint32_t> *primitiveIndices) {
        flags = 1u;
        nPrims |= (np << 1u);
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

    BSP::BSP(std::vector<std::shared_ptr<Primitive>> p,
             uint32_t isectCost, uint32_t traversalCost,
             Float emptyBonus, uint32_t maxPrims, uint32_t maxDepth, uint32_t nbDirections,
             Float splitAlpha, uint32_t alphaType, uint32_t axisSelectionType,
             uint32_t axisSelectionAmount)
            : GenericBSP(std::move(p), isectCost, traversalCost, emptyBonus, maxPrims, maxDepth, nbDirections,
                  splitAlpha, alphaType, axisSelectionType, axisSelectionAmount) {};

    void BSP::printNodes(std::ofstream &os) const {
        for (int i = 0; i < nextFreeNode; i++) {
            os << nodes[i].toString(primitiveIndices) << std::endl;
        }
    }

    bool BSP::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
        ProfilePhase p(Prof::AccelIntersect);
        // Compute initial parametric range of ray inside rbsp-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse rbsp-tree for ray
        PBRT_CONSTEXPR uint32_t maxTodo = 64u;
        BSPToDo<BSPNode> todo[maxTodo];
        uint32_t todoPos = 0;

        // Traverse rbsp-tree nodes in order for ray
        bool hit = false;
        const BSPNode *node = &nodes[0];
        while (node != nullptr) {
            // Bail out if we found a hit closer than the current node
            if (ray.tMax < tMin) break;
            ray.stats.rBSPTreeNodeTraversals++;
            if (!node->IsLeaf()) {
                // Process rbsp-tree interior node

                // Compute parametric distance along ray to split plane
                const Vector3f axis = node->SplitAxis();
                Float projectedO, inverseProjectedD;
                const Float tPlane = planeDistance(axis, node->SplitPos(), ray, projectedO,
                                                   inverseProjectedD);

                // Get node children pointers for ray
                const BSPNode *firstChild, *secondChild;
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

    bool BSP::IntersectP(const Ray &ray) const {
        ProfilePhase p(Prof::AccelIntersectP);
        // Compute initial parametric range of ray inside rbsp-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse rbsp-tree for ray
        PBRT_CONSTEXPR uint32_t maxTodo = 64;
        BSPToDo<BSPNode> todo[maxTodo];
        uint32_t todoPos = 0;
        const BSPNode *node = &nodes[0];
        while (node != nullptr) {
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
                const BSPNode *firstChild, *secondChild;
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
}