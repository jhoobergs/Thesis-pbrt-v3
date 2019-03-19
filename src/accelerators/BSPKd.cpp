//
// Created by jesse on 18.03.19.
//

#include "BSPKd.h"

namespace pbrt {

    BSPKd::BSPKd(std::vector<std::shared_ptr<pbrt::Primitive>> p, uint32_t isectCost,
                           uint32_t traversalCost,
                           Float emptyBonus, uint32_t maxPrims, uint32_t maxDepth, uint32_t nbDirections, uint32_t kdTravCost)
            : GenericBSP(std::move(p), isectCost, traversalCost, emptyBonus, maxPrims, maxDepth, nbDirections,
                         0, 0, 0, 0), kdTraversalCost(kdTravCost) {}

    void BSPKd::printNodes(std::ofstream &os) const {
        for (int i = 0; i < nextFreeNode; i++) {
            os << nodes[i].toString(primitiveIndices) << std::endl; // TODO
        }
    }

    bool BSPKd::Intersect(const Ray &ray, SurfaceInteraction *isect) const {

        ProfilePhase p(Prof::AccelIntersect);
        // Compute initial parametric range of ray inside rbsp-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse rbsp-tree for ray
        Vector3f invDir(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
        PBRT_CONSTEXPR uint32_t maxTodo = 64u;
        BSPToDo<BSPKdNode> todo[maxTodo];
        uint32_t todoPos = 0;

        // Traverse rbsp-tree nodes in order for ray
        bool hit = false;
        const BSPKdNode *node = &nodes[0];
        while (node != nullptr) {
            // Bail out if we found a hit closer than the current node
            if (ray.tMax < tMin) break;
            if(node->isKdNode())
                ray.stats.kdTreeNodeTraversals++;
            else
                ray.stats.rBSPTreeNodeTraversals++;
            if (!node->isLeaf()) {
                // Process rbsp-tree interior node

                // Compute parametric distance along ray to split plane
                const std::pair<Float, bool> intersection = node->intersectInterior(ray, invDir);
                const Float tPlane = intersection.first;
                const bool belowFirst = intersection.second;

                // Get node children pointers for ray
                const BSPKdNode *firstChild, *secondChild;
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
                if(node->intersectLeaf(ray, primitives, primitiveIndices, isect))
                    hit = true;

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

    bool BSPKd::IntersectP(const Ray &ray) const {
        ProfilePhase p(Prof::AccelIntersectP);
        // Compute initial parametric range of ray inside rbsp-tree extent
        Float tMin, tMax;
        if (!bounds.IntersectP(ray, &tMin, &tMax)) {
            return false;
        }

        // Prepare to traverse rbsp-tree for ray
        PBRT_CONSTEXPR uint32_t maxTodo = 64;
        BSPToDo<BSPKdNode> todo[maxTodo];
        uint32_t todoPos = 0;
        const BSPKdNode *node = &nodes[0];
        while (node != nullptr) {
            if(node->isKdNode())
                ray.stats.kdTreeNodeTraversalsP++;
            else
                ray.stats.rBSPTreeNodeTraversalsP++;
            if (node->isLeaf()) {
                // Check for shadow ray intersections inside leaf node
                if (node->intersectPLeaf(ray, primitives, primitiveIndices))
                    return true;

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
                const std::pair<Float, bool> intersection = node->intersectInterior(ray, Vector3f(0,0,0));
                const Float tPlane = intersection.first;
                const bool belowFirst = intersection.second;

                // Get node children pointers for ray
                const BSPKdNode *firstChild, *secondChild;
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