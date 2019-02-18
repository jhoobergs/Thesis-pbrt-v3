//
// Created by jesse on 11.11.18.
//

#ifndef PBRT_V3_GENERIC_RBSP_H
#define PBRT_V3_GENERIC_RBSP_H

#include <core/geometry.h>
#include <algorithm>
#include <core/primitive.h>
#include <fstream>
#include <core/stats.h>


namespace pbrt {
    struct BSPNode;

    enum class EdgeType {
        Start, End
    };

    struct BoundEdge {
        // BoundEdge Public Methods
        BoundEdge() {}

        BoundEdge(Float t, uint32_t primNum, bool starting) : t(t), primNum(primNum) {
            type = starting ? EdgeType::Start : EdgeType::End;
        }

        Float t;
        uint32_t primNum;
        EdgeType type;
    };

    template<class nodeType>
    class GenericBSP : public Aggregate {
    public:

        // KdTreeAccel Public Methods
        GenericBSP(std::vector<std::shared_ptr<Primitive>> p,
                    uint32_t isectCost = 80, uint32_t traversalCost = 1,
                    Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1u, uint32_t nbDirections = 3,
                    Float splitAlpha = 90, uint32_t alphaType = 0, uint32_t axisSelectionType = 0,
                    uint32_t axisSelectionAmount = -1u) : primitives(std::move(p)), isectCost(isectCost),
                                                          traversalCost(traversalCost), emptyBonus(emptyBonus),
                                                          maxPrims(maxPrims), splitAlpha(splitAlpha),
                                                          alphaType(alphaType), axisSelectionType(axisSelectionType) {
            ProfilePhase _(Prof::AccelConstruction);
            nextFreeNode = nAllocedNodes = 0;
            if (maxDepth == -1)
                GenericBSP::maxDepth = calculateMaxDepth(primitives.size());
            else
                this->maxDepth = maxDepth;
            if (axisSelectionAmount == -1)
                GenericBSP::axisSelectionAmount = nbDirections;
            else
                GenericBSP::axisSelectionAmount = axisSelectionAmount;

        }

        Bounds3f WorldBound() const { return bounds; }

        ~GenericBSP() { FreeAligned(nodes); };

        virtual bool Intersect(const Ray &ray, SurfaceInteraction *isect) const = 0;

        virtual bool IntersectP(const Ray &ray) const = 0;

        virtual void printNodes(std::ofstream &os) const = 0;

        friend std::ofstream &operator<<(std::ofstream &os, const GenericBSP<nodeType> &rbspTree) {
            os << rbspTree.directions.size() << std::endl;
            for (auto &direction: rbspTree.directions) {
                os << direction.x << " " << direction.y << " " << direction.z << std::endl;
            }
            os << rbspTree.nextFreeNode << std::endl;
            rbspTree.printNodes(os);
            os << rbspTree.primitives.size() << std::endl;
            for (auto &p: rbspTree.primitives) {
                Normal3f n = p->Normal();
                os << n.x << " " << n.y << " " << n.z << std::endl;
            }
            for (auto &p: rbspTree.primitives) {
                for (auto &direction: rbspTree.directions) {
                    auto b = p->getBounds(direction);
                    os << b.min << " " << b.max << " ";
                }
                os << std::endl;
            }
            for (auto &p: rbspTree.primitives) {
                os << p->getSurfaceArea() << std::endl;
            }
            return os;
        }

    protected:
        // RBSP Private Methods
        virtual void buildTree() = 0;

        // RBSPTreeAccel Private Data
        const uint32_t isectCost, traversalCost, maxPrims, alphaType, axisSelectionType;
        const Float emptyBonus, splitAlpha;
        std::vector<std::shared_ptr<Primitive>> primitives;
        std::vector<uint32_t> primitiveIndices;
        nodeType *nodes;
        uint32_t nAllocedNodes, nextFreeNode, maxDepth, axisSelectionAmount;
        Bounds3f bounds;
        std::vector<Vector3f> directions;
    };

    template<class nodeType>
    struct BSPToDo {
        const nodeType *node;
        Float tMin, tMax;
    };
}
#endif //PBRT_V3_GENERIC_RBSP_H
