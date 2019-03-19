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
#include "kDOPMesh.h"


namespace pbrt {
    /*struct TreeNodeInterface {
        virtual bool IsLeaf() const = 0;

        virtual uint32_t AboveChild() const = 0;

        virtual std::string toString(const std::vector<uint32_t> &primitiveIndices) = 0;

        virtual std::pair<Float, bool>intersectInterior(const Ray &ray, const Vector3f &invDir) const = 0;

        virtual bool intersectLeaf(const Ray &ray, const std::vector<std::shared_ptr<Primitive>> &primitives, const std::vector<uint32_t> &primitiveIndices, SurfaceInteraction *isect) const = 0;

        virtual bool intersectPLeaf(const Ray &ray, const std::vector<std::shared_ptr<Primitive>> &primitives, const std::vector<uint32_t> &primitiveIndices) const = 0;

        virtual void setAboveChild(uint32_t ac) = 0;

        virtual Float SplitPos() const = 0;

        virtual uint32_t nPrimitives() const = 0;

        virtual Vector3f SplitAxis() const = 0;

        virtual void InitLeaf(uint32_t *primNums, uint32_t np, std::vector<uint32_t> *primitiveIndices) = 0;

        virtual void InitInterior(const Vector3f &axis, Float s) = 0;

        virtual void InitInterior(uint32_t &axis, Float s) = 0;

        virtual uint32_t depth(TreeNodeInterface *nodes, int id = 0) const = 0;
    };*/

    //struct TreeNode {
        /*bool IsLeaf() const {return false;};

        uint32_t AboveChild() const {return 0;};

        std::string toString(const std::vector<uint32_t> &primitiveIndices) { return ""; };

        std::pair<Float, bool>intersectInterior(const Ray &ray, const Vector3f &invDir) const { return std::make_pair(0,false); };

        bool intersectLeaf(const Ray &ray, const std::vector<std::shared_ptr<Primitive>> &primitives, const std::vector<uint32_t> &primitiveIndices, SurfaceInteraction *isect) const { return false; };

        bool intersectPLeaf(const Ray &ray, const std::vector<std::shared_ptr<Primitive>> &primitives, const std::vector<uint32_t> &primitiveIndices) const { return false; };

        void setAboveChild(uint32_t ac) {};

        Float SplitPos() const { return 0; };

        uint32_t nPrimitives() const { return 0; };

        Vector3f SplitAxis() const { Vector3f(0,0,0); };

        void InitLeaf(uint32_t *primNums, uint32_t np, std::vector<uint32_t> *primitiveIndices) {};

        virtual void InitInterior(const Vector3f &axis, Float s) {};

        virtual void InitInterior(uint32_t &axis, Float s) {};

        virtual uint32_t depth(TreeNodeInterface *nodes, int id = 0) const {return 0;};*/
    //};

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

        Bounds3f WorldBound() const override { return bounds; }

        ~GenericBSP() override { FreeAligned(nodes); };

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const override = 0;

        bool IntersectP(const Ray &ray) const override = 0;

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

    struct BSPBuildNode {
        BSPBuildNode(uint32_t depth, uint32_t nPrimitives, uint32_t badRefines,
                     KDOPMeshWithDirections kdopMesh,
                     Float kdopMeshArea,
                     uint32_t *primNums, uint32_t parentNum = -1)
                : depth(depth),
                  nPrimitives(nPrimitives), badRefines(badRefines),
                  kDOPMesh(std::move(kdopMesh)), kdopMeshArea(kdopMeshArea), primNums(primNums), parentNum(parentNum) {}

        uint32_t depth;
        uint32_t nPrimitives;
        uint32_t badRefines;
        KDOPMeshWithDirections kDOPMesh;
        Float kdopMeshArea;
        uint32_t *primNums;
        uint32_t parentNum;
    };
}
#endif //PBRT_V3_GENERIC_RBSP_H
