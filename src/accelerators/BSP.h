//
// Created by jesse on 07.03.19.
//

#ifndef PBRT_V3_BSP_H
#define PBRT_V3_BSP_H

#include "genericBSP.h"

namespace pbrt {
    template<class T>
    inline void treeInitLeaf(T *node, uint32_t *primNums, uint32_t np,
                             std::vector<uint32_t> *primitiveIndices) {
        node->flags = 1u;
        node->nPrims |= (np << 1u);
        // Store primitive ids for leaf node
        if (np == 0)
            node->onePrimitive = 0;
        else if (np == 1u)
            node->onePrimitive = primNums[0];
        else {
            node->primitiveIndicesOffset = primitiveIndices->size();
            for (uint32_t i = 0; i < np; ++i) primitiveIndices->push_back(primNums[i]);
        }
    }

    template<class T>
    inline void treeSetAboveChild(T *node, uint32_t ac) {
        node->aboveChild |= (ac << 1u);
    }

    template<class T>
    inline void treeInitInterior(T *node, const Vector3f &axis, Float s) {
        node->split = s;
        node->splitAxis = axis;
        node->flags = 0;
    }

    template<class T>
    inline uint32_t treeDepth(T *node, T *nodes, int id = 0) {
        if (treeIsLeaf(node))
            return 0;
        else
            return 1 + std::max(treeDepth(&nodes[treeAboveChild(node)], nodes, treeAboveChild(node)),
                                treeDepth(&nodes[id + 1], nodes, id + 1));
    }

    template<class T>
    inline bool treeIsLeaf(const T *node) { return (node->flags & 1u) == 1u; }

    template<class T>
    inline uint32_t treeAboveChild(const T *node) { return node->aboveChild >> 1u; }

    template<class T>
    inline Float treeSplitPos(const T *node) { return node->split; }

    template<class T>
    inline uint32_t treenPrimitives(const T *node) {
        return node->nPrims >> 1u; }

    template<class T>
    inline Vector3f treeSplitAxis(const T *node) { return node->splitAxis; }

    template<class T>
    inline std::pair<Float, bool> treeIntersectInterior(const T *node, const Ray &ray, const Vector3f &invDir) {
        const Vector3f axis = treeSplitAxis(node);
        Float projectedO, inverseProjectedD;
        const Float tPlane = planeDistance(axis, treeSplitPos(node), ray, projectedO,
                                           inverseProjectedD);

        // Get node children pointers for ray
        const bool belowFirst =
                (projectedO < treeSplitPos(node)) ||
                (projectedO == treeSplitPos(node) && inverseProjectedD <= 0);

        return std::make_pair(tPlane, belowFirst);
    }

    template<class T>
    inline bool treeIntersectLeaf(const T *node, const Ray &ray, const std::vector<std::shared_ptr<Primitive>> &primitives, const std::vector<uint32_t> &primitiveIndices, SurfaceInteraction *isect) {
        bool hit = false;
        const uint32_t nPrimitives = treenPrimitives(node);
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
        return hit;
    }

    template<class T>
    inline bool treeIntersectPLeaf(const T *node, const Ray &ray, const std::vector<std::shared_ptr<Primitive>> &primitives, const std::vector<uint32_t> &primitiveIndices) {
        const uint32_t nPrimitives = treenPrimitives(node);
        ray.stats.insertLeafNodeIntersectionP(nPrimitives); // Important: not all of these are always tested
        if (nPrimitives == 1) {
            const std::shared_ptr<Primitive> &p =
                    primitives[node->onePrimitive];
            if (p->IntersectP(ray)) return true;

        } else {
            for (uint32_t i = 0; i < nPrimitives; ++i) {
                const uint32_t primitiveIndex =
                        primitiveIndices[node->primitiveIndicesOffset + i];
                const std::shared_ptr<Primitive> &prim =
                        primitives[primitiveIndex];
                if (prim->IntersectP(ray)) return true;
            }
        }
        return false;
    }

    struct BSPNode {
        // BSPNode Methods
        //void InitLeaf(uint32_t *primNums, uint32_t np, std::vector<uint32_t> *primitiveIndices);

        /*void InitInterior(const Vector3f &axis, Float s) {
            split = s;
            splitAxis = axis;
            flags = 0;
        }*/

        /*Float SplitPos() const { return split; }

        uint32_t nPrimitives() const { return nPrims >> 1u; }

        Vector3f SplitAxis() const { return splitAxis; }*/

        //bool IsLeaf() const { return (flags & 1u) == 1u; }

        //uint32_t AboveChild() const { return aboveChild >> 1u; }

        /*uint32_t depth(BSPNode *nodes, int id = 0) {
            if (IsLeaf())
                return 0;
            else
                return 1 + std::max(nodes[AboveChild()].depth(nodes, AboveChild()),
                                    nodes[id + 1].depth(nodes, id + 1));
        }*/

        std::string toString(const std::vector<uint32_t> &primitiveIndices) {
            std::stringstream ss;
            if (treeIsLeaf(this)) {
                uint32_t np = treenPrimitives(this);
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
                ss << "I " << treeSplitAxis(this).x << " " << treeSplitAxis(this).y << " " << treeSplitAxis(this).z << " " << treeSplitPos(this) << " "
                   << treeAboveChild(this);
            }


            return ss.str();
        }

        union {
            Float split;                 // Interior
            uint32_t onePrimitive;            // Leaf
            uint32_t primitiveIndicesOffset;  // Leaf
        };
        union {
            uint32_t flags;       // Both
            uint32_t nPrims;      // Leaf
            uint32_t aboveChild;  // Interior
        };
        Vector3f splitAxis;
    };

    class BSP: public GenericBSP<BSPNode> {
         public:
         BSP(std::vector<std::shared_ptr<Primitive>> p,
             uint32_t isectCost = 80, uint32_t traversalCost = 1,
             Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1u, uint32_t nbDirections = 3,
             Float splitAlpha = 90, uint32_t alphaType = 0, uint32_t axisSelectionType = 0,
             uint32_t axisSelectionAmount = -1u);

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const override;
;

        bool IntersectP(const Ray &ray) const override;

        void printNodes(std::ofstream &os) const override;

        protected:

    };
};


#endif //PBRT_V3_BSP_H
