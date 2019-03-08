//
// Created by jesse on 07.03.19.
//

#ifndef PBRT_V3_BSP_H
#define PBRT_V3_BSP_H

#include "genericBSP.h"
#include "kDOPMesh.h"

namespace pbrt {
    struct BSPNode {
        // BSPNode Methods
        void InitLeaf(uint32_t *primNums, uint32_t np, std::vector<uint32_t> *primitiveIndices);

        void InitInterior(const Vector3f &axis, Float s) {
            split = s;
            splitAxis = axis;
            flags = 0;
        }

        void setAboveChild(uint32_t ac) {
            aboveChild |= (ac << 1u);
        }

        Float SplitPos() const { return split; }

        uint32_t nPrimitives() const { return nPrims >> 1u; }

        Vector3f SplitAxis() const { return splitAxis; }

        bool IsLeaf() const { return (flags & 1u) == 1u; }

        uint32_t AboveChild() const { return aboveChild >> 1u; }

        uint32_t depth(BSPNode *nodes, int id = 0) {
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

    class BSP: public GenericBSP<BSPNode> {
         public:
         BSP(std::vector<std::shared_ptr<Primitive>> p,
             uint32_t isectCost = 80, uint32_t traversalCost = 1,
             Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1u, uint32_t nbDirections = 3,
             Float splitAlpha = 90, uint32_t alphaType = 0, uint32_t axisSelectionType = 0,
             uint32_t axisSelectionAmount = -1u);

        virtual bool Intersect(const Ray &ray, SurfaceInteraction *isect) const override;
;
        virtual bool IntersectP(const Ray &ray) const override;

        void printNodes(std::ofstream &os) const override;

        protected:

    };


    struct BSPBuildNode {
        BSPBuildNode(uint32_t depth, uint32_t nPrimitives, uint32_t badRefines,
                     KDOPMeshCluster kdopMesh,
                     Float kdopMeshArea,
                     uint32_t *primNums, uint32_t parentNum = -1)
                : depth(depth),
                  nPrimitives(nPrimitives), badRefines(badRefines),
                  kDOPMesh(std::move(kdopMesh)), kdopMeshArea(kdopMeshArea), primNums(primNums), parentNum(parentNum) {}

        uint32_t depth;
        uint32_t nPrimitives;
        uint32_t badRefines;
        KDOPMeshCluster kDOPMesh;
        Float kdopMeshArea;
        uint32_t *primNums;
        uint32_t parentNum;
    };
};


#endif //PBRT_V3_BSP_H
