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

        std::pair<Float, bool> intersectInterior(const Ray &ray, const Vector3f &invDir) const {
            const Vector3f axis = this->SplitAxis();
            Float projectedO, inverseProjectedD;
            const Float tPlane = planeDistance(axis, this->SplitPos(), ray, projectedO,
                                               inverseProjectedD);

            // Get node children pointers for ray
            const bool belowFirst =
                    (projectedO < this->SplitPos()) ||
                    (projectedO == this->SplitPos() && inverseProjectedD <= 0);

            return std::make_pair(tPlane, belowFirst);
        }

        bool intersectLeaf(const Ray &ray, const std::vector<std::shared_ptr<Primitive>> &primitives, const std::vector<uint32_t> &primitiveIndices, SurfaceInteraction *isect) const{
            bool hit = false;
            const uint32_t nPrimitives = this->nPrimitives();
            if (nPrimitives == 1) {
                const std::shared_ptr<Primitive> &p =
                        primitives[this->onePrimitive];
                // Check one primitive inside leaf node
                if (p->Intersect(ray, isect)) hit = true;
            } else {
                for (uint32_t i = 0; i < nPrimitives; ++i) {
                    const uint32_t index =
                            primitiveIndices[this->primitiveIndicesOffset + i];
                    const std::shared_ptr<Primitive> &p = primitives[index];
                    // Check one primitive inside leaf node
                    if (p->Intersect(ray, isect)) hit = true;
                }
            }
            return hit;
        }

        bool intersectPLeaf(const Ray &ray, const std::vector<std::shared_ptr<Primitive>> &primitives, const std::vector<uint32_t> &primitiveIndices) const{
            const uint32_t nPrimitives = this->nPrimitives();
            if (nPrimitives == 1) {
                const std::shared_ptr<Primitive> &p =
                        primitives[this->onePrimitive];
                if (p->IntersectP(ray)) return true;

            } else {
                for (uint32_t i = 0; i < nPrimitives; ++i) {
                    const uint32_t primitiveIndex =
                            primitiveIndices[this->primitiveIndicesOffset + i];
                    const std::shared_ptr<Primitive> &prim =
                            primitives[primitiveIndex];
                    if (prim->IntersectP(ray)) return true;
                }
            }
            return false;
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

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const override;
;

        bool IntersectP(const Ray &ray) const override;

        void printNodes(std::ofstream &os) const override;

        protected:

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
};


#endif //PBRT_V3_BSP_H
