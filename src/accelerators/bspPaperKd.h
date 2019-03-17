//
// Created by jesse on 17.03.19.
//

#ifndef PBRT_V3_BSPPAPERKD_H
#define PBRT_V3_BSPPAPERKD_H

#include "BSP.h"

namespace pbrt {
    struct BSPKdNode {

        bool isKdNode() const { return (this->flags & 7u) < 4u; }

        uint32_t kdNodeAxis() const { return (this->flags & 7u); }

        uint32_t nPrimitives() const {
            return this->nPrims >> 3u;
        }

        void initLeaf(uint32_t *primNums, uint32_t np,
                                 std::vector<uint32_t> *primitiveIndices) {
            this->flags = 3u; // 3 is leaf, 4 means, non kd
            this->nPrims |= (np << 3u);
            // Store primitive ids for leaf node
            if (np == 0)
                this->onePrimitive = 0;
            else if (np == 1u)
                this->onePrimitive = primNums[0];
            else {
                this->primitiveIndicesOffset = primitiveIndices->size();
                for (uint32_t i = 0; i < np; ++i) primitiveIndices->push_back(primNums[i]);
            }
        }

        void setAboveChild(uint32_t ac) {
            this->aboveChild |= (ac << 3u);
        }

        void initInterior(const Vector3f &axis, Float s) {
            this->split = s;
            this->splitAxis = axis;
            this->flags = 4; // 4 means non kd
        }

        void initInteriorKd(uint32_t axis, Float s) {
            this->split = s;
            this->flags = axis;
        }

        bool isLeaf() const { return (this->flags & 7u) == 3u; }

        uint32_t AboveChild() const { return this->aboveChild >> 3u; }

        std::pair<Float, bool> intersectInterior(const Ray &ray, const Vector3f &invDir) const {
            if(this->isKdNode()){
                uint32_t axis = this->kdNodeAxis();
                Float tPlane = planeDistance(treeSplitPos(this), ray, invDir, axis);

                // Get node children pointers for ray
                bool belowFirst =
                        (ray.o[axis] < treeSplitPos(this)) ||
                        (ray.o[axis] == treeSplitPos(this) && ray.d[axis] <= 0);
                return std::make_pair(tPlane, belowFirst);
            }
            else {
                const Vector3f axis = treeSplitAxis(this);
                Float projectedO, inverseProjectedD;
                const Float tPlane = planeDistance(axis, treeSplitPos(this), ray, projectedO,
                                                   inverseProjectedD);

                // Get node children pointers for ray
                const bool belowFirst =
                        (projectedO < treeSplitPos(this)) ||
                        (projectedO == treeSplitPos(this) && inverseProjectedD <= 0);

                return std::make_pair(tPlane, belowFirst);
            }
        }

        bool intersectLeaf(const Ray &ray, const std::vector<std::shared_ptr<Primitive>> &primitives, const std::vector<uint32_t> &primitiveIndices, SurfaceInteraction *isect) const {
            bool hit = false;
            const uint32_t nPrimitives = this->nPrimitives();
            ray.stats.insertLeafNodeIntersection(nPrimitives);
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

        bool intersectPLeaf(const Ray &ray, const std::vector<std::shared_ptr<Primitive>> &primitives, const std::vector<uint32_t> &primitiveIndices) const {
            const uint32_t nPrimitives = this->nPrimitives();
            ray.stats.insertLeafNodeIntersectionP(nPrimitives); // Important: not all of these are always tested
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

        std::string toString(const std::vector<uint32_t> &primitiveIndices) { // TODO: update for kd -bsp
            std::stringstream ss;
            if (this->isLeaf()) {
                uint32_t np = this->nPrimitives();
                ss << "L " << np;
                if (np == 1) {
                    ss << " " << this->onePrimitive;
                } else {
                    for (int i = 0; i < np; i++) {
                        uint32_t primitiveIndex =
                                primitiveIndices[primitiveIndicesOffset + i];
                        ss << " " << primitiveIndex;
                    }
                }
            } else {
                if(!this->isKdNode()) {
                    ss << "I " << treeSplitAxis(this).x << " " << treeSplitAxis(this).y << " " << treeSplitAxis(this).z
                       << " " << treeSplitPos(this) << " "
                       << this->AboveChild();
                } else {
                    ss << "I " << this->kdNodeAxis() << " " << treeSplitPos(this) << " "
                                                                         << this->AboveChild();
                }
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

    class BSPPaperKd : public GenericBSP<BSPKdNode> {
    public:

        BSPPaperKd(std::vector<std::shared_ptr<Primitive>> p,
                   uint32_t isectCost = 80, uint32_t traversalCost = 5,
                   Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3, uint32_t kdTraversalCost=1);

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const override;

        bool IntersectP(const Ray &ray) const override;

        void printNodes(std::ofstream &os) const override;
    protected:
        void buildTree() override;



    private:
        const uint32_t kdTraversalCost;

    };

    std::shared_ptr<BSPPaperKd> CreateBSPPaperKdTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}


#endif //PBRT_V3_BSPPAPERKD_H
