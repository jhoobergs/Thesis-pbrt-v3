//
// Created by jesse on 13.02.19.
//

#ifndef PBRT_V3_BSPPAPER_H
#define PBRT_V3_BSPPAPER_H

#include "genericBSP.h"
#include "kDOPMesh.h"

namespace pbrt {
    struct BSPPaperNode;

    class BSPPaper : public GenericBSP<BSPPaperNode> {
    public:

        BSPPaper(std::vector<std::shared_ptr<Primitive>> p,
                 uint32_t isectCost = 80, uint32_t traversalCost = 1,
                 Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3);

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const override;

        bool IntersectP(const Ray &ray) const override;

        void printNodes(std::ofstream &os) const override;

    protected:
        void buildTree() override;

    };

    struct BSPPaperBuildNode {
        BSPPaperBuildNode(uint32_t depth, uint32_t nPrimitives, uint32_t badRefines,
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

    std::shared_ptr<BSPPaper> CreateBSPPaperTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}


#endif //PBRT_V3_BSPPAPER_H
