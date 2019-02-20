//
// Created by jesse on 13.02.19.
//

#ifndef PBRT_V3_RBSPCLUSTER_H
#define PBRT_V3_RBSPCLUSTER_H

#include "genericBSP.h"
#include "kDOPMesh.h"

namespace pbrt {
    struct BSPClusterNode;

    class BSPCluster : public GenericBSP<BSPClusterNode> {
    public:

        BSPCluster(std::vector<std::shared_ptr<Primitive>> p,
                   uint32_t isectCost = 80, uint32_t traversalCost = 1,
                   Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3);

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const override;

        bool IntersectP(const Ray &ray) const override;

        void printNodes(std::ofstream &os) const override;

    protected:
        void buildTree() override;

    private:
        std::vector<Vector3f> calculateClusterMeans(const uint32_t *primNums, const uint32_t np);

        uint32_t calculateIdOfClosestMean(Vector3f &normal, const std::vector<Vector3f> &means);

        Vector3f calculateMeanVector(const std::vector<Vector3f> &vectors);

        Float calculateMaxDifference(const std::vector<Vector3f> &oldMeans, const std::vector<Vector3f> &newMeans);

        uint32_t K;
    };

    struct BSPClusterBuildNode {
        BSPClusterBuildNode(uint32_t depth, uint32_t nPrimitives, uint32_t badRefines,
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

    std::shared_ptr<BSPCluster> CreateBSPClusterTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}


#endif //PBRT_V3_RBSPCLUSTER_H
