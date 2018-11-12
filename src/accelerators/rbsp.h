//
// Created by jesse on 11.11.18.
//

#ifndef PBRT_V3_RBSP_H
#define PBRT_V3_RBSP_H

#include <core/geometry.h>
#include "kdtreeaccel.h"

namespace pbrt {
    struct RBSPNode;
    struct KDOPEdge {
        KDOPEdge(Point3f * v1, Point3f * v2, int faceId1, int faceId2):
                v1(v1), v2(v2), faceId1(faceId1), faceId2(faceId2) {}

        Point3f * v1;
        Point3f * v2;
        int faceId1;
        int faceId2;
    };

    struct KDOPMesh{
        KDOPMesh() {};

        void addEdge(KDOPEdge edge){
            edges.emplace_back(edge);
        }

        float SurfaceArea(const std::vector<Vector3f> &directions){
            std::vector<std::vector<KDOPEdge*>> faces;
            for(size_t i = 0; i < 2 * directions.size(); ++i){
                faces.emplace_back(std::vector<KDOPEdge*>());
            }

            for(auto &edge: edges){
                faces[edge.faceId1].emplace_back(&edge);
                faces[edge.faceId2].emplace_back(&edge);
            }

            float SA = 0;
            for(uint32_t i = 0; i < 2 * directions.size(); ++i){
                Vector3f FSA = Vector3f();
                std::vector<KDOPEdge*>& face = faces[i];
                if(!face.empty()) {
                    uint32_t edgeId = 0;
                    do {
                        KDOPEdge *currentEdge = face[edgeId];
                        FSA += Vector3f(currentEdge->v1->y * currentEdge->v2->z - currentEdge->v1->z * currentEdge->v2->y
                               , -(currentEdge->v1->x * currentEdge->v2->z - currentEdge->v1->z * currentEdge->v2->x)
                               , (currentEdge->v1->x * currentEdge->v2->y - currentEdge->v1->y * currentEdge->v2->x));
                        for (uint32_t j = 0; j < face.size(); ++j) {
                            if(j==edgeId) continue;
                            if (face[j]->v2 == currentEdge->v2) {
                                std::swap(face[j]->v1, face[j]->v2);
                            }
                            if (face[j]->v1 == currentEdge->v2) {
                                edgeId = j;
                                break;
                            }
                        }
                    } while (edgeId != 0);
                }
                SA += std::abs(directions[i/2].dot(FSA));
            }
            return SA / 2.0f;
        }

        std::vector<KDOPEdge> edges;
    };
    class RBSP  : public Aggregate {
    public:

        // KdTreeAccel Public Methods
        RBSP(std::vector<std::shared_ptr<Primitive>> p,
             uint32_t isectCost = 80, uint32_t traversalCost = 1,
                Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1);

        Bounds3f WorldBound() const { return bounds; }

        ~RBSP();

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;

        bool IntersectP(const Ray &ray) const;

    private:
        // RBSP Private Methods
        void buildTree(BoundsMf &rootNodeMBounds, KDOPMesh &kDOPMesh,
                       const std::vector<BoundsMf> &allPrimBounds, const std::vector<Vector3f> &directions, uint32_t maxDepth);

        // KdTreeAccel Private Data
        const uint32_t isectCost, traversalCost, maxPrims;
        const Float emptyBonus;
        std::vector<std::shared_ptr<Primitive>> primitives;
        std::vector<uint32_t> primitiveIndices;
        RBSPNode *nodes;
        uint32_t nAllocedNodes, nextFreeNode;
        Bounds3f bounds;
    };

    struct RBSPBuildNode{
        RBSPBuildNode(uint32_t depth, uint32_t nPrimitives, uint32_t badRefines, BoundsMf nodeBounds, KDOPMesh kdopMesh, uint32_t *primNums, uint32_t parentNum = -1)
                : depth(depth),
                  nPrimitives(nPrimitives), badRefines(badRefines), nodeBounds(std::move(nodeBounds)),
                  kDOPMesh(std::move(kdopMesh)), primNums(primNums), parentNum(parentNum) {}

        uint32_t depth;
        uint32_t nPrimitives;
        uint32_t badRefines;
        BoundsMf nodeBounds;
        KDOPMesh kDOPMesh;
        uint32_t *primNums;
        uint32_t parentNum;
    };

    struct RBSPToDo {
        const RBSPNode *node;
        Float tMin, tMax;
    };

    std::shared_ptr<RBSP> CreateRBSPTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);

}
#endif //PBRT_V3_RBSP_H
