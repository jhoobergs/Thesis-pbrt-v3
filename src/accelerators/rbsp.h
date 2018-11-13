//
// Created by jesse on 11.11.18.
//

#ifndef PBRT_V3_RBSP_H
#define PBRT_V3_RBSP_H

#include <core/geometry.h>
#include "kdtreeaccel.h"
#include <set>

namespace pbrt {
    struct RBSPNode;

    struct KDOPEdge {
        KDOPEdge(Point3f *v1, Point3f *v2, uint32_t faceId1, uint32_t faceId2) :
                v1(v1), v2(v2), faceId1(faceId1), faceId2(faceId2) {}

        Point3f *v1;
        Point3f *v2;
        uint32_t faceId1;
        uint32_t faceId2;
    };

    struct KDOPMesh {
        KDOPMesh() {};

        void addEdge(KDOPEdge edge) {
            edges.emplace_back(edge);
        }

        float SurfaceArea(const std::vector<Vector3f> &directions) {
            std::vector<std::vector<KDOPEdge *>> faces;
            for (size_t i = 0; i < 2 * directions.size(); ++i) {
                faces.emplace_back(std::vector<KDOPEdge *>());
            }

            for (auto &edge: edges) {
                faces[edge.faceId1].emplace_back(&edge);
                faces[edge.faceId2].emplace_back(&edge);
            }

            float SA = 0;
            for (uint32_t i = 0; i < 2 * directions.size(); ++i) {
                Vector3f FSA = Vector3f();
                std::vector<KDOPEdge *> &face = faces[i];
                if (!face.empty()) {
                    uint32_t edgeId = 0;
                    do {
                        KDOPEdge *currentEdge = face[edgeId];
                        FSA += Vector3f(
                                currentEdge->v1->y * currentEdge->v2->z - currentEdge->v1->z * currentEdge->v2->y,
                                -(currentEdge->v1->x * currentEdge->v2->z - currentEdge->v1->z * currentEdge->v2->x),
                                (currentEdge->v1->x * currentEdge->v2->y - currentEdge->v1->y * currentEdge->v2->x));
                        for (uint32_t j = 0; j < face.size(); ++j) {
                            if (j == edgeId) continue;
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
                SA += std::abs(directions[i / 2].dot(FSA));
            }
            return SA / 2.0f;
        }

        std::pair<KDOPMesh, KDOPMesh> cut(uint32_t M, float t, const Vector3f &direction, const uint32_t directionId) {
            KDOPMesh left;
            KDOPMesh right;
            std::vector<std::vector<Point3f *>> faceVertices;
            for (size_t i = 0; i < 2 * M; ++i) {
                faceVertices.emplace_back(std::vector<Point3f *>());
            }
            std::vector<std::pair<Point3f *, std::vector<uint32_t> > > vertexFaces;

            for (auto &edge: edges) {
                float t1 = direction.dot(*edge.v1);
                float t2 = direction.dot(*edge.v2);
                Vector3f d = edge.v2->operator-(*edge.v1);
                if (t1 < t && t2 < t) {
                    left.addEdge(edge);
                } else if (t1 > t && t2 > t) {
                    right.addEdge(edge);
                } else if (t1 < t && t == t2) {
                    left.addEdge(edge);
                    faceVertices[edge.faceId1].emplace_back(edge.v2);
                    faceVertices[edge.faceId2].emplace_back(edge.v2);
                    //Warning("Added (%f,%f,%f) to %d and %d", edge.v2->x, edge.v2->y, edge.v2->z, edge.faceId1, edge.faceId2);
                    //Warning("2 Added (%f,%f,%f) to %d and %d", faceVertices[edge.faceId1].back()->x, faceVertices[edge.faceId1].back()->y, faceVertices[edge.faceId1].back()->z, edge.faceId1, edge.faceId2);
                    /*std::vector<int> faces;
                    faces.emplace_back(edge.faceId1);
                    faces.emplace_back(edge.faceId2);
                    vertexFaces.emplace_back(std::make_pair(edge.v2, faces));*/
                } else if (t1 == t && t < t2) {
                    right.addEdge(edge);
                    faceVertices[edge.faceId1].emplace_back(edge.v1);
                    faceVertices[edge.faceId2].emplace_back(edge.v1);
                    //Warning("Added (%f,%f,%f) to %d and %d", edge.v1->x, edge.v1->y, edge.v1->z, edge.faceId1, edge.faceId2);
                    //Warning("2 Added (%f,%f,%f) to %d and %d", faceVertices[edge.faceId1].back()->x, faceVertices[edge.faceId1].back()->y, faceVertices[edge.faceId1].back()->z, edge.faceId1, edge.faceId2);

                    /*std::vector<int> faces;
                    faces.emplace_back(edge.faceId1);
                    faces.emplace_back(edge.faceId2);
                    vertexFaces.emplace_back(std::make_pair(edge.v1, faces));*/
                } else if (t1 < t && t < t2) {
                    float tAlongEdge = (-(t1 - t) * d.Length()) / (t2 - t1);
                    Point3f *vs = new Point3f(*edge.v1 + tAlongEdge * d / d.Length());
                    left.addEdge(KDOPEdge(edge.v1, vs, edge.faceId1, edge.faceId2));
                    right.addEdge(KDOPEdge(vs, edge.v2, edge.faceId1, edge.faceId2));

                    faceVertices[edge.faceId1].emplace_back(vs);
                    faceVertices[edge.faceId2].emplace_back(vs);
                    //Warning("Added (%f,%f,%f) to %d and %d", vs->x, vs->y, vs->z, edge.faceId1, edge.faceId2);
                    //Warning("2 Added (%f,%f,%f) to %d and %d", faceVertices[edge.faceId1].back()->x, faceVertices[edge.faceId1].back()->y, faceVertices[edge.faceId1].back()->z, edge.faceId1, edge.faceId2);

                    /*std::vector<int> faces;
                    faces.emplace_back(edge.faceId1);
                    faces.emplace_back(edge.faceId2);
                    vertexFaces.emplace_back(std::make_pair(vs.get(), faces));*/
                } else if (t1 == t && t == t2) {
                    left.addEdge(edge);
                    right.addEdge(edge);
                }
                /*for(size_t i = 0; i < 2 * M; ++i){
                    if(faceVertices[i].size() > 0)
                        Warning("E %d (%f,%f,%f)", i, faceVertices[i].back()->x, faceVertices[i].back()->y, faceVertices[i].back()->z);

                }*/
            }

            /*for(size_t i = 0; i < 2 * M; ++i){
                Warning("%d %d", i, faceVertices[i].size());
                if(faceVertices[i].size() > 0)
                    Warning("E (%f,%f,%f)", faceVertices[i].back()->x, faceVertices[i].back()->y, faceVertices[i].back()->z);

            }*/
            Warning("Left %d", left.edges.size());
            Warning("Right %d", right.edges.size());

            std::vector<Point3f *> pairedVertex;
            for (size_t i = 0; i < 2 * M; ++i) {
                pairedVertex.emplace_back(nullptr);
            }

            /*
            int vertexId = 0;
            auto &p = vertexFaces[vertexId];
            int f = p.second[0];
            do{
                if(faceVertices[f].size() == 2){
                    Warning("Creating edge from (%f,%f,%f) to (%f,%f,%f)", faceVertices[f][0]->x, faceVertices[f][0]->y, faceVertices[f][0]->z, faceVertices[f][1]->x, faceVertices[f][1]->y, faceVertices[f][1]->z);
                    for(auto &p2: vertexFaces){
                        if(p2.first != p.first){
                            f = p2.second
                        }
                    }
                }
            } while(f != p.second[0]);

            for(auto &p : vertexFaces){
                for(int f: p.second){
                    if(pairedVertex[f]){

                    }
                    else{
                        pairedVertex[f] = p.first;
                    }
                }
            }*/

            for (size_t i = 0; i < 2 * M; ++i) {
                if (faceVertices[i].size() == 2) {
                    Warning("Creating edge for %d from (%f,%f,%f) to (%f,%f,%f)", i, faceVertices[i][0]->x,
                            faceVertices[i][0]->y, faceVertices[i][0]->z, faceVertices[i][1]->x, faceVertices[i][1]->y,
                            faceVertices[i][1]->z);
                    left.addEdge(KDOPEdge(faceVertices[i][0], faceVertices[i][1], i, 2 * directionId));
                    right.addEdge(KDOPEdge(faceVertices[i][0], faceVertices[i][1], i, 2 * directionId + 1));
                }
            }

            return std::make_pair(left, right);
        }

        std::vector<KDOPEdge> edges;
    };

    class RBSP : public Aggregate {
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
                       const std::vector<BoundsMf> &allPrimBounds, const std::vector<Vector3f> &directions,
                       uint32_t maxDepth);

        // KdTreeAccel Private Data
        const uint32_t isectCost, traversalCost, maxPrims;
        const Float emptyBonus;
        std::vector<std::shared_ptr<Primitive>> primitives;
        std::vector<uint32_t> primitiveIndices;
        RBSPNode *nodes;
        uint32_t nAllocedNodes, nextFreeNode;
        Bounds3f bounds;
    };

    struct RBSPBuildNode {
        RBSPBuildNode(uint32_t depth, uint32_t nPrimitives, uint32_t badRefines, BoundsMf nodeBounds, KDOPMesh kdopMesh,
                      uint32_t *primNums, uint32_t parentNum = -1)
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
