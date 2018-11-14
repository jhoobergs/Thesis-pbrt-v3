//
// Created by jesse on 11.11.18.
//

#ifndef PBRT_V3_RBSP_H
#define PBRT_V3_RBSP_H

#include <core/geometry.h>
#include "kdtreeaccel.h"
#include <algorithm>

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

        void addEdgeIfNeeded(KDOPEdge edge) {
            for (auto &e: edges) {
                if ((e.v1 == edge.v2 && e.v2 == edge.v1) || (e.v1 == edge.v1 && e.v2 == edge.v2)) {
                    return;
                }
            }
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

        void helper(std::vector<Point3f *> *points, Point3f *point) {
            if (std::find(points->begin(), points->end(), point) == points->end()) {
                points->push_back(point);
            }
        }

        std::pair<KDOPMesh, KDOPMesh> cut(uint32_t M, float t, const Vector3f &direction, const uint32_t directionId) {
            //Warning("t %f", t);
            KDOPMesh left;
            KDOPMesh right;
            std::vector<std::vector<Point3f *>> faceVertices;
            for (size_t i = 0; i < 2 * M; ++i) {
                faceVertices.emplace_back(std::vector<Point3f *>());
            }
            std::vector<KDOPEdge> coincidentEdges;

            for (auto &edge: edges) {
                float t1 = direction.dot(*edge.v1);
                float t2 = direction.dot(*edge.v2);
                if (t1 > t2) {
                    std::swap(edge.v1, edge.v2);
                    std::swap(t1, t2);
                }
                Vector3f d = edge.v2->operator-(*edge.v1);
                if (t1 < t && t2 < t) {
                    left.addEdge(edge);
                } else if (t1 > t && t2 > t) {
                    right.addEdge(edge);
                } else if (t1 < t && t == t2) {
                    left.addEdge(edge);
                    helper(&faceVertices[edge.faceId1], edge.v2);
                    helper(&faceVertices[edge.faceId2], edge.v2);
                } else if (t1 == t && t < t2) {
                    right.addEdge(edge);
                    helper(&faceVertices[edge.faceId1], edge.v1);
                    helper(&faceVertices[edge.faceId2], edge.v1);
                } else if (t1 < t && t < t2) {
                    float tAlongEdge = (-(t1 - t)) / (t2 - t1);
                    Point3f *vs = new Point3f(*edge.v1 + tAlongEdge * d);
                    left.addEdge(KDOPEdge(edge.v1, vs, edge.faceId1, edge.faceId2));
                    right.addEdge(KDOPEdge(vs, edge.v2, edge.faceId1, edge.faceId2));

                    helper(&faceVertices[edge.faceId1], vs);
                    helper(&faceVertices[edge.faceId2], vs);
                } else if (t1 == t && t == t2) {
                    coincidentEdges.push_back(edge);
                } /*else{
                    Warning("Strange: %f %f %f", t1, t, t2);
                }*/
            }

            //Warning("Left %d", left.edges.size());
            //Warning("Right %d", right.edges.size());

            //Warning("Coincident size: %d", coincidentEdges.size());
            for (auto &edge: coincidentEdges) {
                bool found = false;
                for (auto &leftEdge: left.edges) {
                    if (leftEdge.faceId1 == edge.faceId1 || leftEdge.faceId2 == edge.faceId1) {
                        left.addEdge(KDOPEdge(edge.v1, edge.v2, edge.faceId1, 2 * directionId));
                        right.addEdge(KDOPEdge(edge.v1, edge.v2, edge.faceId2, 2 * directionId + 1));
                        found = true;
                        break;
                    } else if (leftEdge.faceId1 == edge.faceId2 || leftEdge.faceId2 == edge.faceId2) {
                        left.addEdge(KDOPEdge(edge.v1, edge.v2, edge.faceId2, 2 * directionId));
                        right.addEdge(KDOPEdge(edge.v1, edge.v2, edge.faceId1, 2 * directionId + 1));
                        found = true;
                        break;
                    }
                }
                /*if(!found)
                    Warning("Can't add edge from (%f,%f,%f) to (%f,%f,%f) with faces %d and %d",
                        edge.v1->x, edge.v1->y, edge.v1->z, edge.v2->x, edge.v2->y, edge.v2->z,
                        edge.faceId1, edge.faceId2);
                else{
                    Warning("Added edge from (%f,%f,%f) to (%f,%f,%f)",
                            edge.v1->x, edge.v1->y, edge.v1->z, edge.v2->x, edge.v2->y, edge.v2->z);
                }*/
            }

            /*for(size_t i = 0; i < 2 * M; ++i){
                Warning("%d %d", i, faceVertices[i].size());
                for(uint32_t j = 0; j < faceVertices[i].size(); ++j) {
                    Warning("E %d (%f,%f,%f)", j, faceVertices[i][j]->x, faceVertices[i][j]->y,
                            faceVertices[i][j]->z);
                }

            }*/
            //Warning("Left %d", left.edges.size());
            //Warning("Right %d", right.edges.size());

            std::vector<Point3f *> pairedVertex;
            for (size_t i = 0; i < 2 * M; ++i) {
                pairedVertex.emplace_back(nullptr);
            }

            for (size_t i = 0; i < 2 * M; ++i) {
                if (faceVertices[i].size() == 2) {
                    /* Warning("Creating edge for %d from (%f,%f,%f) to (%f,%f,%f), left %d, right %d", i, faceVertices[i][0]->x,
                            faceVertices[i][0]->y, faceVertices[i][0]->z, faceVertices[i][1]->x, faceVertices[i][1]->y,
                            faceVertices[i][1]->z, 2 * directionId, 2* directionId +1); */
                    left.addEdgeIfNeeded(KDOPEdge(faceVertices[i][0], faceVertices[i][1], i, 2 * directionId));
                    right.addEdgeIfNeeded(KDOPEdge(faceVertices[i][0], faceVertices[i][1], i, 2 * directionId + 1));
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
                       const std::vector<BoundsMf> &allPrimBounds,
                       uint32_t maxDepth);

        // KdTreeAccel Private Data
        const uint32_t isectCost, traversalCost, maxPrims;
        const Float emptyBonus;
        std::vector<std::shared_ptr<Primitive>> primitives;
        std::vector<uint32_t> primitiveIndices;
        RBSPNode *nodes;
        uint32_t nAllocedNodes, nextFreeNode;
        Bounds3f bounds;
        std::vector<Vector3f> directions;
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
