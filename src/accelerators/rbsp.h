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

    struct KDOPEdge { // TODO: Deallocating ? Smart pointers ?
        KDOPEdge(Point3f v1, Point3f v2, uint32_t faceId1, uint32_t faceId2) :
                v1(std::move(v1)), v2(std::move(v2)), faceId1(faceId1), faceId2(faceId2) {};

        Boundsf getBounds(Vector3f &direction) const {
            Boundsf bounds = Boundsf();
            const Float t1 = Dot(direction, v1);
            const Float t2 = Dot(direction, v2);
            bounds.max = std::max(t1, t2);
            bounds.min = std::min(t1, t2);

            return bounds;
        }

        Point3f v1;
        Point3f v2;
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

        Float SurfaceArea(const std::vector<Vector3f> &directions) {
            std::vector<std::vector<KDOPEdge *>> faces;
            for (size_t i = 0; i < 2 * directions.size(); ++i) {
                faces.emplace_back(std::vector<KDOPEdge *>());
            }

            for (auto &edge: edges) {
                faces[edge.faceId1].emplace_back(&edge);
                faces[edge.faceId2].emplace_back(&edge);
            }

            KDOPEdge *currentEdge;
            Float SA = 0;
            uint32_t edgeId;
            for (uint32_t i = 0; i < 2 * directions.size(); ++i) {
                Vector3f FSA = Vector3f();
                const std::vector<KDOPEdge *> &face = faces[i];
                if (!face.empty()) {
                    std::vector<bool> used;
                    used.reserve(face.size());
                    for (auto &edge: face)
                        used.emplace_back(false);
                    edgeId = 0;
                    // uint32_t previousEdgeId = 1;
                    do {
                        //if(edgeId == previousEdgeId){
                        if (used[edgeId]) { // TODO; don't set to zero probably ?
                            //Warning("Breaking from invalid polygon %f %f %f", FSA.x, FSA.y, FSA.z);
                            break;
                        }
                        used[edgeId] = true;
                        // previousEdgeId = edgeId;
                        currentEdge = face[edgeId];
                        FSA += Cross(currentEdge->v1, currentEdge->v2);
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
                SA += std::abs(Dot(directions[i / 2], FSA));
            }
            return SA / 2.0f;
        }

        /*
        Float SurfaceArea(const std::vector<Vector3f> &directions) {
            std::vector<std::vector<KDOPEdge>> faces;
            for (size_t i = 0; i < 2 * directions.size(); ++i) {
                faces.emplace_back(std::vector<KDOPEdge>());
            }

            for (auto &edge: edges) {
                faces[edge.faceId1].emplace_back(edge);
                faces[edge.faceId2].emplace_back(edge);
            }

            Float SA = 0;
            for (uint32_t d = 0; d < 2 * directions.size(); ++d) {
                const Vector3f &N = directions[d / 2];
                std::vector<KDOPEdge> &face = faces[d];
                if (!face.empty()) {
                    // std::unique_ptr<const Point3f*[]> vertices_p = std::unique_ptr<const Point3f*[]>(new const Point3f*[face.size() + 1]);
                    // auto vertices = vertices_p.get();
                    //Point3f** vertices = new Point3f*[face.size() + 1];
                    Point3f *vertices[face.size() + 1];
                    uint32_t vertexId = 0;
                    KDOPEdge firstEdge = face[0];
                    vertices[vertexId++] = firstEdge.v1;
                    vertices[vertexId++] = firstEdge.v2;

                    while (vertexId < face.size() + 1) {
                        for (uint32_t j = 1; j < face.size(); ++j) {
                            if (face[j].v2 == vertices[vertexId - 1]) {
                                std::swap(face[j].v1, face[j].v2);
                            }
                            if (face[j].v1 == vertices[vertexId - 1] && face[j].v2 != vertices[vertexId - 2]) {
                                vertices[vertexId++] = face[j].v2;
                                break;
                            }
                        }
                    }
                    vertices[vertexId] = vertices[0];

                    // THIS PART IS ADAPTED FROM area3D_Polygon at http://geomalgorithms.com/a01-_area.html
                    Float area = 0;
                    Float an, ax, ay, az; // abs value of normal and its coords
                    int coord;           // coord to ignore: 1=x, 2=y, 3=z
                    int i, j, k;         // loop indices

                    if (vertexId > 3) {
                        // select largest abs coordinate to ignore for projection
                        ax = std::abs(N.x);    // abs x-coord
                        ay = std::abs(N.y);    // abs y-coord
                        az = std::abs(N.z);    // abs z-coord

                        coord = 3;                    // ignore z-coord
                        if (ax > ay) {
                            if (ax > az) coord = 1;   // ignore x-coord
                        } else if (ay > az) coord = 2;  // ignore y-coord

                        an = (Float) std::sqrt(ax * ax + ay * ay + az * az); // length of normal vector
                        switch (coord) {
                            case 1:
                                // compute area of the 2D projection
                                for (i = 1, j = 2, k = 0; i < vertexId; i++, j++, k++)
                                    area += (vertices[i]->y * (vertices[j]->z - vertices[k]->z));
                                // wrap-around term
                                area += (vertices[vertexId]->y * (vertices[1]->z - vertices[vertexId - 1]->z));
                                // scale to get area before projection
                                area *= (an / (2 * N.x));
                                break;
                            case 2:
                                for (i = 1, j = 2, k = 0; i < vertexId; i++, j++, k++)
                                    area += (vertices[i]->z * (vertices[j]->x - vertices[k]->x));
                                area += (vertices[vertexId]->z * (vertices[1]->x - vertices[vertexId - 1]->x));
                                area *= (an / (2 * N.y));
                                break;
                            case 3:
                                for (i = 1, j = 2, k = 0; i < vertexId; i++, j++, k++)
                                    area += (vertices[i]->x * (vertices[j]->y - vertices[k]->y));
                                area += (vertices[vertexId]->x * (vertices[1]->y - vertices[vertexId - 1]->y));
                                area *= (an / (2 * N.z));
                                break;
                        }
                    }
                    SA += std::abs(area);
                }

            }
            return SA;
        } */

        void helper(std::vector<Point3f> &points, Point3f point) {
            if (std::find(points.begin(), points.end(), point) == points.end()) { // use shared pointers
                points.push_back(point);
            }
        }

        std::pair<KDOPMesh, KDOPMesh> cut(uint32_t M, Float t, const Vector3f &direction, const uint32_t directionId) {
            //Warning("t %f", t);
            KDOPMesh left; // TODO: ? Allocate right ? Use unique pointers and shared pointers
            KDOPMesh right;
            std::vector<std::vector<Point3f>> faceVertices;
            for (size_t i = 0; i < 2 * M; ++i) {
                faceVertices.emplace_back(std::vector<Point3f>());
            }
            std::vector<KDOPEdge> coincidentEdges;
            Float t1, t2;
            for (auto &edge: edges) {
                t1 = Dot(direction, edge.v1);
                t2 = Dot(direction, edge.v2);
                /*if(std::abs(t1-t) < 0.001 && std::abs(t1/t - 1) < 0.01)
                    t1=t;
                if(std::abs(t2-t) < 0.001 && std::abs(t2/t - 1) < 0.01)
                    t2=t;*/
                if (t1 > t2) {
                    std::swap(edge.v1, edge.v2);
                    std::swap(t1, t2);
                }
                Vector3f d = edge.v2 - edge.v1;
                if (t1 < t && t2 < t) {
                    left.addEdge(edge);
                } else if (t1 > t && t2 > t) {
                    right.addEdge(edge);
                } else if (t1 < t && t == t2) {
                    left.addEdge(edge);
                    helper(faceVertices[edge.faceId1], edge.v2);
                    helper(faceVertices[edge.faceId2], edge.v2);
                } else if (t1 == t && t < t2) {
                    right.addEdge(edge);
                    helper(faceVertices[edge.faceId1], edge.v1);
                    helper(faceVertices[edge.faceId2], edge.v1);
                } else if (t1 < t && t < t2) {
                    Float tAlongEdge = (-(t1 - t)) / (t2 - t1);
                    Point3f vs(edge.v1 + tAlongEdge * d);
                    left.addEdge(KDOPEdge(edge.v1, vs, edge.faceId1, edge.faceId2));
                    right.addEdge(KDOPEdge(vs, edge.v2, edge.faceId1, edge.faceId2));

                    helper(faceVertices[edge.faceId1], vs);
                    helper(faceVertices[edge.faceId2], vs);
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
                /*if (!found)
                    Warning("Can't add edge from (%f,%f,%f) to (%f,%f,%f) with faces %d and %d",
                            edge.v1.x, edge.v1.y, edge.v1.z, edge.v2.x, edge.v2.y, edge.v2.z,
                            edge.faceId1, edge.faceId2);*/
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

            /*std::vector<Point3f *> pairedVertex;
            for (size_t i = 0; i < 2 * M; ++i) {
                pairedVertex.emplace_back(nullptr);
            }*/

            for (uint32_t i = 0; i < 2 * M; ++i) {
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
             Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3,
             Float splitAlpha = 90, uint32_t alphaType = 0, uint32_t axisSelectionType = 0,
             uint32_t axisSelectionAmount = -1);

        Bounds3f WorldBound() const { return bounds; }

        ~RBSP();

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;

        bool IntersectP(const Ray &ray) const;

        friend std::ofstream &operator<<(std::ofstream &os, const RBSP &rbspTree);

    private:
        // RBSP Private Methods
        void buildTree(BoundsMf &rootNodeMBounds, KDOPMesh &kDOPMesh,
                       const std::vector<BoundsMf> &allPrimBounds,
                       uint32_t maxDepth, Float splitAlpha, uint32_t alphaType, uint32_t axisSelectionType,
                       uint32_t axisSelectionAmount);

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
                      Float kdopMeshArea,
                      uint32_t *primNums, uint32_t parentNum = -1)
                : depth(depth),
                  nPrimitives(nPrimitives), badRefines(badRefines), nodeBounds(std::move(nodeBounds)),
                  kDOPMesh(std::move(kdopMesh)), kdopMeshArea(kdopMeshArea), primNums(primNums), parentNum(parentNum) {}

        uint32_t depth;
        uint32_t nPrimitives;
        uint32_t badRefines;
        BoundsMf nodeBounds;
        KDOPMesh kDOPMesh;
        Float kdopMeshArea;
        uint32_t *primNums;
        uint32_t parentNum;
    };

    struct RBSPToDo {
        const RBSPNode *node;
        Float tMin, tMax;
    };

    std::shared_ptr<RBSP> CreateRBSPTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);

    inline std::vector<Vector3f> getDirections(uint32_t N) {
        std::vector<Vector3f> directions;
        if (N >= 100) {
            N -= 100;
            const auto a = (Float) (2 * M_PI / N);
            const auto d = (Float) std::sqrt(a);
            const auto Mpsi = (Float) std::round(M_PI / (2 * d));
            const auto dpsi = (Float) (M_PI / (2 * Mpsi));
            const Float dphi = a / dpsi;
            for (uint32_t m = 0; m < Mpsi; ++m) {
                const auto psi = (Float) (M_PI * (m + 0.5) / (2 * Mpsi));
                const auto Mphi = (Float) std::round(2 * M_PI * std::sin(psi) / dphi);
                for (uint32_t n = 0; n < Mphi; ++n) {
                    const auto phi = (Float) (2 * M_PI * n / Mphi);
                    directions.emplace_back(
                            std::cos(psi), std::sin(psi) * std::sin(phi), std::sin(psi) * std::cos(phi));
                }

            }
            CHECK_EQ(N, directions.size());
        }
        else {
            directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
            directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
            directions.emplace_back(Vector3f(0.0, 0.0, 1.0));

            if (N == 7 || N == 13) {
                directions.emplace_back(Normalize(Vector3f(1.0, 1.0, 1.0)));
                directions.emplace_back(Normalize(Vector3f(1.0, -1.0f, 1.0)));
                directions.emplace_back(Normalize(Vector3f(1.0, 1.0, -1.0f)));
                directions.emplace_back(Normalize(Vector3f(1.0, -1.0f, -1.0f)));
            }

            if (N == 9 || N == 13) {
                directions.emplace_back(Normalize(Vector3f(1.0, 1.0, 0.0)));
                directions.emplace_back(Normalize(Vector3f(1.0, 0.0, 1.0)));
                directions.emplace_back(Normalize(Vector3f(0.0, 1.0, 1.0)));
                directions.emplace_back(Normalize(Vector3f(1.0, -1.0f, 0.0)));
                directions.emplace_back(Normalize(Vector3f(1.0, 0.0f, -1.0f)));
                directions.emplace_back(Normalize(Vector3f(0.0, 1.0, -1.0f)));
            }
            CHECK_EQ(N, directions.size());
        }


        return directions;
    }

}
#endif //PBRT_V3_RBSP_H
