//
// Created by jesse on 13.02.19.
//

#ifndef PBRT_V3_KDOPMESH_H
#define PBRT_V3_KDOPMESH_H

#include <core/geometry.h>
#include <algorithm>
#include <limits.h>

#define clz(x) __builtin_clz(x)

namespace pbrt {
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

    struct KDOPMeshBase {
        KDOPMeshBase() {};

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

        std::vector<KDOPEdge> edges;
    };

    inline void KDOPCutHelper(std::vector<Point3f> &points, Point3f point) {
        if (std::find(points.begin(), points.end(), point) == points.end()) { // use shared pointers
            points.push_back(point);
        }
    }

    inline void
    KDOPCutAddEdge(KDOPMeshBase &left, KDOPMeshBase &right, KDOPEdge edge, std::vector<KDOPEdge> &coincidentEdges,
                   std::vector<std::vector<Point3f>> &faceVertices, Float t, Float t1, Float t2) {
        Vector3f d = edge.v2 - edge.v1;
        if (t1 < t && t2 < t) {
            left.addEdge(edge);
        } else if (t1 > t && t2 > t) {
            right.addEdge(edge);
        } else if (t1 < t && t == t2) {
            left.addEdge(edge);
            KDOPCutHelper(faceVertices[edge.faceId1], edge.v2);
            KDOPCutHelper(faceVertices[edge.faceId2], edge.v2);
        } else if (t1 == t && t < t2) {
            right.addEdge(edge);
            KDOPCutHelper(faceVertices[edge.faceId1], edge.v1);
            KDOPCutHelper(faceVertices[edge.faceId2], edge.v1);
        } else if (t1 < t && t < t2) {
            Float tAlongEdge = (-(t1 - t)) / (t2 - t1);
            Point3f vs(edge.v1 + tAlongEdge * d);
            left.addEdge(KDOPEdge(edge.v1, vs, edge.faceId1, edge.faceId2));
            right.addEdge(KDOPEdge(vs, edge.v2, edge.faceId1, edge.faceId2));

            KDOPCutHelper(faceVertices[edge.faceId1], vs);
            KDOPCutHelper(faceVertices[edge.faceId2], vs);
        } else if (t1 == t && t == t2) {
            coincidentEdges.push_back(edge);
        } /*else{
                    Warning("Strange: %f %f %f", t1, t, t2);
                }*/
    }

    template<class T>
    inline std::pair<T, T>
    KDOPCut(const std::vector<KDOPEdge> &edges, uint32_t M, Float t, const Vector3f &direction,
            const uint32_t directionId) {
        //Warning("t %f", t);
        T left; // TODO: ? Allocate right ? Use unique pointers and shared pointers
        T right;
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
                KDOPCutAddEdge(left, right, KDOPEdge(edge.v2, edge.v1, edge.faceId1, edge.faceId2), coincidentEdges,
                               faceVertices, t, t2, t1);
            } else {
                KDOPCutAddEdge(left, right, edge, coincidentEdges, faceVertices, t, t1, t2);
            }

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

    inline Float KDOPSurfaceArea(std::vector<KDOPEdge> &edges, const std::vector<Vector3f> &directions) {
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
                        if (face[j]->v1 == currentEdge->v2 && !used[j]) {
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

    inline uint32_t log2_fast(const uint32_t x) {
        return sizeof(uint32_t) * CHAR_BIT - clz(x - 1);
    }

    inline const uint32_t getBitOffset(const uint32_t M) {
        return log2_fast(M + 1);
        //return (uint32_t) std::ceil(std::log2(M + 1));
    }

    inline uint32_t getBitMask(const uint32_t M) {
        return ((uint32_t) 1 << getBitOffset(M)) - 1;
    }

    struct KDOPMeshWithDirections : KDOPMeshBase {
        KDOPMeshWithDirections() : KDOPMeshBase() {};

        std::pair<KDOPMeshWithDirections, KDOPMeshWithDirections> cut(Float t, const Vector3f &direction) {
            auto directionId = (uint32_t) directions.size();
            //Warning("SIZE %d", directionId);
            for (uint32_t i = 0; i < directions.size(); i++) {
                auto d = directions[i];
                if (Dot(d, direction) > 0.999961923) {  // cos(0.5°)
                    directionId = i;
                    break;
                }
            }
            auto cut = KDOPCut<KDOPMeshWithDirections>(edges, directions.size(), t, direction, directionId);
            cut.first.directions = directions;
            cut.second.directions = directions;
            if (directionId == directions.size()) {
                cut.first.directions.emplace_back(direction);
                cut.second.directions.emplace_back(direction);
            }
            return cut;
        }

        Float SurfaceArea() {
            return KDOPSurfaceArea(edges, directions);
        }

        std::vector<Vector3f> directions;
    };

}
#endif //PBRT_V3_KDOPMESH_H
