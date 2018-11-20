//
// Created by jesse on 12.11.18.
//

#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "geometry.h"
#include "accelerators/rbsp.h"


using namespace pbrt;

TEST(kDOP, Area) {
    std::vector<Vector3f> directions;
    directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 0.0, 1.0));

    Bounds3f bounds = Bounds3f(Point3f(-5, 2, 0), Point3f(3.5, 3, 22));

    KDOPMesh kDOPMesh;
    Point3f v1 = Point3f(bounds.pMin);
    Point3f v2 = Point3f(bounds.pMin.x, bounds.pMin.y, bounds.pMax.z);
    Point3f v3 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMin.z);
    Point3f v4 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMin.z);
    Point3f v5 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMax.z);
    Point3f v6 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMax.z);
    Point3f v7 = Point3f(bounds.pMax.x, bounds.pMax.y, bounds.pMin.z);
    Point3f v8 = Point3f(bounds.pMax);

    kDOPMesh.addEdge(KDOPEdge(&v1, &v2, 1, 3));
    kDOPMesh.addEdge(KDOPEdge(&v1, &v3, 1, 5));
    kDOPMesh.addEdge(KDOPEdge(&v1, &v4, 3, 5));
    kDOPMesh.addEdge(KDOPEdge(&v2, &v5, 1, 4));
    kDOPMesh.addEdge(KDOPEdge(&v2, &v6, 3, 4));
    kDOPMesh.addEdge(KDOPEdge(&v3, &v5, 1, 2));
    kDOPMesh.addEdge(KDOPEdge(&v3, &v7, 2, 5));
    kDOPMesh.addEdge(KDOPEdge(&v4, &v6, 0, 3));
    kDOPMesh.addEdge(KDOPEdge(&v4, &v7, 0, 5));
    kDOPMesh.addEdge(KDOPEdge(&v5, &v8, 2, 4));
    kDOPMesh.addEdge(KDOPEdge(&v6, &v8, 0, 4));
    kDOPMesh.addEdge(KDOPEdge(&v7, &v8, 0, 2));

    EXPECT_FLOAT_EQ(2 * 8.5 * (1 + 22) + 2 * 1 * 22, kDOPMesh.SurfaceArea(directions));
}

TEST(kDOP, AreaFloat) {
    std::vector<Vector3f> directions;
    directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 0.0, 1.0));

    Bounds3f bounds = Bounds3f(Point3f(-5, 2, 0), Point3f(3.5, 3.3, 22));

    KDOPMesh kDOPMesh;
    Point3f v1 = Point3f(bounds.pMin);
    Point3f v2 = Point3f(bounds.pMin.x, bounds.pMin.y, bounds.pMax.z);
    Point3f v3 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMin.z);
    Point3f v4 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMin.z);
    Point3f v5 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMax.z);
    Point3f v6 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMax.z);
    Point3f v7 = Point3f(bounds.pMax.x, bounds.pMax.y, bounds.pMin.z);
    Point3f v8 = Point3f(bounds.pMax);

    kDOPMesh.addEdge(KDOPEdge(&v1, &v2, 1, 3));
    kDOPMesh.addEdge(KDOPEdge(&v1, &v3, 1, 5));
    kDOPMesh.addEdge(KDOPEdge(&v1, &v4, 3, 5));
    kDOPMesh.addEdge(KDOPEdge(&v2, &v5, 1, 4));
    kDOPMesh.addEdge(KDOPEdge(&v2, &v6, 3, 4));
    kDOPMesh.addEdge(KDOPEdge(&v3, &v5, 1, 2));
    kDOPMesh.addEdge(KDOPEdge(&v3, &v7, 2, 5));
    kDOPMesh.addEdge(KDOPEdge(&v4, &v6, 0, 3));
    kDOPMesh.addEdge(KDOPEdge(&v4, &v7, 0, 5));
    kDOPMesh.addEdge(KDOPEdge(&v5, &v8, 2, 4));
    kDOPMesh.addEdge(KDOPEdge(&v6, &v8, 0, 4));
    kDOPMesh.addEdge(KDOPEdge(&v7, &v8, 0, 2));

    EXPECT_FLOAT_EQ(kDOPMesh.SurfaceArea(directions), 2 * 8.5 * (1.3 + 22) + 2 * 1.3 * 22);
    EXPECT_FLOAT_EQ(kDOPMesh.SurfaceArea(directions), 2 * 8.5 * (1.3 + 22) + 2 * 1.3 * 22);
}

TEST(kDOP, Cut) {
    std::vector<Vector3f> directions;
    directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 0.0, 1.0));

    Bounds3f bounds = Bounds3f(Point3f(-5, 2, 0), Point3f(3.5, 3, 22));

    KDOPMesh kDOPMesh;
    Point3f v1 = Point3f(bounds.pMin);
    Point3f v2 = Point3f(bounds.pMin.x, bounds.pMin.y, bounds.pMax.z);
    Point3f v3 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMin.z);
    Point3f v4 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMin.z);
    Point3f v5 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMax.z);
    Point3f v6 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMax.z);
    Point3f v7 = Point3f(bounds.pMax.x, bounds.pMax.y, bounds.pMin.z);
    Point3f v8 = Point3f(bounds.pMax);

    kDOPMesh.addEdge(KDOPEdge(&v1, &v2, 1, 3));
    kDOPMesh.addEdge(KDOPEdge(&v1, &v3, 1, 5));
    kDOPMesh.addEdge(KDOPEdge(&v1, &v4, 3, 5));
    kDOPMesh.addEdge(KDOPEdge(&v2, &v5, 1, 4));
    kDOPMesh.addEdge(KDOPEdge(&v2, &v6, 3, 4));
    kDOPMesh.addEdge(KDOPEdge(&v3, &v5, 1, 2));
    kDOPMesh.addEdge(KDOPEdge(&v3, &v7, 2, 5));
    kDOPMesh.addEdge(KDOPEdge(&v4, &v6, 0, 3));
    kDOPMesh.addEdge(KDOPEdge(&v4, &v7, 0, 5));
    kDOPMesh.addEdge(KDOPEdge(&v5, &v8, 2, 4));
    kDOPMesh.addEdge(KDOPEdge(&v6, &v8, 0, 4));
    kDOPMesh.addEdge(KDOPEdge(&v7, &v8, 0, 2));

    std::pair<KDOPMesh, KDOPMesh> result = kDOPMesh.cut(directions.size(), 0, directions[0], 0);

    for (auto &edge: result.first.edges) {
        Warning("%d and %d: (%f,%f,%f) to (%f,%f,%f)", edge.faceId1, edge.faceId2, edge.v1->x, edge.v1->y, edge.v1->z,
                edge.v2->x, edge.v2->y, edge.v2->z);
    }

    EXPECT_FLOAT_EQ(2 * 5 * (1 + 22) + 2 * 1 * 22, result.first.SurfaceArea(directions));
    EXPECT_FLOAT_EQ(2 * 3.5 * (1 + 22) + 2 * 1 * 22, result.second.SurfaceArea(directions));

    std::pair<KDOPMesh, KDOPMesh> result2 = result.first.cut(directions.size(), 2.5,
                                                             directions[1], 1);
    EXPECT_EQ(12, result2.first.edges.size());
    EXPECT_EQ(12, result2.second.edges.size());
    EXPECT_FLOAT_EQ(2 * 5 * (0.5 + 22) + 2 * 0.5 * 22, result2.first.SurfaceArea(directions));
    EXPECT_FLOAT_EQ(2 * 5 * (0.5 + 22) + 2 * 0.5 * 22, result2.second.SurfaceArea(directions));
}

TEST(kDOP, CutDiag) {
    std::vector<Vector3f> directions;
    directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 0.0, 1.0));
    directions.emplace_back(
            Vector3f(22 / (float) std::sqrt(22 * 22 + 8.5 * 8.5), 0.0, -8.5 / (float) std::sqrt(22 * 22 + 8.5 * 8.5)));

    Bounds3f bounds = Bounds3f(Point3f(-5, 2, 0), Point3f(3.5, 3, 22));

    KDOPMesh kDOPMesh;
    Point3f v1 = Point3f(bounds.pMin);
    Point3f v2 = Point3f(bounds.pMin.x, bounds.pMin.y, bounds.pMax.z);
    Point3f v3 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMin.z);
    Point3f v4 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMin.z);
    Point3f v5 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMax.z);
    Point3f v6 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMax.z);
    Point3f v7 = Point3f(bounds.pMax.x, bounds.pMax.y, bounds.pMin.z);
    Point3f v8 = Point3f(bounds.pMax);

    kDOPMesh.addEdge(KDOPEdge(&v1, &v2, 1, 3));
    kDOPMesh.addEdge(KDOPEdge(&v1, &v3, 1, 5));
    kDOPMesh.addEdge(KDOPEdge(&v1, &v4, 3, 5));
    kDOPMesh.addEdge(KDOPEdge(&v2, &v5, 1, 4));
    kDOPMesh.addEdge(KDOPEdge(&v2, &v6, 3, 4));
    kDOPMesh.addEdge(KDOPEdge(&v3, &v5, 1, 2));
    kDOPMesh.addEdge(KDOPEdge(&v3, &v7, 2, 5));
    kDOPMesh.addEdge(KDOPEdge(&v4, &v6, 0, 3));
    kDOPMesh.addEdge(KDOPEdge(&v4, &v7, 0, 5));
    kDOPMesh.addEdge(KDOPEdge(&v5, &v8, 2, 4));
    kDOPMesh.addEdge(KDOPEdge(&v6, &v8, 0, 4));
    kDOPMesh.addEdge(KDOPEdge(&v7, &v8, 0, 2));

    std::pair<KDOPMesh, KDOPMesh> result = kDOPMesh.cut(directions.size(),
                                                        -110 / (float) std::sqrt(22 * 22 + 8.5 * 8.5),
                                                        directions[3], 3);
    const float eps = 0.001;
    for (auto &edge: result.first.edges) {
        Warning("%d and %d: (%f,%f,%f) to (%f,%f,%f)", edge.faceId1, edge.faceId2, edge.v1->x, edge.v1->y, edge.v1->z,
                edge.v2->x, edge.v2->y, edge.v2->z);
        if ((edge.faceId1 == 1 && edge.faceId2 == 4) || (edge.faceId2 == 1 && edge.faceId1 == 4)) {
            if (edge.v1->operator-(v2).Length() < eps) {
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v5).Length());
            } else {
                EXPECT_FLOAT_EQ(0, edge.v1->operator-(v5).Length());
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v2).Length());
            }
        } else if ((edge.faceId1 == 1 && edge.faceId2 == 6) || (edge.faceId2 == 1 && edge.faceId1 == 6)) {
            if (edge.v1->operator-(v1).Length() < eps) {
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v3).Length());
            } else {
                EXPECT_FLOAT_EQ(0, edge.v1->operator-(v3).Length());
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v1).Length());
            }
        } else if ((edge.faceId1 == 4 && edge.faceId2 == 6) || (edge.faceId2 == 4 && edge.faceId1 == 6)) {
            if (edge.v1->operator-(v6).Length() < eps) {
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v8).Length());
            } else {
                EXPECT_FLOAT_EQ(0, edge.v1->operator-(v8).Length());
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v6).Length());
            }
        } else if ((edge.faceId1 == 1 && edge.faceId2 == 3) || (edge.faceId2 == 1 && edge.faceId1 == 3)) {
            if (edge.v1->operator-(v1).Length() < eps) {
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v2).Length());
            } else {
                EXPECT_FLOAT_EQ(0, edge.v1->operator-(v2).Length());
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v1).Length());
            }
        } else if ((edge.faceId1 == 1 && edge.faceId2 == 2) || (edge.faceId2 == 1 && edge.faceId1 == 2)) {
            if (edge.v1->operator-(v3).Length() < eps) {
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v5).Length());
            } else {
                EXPECT_FLOAT_EQ(0, edge.v1->operator-(v5).Length());
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v3).Length());
            }
        } else if ((edge.faceId1 == 2 && edge.faceId2 == 4) || (edge.faceId2 == 2 && edge.faceId1 == 4)) {
            if (edge.v1->operator-(v5).Length() < eps) {
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v8).Length());
            } else {
                EXPECT_FLOAT_EQ(0, edge.v1->operator-(v8).Length());
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v5).Length());
            }
        } else if ((edge.faceId1 == 3 && edge.faceId2 == 4) || (edge.faceId2 == 3 && edge.faceId1 == 4)) {
            if (edge.v1->operator-(v2).Length() < eps) {
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v6).Length());
            } else {
                EXPECT_FLOAT_EQ(0, edge.v1->operator-(v6).Length());
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v2).Length());
            }
        } else if ((edge.faceId1 == 2 && edge.faceId2 == 6) || (edge.faceId2 == 2 && edge.faceId1 == 6)) {
            if (edge.v1->operator-(v3).Length() < eps) {
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v8).Length());
            } else {
                EXPECT_FLOAT_EQ(0, edge.v1->operator-(v8).Length());
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v3).Length());
            }
        } else if ((edge.faceId1 == 3 && edge.faceId2 == 6) || (edge.faceId2 == 3 && edge.faceId1 == 6)) {
            if (edge.v1->operator-(v1).Length() < eps) {
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v6).Length());
            } else {
                EXPECT_FLOAT_EQ(0, edge.v1->operator-(v6).Length());
                EXPECT_FLOAT_EQ(0, edge.v2->operator-(v1).Length());
            }
        } else {
            Warning("WRONG combo: %d and %d", edge.faceId1, edge.faceId2);
            EXPECT_FALSE(true);
        }


    }
    EXPECT_EQ(9, result.first.edges.size());
    EXPECT_EQ(9, result.second.edges.size());


    EXPECT_FLOAT_EQ(8.5 * 22 + 22 * 1 + 8.5 * 1 + 1 * (float) std::sqrt(22 * 22 + 8.5 * 8.5),
                    result.first.SurfaceArea(directions));
    EXPECT_FLOAT_EQ(8.5 * 22 + 22 * 1 + 8.5 * 1 + 1 * (float) std::sqrt(22 * 22 + 8.5 * 8.5),
                    result.second.SurfaceArea(directions));
}


TEST(kDOP, CutArb) {
    std::vector<Vector3f> directions;
    directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 0.0, 1.0));
    directions.emplace_back(Vector3f(1 / (float) std::sqrt(2), 1 / (float) std::sqrt(2), 0));
    directions.emplace_back(Vector3f(0, 1 / (float) std::sqrt(2), 1 / (float) std::sqrt(2)));

    Bounds3f bounds = Bounds3f(Point3f(-5, 2, 0), Point3f(3.5, 3, 22));

    KDOPMesh kDOPMesh;
    Point3f v1 = Point3f(bounds.pMin);
    Point3f v2 = Point3f(bounds.pMin.x, bounds.pMin.y, bounds.pMax.z);
    Point3f v3 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMin.z);
    Point3f v4 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMin.z);
    Point3f v5 = Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMax.z);
    Point3f v6 = Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMax.z);
    Point3f v7 = Point3f(bounds.pMax.x, bounds.pMax.y, bounds.pMin.z);
    Point3f v8 = Point3f(bounds.pMax);

    kDOPMesh.addEdge(KDOPEdge(&v1, &v2, 1, 3));
    kDOPMesh.addEdge(KDOPEdge(&v1, &v3, 1, 5));
    kDOPMesh.addEdge(KDOPEdge(&v1, &v4, 3, 5));
    kDOPMesh.addEdge(KDOPEdge(&v2, &v5, 1, 4));
    kDOPMesh.addEdge(KDOPEdge(&v2, &v6, 3, 4));
    kDOPMesh.addEdge(KDOPEdge(&v3, &v5, 1, 2));
    kDOPMesh.addEdge(KDOPEdge(&v3, &v7, 2, 5));
    kDOPMesh.addEdge(KDOPEdge(&v4, &v6, 0, 3));
    kDOPMesh.addEdge(KDOPEdge(&v4, &v7, 0, 5));
    kDOPMesh.addEdge(KDOPEdge(&v5, &v8, 2, 4));
    kDOPMesh.addEdge(KDOPEdge(&v6, &v8, 0, 4));
    kDOPMesh.addEdge(KDOPEdge(&v7, &v8, 0, 2));

    std::pair<KDOPMesh, KDOPMesh> result = kDOPMesh.cut(directions.size(), 2.8 * (float) std::sqrt(2),
                                                        directions[3], 3);
    for (auto &edge: result.first.edges) {
        Warning("FIRST: %d and %d: (%f,%f,%f) to (%f,%f,%f)", edge.faceId1, edge.faceId2, edge.v1->x, edge.v1->y,
                edge.v1->z,
                edge.v2->x, edge.v2->y, edge.v2->z);
    }

    for (auto &edge: result.second.edges) {
        Warning("SECOND: %d and %d: (%f,%f,%f) to (%f,%f,%f)", edge.faceId1, edge.faceId2, edge.v1->x, edge.v1->y,
                edge.v1->z,
                edge.v2->x, edge.v2->y, edge.v2->z);
    }
    EXPECT_EQ(15, result.first.edges.size());
    EXPECT_EQ(9, result.second.edges.size());

    EXPECT_FLOAT_EQ(2 * 7.6 * (22 + 1) + 22 + 0.9 * (22 + 2 * 0.1) + 0.1 * 22 + 0.9 * 0.9 + 0.9 * std::sqrt(2) * 22,
                    result.first.SurfaceArea(directions));
    EXPECT_FLOAT_EQ(2 * 0.9 * 22 + 0.9 * 0.9 + 0.9 * std::sqrt(2) * 22, result.second.SurfaceArea(directions));

    std::pair<KDOPMesh, KDOPMesh> result2 = result.first.cut(directions.size(), 2.8 * (float) std::sqrt(2),
                                                             directions[4], 4);
    EXPECT_EQ(15, result2.first.edges.size());
    EXPECT_EQ(15, result2.second.edges.size());
    Warning("FIRSTAREA: %f, SECONDAREA: %f, SUM: %f, ALMOST SUM %f", result2.first.SurfaceArea(directions),
            result2.second.SurfaceArea(directions),
            result2.first.SurfaceArea(directions) + result2.second.SurfaceArea(directions),
            2 * 8.41 / std::sqrt(2) + result.first.SurfaceArea(directions));
}