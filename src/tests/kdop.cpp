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

    EXPECT_FLOAT_EQ(kDOPMesh.SurfaceArea(directions), 2 * 8.5 * (1 + 22) + 2 * 1 * 22);
}

TEST(kDOP, Cut) {
    std::vector<Vector3f> directions;
    directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 0.0, 1.0));

    Bounds3f bounds = Bounds3f(Point3f(-5, 2, 0), Point3f(3.5, 3, 22));

    /*KDOPMesh kDOPMesh = *new KDOPMesh();
    Point3f *v1 = new Point3f(bounds.pMin);
    Point3f *v2 = new Point3f(bounds.pMin.x, bounds.pMin.y, bounds.pMax.z);
    Point3f *v3 = new Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMin.z);
    Point3f *v4 = new Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMin.z);
    Point3f *v5 = new Point3f(bounds.pMin.x, bounds.pMax.y, bounds.pMax.z);
    Point3f *v6 = new Point3f(bounds.pMax.x, bounds.pMin.y, bounds.pMax.z);
    Point3f *v7 = new Point3f(bounds.pMax.x, bounds.pMax.y, bounds.pMin.z);
    Point3f *v8 = new Point3f(bounds.pMax);

    kDOPMesh.addEdge(*new KDOPEdge(v1, v2, 1, 3));
    kDOPMesh.addEdge(*new KDOPEdge(v1, v3, 1, 5));
    kDOPMesh.addEdge(*new KDOPEdge(v1, v4, 3, 5));
    kDOPMesh.addEdge(*new KDOPEdge(v2, v5, 1, 4));
    kDOPMesh.addEdge(*new KDOPEdge(v2, v6, 3, 4));
    kDOPMesh.addEdge(*new KDOPEdge(v3, v5, 1, 2));
    kDOPMesh.addEdge(*new KDOPEdge(v3, v7, 2, 5));
    kDOPMesh.addEdge(*new KDOPEdge(v4, v6, 0, 3));
    kDOPMesh.addEdge(*new KDOPEdge(v4, v7, 0, 5));
    kDOPMesh.addEdge(*new KDOPEdge(v5, v8, 2, 4));
    kDOPMesh.addEdge(*new KDOPEdge(v6, v8, 0, 4));
    kDOPMesh.addEdge(*new KDOPEdge(v7, v8, 0, 2));*/

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

    EXPECT_FLOAT_EQ(result.first.SurfaceArea(directions), 2 * 5 * (1 + 22) + 2 * 1 * 22);
    EXPECT_FLOAT_EQ(result.second.SurfaceArea(directions), 2 * 3.5 * (1 + 22) + 2 * 1 * 22);
}

TEST(kDOP, CutDiag) {
    std::vector<Vector3f> directions;
    directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 0.0, 1.0));
    directions.emplace_back(Vector3f(22 / std::sqrt(22 * 22 + 8.5 * 8.5), 0.0, 8.5 / std::sqrt(22 * 22 + 8.5 * 8.5)));

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

    std::pair<KDOPMesh, KDOPMesh> result = kDOPMesh.cut(directions.size(), -110 / std::sqrt(22 * 22 + 8.5 * 8.5),
                                                        directions[3], 3);

    for (auto &edge: result.first.edges) {
        Warning("%d and %d: (%f,%f,%f) to (%f,%f,%f)", edge.faceId1, edge.faceId2, edge.v1->x, edge.v1->y, edge.v1->z,
                edge.v2->x, edge.v2->y, edge.v2->z);
    }

    EXPECT_FLOAT_EQ(result.first.SurfaceArea(directions),
                    8.5 * 22 + 22 * 1 + 8.5 * 1 + 1 * std::sqrt(22 * 22 + 8.5 * 8.5));
    EXPECT_FLOAT_EQ(result.second.SurfaceArea(directions),
                    8.5 * 22 + 22 * 1 + 8.5 * 1 + 1 * std::sqrt(22 * 22 + 8.5 * 8.5));
}