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

    Bounds3f bounds = Bounds3f(Point3f(-5,2,0), Point3f(3.5, 3, 22));

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

    EXPECT_FLOAT_EQ(kDOPMesh.SurfaceArea(directions), 2* 8.5 * (1 + 22) + 2* 1 *22);
}