//
// Created by jesse on 29.11.18.
//

#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "geometry.h"


using namespace pbrt;

TEST(speed, Kd) {
    Ray ray = Ray(Point3f(-5,10,5), Normalize(Vector3f(1,2,3)));
    Vector3f invDir(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
    for(uint32_t axis = 0; axis < 3; axis++) {
        for(uint32_t i = 0; i < 1300000; i++){
            const Float tPlane = planeDistance(10, ray, invDir, axis);
        }
    }
}

TEST(speed, ArbitraryPlane) {
    std::vector<Vector3f> directions;
    directions.emplace_back(Vector3f(1.0, 0.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 1.0, 0.0));
    directions.emplace_back(Vector3f(0.0, 0.0, 1.0));
    directions.emplace_back(Normalize(Vector3f(1.0, 1.0, 1.0)));
    directions.emplace_back(Normalize(Vector3f(1.0, -1.0f, 1.0)));
    directions.emplace_back(Normalize(Vector3f(1.0, 1.0, -1.0f)));
    directions.emplace_back(Normalize(Vector3f(1.0, -1.0f, -1.0f)));
    directions.emplace_back(Normalize(Vector3f(1.0, 1.0, 0.0)));
    directions.emplace_back(Normalize(Vector3f(1.0, 0.0, 1.0)));
    directions.emplace_back(Normalize(Vector3f(0.0, 1.0, 1.0)));
    directions.emplace_back(Normalize(Vector3f(1.0, -1.0f, 0.0)));
    directions.emplace_back(Normalize(Vector3f(1.0, 0.0f, -1.0f)));
    directions.emplace_back(Normalize(Vector3f(0.0, 1.0, -1.0f)));


    Ray ray = Ray(Point3f(-5,10,5), Normalize(Vector3f(1,2,3)));
    Float projectedO, inverseProjectedD;
    for(int axis = 0; axis < directions.size(); axis++) {
        for(int i = 0; i < 300000; i++){
            const Float tPlane = planeDistance(directions[axis], 10, ray, projectedO, inverseProjectedD);
            //Warning("%f", tPlane);
        }
    }
}