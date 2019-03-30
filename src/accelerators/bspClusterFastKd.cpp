//
// Created by jesse on 18.03.19.
//

#include "bspClusterFastKd.h"
#include <core/stats.h>
#include <core/progressreporter.h>
#include <bits/random.h>
#include "paramset.h"
#include <set>
#include <random>
#include "clustering.h"

namespace pbrt {

    BSPClusterFastKd::BSPClusterFastKd(std::vector<std::shared_ptr<pbrt::Primitive>> p, uint32_t isectCost,
                                       uint32_t traversalCost,
                                       Float emptyBonus, uint32_t maxPrims, uint32_t maxDepth, uint32_t nbDirections, uint32_t kdTravCost)
            : BSPNodeBasedFastKd(std::move(p), isectCost, traversalCost, emptyBonus, maxPrims, maxDepth, nbDirections,
                  kdTravCost) {
        std::chrono::high_resolution_clock::time_point start =
                std::chrono::high_resolution_clock::now();
        buildTree();
        std::chrono::high_resolution_clock::time_point end =
                std::chrono::high_resolution_clock::now();
        buildTime =
                std::chrono::duration_cast<std::chrono::nanoseconds>(end -
                                                                     start).count();
    }

    std::vector<Vector3f> BSPClusterFastKd::calculateDirections(std::mt19937 &gen, uint32_t K,
                                                                const std::vector<std::shared_ptr<pbrt::Primitive>> &primitives,
                                                                const uint32_t *primNums, uint32_t np) {
        return calculateClusterMeans(gen, K, primitives, primNums, np);
    }

    std::shared_ptr<BSPClusterFastKd> CreateBSPClusterFastKdTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        uint32_t isectCost = (uint32_t) ps.FindOneInt("intersectcost", 80);
        uint32_t travCost = (uint32_t) ps.FindOneInt("traversalcost", 5);
        uint32_t kdTravCost = (uint32_t) ps.FindOneInt("kdtraversalcost", 1);
        Float emptyBonus = ps.FindOneFloat("emptybonus", 0);
        uint32_t maxPrims = (uint32_t) ps.FindOneInt("maxprims", 1);
        uint32_t maxDepth = (uint32_t) ps.FindOneInt("maxdepth", -1);
        uint32_t nbDirections = (uint32_t) ps.FindOneInt("nbDirections", 3);

        return std::make_shared<BSPClusterFastKd>(std::move(prims), isectCost, travCost, emptyBonus,
                                                  maxPrims, maxDepth, nbDirections, kdTravCost);
    }
} // namespace pbrt