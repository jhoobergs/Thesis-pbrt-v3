//
// Created by jesse on 29.03.19.
//

#include "bspRandomWithKd.h"
#include <core/stats.h>
#include <core/progressreporter.h>
#include <bits/random.h>
#include "paramset.h"
#include <set>
#include <random>
#include "randomNormals.h"


namespace pbrt {

    BSPRandomWithKd::BSPRandomWithKd(std::vector<std::shared_ptr<pbrt::Primitive>> p, uint32_t isectCost,
                                           uint32_t traversalCost,
                                           Float emptyBonus, uint32_t maxPrims, uint32_t maxDepth, uint32_t nbDirections)
            : BSPNodeBasedWithKd(std::move(p), isectCost, traversalCost, emptyBonus, maxPrims, maxDepth, nbDirections) {
        std::chrono::high_resolution_clock::time_point start =
                std::chrono::high_resolution_clock::now();
        buildTree();
        std::chrono::high_resolution_clock::time_point end =
                std::chrono::high_resolution_clock::now();
        buildTime =
                std::chrono::duration_cast<std::chrono::nanoseconds>(end -
                                                                     start).count();
    }

    std::vector<Vector3f> BSPRandomWithKd::calculateDirections(std::mt19937 &gen, uint32_t K,
                                                                  const std::vector<std::shared_ptr<pbrt::Primitive>> &primitives,
                                                                  const uint32_t *primNums, uint32_t np) {
        return chooseArbitraryNormals(gen, K, primitives, primNums, np);
    }

    std::shared_ptr<BSPRandomWithKd> CreateBSPRandomWithKdTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps) {
        uint32_t isectCost = (uint32_t) ps.FindOneInt("intersectcost", 80);
        uint32_t travCost = (uint32_t) ps.FindOneInt("traversalcost", 5);
        Float emptyBonus = ps.FindOneFloat("emptybonus", 0);
        uint32_t maxPrims = (uint32_t) ps.FindOneInt("maxprims", 1);
        uint32_t maxDepth = (uint32_t) ps.FindOneInt("maxdepth", -1);
        uint32_t nbDirections = (uint32_t) ps.FindOneInt("nbDirections", 3);

        return std::make_shared<BSPRandomWithKd>(std::move(prims), isectCost, travCost, emptyBonus,
                                                    maxPrims, maxDepth, nbDirections);
    }
} // namespace pbrt