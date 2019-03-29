//
// Created by jesse on 28.03.19.
//

#include <set>
#include <algorithm>
#include "clustering.h"

#ifndef PBRT_V3_RANDOMNORMALS_H
#define PBRT_V3_RANDOMNORMALS_H

namespace pbrt{
    inline std::vector<Vector3f> chooseArbitraryNormals(std::mt19937 &gen, const uint32_t K,
                                                        const std::vector<std::shared_ptr<Primitive>> &primitives,
                                                        const uint32_t *primNums, const uint32_t np){
        std::set<uint32_t> nIds;
        std::vector<Vector3f> arbitraryNormals;
        while (nIds.size() < std::min(np,K))
            nIds.insert(random_int(gen, 0,np));
        arbitraryNormals.reserve(nIds.size());
        for (auto &id: nIds) {
            arbitraryNormals.emplace_back(PositiveX(primitives[primNums[id]]->Normal()));
        }

        return arbitraryNormals;
    }

    inline std::vector<Vector3f> chooseRandomDirections(std::mt19937 &gen, const uint32_t K){
        std::uniform_real_distribution<> disPhi(0, 2*Pi);
        std::uniform_real_distribution<> disCosTheta(-1, 1);

        std::vector<Vector3f> randomDirections;

        for(uint32_t i = 0; i < K; i++){
            Float phi = disPhi(gen);
            Float cosTheta = disCosTheta(gen);
            Float theta = std::acos(cosTheta);
            Float x = std::sin(theta) * std::cos(phi);
            Float y = std::sin(theta) * std::sin(phi);
            Float z = std::cos(theta);
            randomDirections.emplace_back(PositiveX(Vector3f(x,y,z)));
        }

        return randomDirections;

    }
}
#endif //PBRT_V3_RANDOMNORMALS_H
