//
// Created by jesse on 28.03.19.
//

#include <set>
#include <algorithm>
#include "clustering.h"

#ifndef PBRT_V3_RANDOMNORMALS_H
#define PBRT_V3_RANDOMNORMALS_H

namespace pbrt{
    inline std::vector<Vector3f> createRandomNormals(std::mt19937 &gen, const uint32_t K, const std::vector<std::shared_ptr<Primitive>> &primitives, const uint32_t *primNums, const uint32_t np){
        std::set<uint32_t> nIds;
        std::vector<Vector3f> randomNormals;
        while (nIds.size() < std::min(np,K))
            nIds.insert(random_int(gen, 0,np));
        randomNormals.reserve(nIds.size());
        for (auto &id: nIds) {
            randomNormals.emplace_back(PositiveX(primitives[primNums[id]]->Normal()));
        }

        return randomNormals;
    }
}
#endif //PBRT_V3_RANDOMNORMALS_H
