//
// Created by jesse on 22.03.19.
//

#ifndef PBRT_V3_RBSPSHARED_H
#define PBRT_V3_RBSPSHARED_H

#endif //PBRT_V3_RBSPSHARED_H

namespace pbrt {
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

    inline std::vector<Vector3f> getDirections(uint32_t N) {
        std::vector<Vector3f> directions;
        if (N >= 100) { // Not used
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
        } else {
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