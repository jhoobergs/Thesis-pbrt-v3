//
// Created by jesse on 11.11.18.
//

#ifndef PBRT_V3_RBSP_H
#define PBRT_V3_RBSP_H

#include <core/geometry.h>
#include <algorithm>
#include "genericBSP.h"
#include "kDOPMesh.h"

namespace pbrt {
    struct RBSPNode;

    struct KDOPMesh : KDOPMeshBase {
        KDOPMesh() : KDOPMeshBase() {};

        std::pair<KDOPMesh, KDOPMesh> cut(uint32_t M, Float t, const Vector3f &direction, const uint32_t directionId) {
            return KDOPCut<KDOPMesh>(edges, M, t, direction, directionId);
        }

        Float SurfaceArea(const std::vector<Vector3f> &directions) {
            return KDOPSurfaceArea(edges, directions);
        }
    };


    class RBSP : public GenericBSP<RBSPNode> {
    public:

        RBSP(std::vector<std::shared_ptr<Primitive>> p,
             uint32_t isectCost = 80, uint32_t traversalCost = 1,
             Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1, uint32_t nbDirections = 3,
             Float splitAlpha = 90, uint32_t alphaType = 0, uint32_t axisSelectionType = 0,
             uint32_t axisSelectionAmount = -1);

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const override;

        bool IntersectP(const Ray &ray) const override;

        void printNodes(std::ofstream &os) const override;

    protected:
        void buildTree() override;
    };

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
        BoundsMf nodeBounds; // TODO: remove
        KDOPMesh kDOPMesh;
        Float kdopMeshArea;
        uint32_t *primNums;
        uint32_t parentNum;
    };

    std::shared_ptr<RBSP> CreateRBSPTreeAccelerator(
            std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);

    inline std::vector<Vector3f> getDirections(uint32_t N) {
        std::vector<Vector3f> directions;
        if (N >= 100) {
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
#endif //PBRT_V3_RBSP_H
