//
// Created by jesse on 11.11.18.
//

#ifndef PBRT_V3_GENERIC_RBSP_H
#define PBRT_V3_GENERIC_RBSP_H

#include <core/geometry.h>
#include <algorithm>
#include <core/primitive.h>
#include <fstream>
#include <core/stats.h>
#include "kDOPMesh.h"


namespace pbrt {
    STAT_COUNTER("Accelerator/Results/0 BSP-tree node traversals during intersect", nbNodeTraversals);
    STAT_COUNTER("Accelerator/Results/1 BSP-tree node traversals during intersectP", nbNodeTraversalsP);
    STAT_COUNTER("Accelerator/Results/2 BSP-tree nodes", nbNodes);
    STAT_COUNTER("Accelerator/Results/3 BSP-tree kd nodes", nbKdNodes);
    STAT_COUNTER("Accelerator/Results/4 BSP-tree bsp nodes", nbBSPNodes);
    STAT_COUNTER("Accelerator/Results/5 BSP-tree build: splitTests", statNbSplitTests);
    STAT_COUNTER_DOUBLE("Accelerator/Results/6 BSP-tree SA-cost", totalSACost);
    STAT_COUNTER("Accelerator/Results/7 BSP-tree Depth", statDepth);

    STAT_COUNTER("Accelerator/Params/0 BSP-tree param:maxdepth", statParamMaxDepth);
    STAT_COUNTER("Accelerator/Params/1 BSP-tree param:intersectioncost", statParamIntersectCost);
    STAT_COUNTER("Accelerator/Params/2 BSP-tree param:traversalcost", statParamTraversalCost);
    STAT_COUNTER_DOUBLE("Accelerator/Params/4 BSP-tree param:emptybonus", statParamEmptyBonus);
    STAT_COUNTER("Accelerator/Params/5 BSP-tree param:maxprims", statParamMaxPrims);
    STAT_COUNTER("Accelerator/Params/6 BSP-tree param:directions", statParamnbDirections);
    STAT_COUNTER("Accelerator/Params/7 BSP-tree param:axisSelectionType", statParamAxisSelectionType);
    STAT_COUNTER("Accelerator/Params/8 BSP-tree param:axisSelectionAmount", statParamAxisSelectionAmount);
    STAT_COUNTER_DOUBLE("Accelerator/Params/9 BSP-tree param:splitalpha", statParamSplitAlpha);
    STAT_COUNTER_DOUBLE("Accelerator/Params/10 BSP-tree param:alphatype", statParamAlphaType);

    STAT_COUNTER("Timings/Buildtime", buildTime);

    enum class NodeType {
        KD, BSP, LEAF
    };

    enum class EdgeType {
        Start, End
    };

    struct BoundEdge {
        // BoundEdge Public Methods
        BoundEdge() {}

        BoundEdge(Float t, uint32_t primNum, bool starting) : t(t), primNum(primNum) {
            type = starting ? EdgeType::Start : EdgeType::End;
        }

        Float t;
        uint32_t primNum;
        EdgeType type;
    };

    template<class nodeType>
    class GenericBSP : public Aggregate {
    public:

        // KdTreeAccel Public Methods
        GenericBSP(std::vector<std::shared_ptr<Primitive>> p,
                   uint32_t isectCost = 80, uint32_t traversalCost = 1,
                   Float emptyBonus = 0.5, uint32_t maxPrims = 1, uint32_t maxDepth = -1u, uint32_t nbDirections = 3,
                   Float splitAlpha = 90, uint32_t alphaType = 0, uint32_t axisSelectionType = 0,
                   uint32_t axisSelectionAmount = -1u) : primitives(std::move(p)), isectCost(isectCost),
                                                         traversalCost(traversalCost), emptyBonus(emptyBonus),
                                                         maxPrims(maxPrims), splitAlpha(splitAlpha),
                                                         alphaType(alphaType), axisSelectionType(axisSelectionType) {
            ProfilePhase _(Prof::AccelConstruction);
            nextFreeNode = nAllocedNodes = 0;
            if (maxDepth == -1)
                GenericBSP::maxDepth = calculateMaxDepth(primitives.size());
            else
                this->maxDepth = maxDepth;
            if (axisSelectionAmount == -1)
                GenericBSP::axisSelectionAmount = nbDirections;
            else
                GenericBSP::axisSelectionAmount = axisSelectionAmount;

            statParamMaxDepth = maxDepth;
            statParamEmptyBonus = emptyBonus;
            statParamIntersectCost = isectCost;
            statParamTraversalCost = traversalCost;
            statParamMaxPrims = maxPrims;
            statNbSplitTests = 0;
            statParamSplitAlpha = splitAlpha;
            statParamAlphaType = alphaType;
            statParamAxisSelectionType = axisSelectionType;
            statParamAxisSelectionAmount = axisSelectionAmount;

        }

        Bounds3f WorldBound() const override { return bounds; }

        ~GenericBSP() override { FreeAligned(nodes); };

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const override = 0;

        bool IntersectP(const Ray &ray) const override = 0;

        virtual void printNodes(std::ofstream &os) const = 0;

        friend std::ofstream &operator<<(std::ofstream &os, const GenericBSP<nodeType> &rbspTree) {
            os << rbspTree.directions.size() << std::endl;
            for (auto &direction: rbspTree.directions) {
                os << direction.x << " " << direction.y << " " << direction.z << std::endl;
            }
            os << rbspTree.nextFreeNode << std::endl;
            rbspTree.printNodes(os);
            os << rbspTree.primitives.size() << std::endl;
            for (auto &p: rbspTree.primitives) {
                Normal3f n = p->Normal();
                os << n.x << " " << n.y << " " << n.z << std::endl;
            }
            for (auto &p: rbspTree.primitives) {
                for (auto &direction: rbspTree.directions) {
                    auto b = p->getBounds(direction);
                    os << b.min << " " << b.max << " ";
                }
                os << std::endl;
            }
            for (auto &p: rbspTree.primitives) {
                os << p->getSurfaceArea() << std::endl;
            }
            return os;
        }

        void writeNodeTypeDepthMaps(const std::string &filename){
            int x = 0;

            std::map<uint32_t, uint32_t> *depthMaps[3] = {&leafNodeDepths, &bspNodeDepths, &kdNodeDepths};
            std::string names[3] = {"leafNodeDepths", "bspNodeDepths", "kdNodeDepths"};

            for(int i = 0; i < 3; i++) {
                const auto name = names[i];
                std::string textFile = filename.substr(0, filename.find_last_of('.')).append("-").append(name).append(
                        ".txt");
                // Warning("%s", textFile.c_str());
                std::ofstream myfile;
                myfile.open(textFile);

                for (auto &myPair: *depthMaps[i]) {
                    myfile << myPair.first << " " << myPair.second << std::endl;
                }

                myfile.close();
            }
        }


    protected:
        // RBSP Private Methods
        virtual void buildTree() = 0;

        void addNodeDepth(NodeType type, uint32_t depth){
            if(type == NodeType::LEAF){
                auto search = this->leafNodeDepths.find(depth);
                if(search != this->leafNodeDepths.end())
                    this->leafNodeDepths[depth] += 1;
                else
                    this->leafNodeDepths[depth] = 1;
            } else if(type == NodeType::BSP){
                auto search = this->bspNodeDepths.find(depth);
                if(search != this->bspNodeDepths.end())
                    this->bspNodeDepths[depth] += 1;
                else
                    this->bspNodeDepths[depth] = 1;
            } else if(type == NodeType::KD){
                auto search = this->kdNodeDepths.find(depth);
                if(search != this->kdNodeDepths.end())
                    this->kdNodeDepths[depth] += 1;
                else
                    this->kdNodeDepths[depth] = 1;
            }
        }

        // RBSPTreeAccel Private Data
        const uint32_t isectCost, traversalCost, maxPrims, alphaType, axisSelectionType;
        const Float emptyBonus, splitAlpha;
        std::vector<std::shared_ptr<Primitive>> primitives;
        std::vector<uint32_t> primitiveIndices;
        nodeType *nodes;
        uint32_t nAllocedNodes, nextFreeNode, maxDepth, axisSelectionAmount;
        Bounds3f bounds;
        std::vector<Vector3f> directions;
        std::map<uint32_t, uint32_t> kdNodeDepths;
        std::map<uint32_t, uint32_t> bspNodeDepths;
        std::map<uint32_t, uint32_t> leafNodeDepths;
    };

    template<class nodeType>
    struct BSPToDo {
        const nodeType *node;
        Float tMin, tMax;
    };

    struct BSPBuildNode {
        BSPBuildNode(uint32_t depth, uint32_t nPrimitives, uint32_t badRefines,
                     KDOPMeshWithDirections kdopMesh,
                     Float kdopMeshArea,
                     uint32_t *primNums, uint32_t parentNum = -1)
                : depth(depth),
                  nPrimitives(nPrimitives), badRefines(badRefines),
                  kDOPMesh(std::move(kdopMesh)), kdopMeshArea(kdopMeshArea), primNums(primNums), parentNum(parentNum) {}

        uint32_t depth;
        uint32_t nPrimitives;
        uint32_t badRefines;
        KDOPMeshWithDirections kDOPMesh;
        Float kdopMeshArea;
        uint32_t *primNums;
        uint32_t parentNum;
    };
}
#endif //PBRT_V3_GENERIC_RBSP_H
