//
// Created by jesse on 18.03.19.
//

#ifndef PBRT_V3_CLUSTERING_H
#define PBRT_V3_CLUSTERING_H

namespace pbrt {
    inline uint32_t calculateIdOfClosestMean(Vector3f &normal, const std::vector<Vector3f> &means) {
        //Warning("calculateIdOfClosestMean");
        uint32_t closest = 0;
        Float closestAngle = Angle(normal, means[0]);
        for (uint32_t i = 1; i < means.size(); ++i) {
            Float currentAngle = Angle(normal, means[i]);
            if (currentAngle < closestAngle) {
                closest = i;
                closestAngle = currentAngle;
            }
        }
        //Warning("returning calculateIdOfClosestMean");
        return closest;
    }

    inline Vector3f calculateMeanVector(const std::vector<Vector3f> &vectors) {
        //Warning("calculateMeanVector");
        if (vectors.empty())
            return Vector3f();

        auto sumVector = Vector3f();
        for (auto &vector: vectors) {
            sumVector += vector;
        }
        //Warning("returning calculateMeanVector");
        return Normalize(sumVector);
    }

    inline Float calculateMaxDifference(const std::vector<Vector3f> &oldMeans, const std::vector<Vector3f> &newMeans) {
        //Warning("calculateMaxDifference");
        Float maxDiff = 0;
        for (uint32_t i = 0; i < oldMeans.size(); ++i) {
            Vector3f diff = oldMeans[i] - newMeans[i];
            maxDiff = std::max(maxDiff, Dot(diff, diff));
        }
        // Warning("returning calculateMaxDifference");
        return maxDiff;
    }

    inline uint32_t random_int(std::mt19937 &gen, uint32_t from, uint32_t to){
        std::uniform_real_distribution<> dis(from, to);
        return uint32_t(dis(gen));
    }

    inline std::vector<Vector3f> calculateClusterMeans(std::mt19937 &gen, const uint32_t K, const std::vector<std::shared_ptr<Primitive>> &primitives, const uint32_t *primNums, const uint32_t np) {

        //Warning("calculateClusterMeans");

        std::vector<Vector3f> clusterMeans, newClusterMeans;
        std::vector<std::vector<Vector3f>> clusters;

        std::vector<Vector3f> normals;
        normals.reserve(np);
        for (int i = 0; i < np; ++i)
            normals.emplace_back(PositiveX(primitives[primNums[i]]->Normal()));


        if (np <= K) {
            return normals;
        }

        std::set<uint32_t> nIds;
        while (nIds.size() < K)
            nIds.insert(random_int(gen, 0,np));

        for (auto &id: nIds) {
            clusterMeans.emplace_back(normals[id]);
            clusters.emplace_back(std::vector<Vector3f>());
        }
        newClusterMeans = clusterMeans;
        const uint32_t maxIterations = 500;
        uint32_t iterations = 0;
        while (iterations < maxIterations &&
               (iterations == 0 or calculateMaxDifference(clusterMeans, newClusterMeans) > 0.000001)) {

            ++iterations;
            clusterMeans = newClusterMeans;

            for (auto &n: normals)
                clusters[calculateIdOfClosestMean(n, clusterMeans)].emplace_back(n);

            for (int i = 0; i < K; ++i) {
                if (clusters[i].empty()) {
                    std::set<uint32_t> nIds;
                    while (nIds.size() < K)
                        nIds.insert(random_int(gen, 0,np));
                    std::vector<uint32_t> v(nIds.begin(), nIds.end());
                    for (int ii = 0; ii < K; ++ii) {
                        auto id = v[ii];
                        newClusterMeans[ii] = normals[id];
                        clusters[ii].clear();
                    }
                    break;
                }
                newClusterMeans[i] = calculateMeanVector(clusters[i]);
                clusters[i].clear();
                //Warning("Cleared");
            }
        }
        if(iterations == 500)
            Warning("iterations: %d", iterations);

        return newClusterMeans;
    }
}
#endif //PBRT_V3_CLUSTERING_H
