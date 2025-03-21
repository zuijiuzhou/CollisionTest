#pragma once

#include <map>

#include "CollisionDetectorBase.h"

namespace fcl {
template <typename S>
class CollisionGeometry;

using CollisionGeometryf = CollisionGeometry<float>;
}

class FclCollisionDetector : public CollisionDetectorBase {
public:
    FclCollisionDetector();
    virtual ~FclCollisionDetector();

public:
    virtual void addGeometry(const std::string& name, const aiScene* scene, const Eigen::Isometry3d& t) override;

    virtual bool collide() override;

    virtual bool withInDistance(float safety_dist = 10.f) override;

private:
    struct CollisionObjectUserData {
        std::string name;
    };

    void* cm_impl_ = nullptr;

    std::map<const fcl::CollisionGeometryf*, CollisionObjectUserData> obj_userdata_map_;
};