#pragma once

#include <string>

#include <Eigen/Geometry>

struct aiScene;

class CollisionDetectorBase {

public:
    virtual void addGeometry(const std::string& name, const aiScene* scene, const Eigen::Isometry3d& t) = 0;

    virtual bool collide() = 0;

    virtual bool withInDistance(float safety_dist) = 0;
};