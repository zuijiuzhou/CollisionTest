#pragma once

#include "CollisionDetectorBase.h"

#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionDetector.hpp>

class RobWorkCollisionDetector : public CollisionDetectorBase {
public:
    RobWorkCollisionDetector();

public:
    virtual void addGeometry(const std::string& name, const aiScene* scene, const Eigen::Isometry3d& t) override;

    virtual bool collide() override;

    virtual bool withInDistance(float safety_dist) override;

private:
    rw::models::WorkCell::Ptr work_cell_;
    rw::proximity::CollisionDetector::Ptr detector_;
    rw::proximity::CollisionStrategy::Ptr strategy_;
};