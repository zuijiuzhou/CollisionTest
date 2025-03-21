#pragma once

#include <optional>

#include <fcl/narrowphase/collision_object.h>

template <typename S>
class CollisionMatrixFcl {
    std::optional<double> query(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2) const { return 10; };
};

using CollisionMatrixfFcl = CollisionMatrixFcl<float>;
using CollisionMatrixdFcl = CollisionMatrixFcl<double>;
