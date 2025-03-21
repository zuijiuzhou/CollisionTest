#include "collision_manager.h"

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>

/** @brief hyperbrain project */
namespace hyperbrain {
/** @brief robots */
namespace robot {

template <typename S>
bool DefaultCollisionFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, CollisionData<S>& cdata) {
    if (cdata.done) {
        return true;
    }

    auto& request = cdata.request;
    CollisionResult<S> result;

    fcl::collide(o1, o2, request, result);

    if (result.isCollision()) {
        result.o1 = o1;
        result.o2 = o2;
        cdata.results.push_back(result);
        ++cdata.nb_narrow_phase;
        if (request.stop_at_first_contact)
            return true;
    }

    return false;
}
template <typename S>
bool DefaultWithInDistanceFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, WithInDistanceData<S>& cdata) {
    auto& request = cdata.request;
    fcl::DistanceResult<S> result;
    // if (dist == S(0)) {
    //   fcl::collide(o1, o2, request, result);
    // }
    // else
    {
        fcl::distance(o1, o2, request, result);
    }

    if (result.min_distance <= request.safety_dist) {
        // dist = result.min_distance;
        cdata.results.push_back(result);
        ++cdata.nb_narrow_phase;

        if (request.stop_at_first_contact)
            return true;
    }

    return false;
}
template <typename S>
void CollisionManager<S>::registerObjects(const std::vector<fcl::CollisionObject<S>*>& other_objs) {
    for (size_t i = 0; i < other_objs.size(); ++i)
        registerObject(other_objs[i]);
}
template <typename S>
bool CollisionManager<S>::isExcludedBetween(fcl::CollisionObject<S>* obj1, fcl::CollisionObject<S>* obj2) const {
    return isExcludedBetween(collision_matrix_.get(), obj1, obj2);
}
template <typename S>
bool CollisionManager<S>::isExcludedBetween(CollisionMatrix* cm, fcl::CollisionObject<S>* obj1, fcl::CollisionObject<S>* obj2) const {
    if (!cm)
        return false;
    return false;
}
template <typename S>
S CollisionManager<S>::getSafetyDistanceBetween(fcl::CollisionObject<S>* obj1, fcl::CollisionObject<S>* obj2) const {
    return S(0);
}
template <typename S>
S CollisionManager<S>::getSafetyDistanceBetween(CollisionMatrix* cm, fcl::CollisionObject<S>* obj1, fcl::CollisionObject<S>* obj2) const {
    return S(0);
}
template <typename S>
void CollisionManager<S>::setCollisionMatrix(std::shared_ptr<CollisionMatrix> collision_matrix) {
    collision_matrix_ = collision_matrix;
}
template <typename S>
std::shared_ptr<CollisionMatrix> CollisionManager<S>::getCollisionMatrix() const {
    return collision_matrix_;
}
template <typename S>
bool CollisionManager<S>::inTestedSet(fcl::CollisionObject<S>* a, fcl::CollisionObject<S>* b) const {
    if (a < b)
        return tested_set.find(std::make_pair(a, b)) != tested_set.end();
    else
        return tested_set.find(std::make_pair(b, a)) != tested_set.end();
}
template <typename S>
void CollisionManager<S>::insertTestedSet(fcl::CollisionObject<S>* a, fcl::CollisionObject<S>* b) const {
    if (a < b)
        tested_set.insert(std::make_pair(a, b));
    else
        tested_set.insert(std::make_pair(b, a));
}

template bool DefaultCollisionFunction<float>(fcl::CollisionObject<float>* o1, fcl::CollisionObject<float>* o2, CollisionData<float>& cdata);
template bool DefaultCollisionFunction<double>(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, CollisionData<double>& cdata);
template bool DefaultWithInDistanceFunction<float>(fcl::CollisionObject<float>* o1, fcl::CollisionObject<float>* o2, WithInDistanceData<float>& cdata);
template bool DefaultWithInDistanceFunction<double>(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, WithInDistanceData<double>& cdata);

template class CollisionManager<float>;
template class CollisionManager<double>;

} // namespace robot
} // namespace hyperbrain