#pragma once

#include <map>
#include <memory>
#include <set>
#include <vector>

#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>

namespace fcl {

template <typename S>
class CollisionObject;

} // namespace fcl

/** @brief hyperbrain project */
namespace hyperbrain {
/** @brief robots */
namespace robot {

class CollisionMatrix;

template <typename S>
struct CollisionRequest;
template <typename S>
struct CollisionResult;
template <typename S>
struct CollisionData;
template <typename S>
bool DefaultCollisionFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, CollisionData<S>& cdata);

template <typename S>
struct WithInDistanceData;
template <typename S>
struct WithInDistanceRequest;
template <typename S>
bool DefaultWithInDistanceFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, WithInDistanceData<S>& cdata);

/// @brief Callback for collision between two objects. Return value is whether
/// can stop now.
template <typename S>
using CollisionCallBack = bool (*)(
    fcl::CollisionObject<S>* o1,
    fcl::CollisionObject<S>* o2,
    CollisionData<S>& cdata);

/// @brief Callback for distance between two objects, Return value is whether
/// can stop now, also return the minimum distance till now.
template <typename S>
using WithInDistanceCallBack = bool (*)(
    fcl::CollisionObject<S>* o1,
    fcl::CollisionObject<S>* o2,
    WithInDistanceData<S>& cdata);


template <typename S>
class CollisionManager {

public:
    /// @brief add objects to the manager
    virtual void registerObjects(const std::vector<fcl::CollisionObject<S>*>& other_objs);

    /// @brief add one object to the manager
    virtual void registerObject(fcl::CollisionObject<S>* obj) = 0;

    /// @brief remove one object from the managerk
    virtual void unregisterObject(fcl::CollisionObject<S>* obj) = 0;

    /// @brief initialize the manager, related with the specific type of manager
    virtual void setup() = 0;

    /// @brief update the condition of manager
    virtual void update() = 0;

    /// @brief update the manager by explicitly given the object updated
    virtual void update(fcl::CollisionObject<S>* updated_obj) {
        update();
    }

    /// @brief update the manager by explicitly given the set of objects update
    virtual void update(const std::vector<fcl::CollisionObject<S>*>& updated_objs) {
        update();
    }

    /// @brief clear the manager
    virtual void clear() = 0;

    /// @brief return the objects managed by the manager
    virtual void getObjects(std::vector<fcl::CollisionObject<S>*>& objs) const = 0;

    virtual bool empty() const = 0;

    /// @brief the number of objects managed by the manager
    virtual size_t size() const = 0;

    virtual bool isExcludedBetween(fcl::CollisionObject<S>* obj1, fcl::CollisionObject<S>* obj2) const;

    virtual bool isExcludedBetween(CollisionMatrix* cm, fcl::CollisionObject<S>* obj1, fcl::CollisionObject<S>* obj2) const;

    virtual S getSafetyDistanceBetween(fcl::CollisionObject<S>* obj1, fcl::CollisionObject<S>* obj2) const;

    virtual S getSafetyDistanceBetween(CollisionMatrix* cm, fcl::CollisionObject<S>* obj1, fcl::CollisionObject<S>* obj2) const;

    void setCollisionMatrix(std::shared_ptr<CollisionMatrix> collision_matrix);

    std::shared_ptr<CollisionMatrix> getCollisionMatrix() const;

    virtual void collide(CollisionData<S>& cdata, CollisionCallBack<S> callback) const = 0;

    virtual void collide(CollisionManager<S>* other, CollisionData<S>& cdata, CollisionCallBack<S> callback) const = 0;

    virtual void withInDistance(fcl::CollisionObject<S>* obj, WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const = 0;

    virtual void withInDistance(WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const = 0;

    virtual void withInDistance(CollisionManager<S>* other, WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const = 0;

protected:
    bool inTestedSet(fcl::CollisionObject<S>* a, fcl::CollisionObject<S>* b) const;

    void insertTestedSet(fcl::CollisionObject<S>* a, fcl::CollisionObject<S>* b) const;

protected:
    std::shared_ptr<CollisionMatrix> collision_matrix_;
    /// @brief tools help to avoid repeating collision or distance callback for the pairs of objects tested before. It can be useful for some of the broadphase algorithms.
    mutable std::set<std::pair<fcl::CollisionObject<S>*, fcl::CollisionObject<S>*>> tested_set;
    mutable bool enable_tested_set_;
};

using CollisionDataf = CollisionData<float>;
using CollisionDatad = CollisionData<double>;
using CollisionRequestf = CollisionRequest<float>;
using CollisionRequestd = CollisionRequest<double>;
using CollisionResultf = CollisionResult<float>;
using CollisionResultd = CollisionResult<double>;

using WithInDistanceDataf = WithInDistanceData<float>;
using WithInDistanceDatad = WithInDistanceData<double>;
using WithInDistanceRequestf = WithInDistanceRequest<float>;
using WithInDistanceRequestd = WithInDistanceRequest<double>;


#pragma region Collison and WithInDistance data

template <typename S>
struct CollisionRequest : public fcl::CollisionRequest<S> {
    bool stop_at_first_contact = false;
};
template <typename S>
struct CollisionResult : public fcl::CollisionResult<S> {
    fcl::CollisionObject<S>* o1 = nullptr;
    fcl::CollisionObject<S>* o2 = nullptr;
};

template <typename S>
struct CollisionData {
    CollisionRequest<S> request;
    std::vector<CollisionResult<S>> results;
    size_t nb_narrow_phase = 0;
    bool done = false;
};

template <typename S>
struct WithInDistanceRequest : public fcl::DistanceRequest<S> {
    bool stop_at_first_contact = false;
    S safety_dist = S(0);
};

template <typename S>
struct WithInDistanceData {
    WithInDistanceRequest<S> request;
    std::vector<fcl::DistanceResult<S>> results;
    size_t nb_narrow_phase = 0;
    bool done = false;
};

#pragma endregion

} // namespace robot
} // namespace hyperbrain