#pragma once

#include "collision_manager.h"

#include <list>
#include <map>

#include <fcl/math/bv/AABB.h>

/** @brief hyperbrain project */
namespace hyperbrain {
/** @brief robots */
namespace robot {

template <typename S>
class CollisionManagerSaP : public CollisionManager<S> {
    /// @brief SAP interval for one object
    struct SaPAABB;

    /// @brief End point for an interval
    struct EndPoint;

    /// @brief A pair of objects that are not culling away and should further check collision
    struct SaPPair;

    /// @brief Functor to help unregister one object
    class isUnregistered;

    /// @brief Functor to help remove collision pairs no longer valid (i.e., should be culled away)
    class isNotValidPair;

public:
    CollisionManagerSaP();
    virtual ~CollisionManagerSaP();

public:
    void registerObjects(const std::vector<fcl::CollisionObject<S>*>& other_objs) override;

    void registerObject(fcl::CollisionObject<S>* obj) override;

    void unregisterObject(fcl::CollisionObject<S>* obj) override;

    void setup() override;

    void update() override;

    void update(fcl::CollisionObject<S>* updated_obj) override;

    void update(const std::vector<fcl::CollisionObject<S>*>& updated_objs) override;

    void clear() override;

    void getObjects(std::vector<fcl::CollisionObject<S>*>& objs) const override;

    bool empty() const override;

    size_t size() const override;

    virtual void collide(CollisionData<S>& cdata, CollisionCallBack<S> callback) const override;

    virtual void collide(CollisionManager<S>* other, CollisionData<S>& cdata, CollisionCallBack<S> callback) const override;

    virtual void withInDistance(fcl::CollisionObject<S>* obj, WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const override;

    virtual void withInDistance(WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const override;

    virtual void withInDistance(CollisionManager<S>* other, WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const override;

private:
    bool collide_(fcl::CollisionObject<S>* obj, CollisionData<S>& cdata, CollisionCallBack<S> callback) const;

    bool withInDistance_(fcl::CollisionObject<S>* obj, WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const;

    void addToOverlapPairs(const SaPPair& p);

    void removeFromOverlapPairs(const SaPPair& p);

    void update_(SaPAABB* updated_aabb);

    void updateVelist();

private:
    /// @brief End point list for x, y, z coordinates
    EndPoint* elist[3];

    /// @brief vector version of elist, for acceleration
    std::vector<EndPoint*> velist[3];

    /// @brief SAP interval list
    std::list<SaPAABB*> AABB_arr;

    /// @brief The pair of objects that should further check for collision
    std::list<SaPPair> overlap_pairs;

    size_t optimal_axis;

    std::map<fcl::CollisionObject<S>*, SaPAABB*> obj_aabb_map;
};

using CollisionManagerSaPf = CollisionManagerSaP<float>;
using CollisionManagerSaPd = CollisionManagerSaP<double>;

/// @brief SAP interval for one object
template <typename S>
struct CollisionManagerSaP<S>::SaPAABB {
    /// @brief object
    fcl::CollisionObject<S>* obj;

    /// @brief lower bound end point of the interval
    typename CollisionManagerSaP<S>::EndPoint* lo;

    /// @brief higher bound end point of the interval
    typename CollisionManagerSaP<S>::EndPoint* hi;

    /// @brief cached AABB<S> value
    fcl::AABB<S> cached;
};

/// @brief End point for an interval
template <typename S>
struct CollisionManagerSaP<S>::EndPoint {
    /// @brief tag for whether it is a lower bound or higher bound of an interval, 0 for lo, and 1 for hi
    char minmax;

    /// @brief back pointer to SAP interval
    typename CollisionManagerSaP<S>::SaPAABB* aabb;

    /// @brief the previous end point in the end point list
    EndPoint* prev[3];

    /// @brief the next end point in the end point list
    EndPoint* next[3];

    /// @brief get the value of the end point
    const fcl::Vector3<S>& getVal() const;

    /// @brief set the value of the end point
    fcl::Vector3<S>& getVal();

    S getVal(size_t i) const;

    S& getVal(size_t i);
};

/// @brief A pair of objects that are not culling away and should further check collision
template <typename S>
struct CollisionManagerSaP<S>::SaPPair {
    SaPPair(fcl::CollisionObject<S>* a, fcl::CollisionObject<S>* b);

    fcl::CollisionObject<S>* obj1;
    fcl::CollisionObject<S>* obj2;

    bool operator==(const SaPPair& other) const;
};

/// @brief Functor to help unregister one object
template <typename S>
class CollisionManagerSaP<S>::isUnregistered {
    fcl::CollisionObject<S>* obj;

public:
    isUnregistered(fcl::CollisionObject<S>* obj_);

    bool operator()(const SaPPair& pair) const;
};

/// @brief Functor to help remove collision pairs no longer valid (i.e., should be culled away)
template <typename S>
class CollisionManagerSaP<S>::isNotValidPair {
    fcl::CollisionObject<S>* obj1;
    fcl::CollisionObject<S>* obj2;

public:
    isNotValidPair(fcl::CollisionObject<S>* obj1_, fcl::CollisionObject<S>* obj2_);

    bool operator()(const SaPPair& pair);
};

} // namespace robot
} // namespace hyperbrain