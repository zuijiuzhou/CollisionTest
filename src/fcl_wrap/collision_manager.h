#pragma once

#include <map>
#include <memory>
#include <set>
#include <vector>

namespace fcl {
template <typename S>
class CollisionObject;

template <typename S>
using DistanceCallBack = bool (*)(
    CollisionObject<S>* o1,
    CollisionObject<S>* o2, void* cdata, S& dist);
}

#include "collision_matrix.h"

template <typename S>
class CollisionManager {

public:
    /// @brief add objects to the manager
    virtual void registerObjects(const std::vector<fcl::CollisionObject<S>*>& other_objs)
    {
        for (size_t i = 0; i < other_objs.size(); ++i)
            registerObject(other_objs[i]);
    }

    /// @brief add one object to the manager
    virtual void registerObject(fcl::CollisionObject<S>* obj) = 0;

    /// @brief remove one object from the managerk
    virtual void unregisterObject(fcl::CollisionObject<S>* obj) = 0;

    /// @brief initialize the manager, related with the specific type of manager
    virtual void setup() = 0;

    /// @brief update the condition of manager
    virtual void update() = 0;

    /// @brief update the manager by explicitly given the object updated
    virtual void update(fcl::CollisionObject<S>* updated_obj)
    {
        update();
    }

    /// @brief update the manager by explicitly given the set of objects update
    virtual void update(const std::vector<fcl::CollisionObject<S>*>& updated_objs)
    {
        update();
    }

    /// @brief clear the manager
    virtual void clear() = 0;

    /// @brief return the objects managed by the manager
    virtual void getObjects(std::vector<fcl::CollisionObject<S>*>& objs) const = 0;

    virtual bool empty() const = 0;

    /// @brief the number of objects managed by the manager
    virtual size_t size() const = 0;

    virtual void withInDistance(fcl::CollisionObject<S>* obj, void* cdata, fcl::DistanceCallBack<S> callback) = 0;
    virtual void withInDistance(void* cdata, fcl::DistanceCallBack<S> callback) = 0;
    virtual void withInDistance(CollisionManager* other, void* cdata, fcl::DistanceCallBack<S> callback) = 0;

    void setCollisionMatrix(std::shared_ptr<CollisionMatrixFcl<S>> collision_matrix)
    {
        collision_matrix_ = collision_matrix;
    }

    std::shared_ptr<CollisionMatrixFcl<S>> getCollisionMatrix() const
    {
        return collision_matrix_;
    }

protected:
    bool inTestedSet(fcl::CollisionObject<S>* a, fcl::CollisionObject<S>* b) const
    {
        if (a < b)
            return tested_set.find(std::make_pair(a, b)) != tested_set.end();
        else
            return tested_set.find(std::make_pair(b, a)) != tested_set.end();
    }

    void insertTestedSet(fcl::CollisionObject<S>* a, fcl::CollisionObject<S>* b) const
    {
        if (a < b)
            tested_set.insert(std::make_pair(a, b));
        else
            tested_set.insert(std::make_pair(b, a));
    }

protected:
    std::shared_ptr<CollisionMatrixFcl<S>> collision_matrix_;
    /// @brief tools help to avoid repeating collision or distance callback for the pairs of objects tested before. It can be useful for some of the broadphase algorithms.
    mutable std::set<std::pair<fcl::CollisionObject<S>*, fcl::CollisionObject<S>*>> tested_set;
    mutable bool enable_tested_set_;
};