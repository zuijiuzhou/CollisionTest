#pragma once

#include "collision_manager.h"

#include <unordered_map>

#include "fcl/broadphase/detail/hierarchy_tree.h"

template <typename S>
class CollisionManagerAABBTree : public CollisionManager<S> {
public:
    using DynamicAABBNode = fcl::detail::NodeBase<fcl::AABB<S>>;
    using DynamicAABBTable = std::unordered_map<fcl::CollisionObject<S>*, DynamicAABBNode*>;

public:
    CollisionManagerAABBTree();

public:
    /// @brief add objects to the manager
    void registerObjects(const std::vector<fcl::CollisionObject<S>*>& other_objs) override;

    /// @brief add one object to the manager
    void registerObject(fcl::CollisionObject<S>* obj) override;

    /// @brief remove one object from the managerk
    void unregisterObject(fcl::CollisionObject<S>* obj) override;

    /// @brief initialize the manager, related with the specific type of manager
    void setup() override;

    /// @brief update the condition of manager
    void update() override;

    /// @brief update the manager by explicitly given the object updated
    void update(fcl::CollisionObject<S>* updated_obj) override;

    /// @brief update the manager by explicitly given the set of objects update
    void update(const std::vector<fcl::CollisionObject<S>*>& updated_objs) override;

    /// @brief clear the manager
    void clear() override;

    /// @brief return the objects managed by the manager
    void getObjects(std::vector<fcl::CollisionObject<S>*>& objs) const override;

    /// @brief whether the manager is empty
    bool empty() const override;

    /// @brief the number of objects managed by the manager
    size_t size() const override;

    virtual void withInDistance(fcl::CollisionObject<S>* obj, void* cdata, fcl::DistanceCallBack<S> callback) override;

    virtual void withInDistance(void* cdata, fcl::DistanceCallBack<S> callback) override;

    virtual void withInDistance(CollisionManager<S>* other, void* cdata, fcl::DistanceCallBack<S> callback) override;

    const fcl::detail::HierarchyTree<fcl::AABB<S>>& getTree() const;

private:
    void update_(fcl::CollisionObject<S>* updated_obj);

public:
    int max_tree_nonbalanced_level;
    int tree_incremental_balance_pass;
    int& tree_topdown_balance_threshold;
    int& tree_topdown_level;
    int tree_init_level;

    bool octree_as_geometry_collide;
    bool octree_as_geometry_distance;

private:
    bool setup_;

    fcl::detail::HierarchyTree<fcl::AABB<S>> dtree;
    std::unordered_map<fcl::CollisionObject<S>*, DynamicAABBNode*> table;
};

using CollisionManagerAABBTreef = CollisionManagerAABBTree<float>;
using CollisionManagerAABBTreed = CollisionManagerAABBTree<double>;
