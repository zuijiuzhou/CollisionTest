#pragma once

#include "collision_manager.h"

#include <unordered_map>

#include <fcl/broadphase/detail/hierarchy_tree.h>
#include <fcl/geometry/octree/octree.h>

// #include <Eigen/Geometry>

/** @brief hyperbrain project */
namespace hyperbrain {
/** @brief robots */
namespace robot {

template <typename S>
class CollisionManagerAABBTree : public CollisionManager<S> {
public:
    using DynamicAABBNode = fcl::detail::NodeBase<fcl::AABB<S>>;
    using DynamicAABBTable = std::unordered_map<fcl::CollisionObject<S>*, DynamicAABBNode*>;

public:
    CollisionManagerAABBTree();

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

    const fcl::detail::HierarchyTree<fcl::AABB<S>>& getTree() const;

private:
    void update_(fcl::CollisionObject<S>* updated_obj);

    bool collisionRecurse_(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        const fcl::OcTree<S>* tree2,
        const typename fcl::OcTree<S>::OcTreeNode* root2,
        const fcl::AABB<S>& root2_bv,
        const fcl::Transform3<S>& tf2,
        CollisionData<S>& cdata,
        CollisionCallBack<S> callback) const;

    template <typename Derived>
    bool collisionRecurse_(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        const fcl::OcTree<S>* tree2,
        const typename fcl::OcTree<S>::OcTreeNode* root2,
        const fcl::AABB<S>& root2_bv,
        const Eigen::MatrixBase<Derived>& translation2,
        CollisionData<S>& cdata,
        CollisionCallBack<S> callback) const;

    bool collisionRecurse(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        const fcl::OcTree<S>* tree2,
        const typename fcl::OcTree<S>::OcTreeNode* root2,
        const fcl::AABB<S>& root2_bv,
        const fcl::Transform3<S>& tf2,
        CollisionData<S>& cdata,
        CollisionCallBack<S> callback) const;

    bool collisionRecurse(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root2,
        CollisionData<S>& cdata,
        CollisionCallBack<S> callback) const;

    bool collisionRecurse(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root,
        fcl::CollisionObject<S>* query,
        CollisionData<S>& cdata,
        CollisionCallBack<S> callback) const;

    bool selfCollisionRecurse(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root,
        CollisionData<S>& cdata,
        CollisionCallBack<S> callback) const;


    bool withInDistanceRecurse_(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        const fcl::OcTree<S>* tree2,
        const typename fcl::OcTree<S>::OcTreeNode* root2,
        const fcl::AABB<S>& root2_bv,
        const fcl::Transform3<S>& tf2,
        WithInDistanceData<S>& cdata,
        WithInDistanceCallBack<S> callback) const;

    template <typename Derived>
    bool withInDistanceRecurse_(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        const fcl::OcTree<S>* tree2,
        const typename fcl::OcTree<S>::OcTreeNode* root2,
        const fcl::AABB<S>& root2_bv,
        const Eigen::MatrixBase<Derived>& translation2,
        WithInDistanceData<S>& cdata,
        WithInDistanceCallBack<S> callback) const;

    bool withInDistanceRecurse(typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
                               const fcl::OcTree<S>* tree2,
                               const typename fcl::OcTree<S>::OcTreeNode* root2,
                               const fcl::AABB<S>& root2_bv,
                               const fcl::Transform3<S>& tf2,
                               WithInDistanceData<S>& cdata,
                               WithInDistanceCallBack<S> callback) const;

    bool withInDistanceRecurse(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root2,
        WithInDistanceData<S>& cdata,
        WithInDistanceCallBack<S> callback) const;

    bool withInDistanceRecurse(typename CollisionManagerAABBTree<S>::DynamicAABBNode* root,
                               fcl::CollisionObject<S>* query,
                               WithInDistanceData<S>& cdata,
                               WithInDistanceCallBack<S> callback) const;

    bool selfWithInDistanceRecurse(typename CollisionManagerAABBTree<S>::DynamicAABBNode* root,
                                   WithInDistanceData<S>& cdata,
                                   WithInDistanceCallBack<S> callback) const;

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
} // namespace robot
} // namespace hyperbrain