#include "collision_manager_aabb_tree.h"

#include <functional>
#include <limits>
#include <unordered_map>

#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/utility.h>
#include <fcl/math/bv/utility.h>
#include <fcl/narrowphase/collision_object.h>
// #include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/narrowphase/distance.h>

#define TMPL_PREFIX_ template <typename S>

/** @brief hyperbrain project */
namespace hyperbrain {
/** @brief robots */
namespace robot {

TMPL_PREFIX_
CollisionManagerAABBTree<S>::CollisionManagerAABBTree()
  : tree_topdown_balance_threshold(dtree.bu_threshold)
  , tree_topdown_level(dtree.topdown_level) {
    max_tree_nonbalanced_level = 10;
    tree_incremental_balance_pass = 10;
    tree_topdown_balance_threshold = 2;
    tree_topdown_level = 0;
    tree_init_level = 0;
    setup_ = false;

    // from experiment, this is the optimal setting
    octree_as_geometry_collide = true;
    octree_as_geometry_distance = false;
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::registerObjects(const std::vector<fcl::CollisionObject<S>*>& other_objs) {
    if (other_objs.empty())
        return;

    if (size() > 0) {
        CollisionManager<S>::registerObjects(other_objs);
    }
    else {
        std::vector<DynamicAABBNode*> leaves(other_objs.size());
        table.rehash(other_objs.size());
        for (size_t i = 0, size = other_objs.size(); i < size; ++i) {
            DynamicAABBNode* node = new DynamicAABBNode; // node will be managed by the dtree
            node->bv = other_objs[i]->getAABB();
            node->parent = nullptr;
            node->children[1] = nullptr;
            node->data = other_objs[i];
            table[other_objs[i]] = node;
            leaves[i] = node;
        }

        dtree.init(leaves, tree_init_level);

        setup_ = true;
    }
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::registerObject(fcl::CollisionObject<S>* obj) {
    DynamicAABBNode* node = dtree.insert(obj->getAABB(), obj);
    table[obj] = node;
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::unregisterObject(fcl::CollisionObject<S>* obj) {
    DynamicAABBNode* node = table[obj];
    table.erase(obj);
    dtree.remove(node);
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::setup() {
    if (!setup_) {
        int num = dtree.size();
        if (num == 0) {
            setup_ = true;
            return;
        }

        int height = dtree.getMaxHeight();

        if (height - std::log((S)num) / std::log(2.0) < max_tree_nonbalanced_level)
            dtree.balanceIncremental(tree_incremental_balance_pass);
        else
            dtree.balanceTopdown();

        setup_ = true;
    }
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::update() {
    for (auto it = table.cbegin(); it != table.cend(); ++it) {
        fcl::CollisionObject<S>* obj = it->first;
        DynamicAABBNode* node = it->second;
        node->bv = obj->getAABB();
    }

    dtree.refit();
    setup_ = false;

    setup();
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::update(fcl::CollisionObject<S>* updated_obj) {
    update_(updated_obj);
    setup();
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::update(const std::vector<fcl::CollisionObject<S>*>& updated_objs) {
    for (size_t i = 0, size = updated_objs.size(); i < size; ++i)
        update_(updated_objs[i]);
    setup();
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::clear() {
    dtree.clear();
    table.clear();
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::getObjects(std::vector<fcl::CollisionObject<S>*>& objs) const {
    objs.resize(this->size());
    std::transform(table.begin(), table.end(), objs.begin(), std::bind(&DynamicAABBTable::value_type::first, std::placeholders::_1));
}

TMPL_PREFIX_
bool CollisionManagerAABBTree<S>::empty() const {
    return dtree.empty();
}

TMPL_PREFIX_
size_t CollisionManagerAABBTree<S>::size() const {
    return dtree.size();
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::collide(CollisionData<S>& cdata, CollisionCallBack<S> callback) const {
    if (size() == 0) return;
    selfCollisionRecurse(dtree.getRoot(), cdata, callback);
    cdata.done = true;
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::collide(CollisionManager<S>* other, CollisionData<S>& cdata, CollisionCallBack<S> callback) const {
    CollisionManagerAABBTree<S>* other_manager = dynamic_cast<CollisionManagerAABBTree<S>*>(other);
    if ((size() == 0) || (other_manager->size() == 0)) return;
    collisionRecurse(dtree.getRoot(), other_manager->dtree.getRoot(), cdata, callback);
    cdata.done = true;
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::withInDistance(fcl::CollisionObject<S>* obj, WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const {
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::withInDistance(WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const {
    if (this->size() == 0)
        return;

    selfWithInDistanceRecurse(dtree.getRoot(), cdata, callback);

    cdata.done = true;
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::withInDistance(CollisionManager<S>* other, WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const {
    auto other_manager = static_cast<CollisionManagerAABBTree<S>*>(other);
    if ((size() == 0) || (other_manager->size() == 0)) return;
    withInDistanceRecurse(dtree.getRoot(), other_manager->dtree.getRoot(), cdata, callback);
}

TMPL_PREFIX_
const fcl::detail::HierarchyTree<fcl::AABB<S>>& CollisionManagerAABBTree<S>::getTree() const {
    return dtree;
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::update_(fcl::CollisionObject<S>* updated_obj) {
    const auto it = table.find(updated_obj);
    if (it != table.end()) {
        DynamicAABBNode* node = it->second;
        if (!node->bv.equal(updated_obj->getAABB()))
            dtree.update(node, updated_obj->getAABB());
    }
    setup_ = false;
}

//==============================================================================
template <typename S>
bool CollisionManagerAABBTree<S>::collisionRecurse_(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
    const fcl::OcTree<S>* tree2,
    const typename fcl::OcTree<S>::OcTreeNode* root2,
    const fcl::AABB<S>& root2_bv,
    const fcl::Transform3<S>& tf2,
    CollisionData<S>& cdata,
    CollisionCallBack<S> callback) const {
    if (!root2) {
        if (root1->isLeaf()) {
            fcl::CollisionObject<S>* obj1 = static_cast<fcl::CollisionObject<S>*>(root1->data);

            if (!obj1->isFree()) {
                fcl::OBB<S> obb1, obb2;
                fcl::convertBV(root1->bv, fcl::Transform3<S>::Identity(), obb1);
                fcl::convertBV(root2_bv, tf2, obb2);

                if (obb1.overlap(obb2)) {
                    fcl::Box<S>* box = new fcl::Box<S>();
                    fcl::Transform3<S> box_tf;
                    fcl::constructBox(root2_bv, tf2, *box, box_tf);

                    box->cost_density = tree2->getDefaultOccupancy();

                    fcl::CollisionObject<S> obj2(std::shared_ptr<fcl::CollisionGeometry<S>>(box), box_tf);
                    return callback(obj1, &obj2, cdata);
                }
            }
        }
        else {
            if (collisionRecurse_(root1->children[0], tree2, nullptr, root2_bv, tf2, cdata, callback))
                return true;
            if (collisionRecurse_(root1->children[1], tree2, nullptr, root2_bv, tf2, cdata, callback))
                return true;
        }

        return false;
    }
    else if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
        fcl::CollisionObject<S>* obj1 = static_cast<fcl::CollisionObject<S>*>(root1->data);

        if (!tree2->isNodeFree(root2) && !obj1->isFree()) {
            fcl::OBB<S> obb1, obb2;
            fcl::convertBV(root1->bv, fcl::Transform3<S>::Identity(), obb1);
            fcl::convertBV(root2_bv, tf2, obb2);

            if (obb1.overlap(obb2)) {
                fcl::Box<S>* box = new fcl::Box<S>();
                fcl::Transform3<S> box_tf;
                fcl::constructBox(root2_bv, tf2, *box, box_tf);

                box->cost_density = root2->getOccupancy();
                box->threshold_occupied = tree2->getOccupancyThres();

                fcl::CollisionObject<S> obj2(std::shared_ptr<fcl::CollisionGeometry<S>>(box), box_tf);
                return callback(obj1, &obj2, cdata);
            }
            else
                return false;
        }
        else
            return false;
    }

    fcl::OBB<S> obb1, obb2;
    fcl::convertBV(root1->bv, fcl::Transform3<S>::Identity(), obb1);
    fcl::convertBV(root2_bv, tf2, obb2);

    if (tree2->isNodeFree(root2) || !obb1.overlap(obb2)) return false;

    if (!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
        if (collisionRecurse_(root1->children[0], tree2, root2, root2_bv, tf2, cdata, callback))
            return true;
        if (collisionRecurse_(root1->children[1], tree2, root2, root2_bv, tf2, cdata, callback))
            return true;
    }
    else {
        for (unsigned int i = 0; i < 8; ++i) {
            if (tree2->nodeChildExists(root2, i)) {
                const typename fcl::OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
                fcl::AABB<S> child_bv;
                fcl::computeChildBV(root2_bv, i, child_bv);

                if (collisionRecurse_(root1, tree2, child, child_bv, tf2, cdata, callback))
                    return true;
            }
            else {
                fcl::AABB<S> child_bv;
                fcl::computeChildBV(root2_bv, i, child_bv);
                if (collisionRecurse_(root1, tree2, nullptr, child_bv, tf2, cdata, callback))
                    return true;
            }
        }
    }
    return false;
}

//==============================================================================
template <typename S>
template <typename Derived>
bool CollisionManagerAABBTree<S>::collisionRecurse_(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
    const fcl::OcTree<S>* tree2,
    const typename fcl::OcTree<S>::OcTreeNode* root2,
    const fcl::AABB<S>& root2_bv,
    const Eigen::MatrixBase<Derived>& translation2,
    CollisionData<S>& cdata,
    CollisionCallBack<S> callback) const {
    if (!root2) {
        if (root1->isLeaf()) {
            fcl::CollisionObject<S>* obj1 = static_cast<fcl::CollisionObject<S>*>(root1->data);

            if (!obj1->isFree()) {
                const fcl::AABB<S>& root2_bv_t = fcl::translate(root2_bv, translation2);
                if (root1->bv.overlap(root2_bv_t)) {
                    fcl::Box<S>* box = new fcl::Box<S>();
                    fcl::Transform3<S> box_tf;
                    fcl::Transform3<S> tf2 = fcl::Transform3<S>::Identity();
                    tf2.translation() = translation2;
                    fcl::constructBox(root2_bv, tf2, *box, box_tf);

                    box->cost_density = tree2->getOccupancyThres(); // thresholds are 0, 1, so uncertain

                    fcl::CollisionObject<S> obj2(std::shared_ptr<fcl::CollisionGeometry<S>>(box), box_tf);
                    return callback(obj1, &obj2, cdata);
                }
            }
        }
        else {
            if (collisionRecurse_(root1->children[0], tree2, nullptr, root2_bv, translation2, cdata, callback))
                return true;
            if (collisionRecurse_(root1->children[1], tree2, nullptr, root2_bv, translation2, cdata, callback))
                return true;
        }

        return false;
    }
    else if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
        fcl::CollisionObject<S>* obj1 = static_cast<fcl::CollisionObject<S>*>(root1->data);

        if (!tree2->isNodeFree(root2) && !obj1->isFree()) {
            const fcl::AABB<S>& root2_bv_t = translate(root2_bv, translation2);
            if (root1->bv.overlap(root2_bv_t)) {
                fcl::Box<S>* box = new fcl::Box<S>();
                fcl::Transform3<S> box_tf;
                fcl::Transform3<S> tf2 = fcl::Transform3<S>::Identity();
                tf2.translation() = translation2;
                fcl::constructBox(root2_bv, tf2, *box, box_tf);

                box->cost_density = root2->getOccupancy();
                box->threshold_occupied = tree2->getOccupancyThres();

                fcl::CollisionObject<S> obj2(std::shared_ptr<fcl::CollisionGeometry<S>>(box), box_tf);
                return callback(obj1, &obj2, cdata);
            }
            else
                return false;
        }
        else
            return false;
    }

    const fcl::AABB<S>& root2_bv_t = translate(root2_bv, translation2);
    if (tree2->isNodeFree(root2) || !root1->bv.overlap(root2_bv_t)) return false;

    if (!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
        if (collisionRecurse_(root1->children[0], tree2, root2, root2_bv, translation2, cdata, callback))
            return true;
        if (collisionRecurse_(root1->children[1], tree2, root2, root2_bv, translation2, cdata, callback))
            return true;
    }
    else {
        for (unsigned int i = 0; i < 8; ++i) {
            if (tree2->nodeChildExists(root2, i)) {
                const typename fcl::OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
                fcl::AABB<S> child_bv;
                fcl::computeChildBV(root2_bv, i, child_bv);

                if (collisionRecurse_(root1, tree2, child, child_bv, translation2, cdata, callback))
                    return true;
            }
            else {
                fcl::AABB<S> child_bv;
                fcl::computeChildBV(root2_bv, i, child_bv);
                if (collisionRecurse_(root1, tree2, nullptr, child_bv, translation2, cdata, callback))
                    return true;
            }
        }
    }
    return false;
}

//==============================================================================
template <typename S>
bool CollisionManagerAABBTree<S>::collisionRecurse(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
    const fcl::OcTree<S>* tree2,
    const typename fcl::OcTree<S>::OcTreeNode* root2,
    const fcl::AABB<S>& root2_bv,
    const fcl::Transform3<S>& tf2,
    CollisionData<S>& cdata,
    CollisionCallBack<S> callback) const {
    if (tf2.linear().isIdentity())
        return collisionRecurse_(root1, tree2, root2, root2_bv, tf2.translation(), cdata, callback);
    else // has rotation
        return collisionRecurse_(root1, tree2, root2, root2_bv, tf2, cdata, callback);
}

//==============================================================================
template <typename S>
bool CollisionManagerAABBTree<S>::collisionRecurse(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root2,
    CollisionData<S>& cdata,
    CollisionCallBack<S> callback) const {
    if (root1->isLeaf() && root2->isLeaf()) {
        if (this->isExcludedBetween(static_cast<fcl::CollisionObject<S>*>(root1->data), static_cast<fcl::CollisionObject<S>*>(root2->data))) {
            return false;
        }
        if (!root1->bv.overlap(root2->bv)) return false;
        return callback(static_cast<fcl::CollisionObject<S>*>(root1->data), static_cast<fcl::CollisionObject<S>*>(root2->data), cdata);
    }

    if (!root1->bv.overlap(root2->bv)) return false;

    if (root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size()))) {
        if (collisionRecurse(root1->children[0], root2, cdata, callback))
            return true;
        if (collisionRecurse(root1->children[1], root2, cdata, callback))
            return true;
    }
    else {
        if (collisionRecurse(root1, root2->children[0], cdata, callback))
            return true;
        if (collisionRecurse(root1, root2->children[1], cdata, callback))
            return true;
    }
    return false;
}

//==============================================================================
template <typename S>
bool CollisionManagerAABBTree<S>::collisionRecurse(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root,
    fcl::CollisionObject<S>* query,
    CollisionData<S>& cdata,
    CollisionCallBack<S> callback) const {
    if (root->isLeaf()) {
        if (this->isExcludedBetween(static_cast<fcl::CollisionObject<S>*>(root->data), query)) {
            return false;
        }
        if (!root->bv.overlap(query->getAABB())) return false;
        return callback(static_cast<fcl::CollisionObject<S>*>(root->data), query, cdata);
    }

    if (!root->bv.overlap(query->getAABB())) return false;

    int select_res = select(query->getAABB(), *(root->children[0]), *(root->children[1]));

    if (collisionRecurse(root->children[select_res], query, cdata, callback))
        return true;

    if (collisionRecurse(root->children[1 - select_res], query, cdata, callback))
        return true;

    return false;
}

//==============================================================================
template <typename S>
bool CollisionManagerAABBTree<S>::selfCollisionRecurse(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root,
    CollisionData<S>& cdata,
    CollisionCallBack<S> callback) const {
    if (root->isLeaf()) return false;

    if (selfCollisionRecurse(root->children[0], cdata, callback))
        return true;

    if (selfCollisionRecurse(root->children[1], cdata, callback))
        return true;

    if (collisionRecurse(root->children[0], root->children[1], cdata, callback))
        return true;

    return false;
}

//==============================================================================
template <typename S>
bool CollisionManagerAABBTree<S>::withInDistanceRecurse_(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
    const fcl::OcTree<S>* tree2,
    const typename fcl::OcTree<S>::OcTreeNode* root2,
    const fcl::AABB<S>& root2_bv,
    const fcl::Transform3<S>& tf2,
    WithInDistanceData<S>& cdata,
    WithInDistanceCallBack<S> callback) const {
    if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
        if (tree2->isNodeOccupied(root2)) {

            cdata.request.safety_dist = S(0);

            fcl::Box<S>* box = new fcl::Box<S>();
            fcl::Transform3<S> box_tf;
            fcl::constructBox(root2_bv, tf2, *box, box_tf);
            fcl::CollisionObject<S> obj(std::shared_ptr<fcl::CollisionGeometry<S>>(box), box_tf);
            return callback(static_cast<fcl::CollisionObject<S>*>(root1->data), &obj, cdata);
        }
        else
            return false;
    }

    if (!tree2->isNodeOccupied(root2))
        return false;

    S min_dist = std::numeric_limits<S>::max();

    if (!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
        fcl::AABB<S> aabb2;
        fcl::convertBV(root2_bv, tf2, aabb2);

        S d1 = aabb2.distance(root1->children[0]->bv);
        S d2 = aabb2.distance(root1->children[1]->bv);

        if (d2 < d1) {

            if (d2 < min_dist) {
                if (withInDistanceRecurse_(root1->children[1], tree2, root2, root2_bv, tf2, cdata, callback))
                    return true;
            }

            if (d1 < min_dist) {
                if (withInDistanceRecurse_(root1->children[0], tree2, root2, root2_bv, tf2, cdata, callback))
                    return true;
            }
        }
        else {
            if (d1 < min_dist) {
                if (withInDistanceRecurse_(root1->children[0], tree2, root2, root2_bv, tf2, cdata, callback))
                    return true;
            }

            if (d2 < min_dist) {
                if (withInDistanceRecurse_(root1->children[1], tree2, root2, root2_bv, tf2, cdata, callback))
                    return true;
            }
        }
    }
    else {
        for (unsigned int i = 0; i < 8; ++i) {
            if (tree2->nodeChildExists(root2, i)) {
                const typename fcl::OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
                fcl::AABB<S> child_bv;
                fcl::computeChildBV(root2_bv, i, child_bv);

                fcl::AABB<S> aabb2;
                fcl::convertBV(child_bv, tf2, aabb2);
                S d = root1->bv.distance(aabb2);

                if (d < min_dist) {
                    if (withInDistanceRecurse_(root1, tree2, child, child_bv, tf2, cdata, callback))
                        return true;
                }
            }
        }
    }

    return false;
}

//==============================================================================
template <typename S>
template <typename Derived>
bool CollisionManagerAABBTree<S>::withInDistanceRecurse_(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
    const fcl::OcTree<S>* tree2,
    const typename fcl::OcTree<S>::OcTreeNode* root2,
    const fcl::AABB<S>& root2_bv,
    const Eigen::MatrixBase<Derived>& translation2,
    WithInDistanceData<S>& cdata,
    WithInDistanceCallBack<S> callback) const {
    if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
        if (tree2->isNodeOccupied(root2)) {

            cdata.request.safety_dist = S(0);

            fcl::Box<S>* box = new fcl::Box<S>();
            fcl::Transform3<S> box_tf;
            fcl::Transform3<S> tf2 = fcl::Transform3<S>::Identity();
            tf2.translation() = translation2;
            fcl::constructBox(root2_bv, tf2, *box, box_tf);
            fcl::CollisionObject<S> obj(std::shared_ptr<fcl::CollisionGeometry<S>>(box), box_tf);
            return callback(static_cast<fcl::CollisionObject<S>*>(root1->data), &obj, cdata);
        }
        else
            return false;
    }

    if (!tree2->isNodeOccupied(root2))
        return false;

    S min_dist = std::numeric_limits<S>::max();

    if (!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
        const fcl::AABB<S>& aabb2 = translate(root2_bv, translation2);
        S d1 = aabb2.distance(root1->children[0]->bv);
        S d2 = aabb2.distance(root1->children[1]->bv);

        if (d2 < d1) {
            if (d2 < min_dist) {
                if (withInDistanceRecurse_(root1->children[1], tree2, root2, root2_bv, translation2, cdata, callback))
                    return true;
            }

            if (d1 < min_dist) {
                if (withInDistanceRecurse_(root1->children[0], tree2, root2, root2_bv, translation2, cdata, callback))
                    return true;
            }
        }
        else {
            if (d1 < min_dist) {
                if (withInDistanceRecurse_(root1->children[0], tree2, root2, root2_bv, translation2, cdata, callback))
                    return true;
            }

            if (d2 < min_dist) {
                if (withInDistanceRecurse_(root1->children[1], tree2, root2, root2_bv, translation2, cdata, callback))
                    return true;
            }
        }
    }
    else {
        for (unsigned int i = 0; i < 8; ++i) {
            if (tree2->nodeChildExists(root2, i)) {
                const typename fcl::OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
                fcl::AABB<S> child_bv;
                fcl::computeChildBV(root2_bv, i, child_bv);
                const fcl::AABB<S>& aabb2 = translate(child_bv, translation2);

                S d = root1->bv.distance(aabb2);

                if (d < min_dist) {
                    if (withInDistanceRecurse_(root1, tree2, child, child_bv, translation2, cdata, callback))
                        return true;
                }
            }
        }
    }

    return false;
}

//==============================================================================
template <typename S>
bool CollisionManagerAABBTree<S>::withInDistanceRecurse(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
    const fcl::OcTree<S>* tree2,
    const typename fcl::OcTree<S>::OcTreeNode* root2,
    const fcl::AABB<S>& root2_bv,
    const fcl::Transform3<S>& tf2,
    WithInDistanceData<S>& cdata,
    WithInDistanceCallBack<S> callback) const {
    if (tf2.linear().isIdentity())
        return withInDistanceRecurse_(root1, tree2, root2, root2_bv, tf2.translation(), cdata, callback);
    else
        return withInDistanceRecurse_(root1, tree2, root2, root2_bv, tf2, cdata, callback);
}

//==============================================================================
template <typename S>
bool CollisionManagerAABBTree<S>::withInDistanceRecurse(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root2,
    WithInDistanceData<S>& cdata,
    WithInDistanceCallBack<S> callback) const {
    if (root1->isLeaf() && root2->isLeaf()) {
        fcl::CollisionObject<S>* root1_obj = static_cast<fcl::CollisionObject<S>*>(root1->data);
        fcl::CollisionObject<S>* root2_obj = static_cast<fcl::CollisionObject<S>*>(root2->data);
        if (this->isExcludedBetween(root1_obj, root2_obj)) {
            return false;
        }

        cdata.request.safety_dist = this->getSafetyDistanceBetween(root1_obj, root2_obj);
        if (root1_obj->getAABB().distance(root2_obj->getAABB()) > cdata.request.safety_dist)
            return false;

        return callback(root1_obj, root2_obj, cdata);
    }

    S min_dist = std::numeric_limits<S>::max();

    if (root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size()))) {
        S d1 = root2->bv.distance(root1->children[0]->bv);
        S d2 = root2->bv.distance(root1->children[1]->bv);

        if (d2 < d1) {
            if (d2 < min_dist) {
                if (withInDistanceRecurse(root1->children[1], root2, cdata, callback))
                    return true;
            }

            if (d1 < min_dist) {
                if (withInDistanceRecurse(root1->children[0], root2, cdata, callback))
                    return true;
            }
        }
        else {
            if (d1 < min_dist) {
                if (withInDistanceRecurse(root1->children[0], root2, cdata, callback))
                    return true;
            }

            if (d2 < min_dist) {
                if (withInDistanceRecurse(root1->children[1], root2, cdata, callback))
                    return true;
            }
        }
    }
    else {
        S d1 = root1->bv.distance(root2->children[0]->bv);
        S d2 = root1->bv.distance(root2->children[1]->bv);

        if (d2 < d1) {
            if (d2 < min_dist) {
                if (withInDistanceRecurse(root1, root2->children[1], cdata, callback))
                    return true;
            }

            if (d1 < min_dist) {
                if (withInDistanceRecurse(root1, root2->children[0], cdata, callback))
                    return true;
            }
        }
        else {
            if (d1 < min_dist) {
                if (withInDistanceRecurse(root1, root2->children[0], cdata, callback))
                    return true;
            }

            if (d2 < min_dist) {
                if (withInDistanceRecurse(root1, root2->children[1], cdata, callback))
                    return true;
            }
        }
    }

    return false;
}

//==============================================================================
template <typename S>
bool CollisionManagerAABBTree<S>::withInDistanceRecurse(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root,
    fcl::CollisionObject<S>* query,
    WithInDistanceData<S>& cdata,
    WithInDistanceCallBack<S> callback) const {
    if (root->isLeaf()) {
        fcl::CollisionObject<S>* root_obj = static_cast<fcl::CollisionObject<S>*>(root->data);
        if (this->isExcludedBetween(root_obj, query)) {
            return false;
        }

        cdata.request.safety_dist = this->getSafetyDistanceBetween(root_obj, query);

        if (root_obj->getAABB().distance(query->getAABB()) > cdata.request.safety_dist)
            return false;

        return callback(root_obj, query, cdata);
    }

    S min_dist = std::numeric_limits<S>::max();

    S d1 = query->getAABB().distance(root->children[0]->bv);
    S d2 = query->getAABB().distance(root->children[1]->bv);

    if (d2 < d1) {
        if (d2 < min_dist) {
            if (withInDistanceRecurse(root->children[1], query, cdata, callback))
                return true;
        }

        if (d1 < min_dist) {
            if (withInDistanceRecurse(root->children[0], query, cdata, callback))
                return true;
        }
    }
    else {
        if (d1 < min_dist) {
            if (withInDistanceRecurse(root->children[0], query, cdata, callback))
                return true;
        }

        if (d2 < min_dist) {
            if (withInDistanceRecurse(root->children[1], query, cdata, callback))
                return true;
        }
    }

    return false;
}

//==============================================================================
template <typename S>
bool CollisionManagerAABBTree<S>::selfWithInDistanceRecurse(
    typename CollisionManagerAABBTree<S>::DynamicAABBNode* root,
    WithInDistanceData<S>& cdata,
    WithInDistanceCallBack<S> callback) const {
    if (root->isLeaf())
        return false;

    if (selfWithInDistanceRecurse(root->children[0], cdata, callback))
        return true;

    if (selfWithInDistanceRecurse(root->children[1], cdata, callback))
        return true;

    if (withInDistanceRecurse(root->children[0], root->children[1], cdata, callback))
        return true;

    return false;
}


template class CollisionManagerAABBTree<float>;
template class CollisionManagerAABBTree<double>;
/*template bool CollisionManagerAABBTree<float>::collisionRecurse_<float>(typename CollisionManagerAABBTree<float>::DynamicAABBNode*,
                                                                        const fcl::OcTree<float>*,
                                                                        const typename fcl::OcTree<float>::OcTreeNode*,
                                                                        const fcl::AABB<float>&,
                                                                        const Eigen::MatrixBase<float>&,
                                                                        CollisionData<float>&,
                                                                        CollisionCallBack<float>);*/

#undef TMPL_PREFIX_

} // namespace robot

} // namespace hyperbrain