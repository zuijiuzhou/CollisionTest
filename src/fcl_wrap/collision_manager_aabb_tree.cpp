#include "collision_manager_aabb_tree.h"

#include <functional>
#include <limits>
#include <unordered_map>

#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/utility.h>
#include <fcl/math/bv/utility.h>
#include <fcl/narrowphase/collision_object.h>

#define TMPL_PREFIX_ template <typename S>

namespace detail {

namespace dynamic_AABB_tree {

    using namespace fcl;

    //==============================================================================
    template <typename S>
    FCL_EXPORT bool distanceRecurse_(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        const OcTree<S>* tree2,
        const typename OcTree<S>::OcTreeNode* root2,
        const AABB<S>& root2_bv,
        const Transform3<S>& tf2,
        void* cdata,
        DistanceCallBack<S> callback,
        S& min_dist)
    {
        if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
            if (tree2->isNodeOccupied(root2)) {
                Box<S>* box = new Box<S>();
                Transform3<S> box_tf;
                constructBox(root2_bv, tf2, *box, box_tf);
                CollisionObject<S> obj(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
                return callback(static_cast<CollisionObject<S>*>(root1->data), &obj, cdata, min_dist);
            } else
                return false;
        }

        if (!tree2->isNodeOccupied(root2))
            return false;

        if (!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
            AABB<S> aabb2;
            convertBV(root2_bv, tf2, aabb2);

            S d1 = aabb2.distance(root1->children[0]->bv);
            S d2 = aabb2.distance(root1->children[1]->bv);

            if (d2 < d1) {
                if (d2 < min_dist) {
                    if (distanceRecurse_(root1->children[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
                        return true;
                }

                if (d1 < min_dist) {
                    if (distanceRecurse_(root1->children[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
                        return true;
                }
            } else {
                if (d1 < min_dist) {
                    if (distanceRecurse_(root1->children[0], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
                        return true;
                }

                if (d2 < min_dist) {
                    if (distanceRecurse_(root1->children[1], tree2, root2, root2_bv, tf2, cdata, callback, min_dist))
                        return true;
                }
            }
        } else {
            for (unsigned int i = 0; i < 8; ++i) {
                if (tree2->nodeChildExists(root2, i)) {
                    const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
                    AABB<S> child_bv;
                    computeChildBV(root2_bv, i, child_bv);

                    AABB<S> aabb2;
                    convertBV(child_bv, tf2, aabb2);
                    S d = root1->bv.distance(aabb2);

                    if (d < min_dist) {
                        if (distanceRecurse_(root1, tree2, child, child_bv, tf2, cdata, callback, min_dist))
                            return true;
                    }
                }
            }
        }

        return false;
    }

    //==============================================================================
    template <typename S, typename Derived>
    FCL_EXPORT bool distanceRecurse_(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        const OcTree<S>* tree2,
        const typename OcTree<S>::OcTreeNode* root2,
        const AABB<S>& root2_bv,
        const Eigen::MatrixBase<Derived>& translation2,
        void* cdata,
        DistanceCallBack<S> callback,
        S& min_dist)
    {
        if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
            if (tree2->isNodeOccupied(root2)) {
                Box<S>* box = new Box<S>();
                Transform3<S> box_tf;
                Transform3<S> tf2 = Transform3<S>::Identity();
                tf2.translation() = translation2;
                constructBox(root2_bv, tf2, *box, box_tf);
                CollisionObject<S> obj(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
                return callback(static_cast<CollisionObject<S>*>(root1->data), &obj, cdata, min_dist);
            } else
                return false;
        }

        if (!tree2->isNodeOccupied(root2))
            return false;

        if (!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
            const AABB<S>& aabb2 = translate(root2_bv, translation2);
            S d1 = aabb2.distance(root1->children[0]->bv);
            S d2 = aabb2.distance(root1->children[1]->bv);

            if (d2 < d1) {
                if (d2 < min_dist) {
                    if (distanceRecurse_(root1->children[1], tree2, root2, root2_bv, translation2, cdata, callback, min_dist))
                        return true;
                }

                if (d1 < min_dist) {
                    if (distanceRecurse_(root1->children[0], tree2, root2, root2_bv, translation2, cdata, callback, min_dist))
                        return true;
                }
            } else {
                if (d1 < min_dist) {
                    if (distanceRecurse_(root1->children[0], tree2, root2, root2_bv, translation2, cdata, callback, min_dist))
                        return true;
                }

                if (d2 < min_dist) {
                    if (distanceRecurse_(root1->children[1], tree2, root2, root2_bv, translation2, cdata, callback, min_dist))
                        return true;
                }
            }
        } else {
            for (unsigned int i = 0; i < 8; ++i) {
                if (tree2->nodeChildExists(root2, i)) {
                    const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
                    AABB<S> child_bv;
                    computeChildBV(root2_bv, i, child_bv);
                    const AABB<S>& aabb2 = translate(child_bv, translation2);

                    S d = root1->bv.distance(aabb2);

                    if (d < min_dist) {
                        if (distanceRecurse_(root1, tree2, child, child_bv, translation2, cdata, callback, min_dist))
                            return true;
                    }
                }
            }
        }

        return false;
    }

    //==============================================================================
    template <typename S>
    FCL_EXPORT bool distanceRecurse(typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1, const OcTree<S>* tree2, const typename OcTree<S>::OcTreeNode* root2, const AABB<S>& root2_bv, const Transform3<S>& tf2, void* cdata, DistanceCallBack<S> callback, S& min_dist)
    {
        if (tf2.linear().isIdentity())
            return distanceRecurse_(root1, tree2, root2, root2_bv, tf2.translation(), cdata, callback, min_dist);
        else
            return distanceRecurse_(root1, tree2, root2, root2_bv, tf2, cdata, callback, min_dist);
    }

    //==============================================================================
    template <typename S>
    FCL_EXPORT bool distanceRecurse(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root2,
        void* cdata,
        DistanceCallBack<S> callback,
        S& min_dist)
    {
        if (root1->isLeaf() && root2->isLeaf()) {
            CollisionObject<S>* root1_obj = static_cast<CollisionObject<S>*>(root1->data);
            CollisionObject<S>* root2_obj = static_cast<CollisionObject<S>*>(root2->data);
            return callback(root1_obj, root2_obj, cdata, min_dist);
        }

        if (root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size()))) {
            S d1 = root2->bv.distance(root1->children[0]->bv);
            S d2 = root2->bv.distance(root1->children[1]->bv);

            if (d2 < d1) {
                if (d2 < min_dist) {
                    if (distanceRecurse(root1->children[1], root2, cdata, callback, min_dist))
                        return true;
                }

                if (d1 < min_dist) {
                    if (distanceRecurse(root1->children[0], root2, cdata, callback, min_dist))
                        return true;
                }
            } else {
                if (d1 < min_dist) {
                    if (distanceRecurse(root1->children[0], root2, cdata, callback, min_dist))
                        return true;
                }

                if (d2 < min_dist) {
                    if (distanceRecurse(root1->children[1], root2, cdata, callback, min_dist))
                        return true;
                }
            }
        } else {
            S d1 = root1->bv.distance(root2->children[0]->bv);
            S d2 = root1->bv.distance(root2->children[1]->bv);

            if (d2 < d1) {
                if (d2 < min_dist) {
                    if (distanceRecurse(root1, root2->children[1], cdata, callback, min_dist))
                        return true;
                }

                if (d1 < min_dist) {
                    if (distanceRecurse(root1, root2->children[0], cdata, callback, min_dist))
                        return true;
                }
            } else {
                if (d1 < min_dist) {
                    if (distanceRecurse(root1, root2->children[0], cdata, callback, min_dist))
                        return true;
                }

                if (d2 < min_dist) {
                    if (distanceRecurse(root1, root2->children[1], cdata, callback, min_dist))
                        return true;
                }
            }
        }

        return false;
    }

    //==============================================================================
    template <typename S>
    FCL_EXPORT bool distanceRecurse(typename CollisionManagerAABBTree<S>::DynamicAABBNode* root, CollisionObject<S>* query, void* cdata, DistanceCallBack<S> callback, S& min_dist)
    {
        if (root->isLeaf()) {
            CollisionObject<S>* root_obj = static_cast<CollisionObject<S>*>(root->data);
            return callback(root_obj, query, cdata, min_dist);
        }

        S d1 = query->getAABB().distance(root->children[0]->bv);
        S d2 = query->getAABB().distance(root->children[1]->bv);

        if (d2 < d1) {
            if (d2 < min_dist) {
                if (distanceRecurse(root->children[1], query, cdata, callback, min_dist))
                    return true;
            }

            if (d1 < min_dist) {
                if (distanceRecurse(root->children[0], query, cdata, callback, min_dist))
                    return true;
            }
        } else {
            if (d1 < min_dist) {
                if (distanceRecurse(root->children[0], query, cdata, callback, min_dist))
                    return true;
            }

            if (d2 < min_dist) {
                if (distanceRecurse(root->children[1], query, cdata, callback, min_dist))
                    return true;
            }
        }

        return false;
    }

    //==============================================================================
    template <typename S>
    FCL_EXPORT bool selfDistanceRecurse(typename CollisionManagerAABBTree<S>::DynamicAABBNode* root, void* cdata, DistanceCallBack<S> callback, S& min_dist)
    {
        if (root->isLeaf())
            return false;

        if (selfDistanceRecurse(root->children[0], cdata, callback, min_dist))
            return true;

        if (selfDistanceRecurse(root->children[1], cdata, callback, min_dist))
            return true;

        if (distanceRecurse(root->children[0], root->children[1], cdata, callback, min_dist))
            return true;

        return false;
    }

} // namespace dynamic_AABB_tree

}

TMPL_PREFIX_
CollisionManagerAABBTree<S>::CollisionManagerAABBTree()
    : tree_topdown_balance_threshold(dtree.bu_threshold)
    , tree_topdown_level(dtree.topdown_level)
{
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
void CollisionManagerAABBTree<S>::registerObjects(const std::vector<fcl::CollisionObject<S>*>& other_objs)
{
    if (other_objs.empty())
        return;

    if (size() > 0) {
        CollisionManager<S>::registerObjects(other_objs);
    } else {
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
void CollisionManagerAABBTree<S>::registerObject(fcl::CollisionObject<S>* obj)
{
    DynamicAABBNode* node = dtree.insert(obj->getAABB(), obj);
    table[obj] = node;
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::unregisterObject(fcl::CollisionObject<S>* obj)
{
    DynamicAABBNode* node = table[obj];
    table.erase(obj);
    dtree.remove(node);
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::setup()
{
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
void CollisionManagerAABBTree<S>::update()
{
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
void CollisionManagerAABBTree<S>::update(fcl::CollisionObject<S>* updated_obj)
{
    update_(updated_obj);
    setup();
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::update(const std::vector<fcl::CollisionObject<S>*>& updated_objs)
{
    for (size_t i = 0, size = updated_objs.size(); i < size; ++i)
        update_(updated_objs[i]);
    setup();
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::clear()
{
    dtree.clear();
    table.clear();
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::getObjects(std::vector<fcl::CollisionObject<S>*>& objs) const
{
    objs.resize(this->size());
    std::transform(table.begin(), table.end(), objs.begin(), std::bind(&DynamicAABBTable::value_type::first, std::placeholders::_1));
}

TMPL_PREFIX_
bool CollisionManagerAABBTree<S>::empty() const
{
    return dtree.empty();
}

TMPL_PREFIX_
size_t CollisionManagerAABBTree<S>::size() const
{
    return dtree.size();
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::withInDistance(fcl::CollisionObject<S>* obj, void* cdata, fcl::DistanceCallBack<S> callback)
{
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::withInDistance(void* cdata, fcl::DistanceCallBack<S> callback)
{
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::withInDistance(CollisionManager<S>* other, void* cdata, fcl::DistanceCallBack<S> callback)
{
}

TMPL_PREFIX_
const fcl::detail::HierarchyTree<fcl::AABB<S>>& CollisionManagerAABBTree<S>::getTree() const
{
    return dtree;
}

TMPL_PREFIX_
void CollisionManagerAABBTree<S>::update_(fcl::CollisionObject<S>* updated_obj)
{
    const auto it = table.find(updated_obj);
    if (it != table.end()) {
        DynamicAABBNode* node = it->second;
        if (!node->bv.equal(updated_obj->getAABB()))
            dtree.update(node, updated_obj->getAABB());
    }
    setup_ = false;
}

template class CollisionManagerAABBTree<float>;
template class CollisionManagerAABBTree<double>;

#undef TMPL_PREFIX_