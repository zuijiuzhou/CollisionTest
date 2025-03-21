#include "FclCollisionDetector.h"

#include <chrono>

#include <fcl/broadphase/broadphase_SaP.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/common/types.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/utility.h>
#include <fcl/math/bv/utility.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/detail/primitive_shape_algorithm/triangle_distance.h>
#include <fcl/narrowphase/distance.h>

#include <assimp/scene.h>

#include <fcl_wrap/collision_manager_aabb_tree.h>

using BVHM = fcl::BVHModel<fcl::OBBRSS<float>>;

namespace {

using namespace fcl;

void aiProcessNode(std::shared_ptr<BVHM>& fcl_mesh, const aiScene* scene, aiNode* node)
{
    for (int i = 0; i < node->mNumMeshes; i++) {
        auto mesh = scene->mMeshes[node->mMeshes[i]];
        // if (mesh->HasPositions()) {
        //     auto vertices = new Vec3fArray();
        //     vertices->reserve(mesh->mNumVertices);
        //
        //     for (int j = 0; j < mesh->mNumVertices; j++) {
        //         vertices->push_back(Vec3f(mesh->mVertices[j].x, mesh->mVertices[j].y, mesh->mVertices[j].z));
        //         bb.expandBy(vertices->back());
        //     }
        //     geom->setVertexArray(vertices);
        // }

        if (mesh->HasFaces()) {
            for (auto j = 0; j < mesh->mNumFaces; j++) {
                auto& face = mesh->mFaces[j];
                if (face.mNumIndices == 3) {
                    auto& v1 = mesh->mVertices[face.mIndices[0]];
                    auto& v2 = mesh->mVertices[face.mIndices[1]];
                    auto& v3 = mesh->mVertices[face.mIndices[2]];

                    fcl_mesh->addTriangle({ v1.x, v1.y, v1.z }, { v2.x, v2.y, v2.z }, { v3.x, v3.y, v3.z });
                }
            }
        }
    }
    for (int i = 0; i < node->mNumChildren; i++) {
        auto child_node = node->mChildren[i];
        aiProcessNode(fcl_mesh, scene, child_node);
    }
}

template <typename S>
struct CustomCollisionData {
    fcl::CollisionRequest<S> request;
    std::vector<fcl::CollisionResult<S>> results;
};

template <typename S>
bool CustomCollisionFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, void* data)
{
    auto* collision_data = static_cast<CustomCollisionData<S>*>(data);
    const fcl::CollisionRequest<S>& request = collision_data->request;

    fcl::CollisionResult<S> result;
    fcl::collide(o1, o2, request, result);

    if (result.isCollision()) {
        collision_data->results.push_back(result);
        
    }

    return false;
}

template <typename S>
struct CustomDistanceRequest : public fcl::DistanceRequest<S> {
    S safety_dist = S(0);
    bool stop_at_first_contact = false;
};

template <typename S>
struct CustomDistanceData {
    CustomDistanceRequest<S> request;
    std::vector<fcl::DistanceResult<S>> results;
};

template <typename S>
bool CustomDistanceFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, void* data, S& dist)
{
    auto* cdata = static_cast<CustomDistanceData<S>*>(data);
    const CustomDistanceRequest<S>& request = cdata->request;
    fcl::DistanceResult<S> result;

    fcl::distance(o1, o2, request, result);

    if (result.min_distance < dist) {
        // dist = result.min_distance;
        cdata->results.push_back(result);
        if (request.stop_at_first_contact)
            return true;
    }

    return false;
}

template <typename S>
class CustomCollisionManagerSaP : public fcl::SaPCollisionManager<S> {
public:
    void withInDistance(void* cdata, DistanceCallBack<S> callback) const
    {
        if (this->size() == 0)
            return;

        auto cdd = static_cast<CustomDistanceData<S>*>(cdata);
        auto safety_dist = cdd->request.safety_dist;

        this->enable_tested_set_ = true;
        this->tested_set.clear();
        int nb = 0;
        for (auto it1 = this->AABB_arr.begin(), end = this->AABB_arr.end(); it1 != end; ++it1) {
            auto obj1 = (*it1)->obj;
            for (auto it2 = it1, end = this->AABB_arr.end(); it2 != end; ++it2) {
                auto obj2 = (*it2)->obj;
                if (obj1 == obj2)
                    continue;

                if ((*it1)->cached.distance((*it2)->cached) < safety_dist) {
                    nb++;
                    if (callback(obj1, obj2, cdata, safety_dist)) {

                        // return;
                    }
                }
            }
        }

        this->enable_tested_set_ = false;
        this->tested_set.clear();
        printf("\n NB=%i", nb);
    }
};

template <typename S>
class HBCollisionManagerAABB : public CollisionManagerAABBTree<S> {
    //==============================================================================
    template <typename S>
    FCL_EXPORT bool collisionRecurse_(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        const fcl::OcTree<S>* tree2,
        const typename OcTree<S>::OcTreeNode* root2,
        const AABB<S>& root2_bv,
        const Transform3<S>& tf2,
        void* cdata,
        CollisionCallBack<S> callback)
    {
        if (!root2) {
            if (root1->isLeaf()) {
                CollisionObject<S>* obj1 = static_cast<CollisionObject<S>*>(root1->data);

                if (!obj1->isFree()) {
                    OBB<S> obb1, obb2;
                    convertBV(root1->bv, Transform3<S>::Identity(), obb1);
                    convertBV(root2_bv, tf2, obb2);

                    if (obb1.overlap(obb2)) {
                        Box<S>* box = new Box<S>();
                        Transform3<S> box_tf;
                        constructBox(root2_bv, tf2, *box, box_tf);

                        box->cost_density = tree2->getDefaultOccupancy();

                        CollisionObject<S> obj2(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
                        return callback(obj1, &obj2, cdata);
                    }
                }
            } else {
                if (collisionRecurse_<S>(root1->children[0], tree2, nullptr, root2_bv, tf2, cdata, callback))
                    return true;
                if (collisionRecurse_<S>(root1->children[1], tree2, nullptr, root2_bv, tf2, cdata, callback))
                    return true;
            }

            return false;
        } else if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
            CollisionObject<S>* obj1 = static_cast<CollisionObject<S>*>(root1->data);

            if (!tree2->isNodeFree(root2) && !obj1->isFree()) {
                OBB<S> obb1, obb2;
                convertBV(root1->bv, Transform3<S>::Identity(), obb1);
                convertBV(root2_bv, tf2, obb2);

                if (obb1.overlap(obb2)) {
                    Box<S>* box = new Box<S>();
                    Transform3<S> box_tf;
                    constructBox(root2_bv, tf2, *box, box_tf);

                    box->cost_density = root2->getOccupancy();
                    box->threshold_occupied = tree2->getOccupancyThres();

                    CollisionObject<S> obj2(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
                    return callback(obj1, &obj2, cdata);
                } else
                    return false;
            } else
                return false;
        }

        OBB<S> obb1, obb2;
        convertBV(root1->bv, Transform3<S>::Identity(), obb1);
        convertBV(root2_bv, tf2, obb2);

        if (tree2->isNodeFree(root2) || !obb1.overlap(obb2))
            return false;

        if (!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
            if (collisionRecurse_(root1->children[0], tree2, root2, root2_bv, tf2, cdata, callback))
                return true;
            if (collisionRecurse_(root1->children[1], tree2, root2, root2_bv, tf2, cdata, callback))
                return true;
        } else {
            for (unsigned int i = 0; i < 8; ++i) {
                if (tree2->nodeChildExists(root2, i)) {
                    const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
                    AABB<S> child_bv;
                    computeChildBV(root2_bv, i, child_bv);

                    if (collisionRecurse_(root1, tree2, child, child_bv, tf2, cdata, callback))
                        return true;
                } else {
                    AABB<S> child_bv;
                    computeChildBV(root2_bv, i, child_bv);
                    if (collisionRecurse_<S>(root1, tree2, nullptr, child_bv, tf2, cdata, callback))
                        return true;
                }
            }
        }
        return false;
    }

    //==============================================================================
    template <typename S, typename Derived>
    FCL_EXPORT bool collisionRecurse_(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        const OcTree<S>* tree2,
        const typename OcTree<S>::OcTreeNode* root2,
        const AABB<S>& root2_bv,
        const Eigen::MatrixBase<Derived>& translation2,
        void* cdata,
        CollisionCallBack<S> callback)
    {
        if (!root2) {
            if (root1->isLeaf()) {
                CollisionObject<S>* obj1 = static_cast<CollisionObject<S>*>(root1->data);

                if (!obj1->isFree()) {
                    const AABB<S>& root2_bv_t = translate(root2_bv, translation2);
                    if (root1->bv.overlap(root2_bv_t)) {
                        Box<S>* box = new Box<S>();
                        Transform3<S> box_tf;
                        Transform3<S> tf2 = Transform3<S>::Identity();
                        tf2.translation() = translation2;
                        constructBox(root2_bv, tf2, *box, box_tf);

                        box->cost_density = tree2->getOccupancyThres(); // thresholds are 0, 1, so uncertain

                        CollisionObject<S> obj2(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
                        return callback(obj1, &obj2, cdata);
                    }
                }
            } else {
                if (collisionRecurse_<S>(root1->children[0], tree2, nullptr, root2_bv, translation2, cdata, callback))
                    return true;
                if (collisionRecurse_<S>(root1->children[1], tree2, nullptr, root2_bv, translation2, cdata, callback))
                    return true;
            }

            return false;
        } else if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
            CollisionObject<S>* obj1 = static_cast<CollisionObject<S>*>(root1->data);

            if (!tree2->isNodeFree(root2) && !obj1->isFree()) {
                const AABB<S>& root2_bv_t = translate(root2_bv, translation2);
                if (root1->bv.overlap(root2_bv_t)) {
                    Box<S>* box = new Box<S>();
                    Transform3<S> box_tf;
                    Transform3<S> tf2 = Transform3<S>::Identity();
                    tf2.translation() = translation2;
                    constructBox(root2_bv, tf2, *box, box_tf);

                    box->cost_density = root2->getOccupancy();
                    box->threshold_occupied = tree2->getOccupancyThres();

                    CollisionObject<S> obj2(std::shared_ptr<CollisionGeometry<S>>(box), box_tf);
                    return callback(obj1, &obj2, cdata);
                } else
                    return false;
            } else
                return false;
        }

        const AABB<S>& root2_bv_t = translate(root2_bv, translation2);
        if (tree2->isNodeFree(root2) || !root1->bv.overlap(root2_bv_t))
            return false;

        if (!tree2->nodeHasChildren(root2) || (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
            if (collisionRecurse_(root1->children[0], tree2, root2, root2_bv, translation2, cdata, callback))
                return true;
            if (collisionRecurse_(root1->children[1], tree2, root2, root2_bv, translation2, cdata, callback))
                return true;
        } else {
            for (unsigned int i = 0; i < 8; ++i) {
                if (tree2->nodeChildExists(root2, i)) {
                    const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
                    AABB<S> child_bv;
                    computeChildBV(root2_bv, i, child_bv);

                    if (collisionRecurse_(root1, tree2, child, child_bv, translation2, cdata, callback))
                        return true;
                } else {
                    AABB<S> child_bv;
                    computeChildBV(root2_bv, i, child_bv);
                    if (collisionRecurse_<S>(root1, tree2, nullptr, child_bv, translation2, cdata, callback))
                        return true;
                }
            }
        }
        return false;
    }

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
    template <typename S>
    FCL_EXPORT bool collisionRecurse(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        const OcTree<S>* tree2,
        const typename OcTree<S>::OcTreeNode* root2,
        const AABB<S>& root2_bv,
        const Transform3<S>& tf2,
        void* cdata,
        CollisionCallBack<S> callback)
    {
        if (tf2.linear().isIdentity())
            return collisionRecurse_(root1, tree2, root2, root2_bv, tf2.translation(), cdata, callback);
        else // has rotation
            return collisionRecurse_(root1, tree2, root2, root2_bv, tf2, cdata, callback);
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
    FCL_EXPORT bool collisionRecurse(
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root1,
        typename CollisionManagerAABBTree<S>::DynamicAABBNode* root2,
        void* cdata,
        CollisionCallBack<S> callback)
    {
        if (root1->isLeaf() && root2->isLeaf()) {
            if (!root1->bv.overlap(root2->bv))
                return false;
            return callback(static_cast<CollisionObject<S>*>(root1->data), static_cast<CollisionObject<S>*>(root2->data), cdata);
        }

        if (!root1->bv.overlap(root2->bv))
            return false;

        if (root2->isLeaf() || (!root1->isLeaf() && (root1->bv.size() > root2->bv.size()))) {
            if (collisionRecurse(root1->children[0], root2, cdata, callback))
                return true;
            if (collisionRecurse(root1->children[1], root2, cdata, callback))
                return true;
        } else {
            if (collisionRecurse(root1, root2->children[0], cdata, callback))
                return true;
            if (collisionRecurse(root1, root2->children[1], cdata, callback))
                return true;
        }
        return false;
    }

    //==============================================================================
    template <typename S>
    FCL_EXPORT bool collisionRecurse(typename CollisionManagerAABBTree<S>::DynamicAABBNode* root, CollisionObject<S>* query, void* cdata, CollisionCallBack<S> callback)
    {
        if (root->isLeaf()) {
            if (!root->bv.overlap(query->getAABB()))
                return false;
            return callback(static_cast<CollisionObject<S>*>(root->data), query, cdata);
        }

        if (!root->bv.overlap(query->getAABB()))
            return false;

        int select_res = select(query->getAABB(), *(root->children[0]), *(root->children[1]));

        if (collisionRecurse(root->children[select_res], query, cdata, callback))
            return true;

        if (collisionRecurse(root->children[1 - select_res], query, cdata, callback))
            return true;

        return false;
    }

    //==============================================================================
    template <typename S>
    FCL_EXPORT bool selfCollisionRecurse(typename CollisionManagerAABBTree<S>::DynamicAABBNode* root, void* cdata, CollisionCallBack<S> callback)
    {
        if (root->isLeaf())
            return false;

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

public:
    void collide(void* cdata, CollisionCallBack<S> callback){
        if (this->size() == 0)
            return;
        detail::dynamic_AABB_tree::selfCollisionRecurse(this->getTree().getRoot(), cdata, callback);
    }

    void withInDistance(void* cdata, DistanceCallBack<S> callback)
    {
        if (this->size() == 0)
            return;

        auto cdd = static_cast<CustomDistanceData<S>*>(cdata);
        auto safety_dist = cdd->request.safety_dist;

        selfDistanceRecurse(this->getTree().getRoot(), cdata, callback, safety_dist);
    }
};

}

using Impl = HBCollisionManagerAABB<float>;

FclCollisionDetector::FclCollisionDetector()
{
    cm_impl_ = new Impl();
}

FclCollisionDetector::~FclCollisionDetector()
{
    delete cm_impl_;
}

void FclCollisionDetector::addGeometry(const std::string& name, const aiScene* scene, const Eigen::Isometry3d& t)
{
    auto impl = static_cast<Impl*>(cm_impl_);

    auto geom = std::make_shared<BVHM>();

    geom->beginModel();
    aiProcessNode(geom, scene, scene->mRootNode);
    geom->endModel();

    auto obj = new fcl::CollisionObjectf(geom, t.cast<float>());
    obj_userdata_map_.insert({ geom.get(), CollisionObjectUserData { name } });

    impl->registerObject(obj);
}

bool FclCollisionDetector::collide()
{
    auto impl = static_cast<Impl*>(cm_impl_);

    impl->update();

    CustomCollisionData<float> cd;
    // cd.request.num_max_contacts = 1;
    cd.request.enable_cost = false;
    cd.request.enable_cached_gjk_guess = true;
    cd.request.gjk_solver_type = fcl::GST_LIBCCD;
    cd.request.gjk_tolerance = 0.01;

    impl->collide(&cd, CustomCollisionFunction<float>);

    return true;
}

bool FclCollisionDetector::withInDistance(float safety_dist)
{
    auto impl = static_cast<Impl*>(cm_impl_);

    impl->update();

    CustomDistanceData<float> dd;
    dd.request.safety_dist = safety_dist;
    dd.request.enable_signed_distance = false;
    dd.request.enable_nearest_points = false;
    dd.request.distance_tolerance = 0.01;
    dd.request.gjk_solver_type = fcl::GST_LIBCCD;

    auto t1 = std::chrono::high_resolution_clock::now();
    impl->withInDistance(&dd, CustomDistanceFunction<float>);
    auto t2 = std::chrono::high_resolution_clock::now();

    printf("\nFCL::%f ms", (t2 - t1).count() / 1000000.);
    printf("\nFCL Result:");

    for (auto& res : dd.results) {
        printf("\n%s---%s", obj_userdata_map_.at(res.o1).name.data(), obj_userdata_map_.at(res.o2).name.data());
    }
    printf("\n\n");
    return false;
}
