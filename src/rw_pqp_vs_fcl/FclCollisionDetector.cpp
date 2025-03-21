#include "FclCollisionDetector.h"

#include <chrono>

#include <fcl/geometry/bvh/BVH_model.h>

#include <assimp/scene.h>

#include <fcl_wrap/collision_manager_aabb_tree.h>
#include <fcl_wrap/collision_manager_sap.h>

using BVHM = fcl::BVHModel<fcl::OBBRSS<float>>;

using namespace hyperbrain::robot;

namespace {

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

}

using Impl = CollisionManagerAABBTree<float>;

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

    CollisionData<float> cd;
    // cd.request.num_max_contacts = 1;
    cd.request.enable_cost = false;
    cd.request.enable_cached_gjk_guess = true;
    cd.request.gjk_solver_type = fcl::GST_LIBCCD;
    cd.request.gjk_tolerance = 0.01;

    auto t1 = std::chrono::high_resolution_clock::now();
    impl->collide(cd, DefaultCollisionFunction<float>);
    auto t2 = std::chrono::high_resolution_clock::now();

    printf("\nFCL::%f ms", (t2 - t1).count() / 1000000.);
    printf("\nFCL Result:");

    for (auto& res : cd.results) {
        printf("\n%s---%s", obj_userdata_map_.at(res.o1->getCollisionGeometry()).name.data(), obj_userdata_map_.at(res.o2->getCollisionGeometry()).name.data());
    }
    printf("\n\n");

    return true;
}

bool FclCollisionDetector::withInDistance(float safety_dist)
{
    auto impl = static_cast<Impl*>(cm_impl_);

    impl->update();

    WithInDistanceData<float> dd;
    dd.request.safety_dist = safety_dist;
    dd.request.enable_signed_distance = false;
    dd.request.enable_nearest_points = false;
    dd.request.distance_tolerance = 0.01;
    dd.request.gjk_solver_type = fcl::GST_LIBCCD;

    auto t1 = std::chrono::high_resolution_clock::now();
    impl->withInDistance(dd, DefaultWithInDistanceFunction<float>);
    auto t2 = std::chrono::high_resolution_clock::now();

    printf("\nFCL::%f ms", (t2 - t1).count() / 1000000.);
    printf("\nFCL Result:");

    for (auto& res : dd.results) {
        printf("\n%s---%s", obj_userdata_map_.at(res.o1).name.data(), obj_userdata_map_.at(res.o2).name.data());
    }
    printf("\n\n");
    return false;
}
