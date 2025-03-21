#include "RobWorkCollisionDetector.h"

#include <chrono>

#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/loaders/Model3DFactory.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rw/proximity/CollisionSetup.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <assimp/scene.h>

namespace {

void aiProcessNode(rw::geometry::PlainTriMeshN1F::Ptr rw_mesh, const aiScene* scene, aiNode* node)
{
    for (int i = 0; i < node->mNumMeshes; i++) {
        auto mesh = scene->mMeshes[node->mMeshes[i]];
        if (mesh->HasFaces()) {
            for (auto j = 0; j < mesh->mNumFaces; j++) {
                auto& face = mesh->mFaces[j];
                if (face.mNumIndices == 3) {
                    auto& v1 = mesh->mVertices[face.mIndices[0]];
                    auto& v2 = mesh->mVertices[face.mIndices[1]];
                    auto& v3 = mesh->mVertices[face.mIndices[2]];
                    rw::geometry::TriangleN1 tri(rw::math::Vector3Df(v1.x, v1.y, v1.z), rw::math::Vector3Df(v2.x, v2.y, v2.z), rw::math::Vector3Df(v3.x, v3.y, v3.z));
                    rw_mesh->add(tri);
                }
            }
        }
    }
    for (int i = 0; i < node->mNumChildren; i++) {
        auto child_node = node->mChildren[i];
        aiProcessNode(rw_mesh, scene, child_node);
    }
}

class CustomRwCollisionStrategyPQP : public rwlibs::proximitystrategies::ProximityStrategyPQP {
    /**
     * @brief Checks to see if two proximity models are in collision
     * @param a [in] model 1
     * @param wTa [in] transform of model a
     * @param b [in] model 2
     * @param wTb [in] transform of model b
     * @param data [in/out] caching and result container
     * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
     * colliding, false otherwise.
     */
    virtual bool doInCollision(rw::proximity::ProximityModel::Ptr a,
        const rw::math::Transform3D<double>& wTa,
        rw::proximity::ProximityModel::Ptr b,
        const rw::math::Transform3D<double>& wTb,
        rw::proximity::ProximityStrategyData& data) override
    {
        if (is_distance_mode_)
            return doIsWithinDistance(a, wTa, b, wTb, safety_dist_, data);
        else
            return rwlibs::proximitystrategies::ProximityStrategyPQP ::doInCollision(a, wTa, b, wTb, data);
    }

public:
    double safety_dist_ = 10;
    bool is_distance_mode_ = false;
};

class CustomRwCollisionDetector : public rw::proximity::CollisionDetector {
public:
    // bool isWithinDistance(const rw::kinematics::Frame* a, const rw::math::Transform3D<double>& wTa,
    //     const rw::kinematics::Frame* b, const rw::math::Transform3D<double>& wTb,
    //     double distance, class rw::proximity::ProximityStrategyData& data)
    //{
    //     auto stragety = dynamic_cast<rw::proximity::CollisionToleranceStrategy*>(getCollisionStrategy().get());
    //     return stragety->isWithinDistance(a, wTa, b, wTb, distance, data);
    // }
    CustomRwCollisionDetector(rw::core::Ptr<rw::models::WorkCell> workcell,
        rw::core::Ptr<rw::proximity::CollisionStrategy> strategy, rw::core::Ptr<rw::proximity::ProximityFilterStrategy> filter)
        : rw::proximity::CollisionDetector(workcell, strategy, filter)
    {
    }

protected:
};

}

RobWorkCollisionDetector::RobWorkCollisionDetector()
{
    work_cell_ = rw::core::ownedPtr(new rw::models::WorkCell("WorkCell for RobWorkCollisionDetector"));
    // work_cell_->getWorldFrame()->getPropertyMap().set("ProximitySetup", new rw::proximity::CollisionSetup({}, {}, false));

    auto filter = rw::core::ownedPtr(new rw::proximity::BasicFilterStrategy(work_cell_, rw::proximity::CollisionSetup({}, {}, false)));

    strategy_ = rw::core::ownedPtr(new CustomRwCollisionStrategyPQP());
    detector_ = rw::core::ownedPtr(new CustomRwCollisionDetector(work_cell_, strategy_, filter));

    auto setup = work_cell_->getCollisionSetup();
}

void RobWorkCollisionDetector::addGeometry(const std::string& name, const aiScene* scene, const Eigen::Isometry3d& t)
{
    rw::math::Transform3Dd rw_t;

    auto translation = t.translation();
    auto rotation = t.rotation();
    auto rx = rotation.col(0);
    auto ry = rotation.col(1);
    auto rz = rotation.col(2);

    rw_t.P() = rw::math::Vector3Dd(translation.x(), translation.y(), translation.z());
    rw_t.R() = rw::math::Rotation3Dd(rw::math::Vector3Dd(rx.x(), rx.y(), rx.z()), rw::math::Vector3Dd(ry.x(), ry.y(), ry.z()), rw::math::Vector3Dd(rz.x(), rz.y(), rz.z()));

    auto frame = rw::core::ownedPtr(new rw::kinematics::FixedFrame(name, rw_t));
    work_cell_->addFrame(frame);

    auto geom_data = rw::core::ownedPtr(new rw::geometry::PlainTriMeshN1F());
    aiProcessNode(geom_data, scene, scene->mRootNode);

    auto geom = rw::core::ownedPtr(new rw::geometry::Geometry(geom_data, name));
    geom->setFrame(frame.get());
    detector_->addGeometry(frame.get(), geom);
}

bool RobWorkCollisionDetector::collide()
{
    auto strategy = dynamic_cast<CustomRwCollisionStrategyPQP*>(strategy_.get());
    strategy->is_distance_mode_ = false;

    rw::proximity::CollisionDetector::QueryResult result;
    bool status = detector_->inCollision(work_cell_->getDefaultState(), &result, true);
    return status;
}

bool RobWorkCollisionDetector::withInDistance(float safety_dist)
{
    auto strategy = dynamic_cast<CustomRwCollisionStrategyPQP*>(strategy_.get());
    strategy->is_distance_mode_ = true;
    strategy->safety_dist_ = safety_dist;

    rw::proximity::CollisionDetector::QueryResult result;

    auto t1 = std::chrono::high_resolution_clock::now();
    bool status = detector_->inCollision(work_cell_->getDefaultState(), &result, false);
    auto t2 = std::chrono::high_resolution_clock::now();

    printf("\nRW::%f ms", (t2 - t1).count() / 1000000.);
    printf("\nRW Result:");
    for (auto& kv : result.collidingFrames) {
        printf("\n%s---%s", kv.first->getName().c_str(), kv.second->getName().c_str());
    }
    printf("\n\n");

    return status;
}
