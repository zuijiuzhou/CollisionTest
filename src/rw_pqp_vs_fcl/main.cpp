#include <chrono>
#include <iostream>
#include <map>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include "FclCollisionDetector.h"
#include "RobWorkCollisionDetector.h"

int main(int argc, char** argv)
{
    namespace ai = Assimp;

    std::map<std::string, std::string> files = {
        { "ROB1-0", "r:/models/rob1/base.stl" },
        { "ROB1-1", "r:/models/rob1/joint1.stl" },
        { "ROB1-2", "r:/models/rob1/joint2.stl" },
        { "ROB1-3", "r:/models/rob1/joint3.stl" },
        { "ROB1-4", "r:/models/rob1/joint4.stl" },
        { "ROB1-5", "r:/models/rob1/joint5.stl" },
        { "ROB1-6", "r:/models/rob1/joint6.stl" },

        { "ROB2-0", "r:/models/rob2/base.stl" },
        { "ROB2-1", "r:/models/rob2/joint1.stl" },
        { "ROB2-2", "r:/models/rob2/joint2.stl" },
        { "ROB2-3", "r:/models/rob2/joint3.stl" },
        { "ROB2-4", "r:/models/rob2/joint4.stl" },
        { "ROB2-5", "r:/models/rob2/joint5.stl" },
        { "ROB2-6", "r:/models/rob2/joint6.stl" },
    };

    //std::map<std::string, std::string> files = {
    //        { "OBJ1", "r:/models/box.stl" },
    //        { "OBJ2", "r:/models/2030023074779061.stl" },
    //};



    RobWorkCollisionDetector rw_detector;
    FclCollisionDetector fcl_detector;

    Eigen::Isometry3d tr1, tr2;
    tr1.setIdentity();
    tr1.translate(Eigen::Vector3d(0, 0, 100));
    //tr1.rotate(Eigen::AngleAxisd(15 * 3.14 / 180, Eigen::Vector3d::UnitZ()));
    tr2.setIdentity();

    int file_index = 0;

    for (auto& kv : files) {
        ai::Importer im;
        auto scene = im.ReadFile(kv.second, aiProcess_CalcTangentSpace | aiProcess_Triangulate);
        if (scene) {
            rw_detector.addGeometry(kv.first, scene, file_index > 6 ? tr2 : tr1);
            fcl_detector.addGeometry(kv.first, scene, file_index > 6 ? tr2 : tr1);
            ++file_index;
        }
    }

    rw_detector.withInDistance(0);
    fcl_detector.withInDistance(0);

    system("pause");
    return 0;
}