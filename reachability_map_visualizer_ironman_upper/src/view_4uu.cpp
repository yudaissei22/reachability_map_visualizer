#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <iostream>
#include <ros/package.h>
#include <thread>

namespace reachability_map_visualizer_sample{
  void view_4uu(){
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();

    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(ros::package::getPath("ironman_models") + "/IronMan_for_choreonoid_ver2.wrl");
    if(!robot){
      std::cerr << "[view_4uu] failed to load robot model" << std::endl;
      return;
    }

    cnoid::LinkPtr b4Base = robot->joint("B4_BASE"); // z
    cnoid::LinkPtr b4Uu = robot->joint("B4_UU"); // x
    cnoid::LinkPtr shoulderFirst = robot->joint("SHOULDER_FIRST"); // z
    cnoid::LinkPtr b4EndYaw = robot->joint("B4_END_YAW"); // x
    if(!b4Base || !b4Uu || !shoulderFirst || !b4EndYaw){
      std::cerr << "[view_4uu] failed to find required joints" << std::endl;
      return;
    }

    robot->rootLink()->p().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    for(int j=0; j<robot->numJoints(); ++j){
      robot->joint(j)->q() = 0.0;
    }
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();

    viewer->objects(robot);
    viewer->drawObjects();

    std::cerr << "[view_4uu] watching 4UU sign with a = -b" << std::endl;
    std::cerr << "[view_4uu] phase 1: B4_BASE = t, SHOULDER_FIRST = -t" << std::endl;
    std::cerr << "[view_4uu] phase 2: B4_UU = t, B4_END_YAW = -t" << std::endl;

    const double step = 0.01;
    while(true){
      for(double q=0.0; q<=0.25; q+=step){
        b4Base->q() = q;
        shoulderFirst->q() = -q;
        robot->calcForwardKinematics();
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
      }
      for(double q=0.25; q>=-0.15; q-=step){
        b4Base->q() = q;
        shoulderFirst->q() = -q;
        robot->calcForwardKinematics();
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
      }
      for(double q=-0.15; q<=0.0; q+=step){
        b4Base->q() = q;
        shoulderFirst->q() = -q;
        robot->calcForwardKinematics();
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(400));

      for(double q=0.0; q<=0.25; q+=step){
        b4Uu->q() = q;
        b4EndYaw->q() = -q;
        robot->calcForwardKinematics();
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
      }
      for(double q=0.25; q>=-0.25; q-=step){
        b4Uu->q() = q;
        b4EndYaw->q() = -q;
        robot->calcForwardKinematics();
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
      }
      for(double q=-0.25; q<=0.0; q+=step){
        b4Uu->q() = q;
        b4EndYaw->q() = -q;
        robot->calcForwardKinematics();
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(800));
    }
  }
}
