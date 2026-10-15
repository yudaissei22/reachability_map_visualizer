#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <iostream>
#include <ros/package.h>

namespace reachability_map_visualizer_sample{
  // void visualize(){
  std::shared_ptr<choreonoid_viewer::Viewer> visualize() {
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    cnoid::BodyLoader bodyLoader;
    // cnoid::BodyPtr robot = bodyLoader.load(ros::package::getPath("UronMan_models") + "/UronManmain_with_human.wrl");
    cnoid::BodyPtr robot = bodyLoader.load(ros::package::getPath("UronMan_models") + "/EasyHumanmain.wrl");
    if(!robot) std::cerr << "!robot" << std::endl;

    robot->rootLink()->p() = cnoid::Vector3(0,0,0.0);
    robot->rootLink()->v().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,
    };

    for(int j=0; j < robot->numJoints(); ++j){
      robot->joint(j)->q() = reset_manip_pose[j];
    }
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();
    std::shared_ptr<reachability_map_visualizer::ReachabilityMap> map = std::make_shared<reachability_map_visualizer::ReachabilityMap>();

    // ここに平面を出すときは、このyamlから読み取るときにif文で、x=?みたいな感じでやるとよい。
    reachability_map_visualizer::readMap(ros::package::getPath("reachability_map_visualizer_ironman_upper") + "/config/easy_human_upper.yaml",map);
    map->boxSize = cnoid::Vector3(map->posResolution, map->posResolution, 0.001);
    viewer->objects(robot);
    reachability_map_visualizer::visualizeMap(map, viewer);
    // viewer->drawObjects();
    viewer->drawObjects(false);
    return viewer;
  }
}
