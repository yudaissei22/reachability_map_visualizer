#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <iostream>
#include <ros/package.h>

namespace reachability_map_visualizer_sample{
  void jaxon_hand(){
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    std::shared_ptr<reachability_map_visualizer::ReachabilityMapParam> param = std::make_shared<reachability_map_visualizer::ReachabilityMapParam>();

    cnoid::BodyLoader bodyLoader;
    param->robot = bodyLoader.load(ros::package::getPath("ironman_models") + "/IronMan_for_choreonoid.wrl");
    if(!(param->robot)) std::cerr << "!robot" << std::endl;

    param->robot->rootLink()->p() = cnoid::Vector3(0,0,0.0);
    param->robot->rootLink()->v().setZero();
    param->robot->rootLink()->R() = cnoid::Matrix3::Identity();
    param->robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    };

    for(int j=0; j < param->robot->numJoints(); ++j){
      param->robot->joint(j)->q() = reset_manip_pose[j];
    }
    param->robot->calcForwardKinematics();
    param->robot->calcCenterOfMass();
    // joint limit
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
    for(int i=0;i<param->robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = param->robot->joint(i);
      constraints.push_back(constraint);
    }
    param->constraints.push_back(constraints);
    param->variables.push_back(param->robot->joint("SHOULDER_PITCH"));
    param->variables.push_back(param->robot->joint("SHOULDER_ROLL"));
    param->variables.push_back(param->robot->joint("SHOULDER_YAW"));
    param->variables.push_back(param->robot->joint("ELBOW_PITCH"));
    param->variables.push_back(param->robot->joint("WRIST_YAW"));
    reachability_map_visualizer::EndEffector ee;
    ee.parent = param->robot->link("WRIST_YAW");
    param->endEffectors.push_back(ee);
    param->posResolution = 0.05;
    param->pikParam.maxIteration = 30;
    param->testPerGrid = 50;
    param->origin = cnoid::Vector3(0.25, -0.15, 0.0);
    param->size = cnoid::Vector3(0.6, 0.6, 0.6);
    param->weight[5] = 0.0;
    std::shared_ptr<reachability_map_visualizer::ReachabilityMap> map = std::make_shared<reachability_map_visualizer::ReachabilityMap>();
    // reachability_map_visualizer::createMap(param, map);
    reachability_map_visualizer::createMap(param, map, 4);

    reachability_map_visualizer::writeMap(ros::package::getPath("reachability_map_visualizer_ironman_upper") + "/config/ironman_upper_hand.yaml",map);
    viewer->objects(param->robot);
    reachability_map_visualizer::visualizeMap(map, viewer);
    viewer->drawObjects();
  }
}
