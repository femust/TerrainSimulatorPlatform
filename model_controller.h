#ifndef MODEL_PUSH_H
#define MODEL_PUSH_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include<iostream>
#include<iomanip>
#include<iterator>
#include<cmath>
#include<math.h>





namespace gazebo
{
 class ModelPush : public ModelPlugin
  {
   public: ModelPush();
   public: ~ModelPush();
   public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
   public: void OnUpdate(const common::UpdateInfo & /*_info*/); //virtual?


   private: physics::ModelPtr model;

   private: event::ConnectionPtr updateConnection;

   private: physics::Joint_V vectorofJoints;
   private: std::map<std::string, double> jointAngles;
   private: std::map<std::string, double> jointVelocity;
   private: std::map<std::string, double> jointForce;
   private: physics::Joint_V  activeJoints;

   private: common::Time simTime;
   private: common::Time realTime;

   private: std::string motorJointName01;
   private: std::string motorJointName02;
   private: std::string motorJointName03;
   private : physics::JointPtr motorJoint01;
   private: physics::JointPtr motorJoint02;
   private: physics::JointPtr motorJoint03;

   private: std::string robotName;

    private: int attempt;
  };

 }









#endif // MODEL_PUSH_H
