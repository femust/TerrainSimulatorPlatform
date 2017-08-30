#include "model_controller.h"

namespace gazebo
{
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)

   ModelPush::ModelPush()
    {

this->motorJointName01="base_slider1";
this->motorJointName02="base_slider2";
this->motorJointName03="base_slider3";


    }



   ModelPush::~ModelPush()
   {
event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
   }

    void ModelPush::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

      this->model = _parent; // Store the pointer to the model

     this->motorJoint01=this->model->GetJoint(this->motorJointName01);
      this->motorJoint02=this->model->GetJoint(this->motorJointName02);
      this->motorJoint03=this->model->GetJoint(this->motorJointName03);



     if (!this->motorJoint01 || !this->motorJoint02 || !this->motorJoint03)
        {
          gzerr << this->motorJointName01 << std::setw(20) << this->motorJointName02 << std::setw(20) << this->motorJointName03 << std::setw(20) << " some of them weren't found\n";
          return;
        }

     this->activeJoints.push_back(motorJoint01);
     this->activeJoints.push_back(motorJoint02);
     this->activeJoints.push_back(motorJoint03);



      this->vectorofJoints =this->model->GetJoints();

       this->motorJoint01->SetAngle(0,0);
     this->motorJoint02->SetAngle(0,0);
      this->motorJoint03->SetAngle(0,0);




      this->updateConnection = event::Events::ConnectWorldUpdateBegin(  // Listen to the update event. This event is broadcast every
          boost::bind(&ModelPush::OnUpdate, this, _1));                  // simulation iteration.

    }

    // Called by the world update start event
    void ModelPush::OnUpdate(const common::UpdateInfo & /*_info*/)
    {

      simTime = this->model->GetWorld()->GetSimTime();
      realTime = this->model->GetWorld()->GetRealTime();

     // std::cout<<"Sim Time: "<< simTime.Double()<<"Real Time: "<<realTime.Double() <<std::endl;


      /*
     for(physics::Joint_V::iterator jit=vectorofJoints.begin(); jit!=vectorofJoints.end(); ++jit)
      {



      std::cout<<"name:"<<std::setw(40)<<(*jit)->GetName()<<" "<<(*jit)->GetAngle(0)<<" "<<(*jit)->GetVelocity(0)<<" " << std::endl;

     std::cout << "name: " << std::setw(40) << std::left << (*jit)->GetName() << "angle: " << std::setw(10) << std::left << (*jit)->GetAngle(0)<<"velocity: "<< std::setw(10) << std::left << (*jit)->GetVelocity(0)<<std::endl;// << setw(4) << hourlyRate << endl;

        std::cout << "name: ";
        std::cout<<(*jit)->GetName();
        std::cout.width(40);
        std::cout<<(*jit)->GetAngle(0);
        std::cout.width(50);
        std::cout<<(*jit)->GetVelocity(0)<<std::endl;

       jointAngles[(*jit)->GetName()] = (*jit)->GetAngle(0).Radian();
       jointVelocity[(*jit)->GetName()] = (*jit)->GetVelocity(0);
       jointForce[(*jit)->GetName()] = (*jit)->GetForce(0);

      }
*/

 this->motorJoint01->SetVelocity(1,0.01);
       this->motorJoint02->SetVelocity(1,-0.01);

     this->motorJoint03->SetVelocity(1,0.01);




      for(physics::Joint_V::iterator jit=activeJoints.begin(); jit!=activeJoints.end(); ++jit)
      {
       std::cout << "name: " << std::setw(40) << std::left << (*jit)->GetName() << "angle: " << std::setw(15) << std::left << (*jit)->GetAngle(0)<<"velocity: "<< std::setw(15) << std::left << (*jit)->GetVelocity(0)<<std::endl;// << setw(4) << hourlyRate << endl;
      std::cout << "udalo sie";
      }






    }





}
