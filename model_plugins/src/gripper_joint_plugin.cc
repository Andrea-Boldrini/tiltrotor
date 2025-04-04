#include <gazebo/gazebo.hh>                
#include <gazebo/common/common.hh>        
#include <gazebo/physics/physics.hh>       
#include <ignition/math/Vector3.hh>        
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>

namespace gazebo {
class GripperJointPlugin : public ModelPlugin {
    
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    this->model = _model;

    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gripper_joint_plugin", ros::init_options::NoSigintHandler); 
    }

    this->rosNode.reset(new ros::NodeHandle("gripper_joint_plugin"));

    if (_sdf->HasElement("leftFingerJoint")) {
      this->leftFingerJoint = _sdf->Get<std::string>("leftFingerJoint");
    } else {
      gzerr << "No <leftFingerJoint> parameter in SDF." << std::endl;
      return;
    }
    if (_sdf->HasElement("rightFingerJoint")) {
      this->rightFingerJoint = _sdf->Get<std::string>("rightFingerJoint");
    } else {
      gzerr << "No <rightFingerJoint> parameter in SDF." << std::endl;
      return;
    }
    if (_sdf->HasElement("leftFingertipJoint")) {
      this->leftFingertipJoint = _sdf->Get<std::string>("leftFingertipJoint");
    } else {
      gzerr << "No <leftFingertipJoint> parameter in SDF." << std::endl;
      return;
    }
    if (_sdf->HasElement("rightFingertipJoint")) {
      this->rightFingertipJoint = _sdf->Get<std::string>("rightFingertipJoint");
    } else {
      gzerr << "No <rightFingertipJoint> parameter in SDF." << std::endl;
      return;
    }

    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64>(
        "/gripper_joint_position_cmd", 1,
        boost::bind(&GripperJointPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    this->rosSub = this->rosNode->subscribe(so);

    this->rosQueueThread = std::thread(std::bind(&GripperJointPlugin::QueueThread, this)); 

    this->jointName1 = this->model->GetJoint(this->leftFingerJoint);
    this->jointName2 = this->model->GetJoint(this->rightFingerJoint);
    this->jointName3 = this->model->GetJoint(this->leftFingertipJoint);
    this->jointName4 = this->model->GetJoint(this->rightFingertipJoint);

    double p_gain = _sdf->HasElement("p_gain") ? _sdf->Get<double>("p_gain") : 1.0;
    double i_gain = _sdf->HasElement("i_gain") ? _sdf->Get<double>("i_gain") : 0.1;
    double d_gain = _sdf->HasElement("d_gain") ? _sdf->Get<double>("d_gain") : 0.01;

    common::PID pid(p_gain, i_gain, d_gain); 
    this->jointController = this->model->GetJointController();
    this->jointController->SetPositionPID(this->jointName1->GetScopedName(), pid);
    this->jointController->SetPositionPID(this->jointName2->GetScopedName(), pid);
    this->jointController->SetPositionPID(this->jointName3->GetScopedName(), pid);
    this->jointController->SetPositionPID(this->jointName4->GetScopedName(), pid);
    
    this->SetJointPosition(0.0);

    std::cout << "[GripperJointPlugin] Loaded " << std::endl;
  }
  
  void OnRosMsg(const std_msgs::Float64ConstPtr &msg) {
    double position = msg->data;
    this->SetJointPosition(position);
  }

  void SetJointPosition(double position) {
    this->jointController->SetPositionTarget(this->jointName1->GetScopedName(), -position);
    this->jointController->SetPositionTarget(this->jointName2->GetScopedName(), position);
    this->jointController->SetPositionTarget(this->jointName3->GetScopedName(), -position);
    this->jointController->SetPositionTarget(this->jointName4->GetScopedName(), position);
  }

  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

// data members
private:
  physics::ModelPtr model;  
  physics::JointPtr jointName1, jointName2, jointName3, jointName4;
  physics::JointControllerPtr jointController;    
  std::unique_ptr<ros::NodeHandle> rosNode;       
  ros::Subscriber rosSub;                        
  ros::CallbackQueue rosQueue;                    
  std::thread rosQueueThread;                     
  std::string leftFingerJoint, rightFingerJoint, leftFingertipJoint, rightFingertipJoint;
};

GZ_REGISTER_MODEL_PLUGIN(GripperJointPlugin)
} 
