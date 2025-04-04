#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <ros/callback_queue.h>

namespace gazebo {
class JointSetPositionPlugin : public ModelPlugin {  
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;

    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "joint_setposition_plugin", ros::init_options::NoSigintHandler); 
    }

    this->rosNode.reset(new ros::NodeHandle("joint_setposition_plugin"));

    if (_sdf->HasElement("joint_name_1")) {
      this->jointName1 = _sdf->Get<std::string>("joint_name_1");
    } else {
      gzerr << "No <joint_name_1> parameter in SDF. Please specify the first joint name." << std::endl;
      return;
    }

    if (_sdf->HasElement("joint_name_2")) {
      this->jointName2 = _sdf->Get<std::string>("joint_name_2");
    }

    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64>(
        "/joint_position_cmd", 1,
        boost::bind(&JointSetPositionPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    this->rosSub = this->rosNode->subscribe(so);

    this->rosPub = this->rosNode->advertise<std_msgs::Float32>("/gripper_state", 1);

    this->rosQueueThread = std::thread(std::bind(&JointSetPositionPlugin::QueueThread, this)); 

    this->servoJoint1 = this->model->GetJoint(this->jointName1);

    if (!this->servoJoint1) {
      gzerr << "First joint could not be found. Check the joint name in SDF." << std::endl;
      return;
    }

    if (!this->jointName2.empty()) {
      this->servoJoint2 = this->model->GetJoint(this->jointName2);
      if (!this->servoJoint2) {
        gzerr << "Second joint could not be found. Check the joint name in SDF." << std::endl;
      }
    }

    double p_gain = _sdf->HasElement("p_gain") ? _sdf->Get<double>("p_gain") : 1.0;
    double i_gain = _sdf->HasElement("i_gain") ? _sdf->Get<double>("i_gain") : 0.1;
    double d_gain = _sdf->HasElement("d_gain") ? _sdf->Get<double>("d_gain") : 0.01;

    common::PID pid(p_gain, i_gain, d_gain); 
    this->jointController = this->model->GetJointController();
    this->jointController->SetPositionPID(this->servoJoint1->GetScopedName(), pid);

    if (this->servoJoint2) {
      this->jointController->SetPositionPID(this->servoJoint2->GetScopedName(), pid);
    }

    this->SetJointPosition(0.0);

    std::cout << "[JointSetPositionPlugin] Loaded " << std::endl;
  }

  void OnRosMsg(const std_msgs::Float64ConstPtr &msg) {
    double position = msg->data;
    this->SetJointPosition(position);

    std_msgs::Float32 gripper_state_msg;
    if (position > 1.0 || position < -1.0) {
      gripper_state_msg.data = 1;
    }
    else {gripper_state_msg.data = 0;}
    this->rosPub.publish(gripper_state_msg);
  }

  void SetJointPosition(double position) {
    this->jointController->SetPositionTarget(this->servoJoint1->GetScopedName(), position);
    
    if (this->servoJoint2) {
      this->jointController->SetPositionTarget(this->servoJoint2->GetScopedName(), position);
    }
  }

  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

private:
  physics::ModelPtr model;
  physics::JointPtr servoJoint1, servoJoint2;    
  physics::JointControllerPtr jointController;    
  std::unique_ptr<ros::NodeHandle> rosNode;       
  ros::Subscriber rosSub;                         
  ros::Publisher rosPub; 
  ros::CallbackQueue rosQueue;                    
  std::thread rosQueueThread;                     
  std::string jointName1, jointName2;             
};

GZ_REGISTER_MODEL_PLUGIN(JointSetPositionPlugin) 
}  
