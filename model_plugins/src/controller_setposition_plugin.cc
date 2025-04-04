#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
//#include <mavros_msgs/DebugValue.h>
#include <std_msgs/Float64.h>

namespace gazebo {
class ControllerSetPositionPlugin : public ModelPlugin {  
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;

    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "controller_setposition_plugin", ros::init_options::NoSigintHandler); 
    }

    this->rosNode.reset(new ros::NodeHandle("controller_setposition_plugin"));

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
        "/tiltrotor/angle_setpoint", 1,
        boost::bind(&ControllerSetPositionPlugin::OnTiltAngleMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    std::cout << "[ControllerSetPositionPlugin] Subscribing to topic 'tiltrotor/angle_setpoint'." << std::endl;
    this->rosSub = this->rosNode->subscribe(so);
    this->rosQueueThread = std::thread(std::bind(&ControllerSetPositionPlugin::QueueThread, this));

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

    std::cout << "[ControllerSetPositionPlugin] Loaded " << std::endl;
  }

  void OnTiltAngleMsg(const std_msgs::Float64ConstPtr &msg) {
      double tilt_angle = msg->data; 

      this->SetJointPosition(tilt_angle);
      
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
  ros::CallbackQueue rosQueue;                    
  std::thread rosQueueThread;                     

  std::string jointName1, jointName2;            
};


GZ_REGISTER_MODEL_PLUGIN(ControllerSetPositionPlugin) 
}  
