#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {
class TorsionalSpringPlugin : public ModelPlugin {
public: TorsionalSpringPlugin() {}

private:
	physics::ModelPtr model;
	sdf::ElementPtr sdf;
	
	physics::JointPtr joint1;
	physics::JointPtr joint2;
	physics::JointPtr joint3;
	physics::JointPtr joint4;

	double setPoint1;
	double setPoint2;
	double setPoint3;
	double setPoint4;
	double kx;
	
	event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	if (_model->GetJointCount() == 0)
	{
		std::cerr << "There are no joints. Plugin not loaded." << std::endl;
		return;
	}

	this->model = _model;

	this->sdf = _sdf;

	if (_sdf->HasElement("joint1"))
		this->joint1 =  _model->GetJoint(_sdf->Get<std::string>("joint1"));
	else
		std::cerr << "Joint not specified.\n";
	if (_sdf->HasElement("joint2"))
		this->joint2 =  _model->GetJoint(_sdf->Get<std::string>("joint2"));
	else
		std::cerr << "Joint not specified.\n";
	if (_sdf->HasElement("joint3"))
		this->joint3 =  _model->GetJoint(_sdf->Get<std::string>("joint3"));
	else
		std::cerr << "Joint not specified.\n";
	if (_sdf->HasElement("joint4"))
		this->joint4 =  _model->GetJoint(_sdf->Get<std::string>("joint4"));
	else
		std::cerr << "Joint not specified.\n";

	this->kx = 0.0;
	if (_sdf->HasElement("kx"))
		this->kx = _sdf->Get<double>("kx");
	else
		printf("Torsional spring coefficient not specified", this->kx);

	this->setPoint1 = 0.0;
	this->setPoint2 = 0.0;
	this->setPoint3 = 0.0;
	this->setPoint4 = 0.0;

	if (_sdf->HasElement("set_point1"))
		this->setPoint1 = _sdf->Get<double>("set_point1");
	else
		printf("Set point not specified! Defaulting to: %f\n", this->setPoint1);
	if (_sdf->HasElement("set_point2"))
		this->setPoint2 = _sdf->Get<double>("set_point2");
	else
		printf("Set point not specified! Defaulting to: %f\n", this->setPoint2);
	if (_sdf->HasElement("set_point3"))
		this->setPoint3 = _sdf->Get<double>("set_point3");
	else
		printf("Set point not specified! Defaulting to: %f\n", this->setPoint3);
	if (_sdf->HasElement("set_point4"))
		this->setPoint4 = _sdf->Get<double>("set_point4");
	else
		printf("Set point not specified! Defaulting to: %f\n", this->setPoint4);

	std::cout << "Loaded gazebo_joint_torsional_spring." << std::endl;

}

public: void Init()
{
	
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		std::bind (&TorsionalSpringPlugin::OnUpdate, this) );
}

protected: void OnUpdate()
{
	double current_angle1 = this->joint1->Position(0);
	double current_angle2 = this->joint2->Position(0);
	double current_angle3 = this->joint3->Position(0);
	double current_angle4 = this->joint4->Position(0);
	this->joint1->SetForce(0, this->kx*(this->setPoint1-current_angle1));
	this->joint2->SetForce(0, this->kx*(this->setPoint2-current_angle2));
	this->joint3->SetForce(0, this->kx*(this->setPoint3-current_angle3));
	this->joint4->SetForce(0, this->kx*(this->setPoint4-current_angle4));			
}
	
};

GZ_REGISTER_MODEL_PLUGIN(TorsionalSpringPlugin)
}
