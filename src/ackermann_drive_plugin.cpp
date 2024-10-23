#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace gazebo
{
  class AckermannTeleopPlugin : public ModelPlugin
  {
    private: physics::ModelPtr model;
    private: physics::JointPtr frontLeftWheelSteerJoint;
    private: physics::JointPtr frontRightWheelSteerJoint;
    private: physics::JointPtr rearLeftWheelJoint;
    private: physics::JointPtr rearRightWheelJoint;
    private: ros::NodeHandle nh;
    private: ros::Subscriber cmdVelSub;

    public: AckermannTeleopPlugin() {}
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _model;

      // Inicializar ROS e subscrever ao tópico de controle
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("ROS node for Gazebo not initialized, unable to load plugin.");
        return;
      }

      this->cmdVelSub = nh.subscribe("/cmd_vel", 1, &AckermannTeleopPlugin::OnCmdVelReceived, this);

      // Acessar as juntas de direção e tração
      this->frontLeftWheelSteerJoint = this->model->GetJoint("front_left_wheel_steer_joint");
      this->frontRightWheelSteerJoint = this->model->GetJoint("front_right_wheel_steer_joint");
      this->rearLeftWheelJoint = this->model->GetJoint("rear_left_wheel_joint");
      this->rearRightWheelJoint = this->model->GetJoint("rear_right_wheel_joint");

      ROS_INFO("Ackermann Teleop Plugin Loaded.");
    }

    public: void OnCmdVelReceived(const geometry_msgs::TwistConstPtr& msg)
    {
      // Controlar velocidade linear e angular com base nos comandos recebidos
      double steeringAngle = msg->angular.z; // Ângulo de direção
      double velocity = msg->linear.x;       // Velocidade linear

      // Aplicar o ângulo de direção nas rodas frontais
      this->frontLeftWheelSteerJoint->SetPosition(0, steeringAngle);
      this->frontRightWheelSteerJoint->SetPosition(0, steeringAngle);

      // Aplicar velocidade nas rodas traseiras
      this->rearLeftWheelJoint->SetVelocity(0, velocity);
      this->rearRightWheelJoint->SetVelocity(0, velocity);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(AckermannTeleopPlugin)
}
