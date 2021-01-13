#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <jointspace/OptStates.h>
#include "sensor_msgs/JointState.h"
#include <Eigen/Eigen>
#include <kdl/jntarray.hpp>
#include <array>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <std_msgs/Float64.h>//
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <ros/console.h>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/frames.hpp>
#include <jointspace/OptStatesWt.h>
#include <chrono>
#include <ros_action_server/MyMsgAction.h>


using namespace std;
using namespace std::chrono;

#define N_JOINT 6

typedef actionlib::SimpleActionClient <ros_action_server::MyMsgAction> TrajClient;

//rosmsg show ros_action_server/MyMsgAction.msg

class Ur3Arm
{
private:
  TrajClient* traj_client_;
  std::string tauTopicNames[N_JOINT] = {
    "tau_1",
    "tau_2",
    "tau_3",
    "tau_4",
    "tau_5",
    "tau_6",
  };
  std::string dataTopicNames[12] = {
    "pose_error_1",
    "pose_error_2",
    "pose_error_3",
    "pose_error_4",
    "pose_error_5",
    "pose_error_6",
    "velo_error_1",
    "velo_error_2",
    "velo_error_3",
    "velo_error_4",
    "velo_error_5",
    "velo_error_6",
  };


public:
  ros::NodeHandle n;
  std::vector<ros::Publisher> pose_multiple_pub;
  std::vector<ros::Publisher> data_multiple_pub;
  ros::Publisher pose_pub;
  ros::Publisher data_pub;
  ros::Subscriber joint_state_sub;
  ros::Subscriber traj_sub;
  KDL::JntArray jointPosCurrent, jointVelCurrent, jointEffort;
  Eigen::MatrixXf q_cur, qdot_cur;

  KDL::Tree mytree; KDL::Chain mychain;
  std::array<int, N_JOINT> map_joint_states;
  std::array<float, N_JOINT> k_p;
  std::array<float, N_JOINT> k_d;
  std::array<float, N_JOINT> desired_pose;

  Ur3Arm()
  {
    jointPosCurrent.resize(N_JOINT), jointVelCurrent.resize(N_JOINT), jointEffort.resize(N_JOINT);
    q_cur.resize(N_JOINT,1); qdot_cur.resize(N_JOINT,1); // setZero
    map_joint_states={2, 1, 0, 3, 4, 5};
    k_p={16, 16, 16,  10,  10,  10};          // specify p and d gains
    k_d={5, 5, 5,  6,  6,  6};
    desired_pose={0, -1.5, 2, 0, 0, 0};

    // BEGIN
    float mass = 5 ; double Ixx, Iyy, Izz; double l= 0.08, r = l/2.0; // from URDF

    Ixx = (1.0/12.0) * mass * ( 3*r*r + l*l);
    Iyy = Ixx;
    Izz =  (1.0/2.0) * mass * ( r*r );
    double I[6]={Ixx, Iyy, Izz, 0, 0, 0};   // Ixy = Ixz= Iyz = 0;
    double offset[6] = {0, 0, 0, 0, 0, 0} ; std::string tool_name = "new_tool";
    KDL::Vector r_cog(r, r, l/2.0); //! for a cylinder
    KDL::Joint fixed_joint = KDL::Joint(KDL::Joint::None);
    KDL::Frame tip_frame = KDL::Frame(KDL::Rotation::RPY(offset[0],offset[1],offset[2]),KDL::Vector(offset[3],offset[4],offset[5]));

    // rotational inertia in the cog
    KDL::RotationalInertia Inertia_cog = KDL::RotationalInertia(I[0], I[1], I[2], I[3], I[4], I[5]);
    KDL::RigidBodyInertia Inertia = KDL::RigidBodyInertia(mass, r_cog, Inertia_cog);
    KDL::Segment segment = KDL::Segment(tool_name, fixed_joint, tip_frame, Inertia);

    //parse kdl tree from Urdf
    if(!kdl_parser::treeFromFile("/home/mujib/test_ws/src/universal_robot/ur_description/urdf/model.urdf", mytree)){
      ROS_ERROR("[ST] Failed to construct kdl tree for elfin ! ");
    }

    if (!mytree.addSegment(segment, "wrist_3_link")) {  //! adding segment to the tree
      ROS_ERROR("[ST] Could not add segment to kdl tree");
    }

    if (!mytree.getChain("base_link", tool_name, mychain)){
      ROS_ERROR("[ST] Failed to construct kdl chain for elfin ! ");
    }
    // END

    unsigned int nj, ns; // resize variables using # of joints & segments
    nj =  mytree.getNrOfJoints(); ns = mychain.getNrOfSegments();
    if (ns == 0 || nj == 0){
      ROS_ERROR("[ST] Number of segments/joints are zero ! ");
    }
    if(jointPosCurrent.rows()!=nj || jointVelCurrent.rows()!=nj || jointEffort.rows() !=nj )
    {
      ROS_ERROR("[JS] ERROR in size of joint variables ! ");
    }

    traj_sub = n.subscribe("/opt_states", 100, &Ur3Arm::TrajCb, this);
    traj_client_ = new TrajClient("trajectory_action", true); // spin a thread by default
    /*for (short int j = 0; j < N_JOINT; j++)
    {
      pose_pub = n.advertise<std_msgs::Float64>(tauTopicNames[j], 1);
      pose_multiple_pub.push_back(pose_pub);
    }

    for (short int j = 0; j < 12; j++)
    {
      data_pub = n.advertise<std_msgs::Float64>(dataTopicNames[j], 1);
      data_multiple_pub.push_back(data_pub);
    }*/

    while(!traj_client_->waitForServer(ros::Duration(5.0)))
     {
       ROS_INFO("[AC] Waiting for the ros_action server");
     }

    if(traj_client_->isServerConnected()){
      ROS_INFO("[AC] ros_action server is Connected");}
    else {
      ROS_INFO("[AC] ros_action server is Not Connected !");
    }
    cout << "[AC] Constructed !" << endl;
  }

  // -> Destructor of class Ur3Arm
  ~Ur3Arm()
  {
    delete traj_client_;
  }

  void TrajCb(const jointspace::OptStatesWtConstPtr &msg2){

    ROS_INFO("[AC] Reading OptStates");
    Eigen::MatrixXf q_des(N_JOINT,1), qdot_des(N_JOINT,1), qddot_des(N_JOINT,1);
    ros_action_server::MyMsgGoal action;
    double time_ = 0.0;

    action.trajectory.resize(msg2->goal.size());

    for(short int k = 0; k < msg2->goal.size(); k++){

      action.trajectory[k].angle_goal.data.resize(N_JOINT);
      action.trajectory[k].vel_goal.data.resize(N_JOINT);
      action.trajectory[k].acc_goal.data.resize(N_JOINT);
      time_ = k * 3.2/msg2->goal.size();
      cout << "[AC] time_from_start: " << time_ << endl;

      action.trajectory[k].header.stamp = ros::Time::now();
      action.trajectory[k].time_from_start = ros::Duration( time_ );  //  traj duration/nr. of points


      for (short int l=0; l< N_JOINT; l++){

        q_des(l,0) = msg2->goal[k].q.data[l];       // opt_states
        qdot_des(l,0) = msg2->goal[k].qdot.data[l];
        qddot_des(l,0) = msg2->goal[k].qddot.data[l];

        action.trajectory[k].angle_goal.data[l] = q_des(l,0); // Header and time from start
        action.trajectory[k].vel_goal.data[l] = qdot_des(l,0);
        action.trajectory[k].acc_goal.data[l] = qddot_des(l,0);
      }
      // time_from_start = Now () + time_
    }

    action.header.stamp = ros::Time::now() ; //ros::Duration(0.001);
    traj_client_->sendGoalAndWait(action, ros::Duration(0,0), ros::Duration(0,0)); // wait for entire traj to finish
  }


  actionlib::SimpleClientGoalState getState() //! Returns the current state of the action
   {
     return traj_client_->getState();
   }
};


int main (int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "simple_trajectory");
  cout << "[AC] Hello World !" << endl;
  Ur3Arm arm;  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    cout << "[AC] STATE: " << arm.getState().toString() << endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
