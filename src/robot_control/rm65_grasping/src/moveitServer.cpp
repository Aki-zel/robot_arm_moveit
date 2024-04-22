
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>
#include <iostream>

using namespace std;


class MoveIt_Control
{
public:
	
	MoveIt_Control(const ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &arm, const string &PLANNING_GROUP) {
		
		this->arm_ = &arm;
		this->nh_ =nh;
		
		arm_->setGoalPositionTolerance(0.001);
		arm_->setGoalOrientationTolerance(0.01);
		arm_->setGoalJointTolerance(0.001);

		arm_->setMaxAccelerationScalingFactor(0.5);
		arm_->setMaxVelocityScalingFactor(0.5);

		const moveit::core::JointModelGroup* joint_model_group =
			arm_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

		this->end_effector_link = arm_->getEndEffectorLink();
		
		this->reference_frame = "base_link";
		arm_->setPoseReferenceFrame(reference_frame);
		
		arm_->allowReplanning(true);
		arm_->setPlanningTime(5.0);
		arm_->setPlannerId("RRTConnect");

		go_home();

		create_table();

	}	

	void go_home() {
		arm_->setNamedTarget("home");
		arm_->move();
		sleep(0.5);
	}

	bool move_j(const vector<double> &joint_group_positions) {
		arm_->setJointValueTarget(joint_group_positions);
		arm_->move();
		sleep(0.5);
		return true;
	}

	bool move_p(const vector<double> &pose) {
		geometry_msgs::Pose target_pose;
		target_pose.position.x = pose[0];
		target_pose.position.y = pose[1];
		target_pose.position.z = pose[2];

		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(pose[3], pose[4], pose[5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();
		
		arm_->setStartStateToCurrentState();
		arm_->setPoseTarget(target_pose);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::core::MoveItErrorCode success = arm_->plan(plan);
		ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");
		
		if (success) {
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		return false;
	}

	bool move_p(const double (&position)[3]) {
		geometry_msgs::Pose target_pose;
		target_pose.position.x = position[0];
		target_pose.position.y = position[1];
		target_pose.position.z = position[2];

		// keep current pose
		geometry_msgs::Pose current_pose = arm_->getCurrentPose().pose;
		target_pose.orientation = current_pose.orientation;

		arm_->setStartStateToCurrentState();
		arm_->setPoseTarget(target_pose);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::core::MoveItErrorCode success = arm_->plan(plan);
		ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");

		if (success) {
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		return false;
	}

	bool move_p_with_constrains(const vector<double>& pose) {
		geometry_msgs::Pose target_pose;
		target_pose.position.x = pose[0];
		target_pose.position.y = pose[1];
		target_pose.position.z = pose[2];

		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(pose[3], pose[4], pose[5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();

		moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = "Link6";
		ocm.header.frame_id = "base_link";
		ocm.orientation.x = myQuaternion.getX();
		ocm.orientation.y = myQuaternion.getY();
		ocm.orientation.z = myQuaternion.getZ();
		ocm.orientation.w = myQuaternion.getW();
		ocm.absolute_x_axis_tolerance = 0.1;
		ocm.absolute_y_axis_tolerance = 0.1;
		ocm.absolute_z_axis_tolerance = 0.1;
		ocm.weight = 1.0;

		// Now, set it as the path constraint for the group.
		moveit_msgs::Constraints test_constraints;
		test_constraints.orientation_constraints.push_back(ocm);
		arm_->setPathConstraints(test_constraints);

		arm_->setStartStateToCurrentState();
		arm_->setPoseTarget(target_pose);

		// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::core::MoveItErrorCode success = arm_->plan(plan);
		ROS_INFO("move_p_with_constrains :%s", success ? "SUCCESS" : "FAILED");

		arm_->clearPathConstraints();
		if (success) {
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		return false;
	}

	bool move_p_with_constrains(const double (&position)[3]) {
		geometry_msgs::Pose target_pose;
		target_pose.position.x = position[0];
		target_pose.position.y = position[1];
		target_pose.position.z = position[2];

		geometry_msgs::Pose current_pose = arm_->getCurrentPose().pose;

		moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = "Link6";
		ocm.header.frame_id = "base_link";
		ocm.orientation.x = current_pose.orientation.x;
		ocm.orientation.y = current_pose.orientation.y;
		ocm.orientation.z = current_pose.orientation.z;
		ocm.orientation.w = current_pose.orientation.w;
		ocm.absolute_x_axis_tolerance = 0.1;
		ocm.absolute_y_axis_tolerance = 0.1;
		ocm.absolute_z_axis_tolerance = 0.1;
		ocm.weight = 1.0;

		// Now, set it as the path constraint for the group.
		moveit_msgs::Constraints test_constraints;
		test_constraints.orientation_constraints.push_back(ocm);
		arm_->setPathConstraints(test_constraints);

		arm_->setStartStateToCurrentState();
		arm_->setPoseTarget(target_pose);

		// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::core::MoveItErrorCode success = arm_->plan(plan);
		ROS_INFO("move_p_with_constrains :%s", success ? "SUCCESS" : "FAILED");

		arm_->clearPathConstraints();
		if (success) {
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		return false;
	}

	bool move_l(const vector<double>& pose) {

		vector<geometry_msgs::Pose> waypoints;
		geometry_msgs::Pose target_pose;
		target_pose.position.x = pose[0];
		target_pose.position.y = pose[1];
		target_pose.position.z = pose[2];

		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(pose[3], pose[4], pose[5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();
		waypoints.push_back(target_pose);

		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.01;
		double fraction = 0.0;
		int maxtries = 100;   
		int attempts = 0;     

		while (fraction < 1.0 && attempts < maxtries)
		{
			fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");

			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = trajectory;
			
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
			return false;
		}
	}

	bool move_l(const double (&position)[3]) {

		vector<geometry_msgs::Pose> waypoints;
		geometry_msgs::Pose target_pose;
		target_pose.position.x = position[0];
		target_pose.position.y = position[1];
		target_pose.position.z = position[2];

		geometry_msgs::Pose current_pose = arm_->getCurrentPose().pose;
		target_pose.orientation = current_pose.orientation;

		waypoints.push_back(target_pose);

		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.01;
		double fraction = 0.0;
		int maxtries = 100;   
		int attempts = 0;     

		while (fraction < 1.0 && attempts < maxtries)
		{
			fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");

			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = trajectory;
			
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
			return false;
		}
	}

	bool move_l(const vector<vector<double>>& posees) {
		// plan lots of points
		vector<geometry_msgs::Pose> waypoints;
		for (int i = 0; i < posees.size(); i++) {
            geometry_msgs::Pose target_pose;
			target_pose.position.x = posees[i][0];
			target_pose.position.y = posees[i][1];
			target_pose.position.z = posees[i][2];
			
			tf2::Quaternion myQuaternion;
			myQuaternion.setRPY(posees[i][3], posees[i][4], posees[i][5]);
			target_pose.orientation.x = myQuaternion.getX();
			target_pose.orientation.y = myQuaternion.getY();
			target_pose.orientation.z = myQuaternion.getZ();
			target_pose.orientation.w = myQuaternion.getW();
			waypoints.push_back(target_pose);
		}

		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.01;
		double fraction = 0.0;
		int maxtries = 100;   
		int attempts = 0;     

		while (fraction < 1.0 && attempts < maxtries)
		{
			fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");

			
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = trajectory;

			
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
			return false;
		}
	}

	void create_table() {
		// Now let's define a collision object ROS message for the robot to avoid.
		ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    	ros::WallDuration sleep_t(0.5);
    	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    	{
     	 sleep_t.sleep();
    	}
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		moveit_msgs::PlanningScene planning_scene;
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = arm_->getPlanningFrame();

		// The id of the object is used to identify it.
		collision_object.id = "table";

		// Define a box to add to the world.
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = 2;
		primitive.dimensions[primitive.BOX_Y] = 2;
		primitive.dimensions[primitive.BOX_Z] = 0.01;

		// Define a pose for the box (specified relative to frame_id)
		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0.0;
		box_pose.position.y = 0.0;
		box_pose.position.z = -0.01/2 -0.02;

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;

		planning_scene.world.collision_objects.push_back(collision_object);
    	planning_scene.is_diff = true;
    	planning_scene_diff_publisher.publish(planning_scene);

		ROS_INFO("Added an table into the world");
	}
    
	void some_functions_maybe_useful(){
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");

		geometry_msgs::PoseStamped current_pose = this->arm_->getCurrentPose(this->end_effector_link);
		ROS_INFO("current pose:x:%f,y:%f,z:%f,Quaternion:[%f,%f,%f,%f]",current_pose.pose.position.x,current_pose.pose.position.y,
		current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,
		current_pose.pose.orientation.z,current_pose.pose.orientation.w);

		std::vector<double> current_joint_values = this->arm_->getCurrentJointValues();
		ROS_INFO("current joint values:%f,%f,%f,%f,%f,%f",current_joint_values[0],current_joint_values[1],current_joint_values[2],
		current_joint_values[3],current_joint_values[4],current_joint_values[5]);

		std::vector<double> rpy = this->arm_->getCurrentRPY(this->end_effector_link);
		ROS_INFO("current rpy:%f,%f,%f",rpy[0],rpy[1],rpy[2]);

		string planner = this->arm_->getPlannerId();
		ROS_INFO("current planner:%s",planner.c_str());
		std::cout<<"current planner:"<<planner<<endl;

	}

	~MoveIt_Control() {
		
		ros::shutdown();
	}


public:
	string reference_frame;
	string end_effector_link;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface *arm_;
};

int main(int argc, char** argv) {

	ros::init(argc, argv, "moveit_control_server_cpp");
	ros::AsyncSpinner spinner(1);
	ros::NodeHandle nh;
	spinner.start();
	static const std::string PLANNING_GROUP = "arm";
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);

	MoveIt_Control moveit_server(nh,arm,PLANNING_GROUP);

	// Test 

	// test for move_j
	cout<<"-----------------------test for move_j----------------------"<<endl;
	vector<double> joints ={0,0,-1.57,0,0,0};
	moveit_server.move_j(joints);

	// test for move_p and move_l(1 point)
	cout<<"-----------------------test for move_p and move_l---------------------"<<endl;
	vector<double> xyzrpy={0.3,0.1,0.4,-3.1415,0,0};
	moveit_server.move_p(xyzrpy);
	xyzrpy[2]=0.2;
	moveit_server.move_l(xyzrpy);

	// test for move_l (>=2 points)
	cout<<"-----------------------test for move_l(more points)----------------------"<<endl;
	vector<vector<double>> xyzrpys;
	xyzrpys.push_back(xyzrpy);
	xyzrpy[1]=0.2;
	xyzrpys.push_back(xyzrpy);
	xyzrpy[0]=0.4;
	moveit_server.move_l(xyzrpys);

	// test for move_p_with constrains
	cout<<"-----------------------test for move_p_with_constrains----------------------"<<endl;
	vector<double> pose1={0.4,0,0.4,0,3.141592/2,0};
	moveit_server.move_p(pose1);
	vector<double> pose2={0.4,0.2,0.2,0,3.141592/2,0};
	moveit_server.move_p_with_constrains(pose2);
	vector<double> pose3={0.0,0.5,0.3,0,3.141592/2,0};
	moveit_server.move_p_with_constrains(pose3);

	// test for my functions
	cout<<"-----------------------test for my move_function----------------------"<<endl;
	double xyz[3]={0.3,0.1,0.4};
	moveit_server.move_p(xyz);
	xyz[2]=0.2;
	moveit_server.move_l(xyz);
	xyz[1]=0.2;
	moveit_server.move_p_with_constrains(xyz);

	// test for some useful functions
	cout<<"-----------------------test for other functions----------------------"<<endl;
	moveit_server.some_functions_maybe_useful();
	return 0;

}

