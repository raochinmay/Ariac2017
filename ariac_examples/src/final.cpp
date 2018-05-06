#include <algorithm>
#include <vector>
#include <stack>
#include <ros/ros.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/VacuumGripperControl.h>
//#include <osrf_gear/AGVControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/TrajectoryConstraints.h>



//structure for order parts
struct model_with_kit {
  int order_number;
  int kit_number;
  std::string kit_to_send;
  osrf_gear::KitObject models;
};

//structure for parts picked from conveyor belt
struct type_pose {
  std::string type;
  geometry_msgs::Pose pose;
};

//Global variables

//orientation for picking up parts
float w = 0.707;
float x = 0;
float y = 0.707;
int tim = 0;
float z = 0;
float mag = sqrt(w * w + x * x + y * y + z*z);
int conv_flag = 0;
int height_flag = 0;
int break_flag = 0;
int agv1_count =0;
int agv2_count =0;
//extreme values for parts to be kept on agv trays
float agv1_x_max = 0.5;
float agv1_x_min = 0.1;
float agv1_y_max = 3.43;
float agv1_z = 0.75;
float agv2_x_max = 0.5;
float agv2_x_min = 0.1;
float agv2_y_max = -3.43;
float agv2_z = 0.75;

float cam_x = 0;
float cam_y = 0;
float cam_z = 0;
float cam_ow = 0.707;
float cam_ox = 0;
float cam_oy = 0.707;
float cam_oz = 0;
//flag for unrequired parts
bool disregard = 0;

//flag for high priority order
bool high_priority_done = 0;

//vector to store the parts picked from the conveyor belt
std::vector<type_pose> camera_parts;

//parts on agv1 and agv2
std::vector<type_pose> camera1_parts;
std::vector<type_pose> camera2_parts;

//service client for gripper
ros::ServiceClient gripper_client;

//vector storing the order and a dummy list of the part types
std::vector<model_with_kit> Order_parts;
std::vector<std::string> dummy_list;

//flag to check the attached part
bool part_attached;

//positional error in target location and the positions of the faulty parts from agv1 and agv2
float pos_err_x, pos_err_y, pos_err_z, pos_faulty_1x, pos_faulty_1y, pos_faulty_1z, pos_faulty_2x, pos_faulty_2y, pos_faulty_2z, ori_faulty_1x, ori_faulty_2x, ori_faulty_1y, ori_faulty_2y, ori_faulty_1z, ori_faulty_2z, ori_faulty_1w, ori_faulty_2w;
std::string type_faulty_1 = "";
std::string type_faulty_2 = "";

/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
      node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger service;  // Combination of the "request" and the "response".
  start_client.call(service);  // Call the start Service.
  if (!service.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << service.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}

///Class with all the methods and functions
class MyCompetitionClass {
  public:
  bool gripper_status;
  explicit MyCompetitionClass(ros::NodeHandle & node)
    : current_score_(0), has_been_zeroed_(false) {

    //method to publish joint state
    joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
                                      "/ariac/arm/command", 10);

    //client to get material location
    material_locations_client = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

    //client for gripper
    gripper_client = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    ROS_INFO_STREAM("material_locations_client initialized");
  }


  /// Called when a new score is received.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
    if (msg->data != current_score_) {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }
  // void time_callback(rosgraph_msgs::Clock & msg)
  // {
  //   tim = msg.secs;
  // }

  //gets current joint states
  std::vector<double> getCurrentJointState(const std::string& topic) {
    sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.0));
    if (!state)
      throw std::runtime_error("Joint state message capture failed");
    return state->position;
  }

  /// Called when a new JointState message is received.
  void joint_state_callback(
      const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
    current_joint_states_ = *joint_state_msg;
  }

  // Called to check if a part is attached to the gripper
  void part_attached_callback(const osrf_gear::VacuumGripperState::ConstPtr & gripper_msg) {
    part_attached = gripper_msg->attached;
    gripper_status = gripper_msg->enabled;
  }

  /// Check for the faulty parts on AGV1
  void faultyParts_agv1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & faultyPart_msg) {
    for (int i = 0; i < faultyPart_msg->models.size(); i++) {
      type_faulty_1 = faultyPart_msg->models[i].type ;
      pos_faulty_1x = faultyPart_msg->models[i].pose.position.x ;
      pos_faulty_1y = faultyPart_msg->models[i].pose.position.y ;
      pos_faulty_1z = faultyPart_msg->models[i].pose.position.z ;
      ori_faulty_1x = faultyPart_msg->models[i].pose.orientation.x ;
      ori_faulty_1y = faultyPart_msg->models[i].pose.orientation.y ;
      ori_faulty_1z = faultyPart_msg->models[i].pose.orientation.z ;
      ori_faulty_1w = faultyPart_msg->models[i].pose.orientation.w ;
    }
  }

  /// Check for the faulty parts on AGV2
  void faultyParts_agv2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & faultyPart_msg) {
    for (int i = 0; i < faultyPart_msg->models.size(); i++) {
      type_faulty_2 = faultyPart_msg->models[i].type ;
      pos_faulty_2x = faultyPart_msg->models[i].pose.position.x ;
      pos_faulty_2y = faultyPart_msg->models[i].pose.position.y ;
      pos_faulty_2z = faultyPart_msg->models[i].pose.position.z ;
      ori_faulty_2x = faultyPart_msg->models[i].pose.orientation.x ;
      ori_faulty_2y = faultyPart_msg->models[i].pose.orientation.y ;
      ori_faulty_2z = faultyPart_msg->models[i].pose.orientation.z ;
      ori_faulty_2w = faultyPart_msg->models[i].pose.orientation.w ;
    }
  }

  /// Called when a competition state is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done") {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

  /// Called when a new Order message is received.
  void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
    std::string order_id_number_string = (*order_msg).order_id.substr(6);
    int order_id_number = std::stoi(order_id_number_string);
    ROS_INFO_STREAM("order:" << order_id_number);

    for (int j = 0; j < (*order_msg).kits.size(); j++) {
      for (osrf_gear::KitObject currModel : (*order_msg).kits[j].objects) {
        model_with_kit temp;
        temp.order_number = order_id_number;
        temp.kit_number = j;
        temp.kit_to_send = "order_" + std::to_string(order_id_number) + "_kit_" + std::to_string(j);
        temp.models = currModel;
        Order_parts.push_back(temp);
        dummy_list.push_back(currModel.type);


      }
    }
  }
  geometry_msgs::Pose calculate_error(geometry_msgs::Pose target_location) {
    //geometry_msgs::Pose target_location1;
    target_location.position.x -= pos_err_z; //check this for surity
    target_location.position.y += pos_err_y;
    target_location.position.z += 0.1;
    target_location.orientation.x = 0;
    target_location.orientation.y = 0.707;
    target_location.orientation.z = 0;
    target_location.orientation.w = 0.707;
    ROS_INFO_STREAM("new target location \nx:" << target_location);//.position.x
    return target_location;
  }
  /// Called when a new LogicalCameraImage message is received.
  void logical_camera2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    for (int i = 0; i < image_msg->models.size(); i++) {
      if (image_msg->models[i].pose.position.x < 0.65) { //exclude agv and tray
        pos_err_x = image_msg->models[i].pose.position.x - 0.75; //These are the coordinates of agv load point. So make sure to place the ee link on agv load point.
        pos_err_y = image_msg->models[i].pose.position.y ;
        pos_err_z = image_msg->models[i].pose.position.z ;
      } else if (disregard == 1) {
        if (image_msg->models[i].type == "kit_tray" || image_msg->models[i].type == "agv2") {continue;}
        type_pose temp;
        temp.type = image_msg->models[i].type;
        temp.pose.position.x = 0.3 - image_msg->models[i].pose.position.z;
        temp.pose.position.y = 3.14 + image_msg->models[i].pose.position.y;
        temp.pose.position.z = 1.5 - image_msg->models[i].pose.position.x;
        temp.pose.orientation.x = image_msg->models[i].pose.orientation.x;
        temp.pose.orientation.y = image_msg->models[i].pose.orientation.y;
        temp.pose.orientation.z = image_msg->models[i].pose.orientation.z;
        temp.pose.orientation.w = image_msg->models[i].pose.orientation.w;
        camera_parts.push_back(temp);
      }
    }
    disregard = 0;
  }

  void logical_camera1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    for (int i = 0; i < image_msg->models.size(); i++) {
      if (image_msg->models[i].type == "kit_tray" || image_msg->models[i].type == "agv1") {continue;}
      else { //exclude agv and tray
        //
        if ((*image_msg).models[i].pose.position.x > 0.48 && (*image_msg).models[i].pose.position.x < 0.55) {
          cam_ow = (*image_msg).models[i].pose.orientation.w + (*image_msg).pose.orientation.w;
          cam_ox = (*image_msg).models[i].pose.orientation.x + (*image_msg).pose.orientation.x;
          cam_oy = (*image_msg).models[i].pose.orientation.y + (*image_msg).pose.orientation.y;
          cam_oz = (*image_msg).models[i].pose.orientation.z + (*image_msg).pose.orientation.z;
          geometry_msgs::Quaternion quater;
          quater.x = cam_ox;
          quater.y = cam_oy;
          quater.z = cam_oz;
          quater.w = cam_ow;
          ROS_INFO_STREAM("getting orientation:" << quater);
        }
        int size = camera1_parts.size();

        if (camera1_parts.size() <= (*image_msg).models.size() - 2) {
          if ((*image_msg).models[i].pose.position.x > 0.74 && (*image_msg).models[i].pose.position.x < 0.755) {


            //ROS_INFO_STREAM("test x: " << cam_x << " y: " << cam_y << " z: " << cam_z);
            if (camera1_parts.size() == 0) {
              cam_x = (*image_msg).models[i].pose.position.z + 0.3;
              cam_y = (*image_msg).models[i].pose.position.y + 3.14 ;
              cam_z = 1.5 - (*image_msg).models[i].pose.position.x;
              // cam_ow = (*image_msg).models[i].pose.orientation.w + (*image_msg).pose.orientation.w;
              // cam_ox = (*image_msg).models[i].pose.orientation.x + (*image_msg).pose.orientation.x;
              // cam_oy = (*image_msg).models[i].pose.orientation.y + (*image_msg).pose.orientation.y;
              // cam_oz = (*image_msg).models[i].pose.orientation.z + (*image_msg).pose.orientation.z;
              type_pose temp;
              //ROS_INFO_STREAM("test size: " << camera2_parts.size() << " i: " << i);
              temp.type = image_msg->models[i].type;
              temp.pose.position.x = cam_x;
              temp.pose.position.y = cam_y;
              temp.pose.position.z = cam_z;
              temp.pose.orientation.x = 0;
              temp.pose.orientation.y = 0.707;
              temp.pose.orientation.z = 0;
              temp.pose.orientation.w = 0.707;
              camera1_parts.push_back(temp);
              ROS_INFO_STREAM("add part to camera1_parts vector:" << temp.type << "\n pose:" << temp.pose);

            }
            float temp_cam_x = (*image_msg).models[i].pose.position.z + 0.3;
            float temp_cam_y = (*image_msg).models[i].pose.position.y + 3.14 ;
            float temp_cam_z = 1.5 - (*image_msg).models[i].pose.position.x;

            bool found = false;
            for (std::vector<type_pose>::iterator it = camera1_parts.begin(); it != camera1_parts.end(); ++it) {
              //ROS_INFO_STREAM("abs=" << fabs(temp_cam_x - (*it).pose.position.x) << " temp_cam_x:" << temp_cam_x << " temp_cam_y:" << temp_cam_y);
              //ROS_INFO_STREAM("vector_x:" << (*it).pose.position.x << " vector_y:" << (*it).pose.position.y);

              if ((fabs(temp_cam_x - (*it).pose.position.x) < 0.1) && (fabs(temp_cam_y - (*it).pose.position.y) < 0.1) ) {
                found = true;
                //ROS_INFO_STREAM("found" );

                break;
              }
            }
            if (!found) {

              cam_x = (*image_msg).models[i].pose.position.z + 0.3;
              cam_y = (*image_msg).models[i].pose.position.y + 3.14;
              cam_z = 1.5 - (*image_msg).models[i].pose.position.x;
              type_pose temp;
              //ROS_INFO_STREAM("test size: " << camera2_parts.size() << " i: " << i);
              temp.type = image_msg->models[i].type;
              temp.pose.position.x = cam_x;
              temp.pose.position.y = cam_y;
              temp.pose.position.z = cam_z;
              // cam_ow = (*image_msg).models[i].pose.orientation.w + (*image_msg).pose.orientation.w;
              // cam_ox = (*image_msg).models[i].pose.orientation.x + (*image_msg).pose.orientation.x;
              // cam_oy = (*image_msg).models[i].pose.orientation.y + (*image_msg).pose.orientation.y;
              // cam_oz = (*image_msg).models[i].pose.orientation.z + (*image_msg).pose.orientation.z;
              if ((*image_msg).models.size() - 2 == camera1_parts.size()) {
                //ROS_INFO_STREAM("pop part from camera1_parts vector");
                camera1_parts.pop_back();
              }
              camera1_parts.push_back(temp);
              ROS_INFO_STREAM("add part to camera1_parts vector:" << temp.type << "\n pose:" << temp.pose);
            }
          }
        }
      }
    }
  }



  std::string query_part_location(std::string part_name) {

    std::vector<std::string> part_location;

    osrf_gear::GetMaterialLocations query_location ;
    query_location.request.material_type = part_name;
    std::vector<osrf_gear::StorageUnit> location_vector; //(received_arr, received_arr + sizeof(received_arr) / sizeof(Point));
    ROS_INFO_STREAM("Getting " << part_name << " location\n");
    std::string loc;

    if (material_locations_client.call(query_location)) {
      ROS_INFO_STREAM("Got " << part_name << " location:");
      location_vector = query_location.response.storage_units;
      for (std::vector<osrf_gear::StorageUnit>::iterator it = location_vector.begin(); it != location_vector.end(); ++it) {

        std::ostringstream oss;
        oss << (*it);
        std::string location_string = oss.str().substr(9, 12);
        location_string.erase(std::remove(location_string.begin(), location_string.end(), '\n'), location_string.end());
        ROS_INFO_STREAM("location: " << location_string );

        part_location.push_back(location_string);

      }

      for (std::vector<std::string>::iterator it = part_location.begin(); it != part_location.end(); ++it) {
        loc = *it;
      }
    } else {
      ROS_ERROR_STREAM("Material location service call error");
    }
    return loc;
  }

  //remove the unwanted parts from the agv2 collected from the belt
  void rmv_unwanted_parts() {
    std::vector<type_pose> unwanted_parts;
    osrf_gear::VacuumGripperControl srv;

    //copy all parts from camera parts vector to the unwanted parts
    for (int i = 0; i < camera_parts.size(); i++) {unwanted_parts.push_back(camera_parts[i]);}

    //pop all the wanted parts from the unwanted parts vector
    for (int i = 0; i < camera_parts.size(); i++) {
      for (int j = 0; j < dummy_list.size(); j++) {
        if (camera_parts[i].type == dummy_list[j]) {
          unwanted_parts.erase(unwanted_parts.begin() + i);
          break;
        }
      }
    }
    static const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(15.0);
    move_group.setNumPlanningAttempts(20);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    for (int i = 0; i < unwanted_parts.size(); i++) {
      geometry_msgs::Pose target_pose1;
      target_pose1 = unwanted_parts[i].pose;
      target_pose1.orientation.x = 0;
      target_pose1.orientation.y = 0.707;
      target_pose1.orientation.z = 0;
      target_pose1.orientation.w = 0.707;
      move_group.setPoseTarget(target_pose1);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = false;
      int lc = 0;
      while (!success) {
        if (lc > 2)
          break;
        success = (move_group.plan(my_plan) ==
                   moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        lc++;
      }
      //sleep(6.0);
      osrf_gear::VacuumGripperControl srv;
      srv.request.enable = true;
      gripper_client.call(srv);
      move_group.execute(my_plan);
      ros::Duration(3).sleep();

      joint_state_control("agv2_loadpoint");
      ros::spinOnce();
      joint_state_control("disregard");
      ros::spinOnce();
      srv.request.enable = false;
      gripper_client.call(srv);
      ros::spinOnce();
    }

  }

  void joint_state_control(std::string where) {
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.clear();
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.points.resize(1);
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    if (where == "home") {
      msg.points[0].positions = {2.1, 0, -1.5, -3, 3.45, -1.51, 0.0};
    } else if (where == "moving_state") {
      msg.points[0].positions = {2.5, 0, -1.5, 1, 3.5, -1.51, 0.0};
    } else if (where == "moving_to_agv1") {
      msg.points[0].positions = {2.5, 2.1, -1.5, -3, 3.5, -1.51, 0.0};
    }
    //else if (where == "agv1_loadpoint"){msg.points[0].positions = {1.5, 2.1, -0.75, -4.75, 3.95, -1.51, 0.00};}
    // else if (where == "agv1") {msg.points[0].positions = {1.7, 2.1, -0.52, -4.9, 3.45, -1.51, -0.972};}
    else if (where == "agv1") {msg.points[0].positions = {1.385111048783651, 2.1000175321311536, -0.5, 1.412568909255488, 3.963771182986521, -1.573734238178758, 1.4119492775358617, 0.0};}
    else if (where == "agv10") {msg.points[0].positions = {1.385111048783651, 2.1000175321311536, -0.5, 1.46, 3.963771182986521, -1.573734238178758, 1.4119492775358617, 0.0};}
    else if (where == "agv11") {msg.points[0].positions = {1.385111048783651, 2.1000175321311536, -0.5, 1.32, 3.963771182986521, -1.573734238178758, 1.4119492775358617, 0.0};}
    else if (where == "agv12") {msg.points[0].positions = {1.42, 2.1000175321311536, -0.5, 1.35, 3.963771182986521, -1.573734238178758, 1.4119492775358617, 0.0};}
    else if (where == "agv1_camera") {msg.points[0].positions = {1.385111048783651, 2.1000175321311536, -0.6238696945009714, 1.412568909255488, 3.963771182986521, -1.573734238178758, 1.4119492775358617, 0.0};}

    else if (where == "moving_to_agv2") {
      msg.points[0].positions = {2.1, -2.1, -1.5, 1, 3.5, -1.51, 0.0};}
    // else if (where == "agv2") {msg.points[0].positions = {1.75, -2.1, -0.75, -1.8, 3.45, -1.51, -0.9};}
    else if (where == "agv2") {msg.points[0].positions = {1.75, -2, -0.53, -1.9, 3.75, 0, 0};}
    else if (where == "agv20") {msg.points[0].positions = {1.75, -2, -0.53, -1.95, 3.75, 0, 0};}
    else if (where == "agv21") {msg.points[0].positions = {1.75, -2, -0.53, -1.83, 3.75, 0, 0};}
    else if (where == "conveyor_pickup1") {msg.points[0].positions = {1.65, 2.1, -0.58, 0.0, 3.75, 0, 0.00};}
    else if (where == "conveyor_idle") {msg.points[0].positions = {1.6, -1.5, -0.9, -0.5, 3.75, -1.51, 0.00};}
    // else if (where == "before_conveyor_pickup1") {msg.points[0].positions = {1.6, 2.1, -0.58, 0.0, 3.75, 0, 0.00};}
    // else if (where == "after_conveyor_pickup1") {msg.points[0].positions = {1.6, 2, -0.9, 1.0, 3.5, 0, 0.00};}
    // else if (where == "after_conveyor_pickup3") {msg.points[0].positions = {1.6, 2.1, -0.9, 1, 3.5, 0, 0.00};}
    else if (where == "agv1_drop_1") {msg.points[0].positions = {1.6, 2, -0.515, -4.75, 3.75, -1.51, 0.00};}
    else if (where == "agv1_drop_2") {msg.points[0].positions = {1.6, 2, -0.515, -4.875, 3.75, -1.51, 0.00}; }
    else if (where == "agv1_drop_3") {msg.points[0].positions = {1.6, 2, -0.515, -5, 3.75, -1.51, 0.00}; }

    // else if (where == "conveyor_pickup2") {msg.points[0].positions = {1.6, -1.8, -0.575, 0.0, 3.75, 0, 0.00};}
        else if (where == "conveyor_pickup2") {msg.points[0].positions = {1.6, -2, -0.575, 0.0, 3.75, 0, 0.00};}
    else if (where == "agv2wp") {msg.points[0].positions = {1.4, -2, -0.7, 0.0, 3.75, 0, 0.00};}

    else if (where == "before_conveyor_pickup2") {msg.points[0].positions = {2.55, 0, -1.3, -0.50, 3.0, -1.6, 0.00};}
    else if (where == "after_conveyor_pickup2") {msg.points[0].positions = {1.6, -2.1, -0.578, -1.0, 3.5, 0, 0.00};}
    else if (where == "agv2_drop_1") {msg.points[0].positions = {1.6, -2, -0.475, -1.725, 3.75, 0, 0.00};}
    else if (where == "agv2_drop_2") {msg.points[0].positions = {1.6, -2, -0.475, -1.9, 3.75, 0, 0.00}; }
    else if (where == "agv2_drop_3") {msg.points[0].positions = {1.6, -2, -0.475, -2.075, 3.75, 0, 0.00}; }
    else if (where == "agv2_drop_4") {msg.points[0].positions = {1.3, -2.2, -0.35, -1.7, 3.75, 0, 0.00}; }
    else if (where == "agv2_drop_5") {msg.points[0].positions = {1.2, -2.2, -0.35, -1.85, 3.75, 0, 0.00}; }
    else if (where == "agv2_drop_6") {msg.points[0].positions = {1.2, -2.2, -0.35, -2.0, 3.75, 0, 0.00}; }

    else if (where == "agv2_pick_1") {msg.points[0].positions = {1.55, -2, -0.43, -1.6, 3.5, -1.51, 0.00};}
    else if (where == "agv2_pick_2") {msg.points[0].positions = {1.6, -2, -0.43, -1.725, 3.5, -1.51, 0.00}; }
    else if (where == "agv2_pick_3") {msg.points[0].positions = {1.6, -2, -0.45, -1.9, 3.5, -1.51, 0.00}; }
    else if (where == "agv2_pick_4") {msg.points[0].positions = {1.3, -2.2, -0.3, -1.525, 3.5, -1.51, 0.00}; }
    else if (where == "agv2_pick_5") {msg.points[0].positions = {1.2, -2.2, -0.3, -1.675, 3.5, -1.51, 0.00}; }
    else if (where == "agv2_pick_6") {msg.points[0].positions = {1.2, -2.2, -0.3, -1.875, 3.5, -1.51, 0.00}; }
    else if (where == "agv2_pic_1") {msg.points[0].positions = {1.62, -2, -0.33, -1.6, 3.5, -1.51, 0.00};}
    else if (where == "agv2_pic_2") {msg.points[0].positions = {1.63, -2, -0.33, -1.725, 3.5, -1.51, 0.00}; }
    else if (where == "agv2_pic_3") {msg.points[0].positions = {1.63, -2, -0.35, -1.9, 3.5, -1.51, 0.00}; }
    else if (where == "disregard") {msg.points[0].positions = {0.6, -2.1, -0.5, -2.3, 4.1, -1.51, 0.00}; }

    else if (where == "faulty_out_agv1") {msg.points[0].positions = {1.488966, 2.10, -0.67102883, 1.756163956, 3.90240685, -1.571011107, -0.972};}
    else if (where == "faulty_out_agv2") {msg.points[0].positions = {1, -2.1, -0.4, -2, 4.1, -1.51, 0};}

    msg.points[0].time_from_start = ros::Duration(0.001);
    joint_trajectory_publisher_.publish(msg);
    ros::Duration(1.5).sleep();
  }

  //pickup part on the bin using offset methods
  void pickup_part_ik(int bin, float count) {
    if (count > 3) {
      break_flag = 1;
      Order_parts.erase(Order_parts.begin());
      dummy_list.erase(dummy_list.begin());
    }
    ROS_INFO_STREAM(count);
    int k = count;
    if ( count > 2.2)
      k = 2.2;
    float offsetx = 0.1 * k;
    float offsety = 0.1 * k;
    float y = 1 - (8 - bin) * 0.765;
    float x = -0.3;
    if (bin == 5) {
      y = -1.33;
      if (bin5_counter == 0) {
        x += offsetx;
        y += offsety;
      }
      if (bin5_counter == 3) {
        x -= offsetx;
        y += offsety;
      }
      if (bin5_counter == 2) {
        x += offsetx;
        y -= offsety;
      }
      if (bin5_counter == 1) {
        x -= offsetx;
        y -= offsety;
      }
      bin5_counter++;
      bin5_counter = bin5_counter % 4;
    }
    if (bin == 6) {
      y = -0.535;
      if (bin6_counter == 0) {
        x += offsetx;
        y += offsety;
      }
      if (bin6_counter == 3) {
        x -= offsetx;
        y += offsety;
      }
      if (bin6_counter == 2) {
        x += offsetx;
        y -= offsety;
      }
      if (bin6_counter == 1) {
        x -= offsetx;
        y -= offsety;
      }
      bin6_counter++;
      bin6_counter = bin6_counter % 4;
    }
    if (bin == 7) {
      y = 0.23;
      if (bin7_counter == 0) {
        x += offsetx;
        y += offsety;
      }
      if (bin7_counter == 1) {
        x -= offsetx;
        y += offsety;
      }
      if (bin7_counter == 2) {
        x += offsetx;
        y -= offsety;
      }
      if (bin7_counter == 3) {
        x -= offsetx;
        y -= offsety;
      }
      bin7_counter++;
      bin7_counter = bin7_counter % 4;
    }
    if (bin == 8) {
      y = 0.995;
      if (bin8_counter == 0) {
        x += offsetx;
        y += offsety;
      }
      if (bin8_counter == 1) {
        x -= offsetx;
        y += offsety;
      }
      if (bin8_counter == 2) {
        x += offsetx;
        y -= offsety;
      }
      if (bin8_counter == 3) {
        x -= offsetx;
        y -= offsety;
      }
      bin8_counter++;
      bin8_counter = bin8_counter % 5;
    }

    geometry_msgs::Pose target_pose1;

    target_pose1.position.x = x;
    target_pose1.position.y = y;
    if (height_flag == 1) {
      target_pose1.position.z = 0.795;
    } else {
      target_pose1.position.z = 0.745;
    }
    target_pose1.orientation.x = 0;
    target_pose1.orientation.y = 0.707;
    target_pose1.orientation.z = 0;
    target_pose1.orientation.w = 0.707;
    move_arm_to(target_pose1);
    osrf_gear::VacuumGripperControl srv;
    srv.request.enable = true;
    gripper_client.call(srv);

    // target_pose1.position.z = 0.745;
    move_arm_to(target_pose1);

    ros::Duration(2).sleep();
  }

  void place_part_agv_ik(geometry_msgs::Pose target_pose) {
    ROS_INFO_STREAM("placing part planner");
    geometry_msgs::Pose target_pose1;

    target_pose1.position.x = target_pose.position.x;
    target_pose1.position.y = target_pose.position.y;
    target_pose1.position.z = 0.85;
    target_pose1.orientation.x = 0;
    target_pose1.orientation.y = 0.707;
    target_pose1.orientation.z = 0;
    target_pose1.orientation.w = 0.707;

    ROS_INFO_STREAM("place part location:" << target_pose1);

    move_arm_to(target_pose1);

    //ros::Duration(3).sleep();
    osrf_gear::VacuumGripperControl srv;
    srv.request.enable = false;
    gripper_client.call(srv);
  }

  //hold the part at the load point to check for offset errors
  void hold_part_to_load_point_ik(int agv) {

    ROS_INFO_STREAM("Here in local planner");
    static const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(25);
    //name: 'allowed_start_tolerance', value: 0.0
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    geometry_msgs::Pose target_pose_loadpoint;
    if (agv == 1) {
      target_pose_loadpoint.position.x = 0.3;
      target_pose_loadpoint.position.y = 3.14;
      target_pose_loadpoint.position.z = 1;
      target_pose_loadpoint.orientation.x = 0;
      target_pose_loadpoint.orientation.y = 0.707;
      target_pose_loadpoint.orientation.z = 0;
      target_pose_loadpoint.orientation.w = 0.707;
    }
    if (agv == 2) {
      target_pose_loadpoint.position.x = 0.3;
      target_pose_loadpoint.position.y = -3.14;
      target_pose_loadpoint.position.z = 1;
      target_pose_loadpoint.orientation.x = 0;
      target_pose_loadpoint.orientation.y = 0.707;
      target_pose_loadpoint.orientation.z = 0;
      target_pose_loadpoint.orientation.w = 0.707;
    }


    move_group.setPoseTarget(target_pose_loadpoint);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;
    while (!success) {
      success = (move_group.plan(my_plan) ==
                 moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    }
    move_group.execute(my_plan);
    ros::Duration(3).sleep();
    ROS_INFO_STREAM("Here at end of local planner moving to load point");
  }
  void check_rotation(geometry_msgs::Pose target_pose) {
    geometry_msgs::Quaternion q0;
    q0.w = cam_ow;
    q0.x = cam_ox;
    q0.y = cam_oy;
    q0.z = cam_oz;
    float yaw0 = qtoa(q0);
    float yaw_goal = qtoa(target_pose.orientation);
    //ros::Duration(1).sleep();
    ROS_INFO_STREAM("target yaw: " << yaw_goal);
    part_rotation(yaw_goal - yaw0);
  }
  void check_orientation(geometry_msgs::Pose target_pose) {
    ROS_INFO_STREAM("check orientation");
    geometry_msgs::Pose temp;
    // ROS_INFO_STREAM("temp_vec size:" << temp_vec.size());// << "\ntype: " << temp_vec.end()->type);

    float xp =  cam_x ;
    float yp =  cam_y ;
    float zp;
    if(height_flag=1){
      zp = 0.795;
    }else{
      zp =  0.743 ;
  }



    geometry_msgs::Pose target_location;
    target_location.position.x = xp;
    target_location.position.y = yp;
    target_location.position.z = zp;
    target_location.orientation.w = 0.707;
    target_location.orientation.x = 0;
    target_location.orientation.y = 0.707;
    target_location.orientation.z = 0;
    //geometry_msgs::Pose part_original_location = temp.pose;//get the latest part location
    ROS_INFO_STREAM("reorientation pick up part location:" << target_location);
    osrf_gear::VacuumGripperControl srv;
    srv.request.enable = true;
    gripper_client.call(srv);

    move_arm_to(target_location);
    // while (!part_attached) {
    //   // target_location.position.z -= 0.015;
    //   move_arm_to(target_location);
    // }


    //target_location.position.z = zp + 0.2;
    // part_original_location.position.x += 0.3; //world coord = camera coord + part coord wrt camera
    // part_original_location.position.y -= 3.14;
    // part_original_location.position.z += 1;
    // part_original_location.orientation.x = 0;
    // part_original_location.orientation.y = 0.707;
    // part_original_location.orientation.z = 0;
    // part_original_location.orientation.w = 0.707;
    //ROS_INFO_STREAM("reorientation pick up part world location:" << part_original_location);

    //move_arm_to(target_location);


    // ros::Duration(1).sleep();
    // joint_state_control("agv1");
    ros::spinOnce();
    ros::Duration(1).sleep();
    // target_pose.position.x = 0.3 - target_pose.position.x;
    // target_pose.position.y = 3.14 - target_pose.position.y;
    // if (target_pose.position.y > 3.43) {target_pose.position.y = 3.43;}
    //target_pose.position.z = 0.85;


    target_location = target_pose;
//    float mag_o = sqrt(w * w + x * x + y * y + target_pose.orientation.z * target_pose.orientation.z);

    target_location.orientation.w = 0.707 ;
    target_location.orientation.x = 0 ;
    target_location.orientation.y = 0.707;
    target_location.orientation.z = 0 ;
    target_location.position.z = 0.85;


    //target_pose.position.z = 0.75 + target_pose.position.z;
    ROS_INFO_STREAM("reorientation move part to location:" << target_location);
    //ros::Duration(1).sleep();
    move_arm_to(target_location);
    //set_arm_orientation(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
    ros::Duration(1.5).sleep();

    //osrf_gear::VacuumGripperControl srv;
    srv.request.enable = false;
    gripper_client.call(srv);

  }


  void set_arm_orientation(float x, float y, float z, float w) {
    ROS_INFO_STREAM("Local planner for moving the arm");
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(25);
    //name: 'allowed_start_tolerance', value: 0.0
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    move_group.setOrientationTarget(x, y, z, w, "ee_link");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;
    int counter = 0;
    while (!success) {
      if (counter > 15) {
        ROS_INFO("cannot solve giving up");
        break;
      }
      success = (move_group.plan(my_plan) ==
                 moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
      counter++;
    }
    move_group.execute(my_plan);
    ros::Duration(2).sleep();
  }


  //move arm to a particular pose
  void move_arm_to(geometry_msgs::Pose target_pose) {
    ROS_INFO_STREAM("Local planner for moving the arm");
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(25);
    //name: 'allowed_start_tolerance', value: 0.0
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;
    int counter = 0;
    while (!success) {
      if (counter > 2) {
        ROS_INFO("cannot solve giving up");
        break;
      }
      success = (move_group.plan(my_plan) ==
                 moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
      counter++;
    }
    move_group.execute(my_plan);
    ros::Duration(2).sleep();
  }

  void move_part_to_agv1(geometry_msgs::Pose target_location) {
    agv1_count++;
    agv1_count =agv1_count%3;
    joint_state_control("moving_state");
    ros::spinOnce();
    joint_state_control("moving_to_agv1");
    ros::spinOnce();
    // joint_state_control("agv1");
    // ros::spinOnce();

    joint_state_control("agv1_camera");
    //hold_part_to_load_point_ik(1);
    ros::Duration(2).sleep();
    // check_rotation(target_location);
    // ros::spinOnce();
    std::string r = "agv1"+std::to_string(agv1_count);
    joint_state_control(r);
    ros::spinOnce();
    if (type_faulty_1 != "") {
      osrf_gear::VacuumGripperControl srv;
      joint_state_control("faulty_out_agv1");
      srv.request.enable = false;
      gripper_client.call(srv);
    } else {
      ros::Duration(1).sleep();
      //hold_part_to_load_point_ik(1);
      //place_part_agv_ik(target_location);
      //ros::spinOnce();
      osrf_gear::VacuumGripperControl srv;
      srv.request.enable = false;
      gripper_client.call(srv);
      // joint_state_control("agv1");
      // ros::spinOnce();
      ros::Duration(3).sleep();
      // check_orientation(target_location);
      // ros::spinOnce();
      joint_state_control("moving_to_agv1");
      ros::spinOnce();
      joint_state_control("moving_state");
      ros::spinOnce();
    }
  }

  //state machine to discard faulty parts and reorient the other parts for agv2
  void move_part_to_agv2(geometry_msgs::Pose target_location) {
     agv2_count++;
    agv2_count =agv2_count%3;
    joint_state_control("moving_state");
    ros::spinOnce();
    joint_state_control("moving_to_agv2");
    ros::spinOnce();
    joint_state_control("ag2wp");
    ros::spinOnce();
    std::string r = "agv2_pic_"+std::to_string(agv2_count+1);
    joint_state_control(r);
    ros::spinOnce();
    if (type_faulty_1 != "") {
      osrf_gear::VacuumGripperControl srv;
      joint_state_control("faulty_out_agv2");
      srv.request.enable = false;
      gripper_client.call(srv);
    } else {
      ros::Duration(1).sleep();
      //hold_part_to_load_point_ik(1);
      //place_part_agv_ik(target_location);
      //ros::spinOnce();
      osrf_gear::VacuumGripperControl srv;
      srv.request.enable = false;
      gripper_client.call(srv);
      // joint_state_control("agv1");
      // ros::spinOnce();
      ros::Duration(3).sleep();
      // check_orientation(target_location);
      // ros::spinOnce();
      joint_state_control("moving_to_agv2");
      ros::spinOnce();
      joint_state_control("moving_state");
      ros::spinOnce();
    }
  }

  //joint states for the bins
  void move_arm_over_bin(int bin) {
    int count = 1;
    while (!part_attached) {
      if (break_flag == 1) {
        // break_flag = 0;
        break;
      }
      send_arm_to_bin(bin);
      ros::spinOnce();
      pickup_part_ik(bin, count);
      ros::spinOnce();
      count = count + 0.2;
    }
    send_arm_to_bin(bin);
    ros::spinOnce();

  }
  void part_rotation(float yaw) {
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.clear();
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.points.resize(1);

    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    msg.points[0].positions = {1.385111048783651, 2.05, -0.6238696945009714, 1.412568909255488, 3.963771182986521, -1.573734238178758, yaw};
    msg.points[0].time_from_start = ros::Duration(0.001);
    ROS_INFO_STREAM("rotate yaw:" << yaw);
    joint_trajectory_publisher_.publish(msg);

    ros::Duration(2).sleep();
  }

  //THis makes it go to the parts on the agv
  void Go_to_camera_parts(std::string part_type) {
    joint_state_control("moving_state");
    ros::spinOnce();
    joint_state_control("agv2");
    ros::spinOnce();
    static const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(15.0);
    move_group.setNumPlanningAttempts(20);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //this needs to pick the part up and place it on the same or different agv
    for (int i = 0; i < camera_parts.size(); i++) {
      if (camera_parts[i].type == part_type) {
        geometry_msgs::Pose target_pose1;
        target_pose1 = camera_parts[i].pose;
        target_pose1.position.z = + 0.2;
        target_pose1.orientation.x = 0;
        target_pose1.orientation.y = 0.707;
        target_pose1.orientation.z = 0;
        target_pose1.orientation.w = 0.707;
        move_group.setPoseTarget(target_pose1);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = false;
        int ll = 0;
        while (!success) {
          if (ll > 2)
            break;
          success = (move_group.plan(my_plan) ==
                     moveit::planning_interface::MoveItErrorCode::SUCCESS);
          ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
          ll++;
        }
        //sleep(6.0);
        osrf_gear::VacuumGripperControl srv;
        srv.request.enable = true;
        gripper_client.call(srv);
        move_group.execute(my_plan);
        ros::Duration(3).sleep();
        break;
      }
    }
    joint_state_control("agv2");
    ros::spinOnce();
  }

  // function to pick up parts from the conveyor
  void pick_up_conveyor2(int u) {
    osrf_gear::VacuumGripperControl srv;
    joint_state_control("conveyor_pickup2");
    ros::spinOnce();
    srv.request.enable = true;
    gripper_client.call(srv);
    ros::spinOnce();
    ROS_INFO_STREAM(part_attached);
    while (part_attached==false) {
    ;
    }
    joint_state_control("after_conveyor_pickup2");
    ros::spinOnce();
    std::string m = "agv2_drop_" + std::to_string(u + 1);
    joint_state_control(m);
    ros::spinOnce();
    srv.request.enable = false;
    gripper_client.call(srv);
    ros::spinOnce();
    ros::Duration(0.3).sleep();
  }


  //function to send the arm to a particular bin
  void send_arm_to_bin(int bin) {
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.clear();
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.points.resize(1);

    float y = 1.2 - (8 - bin) * 0.5;
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    msg.points[0].positions = {2, y, -1, -2.5, 3.5, -1.51, 0.0};
    msg.points[0].time_from_start = ros::Duration(0.001);
    ROS_INFO_STREAM("Sending arm to bin " << bin);
    joint_trajectory_publisher_.publish(msg);
    ros::Duration(2).sleep();
  }
  double qtoa(geometry_msgs::Quaternion q) {
    double roll, pitch, yaw;
    tf::Quaternion quater;
    // quater.x = x;
    // quater.y = y;
    // quater.z = z;
    // quater.w = w;
    tf::quaternionMsgToTF(q, quater);
    tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
    yaw = angles::normalize_angle_positive(yaw);
    ROS_INFO_STREAM("yaw = " << yaw);
    return yaw;
  }


  //public variables of the class
  tf::TransformListener listener;
  std::string competition_state_;

  private:
  //private variables of the class
  double current_score_;
  ros::Publisher joint_trajectory_publisher_;
  ros::ServiceClient material_locations_client;
  std::vector<osrf_gear::Order> received_orders_;
  sensor_msgs::JointState current_joint_states_;
  bool has_been_zeroed_;
  int bin5_counter = 0;
  int bin6_counter = 0;
  int bin7_counter = 0;
  int bin8_counter = 0;

};

bool check_part_in_camera_parts(std::string current_type) {
  for (int i = 0; i < camera_parts.size(); i++) {
    if (camera_parts[i].type == current_type) {return true;}
  }

  return false;
}




int main(int argc, char ** argv) {
  // initiate a node
  ros::init(argc, argv, "ariac_example_node");

  //Initiated an asynchronous spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //A node handle initiation
  ros::NodeHandle node;
  double secs = ros::Time::now().toSec();

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/current_score' topic.
  ros::Subscriber current_score_subscriber = node.subscribe(
              "/ariac/current_score", 10,
              &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' topic.
  ros::Subscriber competition_state_subscriber = node.subscribe(
              "/ariac/competition_state", 10,
              &MyCompetitionClass::competition_state_callback, &comp_class);


  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe(
                                          "/ariac/orders", 10,
                                          &MyCompetitionClass::order_callback, &comp_class);


  // Subscribe to the '/ariac/joint_states' topic.
  ros::Subscriber joint_state_subscriber = node.subscribe(
              "/ariac/joint_states", 10,
              &MyCompetitionClass::joint_state_callback, &comp_class);

  // Subscribe to the '/ariac/gripper/state' topic.
  ros::Subscriber logical_camera_1_subscriber = node.subscribe(
              "/ariac/logical_camera_1", 10,
              &MyCompetitionClass::logical_camera1_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_2' topic.
  ros::Subscriber logical_camera_2_subscriber = node.subscribe(
              "/ariac/logical_camera_2", 10,
              &MyCompetitionClass::logical_camera2_callback, &comp_class);
  ros::Subscriber part_attached_subscriber = node.subscribe(
              "/ariac/gripper/state", 10,
              &MyCompetitionClass::part_attached_callback, &comp_class);

  // Subscribe to the '/ariac/gripper/state' topic.
  ros::Subscriber qualitySensor1_subscriber = node.subscribe(
              "/ariac/quality_control_sensor_1", 10,
              &MyCompetitionClass::faultyParts_agv1_callback, &comp_class);


  ros::Subscriber qualitySensor2_subscriber = node.subscribe(
              "/ariac/quality_control_sensor_2", 10,
              &MyCompetitionClass::faultyParts_agv2_callback, &comp_class);

  ros::ServiceClient submitAGV1_client = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
  if (!submitAGV1_client.exists()) {
    submitAGV1_client.waitForExistence();
  }
  osrf_gear::AGVControl srv1;
  ros::ServiceClient submitAGV2_client = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
  if (!submitAGV2_client.exists()) {
    submitAGV2_client.waitForExistence();
  }
  osrf_gear::AGVControl srv2;
  ros::ServiceClient endComp_client =  node.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

  ROS_INFO("Setup complete.");
  start_competition(node);
  //ros::Duration(3).sleep();
 comp_class.joint_state_control("home");
  ros::spinOnce();
  int global_count=0;
  for (int i = 0; i < Order_parts.size(); i++) {
    if (comp_class.query_part_location(Order_parts[i].models.type) == std::string("belt")){
      global_count +=1;
    }
  }
  if (global_count <= 3){
    global_count= global_count*2;
  }
  ROS_INFO_STREAM(global_count);
  comp_class.joint_state_control("before_conveyor_pickup2");
  ros::spinOnce();
  ROS_INFO_STREAM("HERE");
  for (int i = 0; i < global_count; i++) {
    // if (conv_flag == 1) {
    //   break;
    // }
    comp_class.pick_up_conveyor2(i);
  }
  
  global_count=global_count/2;

  //picking parts from conveyor 2 and placing on conveyor 1
  for(int i=0;i<global_count;i++){
    comp_class.joint_state_control("agv2");
    ros::spinOnce();
    osrf_gear::VacuumGripperControl srv;
    srv.request.enable = true;
    gripper_client.call(srv);
    ros::spinOnce();
    comp_class.joint_state_control("agv2_pick_" + std::to_string(i + 1));
    ros::spinOnce();
    if(i==0){
      comp_class.joint_state_control("after_conveyor_pickup2");
      ros::spinOnce();}
    else{
      comp_class.joint_state_control("agv2");
      ros::spinOnce();}
    comp_class.joint_state_control("moving_state");
    ros::spinOnce();
    comp_class.joint_state_control("moving_to_agv1");
    ros::spinOnce();
    comp_class.joint_state_control("agv1_drop_" + std::to_string(i + 1));
    ros::spinOnce();
    srv.request.enable = false;
    gripper_client.call(srv);
    ros::spinOnce();
    comp_class.joint_state_control("moving_to_agv1");
    ros::spinOnce();
    comp_class.joint_state_control("moving_state");
    ros::spinOnce();

  }   

  disregard = 1;
  std:: string current_agv = "agv1";
  std::string low_priority_kit = Order_parts.back().kit_to_send;
  std::string high_priority_kit = Order_parts[0].kit_to_send;
  ROS_INFO_STREAM("low_priority_kit  " << low_priority_kit );
  ROS_INFO_STREAM("high_priority_kit " << low_priority_kit );
  //Run the loops till whole competition is not done
  ROS_INFO_STREAM("ASASAS");
  int thresh = 550;
  while (ros::ok() && secs < thresh) {
    secs = ros::Time::now().toSec();
    while ((!Order_parts.empty()) && (secs < thresh)) {
      int loop_count = 1;
      ROS_INFO_STREAM("Total parts remaining to complete" << Order_parts.size());
      std::string current_type = Order_parts[0].models.type;
      // disregard=1;
      std::string loc = comp_class.query_part_location(current_type);
      if (secs > thresh) {
        break;
      }
      if (current_type == "pulley_part") {
        height_flag = 1;
      }
      if (loc == "bin5") {
        comp_class.move_arm_over_bin(5);
      }
      if (loc == "bin6") {
        comp_class.move_arm_over_bin(6);
      }
      if (loc == "bin7") {
        comp_class.move_arm_over_bin(7);
      }
      if (loc == "bin8") {
        comp_class.move_arm_over_bin(8);
      }
      if (loc == "belt") {
        ROS_INFO_STREAM("Need more parts from belt");
        Order_parts.erase(Order_parts.begin());
        dummy_list.erase(dummy_list.begin());
        for (int i=0;i<Order_parts.size();i++){ROS_INFO_STREAM(Order_parts[0].models.type);}
        continue;
      }
       ROS_INFO_STREAM(high_priority_done);
      if (Order_parts[0].kit_to_send == high_priority_kit) {
        comp_class.move_part_to_agv1(Order_parts[0].models.pose);
      } else {
        srv1.request.kit_type = high_priority_kit;
        ros::spinOnce();
        if (!submitAGV1_client.exists()) {
          submitAGV1_client.waitForExistence();
          ros::spinOnce();
        }
        submitAGV1_client.call(srv1);
        ros::spinOnce();

        comp_class.move_part_to_agv2(Order_parts[0].models.pose);
      }

      Order_parts.erase(Order_parts.begin());
      dummy_list.erase(dummy_list.begin());
      // if (high_priority_done) {
      //   srv1.request.kit_type = high_priority_kit;
      //   if (!submitAGV1_client.exists()) {
      //     submitAGV1_client.waitForExistence();
      //   }
      //   submitAGV1_client.call(srv1);
      //   ros::spinOnce();
      // }
      height_flag = 0;
      ROS_INFO_STREAM(Order_parts[0].models.type);
    }
  srv1.request.kit_type = high_priority_kit;
  ros::spinOnce();
  if (!submitAGV1_client.exists()) {
    submitAGV1_client.waitForExistence();
    ros::spinOnce();
  }
  submitAGV1_client.call(srv1);
  ros::spinOnce();
  srv2.request.kit_type = low_priority_kit;
  ros::spinOnce();
  if (!submitAGV2_client.exists()) {
    submitAGV2_client.waitForExistence();
    ros::spinOnce();
  }
  submitAGV2_client.call(srv2);
  ros::spinOnce();
}
  ros::spin();
  return 0;
}
