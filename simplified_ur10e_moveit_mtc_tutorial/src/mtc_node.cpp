#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <type_traits>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace {
/**
 * @brief Transform a vector of numbers into a 3D position and orientation.
 * @param values Vector containing position and orientation values.
 * @return Eigen::Isometry3d representing the transformation.
 */
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
  return Eigen::Translation3d(values[0], values[1], values[2]) *
         Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

/**
 * @brief Convert a vector of numbers to a geometry_msgs::msg::Pose.
 * @param values Vector containing position and orientation values.
 * @return geometry_msgs::msg::Pose representing the pose.
 */
geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
  return tf2::toMsg(vectorToEigen(values));
};
}  // namespace

// The next line gets a logger for our new node. We also create a namespace alias for moveit::task_constructor for convenience.
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

/**
 * @brief Class representing the MTC Task Node.
 */
class MTCTaskNode : public rclcpp::Node
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  // rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  // rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Constructor for the MTCTaskNode class.
 * @param options Node options for configuration.
 */
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : Node("mtc_node", options)
{
  auto declare_parameter = [this](const std::string& name, const auto& default_value, const std::string& description = "") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
      
    if (!this->has_parameter(name)) {
      this->declare_parameter(name, default_value, descriptor);
    }
  };

  // General parameters
  declare_parameter("execute", false, "Whether to execute the planned task");
  declare_parameter("max_solutions", 10, "Maximum number of solutions to compute");
  // declare_parameter("spawn_table", true, "Whether to spawn a table in the planning scene");

  // Controller parameters
  declare_parameter("controller_names", std::vector<std::string>{"arm_controller", "grip_action_controller"}, "Names of the controllers to use");

  // Robot configuration parameters
  declare_parameter("arm_group_name", "ur_arm", "Name of the arm group in the SRDF");
  declare_parameter("hand_group_name", "ur_grip", "Name of the gripper group in the SRDF");
  declare_parameter("hand_frame", "flange_base", "Name of the gripper frame");
  declare_parameter("hand_open_pose", "open", "Name of the gripper open pose");
  declare_parameter("hand_close_pose", "close", "Name of the gripper closed pose");
  declare_parameter("arm_home_pose", "home", "Name of the arm home pose");
  declare_parameter("arm_official_pose", "pose_official", "Name of the arm home pose");

  // Scene frame parameters
  declare_parameter("world_frame", "world", "Name of the world frame");

  // Table parameters
  // declare_parameter("table_name", "table", "Name of the table in the planning scene");
  // declare_parameter("table_reference_frame", "base_link", "Reference frame for the table");
  // declare_parameter("table_dimensions", std::vector<double>{0.10, 0.20, 0.03}, "Dimensions of the table [x, y, z]");
  // declare_parameter("table_pose", std::vector<double>{0.22, 0.12, 0.0, 0.0, 0.0, 0.0}, "Pose of the table [x, y, z, roll, pitch, yaw]");

  // Object parameters
  declare_parameter("object_name", "object", "Name of the object to be manipulated");
  declare_parameter("object_reference_frame", "world", "Reference frame for the object");
  declare_parameter("object_dimensions", std::vector<double>{0.1, 0.02}, "Dimensions of the object [height, radius]");
  declare_parameter("object_pose", std::vector<double>{0.5, -0.25, 0.0, 0.0, 0.0, 0.0}, "Initial pose of the object [x, y, z, roll, pitch, yaw]");

  // Timeout parameters
  declare_parameter("move_to_pick_timeout", 5.0, "Timeout for move to pick stage (seconds)");
  declare_parameter("move_to_place_timeout", 5.0, "Timeout for move to place stage (seconds)");

  // Motion planning parameters
  declare_parameter("approach_object_min_dist", 0.1, "Minimum approach distance to the object");
  declare_parameter("approach_object_max_dist", 0.15, "Maximum approach distance to the object");
  declare_parameter("lift_object_min_dist", 0.1, "Minimum lift distance for the object");
  declare_parameter("lift_object_max_dist", 0.3, "Maximum lift distance for the object");
  declare_parameter("lower_object_min_dist", 0.005, "Minimum distance for lowering object");
  declare_parameter("lower_object_max_dist", 0.4, "Maximum distance for lowering object");

  // Direction vector parameters
  declare_parameter("approach_object_direction_z", 1.0, "Z component of approach object direction vector");
  declare_parameter("lift_object_direction_z", 1.0, "Z component of lift object direction vector");
  declare_parameter("lower_object_direction_z", -1.0, "Z component of lower object direction vector");
  declare_parameter("retreat_direction_z", -0.5, "Z component of retreat direction vector");

  // Grasp generation parameters
  declare_parameter("grasp_pose_angle_delta", M_PI / 12, "Angular resolution for sampling grasp poses (radians)");
  declare_parameter("grasp_pose_max_ik_solutions", 8, "Maximum number of IK solutions for grasp pose generation");
  declare_parameter("grasp_pose_min_solution_distance", 1.0, "Minimum distance in joint-space units between IK solutions for grasp pose");

  // Grasp and place parameters
  declare_parameter("grasp_frame_transform", std::vector<double>{0.0, 0.0, 0.1, 0.0, M_PI / 2, 0.0}, "Transform from gripper frame to grasp frame [x, y, z, roll, pitch, yaw]"); //所夹持物体的坐标系，和hand_group的parant_link坐标系之间的关系。
  declare_parameter("place_pose", std::vector<double>{0.0, 0.5, 0.0, 0.0, 0.0, 0.0}, "Pose where the object should be placed [x, y, z, roll, pitch, yaw]");
  declare_parameter("place_surface_offset", -0.03, "Offset from the surface when placing the object");


  // Place generation parameters
  declare_parameter("place_pose_max_ik_solutions", 2, "Maximum number of IK solutions for place pose generation");

  // Cartesian planner parameters
  declare_parameter("cartesian_max_velocity_scaling", 1.0, "Max velocity scaling factor for Cartesian planner");
  declare_parameter("cartesian_max_acceleration_scaling", 1.0, "Max acceleration scaling factor for Cartesian planner");
  declare_parameter("cartesian_step_size", 0.00025, "Step size for Cartesian planner");


  // Other parameters
  declare_parameter("place_pose_z_offset_factor", 0.5, "Factor to multiply object height for place pose Z offset");
  declare_parameter("retreat_min_distance", 0.1, "Minimum distance for retreat motion");
  declare_parameter("retreat_max_distance", 0.3, "Maximum distance for retreat motion");

  RCLCPP_INFO(this->get_logger(), "All parameters have been declared with descriptions");
}

// // These next lines define a getter function to get the node base interface, which will be used for the executor later.
// rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
// {
//   return node_->get_node_base_interface();
// }

void MTCTaskNode::setupPlanningScene()
{
  // Create a planning scene interface to interact with the world
  moveit::planning_interface::PlanningSceneInterface psi;

  // Get object parameters
  auto object_name = this->get_parameter("object_name").as_string();
  auto object_dimensions = this->get_parameter("object_dimensions").as_double_array();
  auto object_pose_param = this->get_parameter("object_pose").as_double_array();
  auto object_reference_frame = this->get_parameter("object_reference_frame").as_string();

  // Create a cylinder collision object
  moveit_msgs::msg::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { object_dimensions.at(0), object_dimensions.at(1) };

  geometry_msgs::msg::Pose cylinder_pose = vectorToPose(object_pose_param);
  auto place_pose_z_offset_factor = this->get_parameter("place_pose_z_offset_factor").as_double();
  cylinder_pose.position.z += place_pose_z_offset_factor * object_dimensions[0]; // Adjust z position before creating the object
  object.primitive_poses.push_back(cylinder_pose);

  // Add the cylinder to the planning scene
  if (!psi.applyCollisionObject(object)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to spawn object: %s", object.id.c_str());
    throw std::runtime_error("Failed to spawn object: " + object.id);
  }
  RCLCPP_INFO(this->get_logger(), "Added object to planning scene");

  RCLCPP_INFO(this->get_logger(), "Planning scene setup completed");

}

/**
 * @brief Plan and/or execute the pick and place task.
 */
void MTCTaskNode::doTask()
{
  RCLCPP_INFO(this->get_logger(), "Starting the pick and place task");

  task_ = createTask();
  
  RCLCPP_INFO(this->get_logger(), "createTask() finished");

  // Get parameters
  auto execute = this->get_parameter("execute").as_bool();
  auto max_solutions = this->get_parameter("max_solutions").as_int();

  try
  {
    task_.init();
    RCLCPP_INFO(this->get_logger(), "Task initialized successfully");
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Task initialization failed: %s", e.what());
    return;
  }

  // task.plan(5) generates a plan, stopping after 5 successful plans are found. 
  if (!task_.plan(max_solutions))
  {
    RCLCPP_ERROR(this->get_logger(), "Task planning failed");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Task planning succeeded");

  // Publish the planned solution for visualization
  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(this->get_logger(), "Published solution for visualization");


  if (execute)
  {
    // Execute the planned task
    RCLCPP_INFO(this->get_logger(), "Executing the planned task");
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Task execution failed with error code: %d", result.val);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Task executed successfully");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Execution skipped as per configuration");
  }

  return;
}

/**
 * @brief Create the MTC task with all necessary stages.
 * @return The created MTC task.
 */
mtc::Task MTCTaskNode::createTask()
{
  RCLCPP_INFO(this->get_logger(), "Creating MTC task");

  // Create a new Task
  mtc::Task task;
  
  // Set the name of the task
  task.stages()->setName("demo task");
  
  // Load the robot model into the task
  task.loadRobotModel(shared_from_this(), "robot_description");

  // Get parameters
  // Robot configuration parameters
  auto arm_group_name = this->get_parameter("arm_group_name").as_string();
  auto hand_group_name = this->get_parameter("hand_group_name").as_string();
  auto hand_frame = this->get_parameter("hand_frame").as_string();
  auto arm_home_pose = this->get_parameter("arm_home_pose").as_string();
  auto arm_official_pose = this->get_parameter("arm_official_pose").as_string();
  auto hand_open_pose = this->get_parameter("hand_open_pose").as_string();
  auto hand_close_pose = this->get_parameter("hand_close_pose").as_string();

  // Frame parameters
  auto world_frame = this->get_parameter("world_frame").as_string();

  // Controller parameters
  // auto controller_names = this->get_parameter("controller_names").as_string_array();

  // Object parameters
  auto object_name = this->get_parameter("object_name").as_string();
  auto object_reference_frame = this->get_parameter("object_reference_frame").as_string();
  auto object_dimensions = this->get_parameter("object_dimensions").as_double_array();
  auto object_pose = this->get_parameter("object_pose").as_double_array();

  // Table parameters
  // auto table_name = this->get_parameter("table_name").as_string();
  // auto table_reference_frame = this->get_parameter("table_reference_frame").as_string();

  // Grasp and place parameters
  auto grasp_frame_transform = this->get_parameter("grasp_frame_transform").as_double_array();
  auto place_pose = this->get_parameter("place_pose").as_double_array();
  // auto place_surface_offset = this->get_parameter("place_surface_offset").as_double();

  // Motion planning parameters
  auto approach_object_min_dist = this->get_parameter("approach_object_min_dist").as_double();
  auto approach_object_max_dist = this->get_parameter("approach_object_max_dist").as_double();
  auto lift_object_min_dist = this->get_parameter("lift_object_min_dist").as_double();
  auto lift_object_max_dist = this->get_parameter("lift_object_max_dist").as_double();
  // auto lower_object_min_dist = this->get_parameter("lower_object_min_dist").as_double();
  // auto lower_object_max_dist = this->get_parameter("lower_object_max_dist").as_double();

  // Timeout parameters
  auto move_to_pick_timeout = this->get_parameter("move_to_pick_timeout").as_double();
  auto move_to_place_timeout = this->get_parameter("move_to_place_timeout").as_double();

  // Grasp generation parameters
  auto grasp_pose_angle_delta = this->get_parameter("grasp_pose_angle_delta").as_double();
  auto grasp_pose_max_ik_solutions = this->get_parameter("grasp_pose_max_ik_solutions").as_int();
  auto grasp_pose_min_solution_distance = this->get_parameter("grasp_pose_min_solution_distance").as_double();

  // Place generation parameters
  auto place_pose_max_ik_solutions = this->get_parameter("place_pose_max_ik_solutions").as_int();

  // Cartesian planner parameters
  auto cartesian_max_velocity_scaling = this->get_parameter("cartesian_max_velocity_scaling").as_double();
  auto cartesian_max_acceleration_scaling = this->get_parameter("cartesian_max_acceleration_scaling").as_double();
  auto cartesian_step_size = this->get_parameter("cartesian_step_size").as_double();

  // Direction vector parameters
  auto approach_object_direction_z = this->get_parameter("approach_object_direction_z").as_double();
  auto lift_object_direction_z = this->get_parameter("lift_object_direction_z").as_double();
  // auto lower_object_direction_z = this->get_parameter("lower_object_direction_z").as_double();
  auto retreat_direction_z = this->get_parameter("retreat_direction_z").as_double();

  // Other parameters
  // auto place_pose_z_offset_factor = this->get_parameter("place_pose_z_offset_factor").as_double();
  auto retreat_min_distance = this->get_parameter("retreat_min_distance").as_double();
  auto retreat_max_distance = this->get_parameter("retreat_max_distance").as_double();

  RCLCPP_INFO(this->get_logger(), "all parameter get for create task");

  // Solvers are used to define the type of robot motion. 
  // MoveIt Task Constructor has three options for solvers:

  // PipelinePlanner uses MoveIt’s planning pipeline, which typically defaults to OMPL.
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());
  RCLCPP_INFO(this->get_logger(), "OMPL planner created for the arm group");

  // JointInterpolation is a simple planner that interpolates between the start and goal joint states. 
  // It is typically used for simple motions as it computes quickly but doesn’t support complex motions.
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  RCLCPP_INFO(this->get_logger(), "Joint Interpolation planner created for the gripper group");

  // CartesianPath is used to move the end effector in a straight line in Cartesian space.
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(cartesian_max_velocity_scaling);
  cartesian_planner->setMaxAccelerationScalingFactor(cartesian_max_acceleration_scaling);
  cartesian_planner->setStepSize(cartesian_step_size);

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  // Pointer to store the current state (will be used during the grasp pose generation stage)
  mtc::Stage* current_state_ptr = nullptr;

  // Add a stage to capture the current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  /****************************************************
   *                                                  *
   *               Open Gripper                       *
   *                                                  *
   ***************************************************/
  // This stage is responsible for opening the robot's gripper in preparation for picking 
  // up an object in the pick-and-place task. 

  // The following lines use a MoveTo stage (a propagator stage). 
  // Since opening the hand is a relatively simple movement, 
  // we can use the joint interpolation planner.
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal(hand_open_pose);
  task.add(std::move(stage_open_hand));


  /****************************************************
   *                                                  *
   *               Move to Pick                       *
   *                                                  *
   ***************************************************/    
  // Create a stage to move the arm to a pre-grasp position
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ 
        { arm_group_name, sampling_planner } 
      });
  stage_move_to_pick->setTimeout(move_to_pick_timeout);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));


  // Create a pointer for the stage that will attach the object (to be used later)
  // By declaring it at the top level of the function, it can be accessed throughout 
  // the entire task creation process. 
  // This allows different parts of the code to use and modify this pointer.
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator 
	  
  /****************************************************
   *                                                  *
   *               Pick Object                        *
   *                                                  *
   ***************************************************/
  {
    // Create a serial container for the grasping action
    // This container will hold stages (in order) that will accomplish the picking action
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    
    /****************************************************
---- *               Approach Object                    *
     ***************************************************/
    {
      // Create a stage for moving the gripper close to the object before trying to grab it.	
      // We are doing a movement that is relative to our current position.	
      // Cartesian planner will move the gripper in a straight line	
      auto stage_approach_object =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage_approach_object->properties().set("marker_ns", "approach_object");
      stage_approach_object->properties().set("link", hand_frame);
      stage_approach_object->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage_approach_object->setMinMaxDistance(approach_object_min_dist, approach_object_max_dist);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame; // Set the frame for the vector
      vec.vector.z = approach_object_direction_z; // Set the direction (in this case, along the z-axis of the gripper frame)
      stage_approach_object->setDirection(vec);
      grasp->insert(std::move(stage_approach_object));       
    }
    
    /****************************************************
---- *               Generate Grasp Pose               *
     ***************************************************/    
    {
      // Sample grasp pose
      auto stage_generate_grasp_pose = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage_generate_grasp_pose->properties().configureInitFrom(mtc::Stage::PARENT);
      stage_generate_grasp_pose->properties().set("marker_ns", "grasp_pose");
      stage_generate_grasp_pose->setPreGraspPose(hand_open_pose);
      stage_generate_grasp_pose->setObject(object_name);
      stage_generate_grasp_pose->setAngleDelta(grasp_pose_angle_delta);
      stage_generate_grasp_pose->setMonitoredStage(current_state_ptr);  // Hook into current state 

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage_generate_grasp_pose));
      wrapper->setMaxIKSolutions(grasp_pose_max_ik_solutions);
      wrapper->setMinSolutionDistance(grasp_pose_min_solution_distance);
      wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper)); 
    }

    /****************************************************
---- *            Allow Collision (gripper,  object)   *
     ***************************************************/
    {
      // Modify planning scene (w/o altering the robot's pose) to allow touching the object for picking
      auto stage_allow_collision =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage_allow_collision->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage_allow_collision));
    }

    /****************************************************
---- *               Close Gripper                     *
     ***************************************************/
    {
      auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage_close_hand->setGroup(hand_group_name);
      stage_close_hand->setGoal(hand_close_pose);
      grasp->insert(std::move(stage_close_hand));
    }

    /****************************************************
---- *               Attach Object                     *
     ***************************************************/
    {
      auto stage_attach_object = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage_attach_object->attachObject(object_name, hand_frame);
      attach_object_stage = stage_attach_object.get();
      grasp->insert(std::move(stage_attach_object));
    }
    
    /****************************************************
---- *       Lift object                               *
     ***************************************************/
    {
      auto stage_lift_object =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage_lift_object->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage_lift_object->setMinMaxDistance(lift_object_min_dist, lift_object_max_dist);
      stage_lift_object->setIKFrame(hand_frame); //干什么用？为何approach object stage没有设置此参数
      stage_lift_object->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;
      vec.vector.z = lift_object_direction_z;
      stage_lift_object->setDirection(vec);
      grasp->insert(std::move(stage_lift_object));
    }

    task.add(std::move(grasp));
  }

  /******************************************************
   *                                                    *
   *          Move to Place                             *
   *                                                    *
   *****************************************************/
  {
    // Connect the grasped state to the pre-place state, i.e. realize the object transport
    // In other words, this stage plans the motion that transports the object from where it was picked up 
    // to where it will be placed.
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(move_to_place_timeout);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place)); 
  }

  /******************************************************
   *                                                    *
   *          Place Object                              *
   *                                                    *
   *****************************************************/
   // All placing sub-stages are collected within a serial container 
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    /******************************************************
---- *          Generate Place Pose                       *
     *****************************************************/
    {
      // Sample place pose
      auto stage_generate_place_pose = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage_generate_place_pose->properties().configureInitFrom(mtc::Stage::PARENT);
      stage_generate_place_pose->properties().set("marker_ns", "place_pose");
      stage_generate_place_pose->setObject(object_name);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = object_name;
      target_pose_msg.pose = vectorToPose(place_pose);
      stage_generate_place_pose->setPose(target_pose_msg);
      stage_generate_place_pose->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage_generate_place_pose));
      wrapper->setMaxIKSolutions(place_pose_max_ik_solutions);
      // wrapper->setMinSolutionDistance(2.0);
      // wrapper->setIKFrame(object_name);
      wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    /******************************************************
---- *          Open Gripper                              *
     *****************************************************/
    {
      auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage_open_hand->setGroup(hand_group_name);
      stage_open_hand->setGoal(hand_open_pose);
      place->insert(std::move(stage_open_hand));
    }

    /******************************************************
---- *          Forbid collision (gripper, object)        *
     *****************************************************/
    {
      auto stage_forbid_collision =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage_forbid_collision->allowCollisions(object_name,
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      place->insert(std::move(stage_forbid_collision));
    }

    /******************************************************
---- *          Detach Object                             *
     *****************************************************/
    {
      auto stage_detach_object = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage_detach_object->detachObject(object_name, hand_frame);
      place->insert(std::move(stage_detach_object));
    }

    /******************************************************
---- *          Retreat Motion                            *
     *****************************************************/
    {
      auto stage_retreat = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage_retreat->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage_retreat->setMinMaxDistance(retreat_min_distance, retreat_max_distance);
      stage_retreat->setIKFrame(hand_frame);
      stage_retreat->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = retreat_direction_z;
      stage_retreat->setDirection(vec);
      place->insert(std::move(stage_retreat));   
    }

    // Add place container to task
    task.add(std::move(place));
  }

  /******************************************************
   *                                                    *
   *          Move to Home                              *
   *                                                    *
   *****************************************************/
  {
    auto stage_return_home = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage_return_home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage_return_home->setGoal(arm_official_pose);
    task.add(std::move(stage_return_home));                                              
  }                      

  return task;
}

/**
 * @brief Main function to run the MTC task node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status.
 */
int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Set up node options
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  // Create the MTC task node
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  // Set up a multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_task_node);

  // auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
  //   executor.add_node(mtc_task_node->getNodeBaseInterface());
  //   executor.spin();
  //   executor.remove_node(mtc_task_node->getNodeBaseInterface());
  // });

   // Set up the planning scene and execute the task
  try {
    RCLCPP_INFO(mtc_task_node->get_logger(), "Setting up planning scene");
    mtc_task_node->setupPlanningScene();
    RCLCPP_INFO(mtc_task_node->get_logger(), "Executing task");
    mtc_task_node->doTask();
    RCLCPP_INFO(mtc_task_node->get_logger(), "Task execution completed. Keeping node alive for visualization. Press Ctrl+C to exit.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(mtc_task_node->get_logger(), "An error occurred: %s", e.what());
  }

  // Keep the node running until Ctrl+C is pressed
  executor.spin();

  // Cleanup
  rclcpp::shutdown();

  // mtc_task_node->setupPlanningScene();
  // mtc_task_node->doTask();
  // spin_thread->join();
  // rclcpp::shutdown();
  return 0;
}