/**
 * @file alternative_path_costs.cpp
 * @brief Demonstrates different path planning strategies using MoveIt Task Constructor.
 *
 * This program sets up a Task Constructor task to plan robot motion using different
 * cost optimization strategies. It creates multiple path planning alternatives and
 * evaluates them based on different cost terms such as path length, trajectory duration,
 * end-effector motion, and elbow motion.
 *
 * @author Addison Sears-Collins
 * @date August 16, 2024
 */


#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/cost_terms.h>
#include <iostream> 

using namespace moveit::task_constructor;

/**
 * @brief Main function to demonstrate alternative path costs.
 *
 * This function sets up a ROS 2 node, creates a Task Constructor task,
 * and demonstrates different path planning strategies using various cost terms.
 * FixedState (Generator Stage) - Connect (Connector Stage) - FixedState (Generator Stage)
 *
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return int Exit status
 */
int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    std::cout << "ROS 2 initialized for alternative_path_costs_demo" << std::endl;
    
    // Set up ROS 2 node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    // Create a ROS 2 node
    auto node = rclcpp::Node::make_shared("alternative_path_costs_demo", node_options);
    std::cout << "Created ROS 2 node: alternative_path_costs_demo" << std::endl;
    
    // Start a separate thread to handle ROS 2 callbacks
    std::thread spinning_thread([node] { rclcpp::spin(node); });

    // Create a Task Constructor task
    Task t;
    t.stages()->setName("IK test");
    std::cout << "Created Task Constructor task: IK test" << std::endl;
    
    // Load the robot model
    t.loadRobotModel(node);
    std::cout << "Loaded robot model: " << t.getRobotModel()->getName() << std::endl;

    // Ensure the correct robot model is loaded
    assert(t.getRobotModel()->getName() == "ur10e");

    // Create a planning scene
    auto scene{ std::make_shared<planning_scene::PlanningScene>(t.getRobotModel()) };
    std::cout << "Created planning scene" << std::endl;
    
    // Add a collision object (box) to the scene
    scene->processCollisionObjectMsg([]() {
      moveit_msgs::msg::CollisionObject co;
      co.id = "table";
      co.header.frame_id = "base_link";
      co.operation = co.ADD;
      co.pose = []() {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.0;
        p.position.y = 0.0;
        p.position.z = -0.005;
        p.orientation.w = 1.0;
        return p;
      }();
      co.primitives.push_back([]() {
        shape_msgs::msg::SolidPrimitive sp;
        sp.type = sp.BOX;
        sp.dimensions = { 10, 10, 0.01 };
        return sp;
      }());
      return co;
    }());

    // Get the current robot state and set it to default values
    auto& robot_state{ scene->getCurrentStateNonConst() };
    robot_state.setToDefaultValues();
    
    // robot_state.setToDefaultValues(robot_state.getJointModelGroup("ur_arm"), "home");
    std::cout << "Initialized robot state in planning scene to default values and 'home' position" << std::endl;

    // Create a interpolation planner
    // The "pipeline" part means it can chain together multiple planning attempts or strategies. 
    // If one method fails, it can try another
    // auto pipeline{ std::make_shared<solvers::PipelinePlanner>(node) };
    auto interpolation_planner = std::make_shared<solvers::JointInterpolationPlanner>();
    std::cout << "Created interpolation planner" << std::endl;

    // OMPL planner
    auto ompl_planner_arm = std::make_shared<solvers::PipelinePlanner>(node);
    std::cout << "OMPL planner created for the arm group"<< std::endl;
	

    // Create and add the initial state to the task
    // This step gives MoveIt 2 a clear starting point from which to plan the robot's movements.
    auto initial = std::make_unique<stages::FixedState>("start");
    initial->setState(scene);
    t.add(std::move(initial));
    std::cout << "Added initial state to the task" << std::endl;

    // Create an Alternatives container for different path planning strategies
    // Alternatives container is a type of Parallel container. It's used to group multiple stages that 
    // represent different ways to accomplish the same task.
    // In this case, it's used to create different strategies for connecting the intial state to the goal state. 
    // The planner will try all these strategies and choose the best one based on their respective cost terms.
    
    // auto alternatives = std::make_unique<Alternatives>("MoveTo");
    // std::cout << "Created Alternatives container for path planning strategies" << std::endl;
    
    // {
    //   auto stage = std::make_unique<stages::MoveTo>("move to ready (joint space goal: interpolation planner)", interpolation_planner);//ompl_planner_arm,interpolation_planner
    //   stage->properties().set("marker_ns", "move_to_ready");
    //   stage->setGroup("ur_arm");
    //   stage->setGoal("pose_random1");
    //   stage->restrictDirection(stages::MoveTo::FORWARD);
    //   alternatives->add(std::move(stage));
    // }

    // {
    //   auto stage = std::make_unique<stages::MoveTo>("move to ready (joint space goal: ompl planner)", ompl_planner_arm);//ompl_planner_arm,interpolation_planner
    //   stage->properties().set("marker_ns", "move_to_ready");
    //   stage->setGroup("ur_arm");
    //   stage->setGoal("pose_random1");
    //   stage->restrictDirection(stages::MoveTo::FORWARD);
    //   alternatives->add(std::move(stage));
    // }

    // {
    //   auto stage = std::make_unique<stages::MoveTo>("move to ready (cartesian space goal: interpolation planner)", interpolation_planner);//ompl_planner_arm,interpolation_planner
    //   stage->properties().set("marker_ns", "move_to_ready");
    //   stage->setGroup("ur_arm");
    //   geometry_msgs::msg::PoseStamped ik_pose_msg;
    //   ik_pose_msg.header.frame_id = "base_link";
    //   ik_pose_msg.pose.position.x = 0.173923;
    //   ik_pose_msg.pose.position.y = -0.452194;
    //   ik_pose_msg.pose.position.z = 0.959854;
    //   ik_pose_msg.pose.orientation.x = -0.000398163;
    //   ik_pose_msg.pose.orientation.y = 0.000398163;
    //   ik_pose_msg.pose.orientation.z = -0.000796485;
    //   ik_pose_msg.pose.orientation.w = 1;
    //   stage->setGoal(ik_pose_msg);
    //   stage->restrictDirection(stages::MoveTo::FORWARD);
    //   alternatives->add(std::move(stage));
    // }

    // {
    //   auto stage = std::make_unique<stages::MoveTo>("move to ready (cartesian space goal: ompl planner)", ompl_planner_arm);//ompl_planner_arm,interpolation_planner
    //   stage->properties().set("marker_ns", "move_to_ready");
    //   stage->setGroup("ur_arm");
    //   geometry_msgs::msg::PoseStamped ik_pose_msg;
    //   ik_pose_msg.header.frame_id = "base_link";
    //   ik_pose_msg.pose.position.x = 0.173923;
    //   ik_pose_msg.pose.position.y = -0.452194;
    //   ik_pose_msg.pose.position.z = 0.959854;
    //   ik_pose_msg.pose.orientation.x = -0.000398163;
    //   ik_pose_msg.pose.orientation.y = 0.000398163;
    //   ik_pose_msg.pose.orientation.z = -0.000796485;
    //   ik_pose_msg.pose.orientation.w = 1;
    //   stage->setGoal(ik_pose_msg);
    //   stage->restrictDirection(stages::MoveTo::FORWARD);
    //   alternatives->add(std::move(stage));
    // }

    {
      auto stage = std::make_unique<stages::MoveTo>("move to ready (cartesian space goal: ompl planner)", ompl_planner_arm);//ompl_planner_arm,interpolation_planner
      stage->properties().set("marker_ns", "move_to_ready");
      stage->setGroup("ur_arm");
      geometry_msgs::msg::PoseStamped ik_pose_msg;
      ik_pose_msg.header.frame_id = "base_link";
      ik_pose_msg.pose.position.x = 0.173923;
      ik_pose_msg.pose.position.y = -0.452194;
      ik_pose_msg.pose.position.z = 0.959854;
      ik_pose_msg.pose.orientation.x = -0.000398163;
      ik_pose_msg.pose.orientation.y = 0.000398163;
      ik_pose_msg.pose.orientation.z = -0.000796485;
      ik_pose_msg.pose.orientation.w = 1;
      stage->setGoal(ik_pose_msg);
      stage->restrictDirection(stages::MoveTo::FORWARD);
      stage->setTimeout(5.0);
      // alternatives->add(std::move(stage));
      t.add(std::move(stage));
    }
    
    // Add the Alternatives container to the task
    // By adding this Alternatives container to the task, we're giving the task planner multiple options for 
    // solving a particular part of the robot's movement. The planner can now choose the best strategy based on the current situation.
    // t.add(std::move(alternatives));
    std::cout << "Added all strategies in the Alternatives Container to the task" << std::endl;
    
    // Plan the task
    std::cout << "Starting task planning..." << std::endl;
    try {
      moveit::core::MoveItErrorCode error_code = t.plan(2); // The 0 parameter means it will generate as many solutions as possible
      if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
        std::cout << "Task planning completed successfully" << std::endl;
      } else {
        std::cout << "Task planning failed with error code: " << error_code.val << std::endl;
        std::ostringstream explanation;
        t.explainFailure(explanation);
        std::cout << "explanation: " << explanation.str().c_str() << std::endl;
      }
      
      // Print the results
      std::cout << "Planning results:" << std::endl;
      t.printState();
           
    } catch (const InitStageException& e) {
        std::cout << "Task planning failed: " << e << std::endl;
    }
    
    // Keep the node alive for interactive inspection in RViz
    std::cout << "Keeping node alive for RViz inspection. Press Ctrl+C to exit." << std::endl;
    spinning_thread.join();

    return 0;
}

