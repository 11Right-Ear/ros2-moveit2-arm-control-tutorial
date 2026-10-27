#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

// class dengyinghao{
//     public:
//     dengyinghao() = default;
//     dengyinghao(std::string s) : s_(s){return s};
//     std::string Get(){return s_};
//     void Set(std::string& s){return s_ = s;}

//     int a
//     private:
//     std::string s_;
// }

//VERSION 扩展：ROS1 && ROS2
//使用类更常见

int main(int argc, char **argv)
{
    // std::string s = "hello deng";
    // dengyinghao d(s);
    
    rclcpp::init(argc, argv);
    //ros::init(argc, argv, "test_moveit");

    // auto node = std::make_shared<rclcpp::Node>("test_moveit");
    // auto 本质上先声明了一个共享指针类型是rclcpp::Node的，再将这个变量指向构造的make_shared<rclcpp::Node>
    std::shared_ptr<rclcpp::Node> nodemain;
    nodemain = std::make_shared<rclcpp::Node>("test_moveit");
    // ros1 中是这样写
    // ros::NodeHandle nh; //声明一个句柄持有者，它类似node能pub,sub。

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nodemain);
    // ros::AsyncSpinner spinner(1); // 异步spinner 1个线程
    auto spinner = std::thread([&executor]() {
        executor.spin();
    });
    // spinner.start(); 等价于executor.spin();

    // auto arm = moveit::planning_interface::MoveGroupInterface(nodemain, "arm");
    moveit::planning_interface::MoveGroupInterface arm1(nodemain, "arm");
    //这样写的目的是要换arm只需该arm1，之后所有的都不用改。
    moveit::planning_interface::MoveGroupInterface& arm = arm1;

    //设置机械臂的速度和加速度范围 - > Scaling 单位应该是m/s
    //这个类在源码里，此仓库看不了，找到它是通过.so动态库找到的
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    // auto gripper = moveit::planning_interface::MoveGroupInterface(nodemain, "gripper");
    moveit::planning_interface::MoveGroupInterface gripper1(nodemain, "gripper");
    moveit::planning_interface::MoveGroupInterface& gripper = gripper1;


    // // Named goal
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;

    // bool success1 = static_cast<bool>(arm.plan(plan1));

    // if (success1)
    // {
    //     arm.execute(plan1);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    // }

    // // Named goal
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;

    // bool success2 = static_cast<bool>(arm.plan(plan2));

    // if (success2)
    // {
    //     arm.execute(plan2);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    // }

    //--------------------------------------------------------------------------------

    // //Joint Goal
    // std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };

    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;

    // bool success1 =
    //     (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1)
    // {
    //     arm.execute(plan1);
    // }


    //---------------------------------------------------------------------------------

    // Pose Goal
    //默认构造了一个表示姿态的对象，位置，角度,里面的方法可以相互转换
    tf2::Quaternion q;
    //参数顺序很重要
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    //没有基于frame_id的参考坐标系，下面每一个数都没有意义。
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    //内部做了什么？？？
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    //使用了什么规划方法？
    moveit::planning_interface::MoveGroupInterface::Plan plan1;


    bool success1 =
        (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1)
    {
        //执行器用的是什么跟踪算法去跟踪的？
        arm.execute(plan1);
    }

    // Cartesian Path

    std::vector<geometry_msgs::msg::Pose> waypoints;

    //这个数据结构放在这里是为了起什么作用？？
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.z += 0.2;
    waypoints.push_back(pose1);

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = arm.computeCartesianPath(
        waypoints,
        0.01,
        0.0,
        trajectory,
        true,
        nullptr
    );

    if (fraction == 1.0)
    {
        arm.execute(trajectory);
    }

    rclcpp::shutdown();
    spinner.join();

    return 0;
}