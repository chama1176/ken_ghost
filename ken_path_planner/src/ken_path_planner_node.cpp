#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <chrono>
#include <functional>

#include <eigen3/Eigen/Dense>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "ken_msgs/msg/mission_target_array.hpp"
#include "ken_msgs/msg/mission_trajectory.hpp"
#include "ken_path_planner/ken_fk.hpp"
#include "ken_path_planner/ken_ik.hpp"

using std::placeholders::_1;

class KenPathPlanner : public rclcpp::Node
{
public:
  /* 
  * コンストラクタ
  */
  KenPathPlanner();
  /*
  * デストラクタ
  */
  ~KenPathPlanner();

private:
  void makeMoveHomeTrajectory(trajectory_msgs::msg::JointTrajectory & jtm);
  void makeMoveKamaeTrajectory(trajectory_msgs::msg::JointTrajectory & jtm);
  void makeMenTrajectory(
    trajectory_msgs::msg::JointTrajectory & jtm, const geometry_msgs::msg::Pose pose);

  void pushInterpolateTrajectoryPoints(
    trajectory_msgs::msg::JointTrajectory & jtm,
    const trajectory_msgs::msg::JointTrajectoryPoint start,
    const trajectory_msgs::msg::JointTrajectoryPoint end, int interpolate_num);

  void publishIKlog(const KenIK & ik);

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void mission_target_callback(const ken_msgs::msg::MissionTargetArray::SharedPtr msg);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fk_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ik_debug_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_x_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_y_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_z_debug_pub_;

  rclcpp::Publisher<ken_msgs::msg::MissionTrajectory>::SharedPtr mission_trajectory_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Subscription<ken_msgs::msg::MissionTargetArray>::SharedPtr mission_target_sub_;

  int64_t joint_num_;
  double move_time_;

  const std::string base_frame_id_;

  std::vector<std::string> name_vec_;
  std::vector<double> current_pos_;
  std::vector<double> kamae_pos_;

  bool received_joint_state_msg_;
};

KenPathPlanner::KenPathPlanner() : Node("ken_path_planner"), base_frame_id_("base_link")
{
  this->declare_parameter("joint_num", -1);
  this->get_parameter("joint_num", joint_num_);

  this->declare_parameter("move_time", 0.0);
  this->get_parameter("move_time", move_time_);

  this->declare_parameter("ik_test_pos_x", 0.0);
  this->declare_parameter("ik_test_pos_y", 0.0);
  this->declare_parameter("ik_test_pos_z", 0.0);

  if (joint_num_ > 0) {
    this->declare_parameter("name", std::vector<std::string>(joint_num_, ""));
    this->get_parameter("name", name_vec_);

    this->declare_parameter("kamae_pos", std::vector<double>(joint_num_, 0.0));
    this->get_parameter("kamae_pos", kamae_pos_);

    current_pos_.resize(joint_num_);

    for (size_t i = 0; i < (size_t)joint_num_; ++i) {
      RCLCPP_INFO(this->get_logger(), "Axis %d %s", i, name_vec_[i].c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Move time: %f", move_time_);
  RCLCPP_INFO(this->get_logger(), "Joint num: %d", joint_num_);

  received_joint_state_msg_ = false;

  cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "ken_joint_trajectory_controller/joint_trajectory", 1);
  fk_debug_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/fk_pose", 1);
  ik_debug_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("debug/ik_pose", 1);
  point_x_debug_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("debug/point_x", 1);
  point_y_debug_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("debug/point_y", 1);
  point_z_debug_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("debug/point_z", 1);

  mission_trajectory_pub_ =
    this->create_publisher<ken_msgs::msg::MissionTrajectory>("mission_trajectory", 1);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 1, std::bind(&KenPathPlanner::joint_state_callback, this, _1));

  mission_target_sub_ = this->create_subscription<ken_msgs::msg::MissionTargetArray>(
    "mission_target", 1, std::bind(&KenPathPlanner::mission_target_callback, this, _1));

  std::cout << "Finish Initialization" << std::endl;
}

KenPathPlanner::~KenPathPlanner() {}

void KenPathPlanner::pushInterpolateTrajectoryPoints(
  trajectory_msgs::msg::JointTrajectory & jtm,
  const trajectory_msgs::msg::JointTrajectoryPoint start,
  const trajectory_msgs::msg::JointTrajectoryPoint end, int interpolate_num)
{
  rclcpp::Duration duration_btwn_point = end.time_from_start;
  duration_btwn_point = duration_btwn_point - start.time_from_start;

  for (int i = 0; i < interpolate_num; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start =
      rclcpp::Duration(duration_btwn_point.nanoseconds() / interpolate_num * i) +
      start.time_from_start;
    for (size_t idx = 0; idx < start.positions.size(); ++idx) {
      point.positions.push_back(
        (end.positions[idx] - start.positions[idx]) / interpolate_num * i + start.positions[idx]);
    }
    jtm.points.push_back(point);
  }
  jtm.points.push_back(end);
}

void KenPathPlanner::makeMoveHomeTrajectory(trajectory_msgs::msg::JointTrajectory & jtm)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start.sec = 0;
  start_point.time_from_start.nanosec = 0;
  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    start_point.positions.push_back(current_pos_[i]);
  }
  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.time_from_start.sec = (uint32_t)move_time_;
  end_point.time_from_start.nanosec =
    (uint32_t)((move_time_ - end_point.time_from_start.sec) * 1e9);
  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    end_point.positions.push_back(0.0);
  }

  pushInterpolateTrajectoryPoints(jtm, start_point, end_point, 100);
}

void KenPathPlanner::makeMoveKamaeTrajectory(trajectory_msgs::msg::JointTrajectory & jtm)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start.sec = 0;
  start_point.time_from_start.nanosec = 0;
  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    start_point.positions.push_back(current_pos_[i]);
  }

  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.time_from_start.sec = (uint32_t)move_time_;
  end_point.time_from_start.nanosec =
    (uint32_t)((move_time_ - end_point.time_from_start.sec) * 1e9);
  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    end_point.positions.push_back(kamae_pos_[i]);
  }

  pushInterpolateTrajectoryPoints(jtm, start_point, end_point, 100);
}

void KenPathPlanner::makeMenTrajectory(
  trajectory_msgs::msg::JointTrajectory & jtm, const geometry_msgs::msg::Pose pose)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start.sec = 0;
  start_point.time_from_start.nanosec = 0;
  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    start_point.positions.push_back(current_pos_[i]);
  }

  trajectory_msgs::msg::JointTrajectoryPoint via_point;
  via_point.time_from_start.sec = (uint32_t)move_time_ / 2;
  via_point.time_from_start.nanosec =
    (uint32_t)((move_time_ / 2 - via_point.time_from_start.sec) * 1e9);

  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.time_from_start.sec = (uint32_t)move_time_;
  end_point.time_from_start.nanosec =
    (uint32_t)((move_time_ - end_point.time_from_start.sec) * 1e9);

  Eigen::Vector3d target_pos = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  KenIK ik;
  std::vector<double> ik_ans;
  bool is_succeed_ik = ik.calcPositionIK(target_pos, kamae_pos_, ik_ans);
  publishIKlog(ik);
  // TODO: write when ik solver is failed
  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    end_point.positions.push_back(ik_ans[i]);
  }

  via_point.positions = end_point.positions;
  via_point.positions[3] = 0.0;
  via_point.positions[4] = 0.0;

  pushInterpolateTrajectoryPoints(jtm, start_point, via_point, 100);
  pushInterpolateTrajectoryPoints(jtm, via_point, end_point, 100);
}

void KenPathPlanner::publishIKlog(const KenIK & ik)
{
  // Publish IK calculation log
  std::vector<Eigen::Matrix4d> ik_Tbe_log = ik.getIKlogTbe();

  geometry_msgs::msg::PoseArray ik_debug_pose_array;
  ik_debug_pose_array.header.frame_id = base_frame_id_;
  ik_debug_pose_array.header.stamp = rclcpp::Time(0);
  for (size_t i = 0; i < ik_Tbe_log.size(); ++i) {
    geometry_msgs::msg::Pose Tbepose;
    Tbepose.position.x = ik_Tbe_log[i](0, 3);
    Tbepose.position.y = ik_Tbe_log[i](1, 3);
    Tbepose.position.z = ik_Tbe_log[i](2, 3);
    Eigen::Quaterniond Tbe_q(ik_Tbe_log[i].block<3, 3>(0, 0));
    Tbepose.orientation.x = Tbe_q.x();
    Tbepose.orientation.y = Tbe_q.y();
    Tbepose.orientation.z = Tbe_q.z();
    Tbepose.orientation.w = Tbe_q.w();

    ik_debug_pose_array.poses.push_back(Tbepose);
  }
  ik_debug_pose_pub_->publish(ik_debug_pose_array);
}

void KenPathPlanner::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() == (size_t)joint_num_) {
    for (size_t i = 0; i < (size_t)joint_num_; ++i) {
      current_pos_[i] = msg->position[i];
    }
    received_joint_state_msg_ = true;
  }

  // Code for checking forward kinematics

  KenFK fk(current_pos_);
  Eigen::Matrix4d Tbe = fk.computeTbe();

  Eigen::Quaterniond Tbe_q(Tbe.block<3, 3>(0, 0));
  geometry_msgs::msg::PoseStamped pose_pub;
  pose_pub.header.frame_id = base_frame_id_;
  pose_pub.header.stamp = rclcpp::Time(0);
  pose_pub.pose.position.x = Tbe(0, 3);
  pose_pub.pose.position.y = Tbe(1, 3);
  pose_pub.pose.position.z = Tbe(2, 3);
  pose_pub.pose.orientation.x = Tbe_q.x();
  pose_pub.pose.orientation.y = Tbe_q.y();
  pose_pub.pose.orientation.z = Tbe_q.z();
  pose_pub.pose.orientation.w = Tbe_q.w();

  fk_debug_pub_->publish(pose_pub);

  // Code for checking Jacobian
  Eigen::MatrixXd Jv = fk.computeJv();
  // std::cout << "Jv" << std::endl;
  // std::cout << Jv << std::endl;
  Eigen::MatrixXd A = Jv * Jv.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A.block<3, 3>(0, 0));
  // if (es.info() == Eigen::Success) {
  //   std::cout << "A es" << std::endl;
  //   std::cout << es.eigenvalues() << std::endl;
  //   std::cout << es.eigenvectors() << std::endl;
  // }
  geometry_msgs::msg::PointStamped point_x;
  point_x.header.frame_id = base_frame_id_;
  point_x.header.stamp = rclcpp::Time(0);
  point_x.point.x = es.eigenvectors().col(0)(0) * es.eigenvalues()(0) + Tbe(0, 3);
  point_x.point.y = es.eigenvectors().col(0)(1) * es.eigenvalues()(0) + Tbe(1, 3);
  point_x.point.z = es.eigenvectors().col(0)(2) * es.eigenvalues()(0) + Tbe(2, 3);
  point_x_debug_pub_->publish(point_x);

  geometry_msgs::msg::PointStamped point_y;
  point_y.header.frame_id = base_frame_id_;
  point_y.header.stamp = rclcpp::Time(0);
  point_y.point.x = es.eigenvectors().col(1)(0) * es.eigenvalues()(1) + Tbe(0, 3);
  point_y.point.y = es.eigenvectors().col(1)(1) * es.eigenvalues()(1) + Tbe(1, 3);
  point_y.point.z = es.eigenvectors().col(1)(2) * es.eigenvalues()(1) + Tbe(2, 3);
  point_y_debug_pub_->publish(point_y);

  geometry_msgs::msg::PointStamped point_z;
  point_z.header.frame_id = base_frame_id_;
  point_z.header.stamp = rclcpp::Time(0);
  point_z.point.x = es.eigenvectors().col(2)(0) * es.eigenvalues()(2) + Tbe(0, 3);
  point_z.point.y = es.eigenvectors().col(2)(1) * es.eigenvalues()(2) + Tbe(1, 3);
  point_z.point.z = es.eigenvectors().col(2)(2) * es.eigenvalues()(2) + Tbe(2, 3);
  point_z_debug_pub_->publish(point_z);
}

void KenPathPlanner::mission_target_callback(const ken_msgs::msg::MissionTargetArray::SharedPtr msg)
{
  ken_msgs::msg::MissionTrajectory mtm;
  mtm.header.frame_id = base_frame_id_;
  mtm.header.stamp = rclcpp::Time(0);
  mtm.plan_result = true;
  mtm.type = msg->type;

  for (int i = (int)msg->type.size() - 1; i >= 0; --i) {
    trajectory_msgs::msg::JointTrajectory jt;
    std::string jt_name;
    if (msg->type[i] == msg->HOME) {
      makeMoveHomeTrajectory(jt);
      jt_name = "home";
    } else if (msg->type[i] == msg->KAMAE) {
      makeMoveKamaeTrajectory(jt);
      jt_name = "kamae";
    } else if (msg->type[i] == msg->MEN) {
      // TODO: IKが失敗したときにもfalseを返すようにする
      makeMenTrajectory(jt, msg->poses[i]);
      jt_name = "men";
    } else {
      mtm.plan_result = false;
      break;
    }

    mtm.trajectories.push_back(jt);
    mtm.type_names.push_back(jt_name);
  }

  mission_trajectory_pub_->publish(mtm);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenPathPlanner>());
  rclcpp::shutdown();
  return 0;
}