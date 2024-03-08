// Copyright 2023 MOBILE ROBOTICS LAB. LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// author : Robotdory (Taehyeon Kim)

#include "dwa_planner/dwa_planner.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

DWAPlanner::DWAPlanner(void)
    : local_nh_("~"),
      robot_aligned_(false),
      scan_updated_(false),
      robot_reached_(false) {
  local_nh_.param<std::string>("ROBOT_FRAME", robot_frame_, {"base_link"});
  local_nh_.param<std::string>("GOAL_FRAME_ID", goal_.header.frame_id, {"map"});
  local_nh_.param<double>("MAX_VELOCITY", max_velocity_, {0.1});
  local_nh_.param<double>("MIN_VELOCITY", min_velocity_, {0.0});
  local_nh_.param<double>("MAX_YAWRATE", max_yawrate_, {0.5});
  local_nh_.param<double>("MAX_ACCELERATION", max_acceleration_, {0.5});
  local_nh_.param<double>("MAX_DECELERATION", max_deceleration_, {1.0});
  local_nh_.param<double>("MAX_D_YAWRATE", max_d_yawrate_, {3.2});
  local_nh_.param<double>("PREDICT_TIME", predict_time_, {3.0});
  local_nh_.param<double>("DT", dt_, {0.1});
  local_nh_.param<double>("OBSTACLE_COST_GAIN", obs_cost_gain_, {1.0});
  local_nh_.param<double>("OBS_TH", obs_th_, {0.1});
  local_nh_.param<double>("DIST_COST_GAIN", dist_cost_gain_, {1.0});
  local_nh_.param<double>("DIRECTION_COST_GAIN", velocity_cost_gain_, {1.0});
  local_nh_.param<double>("DIST_THRESHOLD", dist_to_goal_th_, {0.4});
  local_nh_.param<double>("DIST_THRESHOLD", initial_dist_cost_gain, {0.4});
  local_nh_.param<double>("ANGLE_THRESHOLD", ang_to_goal_th_, {0.03});
  local_nh_.param<double>("OBS_RANGE", obs_range_, {3.0});
  local_nh_.param<int>("VELOCITY_SAMPLES", velocity_samples_, {3});
  local_nh_.param<int>("YAWRATE_SAMPLES", yawrate_samples_, {15});
  local_nh_.param<double>("GOAL_ORIENTATION", goal_.pose.orientation.w, {1.0});

  local_nh_.param<double>("X_1", goal_.pose.position.x, {2.0});
  local_nh_.param<double>("Y_1", goal_.pose.position.y, {1.0});
  local_nh_.param<double>("YAW_1", goal_yaw, {0.0});

  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  scan_sub_ = nh_.subscribe("/scan", 1, &DWAPlanner::scan_callback, this);
}
// 클래스 정의 및 초기화
DWAPlanner::State::State(void)
    : x_(0.0), y_(0.0), yaw_(0.0), velocity_(0.0), yawrate_(0.0) {}

DWAPlanner::State::State(const double x, const double y, const double yaw,
                         const double velocity, const double yawrate)
    : x_(x), y_(y), yaw_(yaw), velocity_(velocity), yawrate_(yawrate) {}

DWAPlanner::Window::Window(void)
    : min_velocity_(0.0),
      max_velocity_(0.0),
      min_yawrate_(0.0),
      max_yawrate_(0.0) {}

DWAPlanner::Window::Window(const double min_v, const double max_v,
                           const double min_y, const double max_y)
    : min_velocity_(min_v),
      max_velocity_(max_v),
      min_yawrate_(min_y),
      max_yawrate_(max_y) {}

DWAPlanner::Cost::Cost(void)
    : dist_cost_(0.0), obs_cost_(0.0), velocity_cost_(0.0), total_cost_(0.0) {}

DWAPlanner::Cost::Cost(const float dist_cost, const float obs_cost,
                       const float direction_cost, const float total_cost)
    : dist_cost_(dist_cost), obs_cost_(obs_cost), total_cost_(total_cost) {}

// 실시간 장애물 탐지를 위한 scan DATA
void DWAPlanner::scan_callback(const sensor_msgs::LaserScanConstPtr
                                   &msg) {  // 장애물 정보로 해당 값을 변환
  scan_obstacles(*msg);
  scan_updated_ = true;  // 스캔 값 업데이트를 참으로 변경
  scan_count++;
}
// 로봇의 현재 위치와 목표 위치를 계산하는 함수

bool DWAPlanner::calculate_pose(tf::StampedTransform &robot_transform) {
  try {  // "map" 좌표계에서 "base_link" 좌표의 변환 정보를 추출
    tf_listener.lookupTransform(goal_.header.frame_id, robot_frame_,
                                ros::Time(0), robot_transform);
    // 로봇의 Pose를 추출 및 대입
    robot_x = robot_transform.getOrigin().x();
    robot_y = robot_transform.getOrigin().y();
    robot_yaw = tf::getYaw(robot_transform.getRotation());
    // 목표 위치 추출 및 대입
    goal_x = goal_.pose.position.x;
    goal_y = goal_.pose.position.y;

    // 2개 좌표의 차이를 x,y로 분할해 계산
    double diff_x = goal_x - robot_x;
    double diff_y = goal_y - robot_y;
    // x,y 차이를 통해 각도 및 유클리디안 거리 계산
    target_yaw = atan2(diff_y, diff_x);
    target_angle_diff = target_yaw - robot_yaw;
    goal_angle_diff = goal_yaw - robot_yaw;
    distance_diff = hypot(diff_x, diff_y);
    return true;
  }

  catch (tf::TransformException &ex) {
    return false;  // 예외처리
  }
}

// 총 비용을 계산하는 함수
void DWAPlanner::Cost::calc_total_cost(void) {
  total_cost_ = dist_cost_ + obs_cost_ + velocity_cost_;
}

void DWAPlanner::update_local_goal() {
  if (distance_diff < dist_to_goal_th_ + 0.2) {
    if (goal_index < goals.size()) {
      goal_.pose.position.x = goals[goal_index].x;
      goal_.pose.position.y = goals[goal_index].y;
      goal_yaw = goals[goal_index].yaw;
      goal_index++;
    } else {
    }
    // 로봇의 상태를  재설정
    scan_count = 0;
    robot_aligned_ = false;
    robot_reached_ = false;
  }
}

std::vector<DWAPlanner::State> DWAPlanner::dwa_planner(
    const Eigen::Vector3d &goal,
    std::vector<std::pair<std::vector<State>, bool>> &trajectories) {
  Cost min_cost(0.0, 0.0, 0.0, 1e6);                // 비용 초기화
  Window dynamic_window = create_dynamic_window();  // 동적 창을 계산함

  const size_t trajectory_size = predict_time_ / dt_;  // 경로 예측 스텝 수 계산
  std::vector<State> best_trajectory;  // 가장 적합한 경로를 저장할 벡터 초기화
  best_trajectory.resize(trajectory_size);
  std::vector<Cost> costs;  // 비용 벡터 초기화
  // 속도 및 각속도 조합의 총 수 계산
  const size_t costs_size = velocity_samples_ * (yawrate_samples_ + 1);
  costs.reserve(costs_size);
  // 현재 선속도의 해상도는 0.05 m/s (0.2/4)
  // 현재 각속도의 해상도는 0.04 rad/s (0.6/16)
  const double velocity_resolution =
      (dynamic_window.max_velocity_ - dynamic_window.min_velocity_) /
      (velocity_samples_ - 1);
  const double yawrate_resolution =
      (dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) /
      (yawrate_samples_ - 1);
  // 유효한 경우의 수 초기화
  int available_traj_count = 0;

  // 목표까지 거리가 임계 거리 이하일 경우 gain 값을 증가시켜줌

  if (distance_diff <= 1.5) {
    dist_cost_gain_ = 2 * initial_dist_cost_gain;
  }

  // 가능한 선속도에 대해 계산
  for (int i = 0; i < velocity_samples_; i++) {
    const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
    // 가능한 각속도에 대해서도 계산
    for (int j = 0; j < yawrate_samples_; j++) {
      std::pair<std::vector<State>, bool> traj;
      const double y = dynamic_window.min_yawrate_ + yawrate_resolution * j;
      traj.first = generate_trajectory(v, y);
      Cost cost = evaluate_trajectory(traj.first, goal);
      costs.push_back(cost);

      // 장애물 충돌 예상 경로 및 정지 경로를 false로 지정
      if (cost.obs_cost_ == 1e6) {
        traj.second = false;
      } else {
        traj.second = true;  // 궤적을 유효하다 판단하여 카운트
        available_traj_count++;
      }
      trajectories.push_back(traj);
    }
  }
  // 유효한 경로가 없는 경우 로봇을 정지
  if (available_traj_count <= 0) {
    best_trajectory = generate_trajectory(0.00, 0.05);  // 충돌 방지
  } else {  // 비용 정규화 및 가중치 적용
    normalize_cost(costs);
    // 최적 경로 선택
    for (int i = 0; i < costs.size(); i++) {
      if (costs[i].obs_cost_ != 1e6)  // 유효한 궤적들에 대해서
      {
        costs[i].dist_cost_ *= dist_cost_gain_;  // 거리 가중치 추가
        costs[i].obs_cost_ *= obs_cost_gain_;    // 장애물 가중치 추가
        costs[i].velocity_cost_ *= velocity_cost_gain_;  // 속도 가중치 추가
        costs[i]
            .calc_total_cost();  // 정규화 및 가중치가 추가된 최종 비용을 도출

        // 비용이 가장 낮은 경로을 추출
        if (costs[i].total_cost_ < min_cost.total_cost_) {
          min_cost = costs[i];
          // min_cost 값 출력 및 최적의 궤적을 best_trajectory 변수에 대입
          best_trajectory = trajectories[i].first;
        }
      }
    }
  }
  // 비용 계산 및 경로 선택 로직 후, dist_cost 가중치를 원래 값으로 복원
  dist_cost_gain_ = initial_dist_cost_gain;
  return best_trajectory;  // 최적의 궤적 반환
}

geometry_msgs::Twist DWAPlanner::move_robot(void) {
  geometry_msgs::Twist cmd_vel;  // 초기화된 명령 속도 객체를 생성
  // 최적 경로 및 궤적을 저장할 변수 및 벡터를 선언
  std::pair<std::vector<State>, bool> best_trajectory;
  std::vector<std::pair<std::vector<State>, bool>> trajectories;
  // 가능한 궤적 개수를 계산하고 벡터 크기를 할당
  const size_t trajectories_size = velocity_samples_ * (yawrate_samples_ + 1);
  trajectories.reserve(trajectories_size);
  // 목표 위치를 Eigen 형식으로 변환
  const Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y,
                             tf::getYaw(goal_.pose.orientation));

  // 거리가 dist_to_goal_th_보다 큰 경우에만 최적의 경로 계산과 로봇 구동을 수행
  if (distance_diff > dist_to_goal_th_) {
    best_trajectory.first = dwa_planner(goal, trajectories);

    // 속도와 각속도가 모두 0인 경우, 다음으로 낮은 비용을 대입
    if (best_trajectory.first.front().velocity_ == 0.0 &&
        best_trajectory.first.front().yawrate_ == 0.0) {
      for (auto &trajectory : trajectories) {
        if (trajectory.first.front().velocity_ != 0.0 ||
            trajectory.first.front().yawrate_ != 0.0) {
          best_trajectory = trajectory;
          break;  // 다음으로 낮은 비용의 궤적을 찾으면 반복 중지
        }
      }
    }
    cmd_vel.linear.x = best_trajectory.first.front().velocity_;
    cmd_vel.angular.z = best_trajectory.first.front().yawrate_;
    ROS_INFO("vel : %.3f, yawrate : %.3f", cmd_vel.linear.x, cmd_vel.angular.z);
    prev_yawrate = cmd_vel.angular.z;
  } else {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    dist_cost_gain_ = initial_dist_cost_gain;
    robot_aligned_ = false;
    robot_reached_ = true;
    ROS_INFO("FINISH");
  }
  return cmd_vel;
}

geometry_msgs::Twist DWAPlanner::align_robot(void) {
  geometry_msgs::Twist cmd_vel;  // 명령 속도 메시지 생성

  // dist_to_goal_th_에 0.2를 더해줌
  dist_to_goal_th_ += 0.2;

  robot_aligned_ = false;

  if (!robot_reached_) {
    if (target_angle_diff >
        ang_to_goal_th_)  // 목표 방향과의 각도 차이가 양수인 경우
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.2;  // 양수 각속도로 회전 (시계 방향)
    } else if (target_angle_diff < -ang_to_goal_th_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = -0.2;  // 음수 각속도로 회전 (반시계 방향)
    } else {
      robot_aligned_ = true;
    }

    // dist_to_goal_th_에서 다시 0.2를 빼줌
    dist_to_goal_th_ -= 0.2;
  } else {
    if (goal_angle_diff > ang_to_goal_th_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.2;  // 양수 각속도로 회전 (시계 방향)
    } else if (goal_angle_diff < -ang_to_goal_th_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = -0.2;  // 음수 각속도로 회전 (반시계 방향)
    } else {
      robot_aligned_ = true;
    }

    // dist_to_goal_th_에서 다시 0.2를 빼줌
    dist_to_goal_th_ -= 0.2;
  }

  return cmd_vel;
}

DWAPlanner::Window DWAPlanner::create_dynamic_window(void) {
  // 초기 동적 윈도우를 생성하고 현재 설정된 속도 및 회전율 범위를 할당
  Window window(min_velocity_, max_velocity_, -max_yawrate_, max_yawrate_);
  // 현재 속도를 기반으로 윈도우의 범위를 수정
  // 선 속도의 최소값을 계산하고, 현재 속도에서 최대 감속도를 고려
  window.min_velocity_ = std::max(
      (current_cmd_vel_.linear.x - max_deceleration_ * dt_), min_velocity_);
  // 선 속도의 최대값을 계산하고, 현 속도에서 최대 감속도를 고려
  window.max_velocity_ = std::min(
      (current_cmd_vel_.linear.x + max_acceleration_ * dt_), max_velocity_);
  // 각 속도의 최소값을 계산하고, 현 속도에서 최대 감속도를 고려
  window.min_yawrate_ = std::max(
      (current_cmd_vel_.angular.z - max_d_yawrate_ * dt_), -max_yawrate_);
  // 각 속도의 최대값을 계산하고, 현 속도에서 최대 감속도를 고려
  window.max_yawrate_ = std::min(
      (current_cmd_vel_.angular.z + max_d_yawrate_ * dt_), max_yawrate_);
  return window;
}

void DWAPlanner::normalize_cost(std::vector<DWAPlanner::Cost> &costs) {
  Cost min_cost(1e6, 1e6, 1e6, 1e6), max_cost;

  for (const auto &cost : costs) {
    if (cost.obs_cost_ != 1e6)  // 장애물 충돌에 대한 비용을 무시
    {
      // 장애물 회피, 목표 도달 및 방향 비용에 대한 최소 및 최대 비용을 갱신
      min_cost.obs_cost_ = std::min(min_cost.obs_cost_, cost.obs_cost_);
      max_cost.obs_cost_ = std::max(max_cost.obs_cost_, cost.obs_cost_);
      min_cost.dist_cost_ = std::min(min_cost.dist_cost_, cost.dist_cost_);
      max_cost.dist_cost_ = std::max(max_cost.dist_cost_, cost.dist_cost_);
      min_cost.velocity_cost_ =
          std::min(min_cost.velocity_cost_, cost.velocity_cost_);
      max_cost.velocity_cost_ =
          std::max(max_cost.velocity_cost_, cost.velocity_cost_);
    }
  }

  for (auto &cost : costs) {
    if (cost.obs_cost_ != 1e6)  // 장애물 충돌에 대한 비용을 무시
    {
      // 장애물 회피, 목표 도달 및 방향 비용을 정규화
      cost.obs_cost_ = (cost.obs_cost_ - min_cost.obs_cost_) /
                       (max_cost.obs_cost_ - min_cost.obs_cost_ + DBL_EPSILON);
      cost.dist_cost_ =
          (cost.dist_cost_ - min_cost.dist_cost_) /
          (max_cost.dist_cost_ - min_cost.dist_cost_ + DBL_EPSILON);
      cost.velocity_cost_ =
          (cost.velocity_cost_ - min_cost.velocity_cost_) /
          (max_cost.velocity_cost_ - min_cost.velocity_cost_ + DBL_EPSILON);
    }
  }
}
// 궤적의 목표 지점까지의 Cost를 계산하는 함수
float DWAPlanner::calc_dist_cost(const std::vector<State> &traj,
                                 const Eigen::Vector3d &goal) {
  // 궤적의 마지막 상태를 가져와 위치 및 방향 값을 추출해 변수에 대입
  Eigen::Vector3d last_position(traj.back().x_, traj.back().y_,
                                traj.back().yaw_);
  // 목표 지점과 궤적의 마지막 위치 간의 거리를 추출
  Eigen::Vector2d position_diff =
      last_position.segment(0, 2) - goal.segment(0, 2);
  return position_diff.norm();
}

float DWAPlanner::calc_velocity_cost(const std::vector<State> &trajectory,
                                     const double prev_yawrate) {
  double yawrate_diff = 0.0;
  double velocity_cost_ = 0.0;
  double vel_th = 0.02;
  double yaw_rate_zero_cost = 0.3;  // Reduced cost for zero yaw rate

  if (trajectory.empty()) {
    yawrate_diff = 1e6;
    return yawrate_diff;  // If trajectory is empty, return a high cost
  }

  // Get the yaw rate used in the current trajectory
  double traj_yawrate = trajectory[0].yawrate_;
  double current_velocity = trajectory.front().velocity_;

  // Calculate the absolute difference between the previous and current yaw
  // rates
  yawrate_diff = std::abs(traj_yawrate - prev_yawrate);

  // Apply a lower cost if the yaw rate is zero
  if (-0.05 <= traj_yawrate <= 0.05) {
    yawrate_diff *= yaw_rate_zero_cost;
  }

  // Impose a high cost if the linear velocity is below the threshold
  if (current_velocity <= vel_th) {
    velocity_cost_ = 0.8 - current_velocity;
  }
  return yawrate_diff + velocity_cost_;
}

float DWAPlanner::calc_obstacle_cost(const std::vector<State> &traj) {
  // 초기 최소 거리를 장애물의 감지 범위로 설정
  float min_dist = obs_range_;

  // 주어진 궤적과 장애물에 대해 2중 반복문 처리
  for (const auto &state : traj) {
    for (const auto &obs : obs_list_.poses) {
      float dist;
      // 현재 위치와 장애물과의 위치의 유클리디안 거리를 계산
      dist = hypot((state.x_ - obs.position.x), (state.y_ - obs.position.y));

      // 해당 값이 0 이하일 경우 큰 값을 반환 (궤적 탈락 목적)
      if (dist <= DBL_EPSILON) return 1e6;
      // 현재 거리와 최소 거리를 비교해 갱신
      min_dist = std::min(min_dist, dist);
    }
  }
  // 장애물 감지 범위에서 최소 거리를 뺀 값을 반환
  return obs_range_ - min_dist;
}

DWAPlanner::Cost DWAPlanner::evaluate_trajectory(
    const std::vector<State> &trajectory, const Eigen::Vector3d &goal) {
  // 궤적에 대한 비용을 계산하기 위한 Cost 객체를 초기화
  Cost cost;
  // 목표 지점까지의 비용을 계산
  cost.dist_cost_ = calc_dist_cost(trajectory, goal);
  // 장애물 회피 비용을 계산
  cost.obs_cost_ = calc_obstacle_cost(trajectory);
  // 방향 비용을 계산
  cost.velocity_cost_ = calc_velocity_cost(trajectory, prev_yawrate);

  cost.calc_total_cost();
  // 비용 값 반환
  return cost;
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(
    const double velocity,
    const double yawrate) {  // 예측 시간을 기반으로 생성할 궤적의 크기를 계산합
  const size_t trajectory_size = predict_time_ / dt_;
  // 궤적을 저장할 벡터를 초기화
  std::vector<State> trajectory;
  trajectory.resize(trajectory_size);
  // 로봇의 상태를 나타내는 State 객체를 생성
  State state;
  state.x_ = robot_x;
  state.y_ = robot_y;
  state.yaw_ = robot_yaw;
  for (int i = 0; i < trajectory_size;
       i++) {  // 주어진 속도와 회전율로부터 로봇의 운동을 시뮬레이션
    state_simulation(state, velocity, yawrate);
    // 계산된 로봇의 상태를 궤적 벡터에 저장
    trajectory[i] = state;
  }
  return trajectory;  // 생성된 궤적을 반환
}

void DWAPlanner::state_simulation(State &state, const double velocity,
                                  const double yawrate) {
  // 주어진 속도와 각속도를 사용하여 로봇의 새로운 위치와 방향을 계산
  state.x_ += velocity * std::cos(state.yaw_) * dt_;
  state.y_ += velocity * std::sin(state.yaw_) * dt_;
  state.yaw_ += yawrate * dt_;

  // 속도와 각속도를 업데이트
  state.velocity_ = velocity;
  state.yawrate_ = yawrate;
}

void DWAPlanner::scan_obstacles(const sensor_msgs::LaserScan &scan) {
  obs_list_.poses.clear();
  float angle = scan.angle_min;

  for (auto r : scan.ranges) {
    // 스캔 데이터의 값이 NaN 인 경우를 확인
    if (std::isnan(r)) {
      r = 10.0;  // 스캔 데이터가 NaN이면 8.0으로 설정함
    }

    // 스캔 데이터가 없는 영역에 대해서는 r 값을 0.5로 통일해서 지정 -> 미지의
    // 영역으로의 궤적 생성 방지
    if (angle < scan.angle_min || angle > scan.angle_max) {
      r = 0.5;
    }
    // depth image에서 추출된 스캔 데이터를 통해 장애물 위치의 좌표 추출
    geometry_msgs::Pose original_pose;
    original_pose.position.x = robot_x + r * cos(angle + robot_yaw);
    original_pose.position.y = robot_y + r * sin(angle + robot_yaw);
    obs_list_.poses.push_back(original_pose);

    angle += scan.angle_increment;
  }
}

void DWAPlanner::process(void) {
  ros::Rate loop_rate(10);  // 10hz로 로봇을 구동
  while (ros::ok()) {
    geometry_msgs::Twist cmd_vel;  // 속도 변수를 초기화+
    calculate_pose(robot_transform);  // 로봇의 현재위치 및 목표 지점을 계산

    if (!robot_aligned_ && robot_reached_)  // 로봇이 정렬되지 않으면
    {
      cmd_vel = align_robot();  // 로봇 정렬을 수행
    } else {
      cmd_vel = move_robot();
    }
    current_cmd_vel_ = cmd_vel;  // 도출된 속도를 current_cmd_vel 변수에저장
    velocity_pub_.publish(cmd_vel);  // 로봇을 제어하는 함수
    scan_updated_ = false;           // scan_update 함수를 false로 초기화
    ros::spinOnce();
    loop_rate.sleep();
  }
}
