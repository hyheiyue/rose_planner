#include "rose_planner.hpp"
#include "control/osqp_mpc.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "parameters.hpp"
#include "path_search/a*.hpp"
#include "replan_fsm.hpp"
#include "rose_map/rose_map.hpp"
#include "trajectory_optimize/trajectory_opt.hpp"
#include "trajectory_optimize/trajectory_sampler.hpp"
#include <angles.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
//clang-format off
#include "control/acado_mpc.hpp"
//clang-format on
namespace rose_planner {
struct RosePlanner::Impl {
public:
    using SearchType = AStar;
    using MpcType = control::OsqpMpc;
    double default_wz = 0.0;
    ~Impl() {
        running_ = false;
        if (timer_thread_.joinable())
            timer_thread_.join();
        if (control_timer_thread_.joinable())
            control_timer_thread_.join();
    }

    Impl(rclcpp::Node& node) {
        node_ = &node;
        tf_ = TF::create(node);
        parameters_.load(node);

        rose_map_ = std::make_shared<rose_map::RoseMap>(node);
        path_search_ = SearchType::create(rose_map_, parameters_);
        traj_opt_ = TrajectoryOpt::create(rose_map_, parameters_);
        traj_sampler_ = TrajectorySampler2D::create(parameters_);
        mpc_ = MpcType::create(rose_map_, parameters_);
        default_wz = node.declare_parameter<double>("default_wz", 0.0);
        target_frame_ = node.declare_parameter<std::string>("target_frame", "");
        std::string odom_topic = node.declare_parameter<std::string>("odom_topic", "");
        odometry_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&RosePlanner::Impl::odomCallback, this, std::placeholders::_1)
        );

        std::string goal_topic = node.declare_parameter<std::string>("goal_topic", "");
        goal_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic,
            10,
            std::bind(&RosePlanner::Impl::goalPoseCallback, this, std::placeholders::_1)
        );

        goal_point_sub_ = node.create_subscription<geometry_msgs::msg::PointStamped>(
            goal_topic,
            10,
            std::bind(&RosePlanner::Impl::goalPointCallback, this, std::placeholders::_1)
        );

        raw_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("raw_path", 10);
        opt_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("opt_path", 10);
        predict_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("predict_path", 10);
        cmd_vel_pub_ = node.create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        cmd_vel_norm_pub_ = node.create_publisher<std_msgs::msg::Float64>("/cmd_vel_norm", 10);
        vel_marker_pub_ = node.create_publisher<visualization_msgs::msg::Marker>("/vel_marker", 10);
        opt_marker_pub_ =
            node.create_publisher<visualization_msgs::msg::Marker>("/opt_traj_marker", 10);
        timer_thread_ = std::thread([this]() {
            while (running_) {
                auto start = std::chrono::steady_clock::now();
                timerCallback();
                auto end = std::chrono::steady_clock::now();
                auto cost =
                    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                if (cost < 100)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100 - cost));
            }
        });

        control_timer_thread_ = std::thread([this]() {
            while (running_) {
                auto start = std::chrono::steady_clock::now();
                mpcCallback();
                auto end = std::chrono::steady_clock::now();
                auto cost =
                    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                if (cost < 1000 / parameters_.mpc_params.fps)
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(1000 / parameters_.mpc_params.fps - cost)
                    );
            }
        });
        use_control_output_ = node.declare_parameter<bool>("use_control_output", false);
        replan_fsm_.state_ = ReplanFSM::INIT;
        dynamic_replan_ = node.declare_parameter<bool>("dynamic_replan", false);
        RCLCPP_INFO(node_->get_logger(), "RosePlanner initialized");
    }
    Eigen::Vector2d getCurrentVel() {
        Eigen::Vector2d v(current_odom_.twist.twist.linear.x, current_odom_.twist.twist.linear.y);
        return v;
    }
    geometry_msgs::msg::PoseStamped getCurrentPose() {
        geometry_msgs::msg::PoseStamped predicted_pose;
        predicted_pose.header.stamp = node_->now();
        predicted_pose.header.frame_id = current_odom_.header.frame_id;

        double px = current_odom_.pose.pose.position.x;
        double py = current_odom_.pose.pose.position.y;
        double pz = current_odom_.pose.pose.position.z;

        Eigen::Quaterniond q(
            current_odom_.pose.pose.orientation.w,
            current_odom_.pose.pose.orientation.x,
            current_odom_.pose.pose.orientation.y,
            current_odom_.pose.pose.orientation.z
        );

        Eigen::Vector3d v(
            current_odom_.twist.twist.linear.x,
            current_odom_.twist.twist.linear.y,
            current_odom_.twist.twist.linear.z
        );

        Eigen::Vector3d w(
            current_odom_.twist.twist.angular.x,
            current_odom_.twist.twist.angular.y,
            current_odom_.twist.twist.angular.z
        );

        static rclcpp::Time last_time = rclcpp::Time(current_odom_.header.stamp, RCL_ROS_TIME);
        double dt = (node_->now() - last_time).seconds();
        last_time = node_->now();
        Eigen::Quaterniond dq = Eigen::Quaterniond(Eigen::AngleAxisd(
            dt * w.norm(),
            (w.norm() > 1e-6) ? w.normalized() : Eigen::Vector3d::UnitZ()
        ));

        Eigen::Quaterniond q_pred = q * dq;
        q_pred.normalize();
        Eigen::Vector3d p_pred(px, py, pz);
        p_pred += v * dt;
        predicted_pose.pose.position.x = p_pred.x();
        predicted_pose.pose.position.y = p_pred.y();
        predicted_pose.pose.position.z = p_pred.z();

        predicted_pose.pose.orientation.w = q_pred.w();
        predicted_pose.pose.orientation.x = q_pred.x();
        predicted_pose.pose.orientation.y = q_pred.y();
        predicted_pose.pose.orientation.z = q_pred.z();

        return predicted_pose;
    }

    void timerCallback() {
        static auto last_fps_time = node_->now();
        static int frame_count = 0;

        ++frame_count;

        auto current_time = node_->now();
        double elapsed_sec = (current_time - last_fps_time).seconds();

        if (elapsed_sec >= 1.0) {
            double fps = frame_count / elapsed_sec;
            RCLCPP_INFO(
                node_->get_logger(),
                "Planner callback frequency: %.2f Hz (%d frames in %.1f s)",
                fps,
                frame_count,
                elapsed_sec
            );

            frame_count = 0;
            last_fps_time = current_time;
        }
        switch (replan_fsm_.state_) {
            case ReplanFSM::INIT: {
                RCLCPP_DEBUG(node_->get_logger(), "FSM: INIT");
            }

            break;

            case ReplanFSM::WAIT_GOAL: {
                static auto t_start = std::chrono::steady_clock::now();
                static bool timing = false;
                if (change_to_wait_) {
                    change_to_wait_ = false;
                    t_start = std::chrono::steady_clock::now();
                    timing = true;
                    RCLCPP_INFO(node_->get_logger(), "change to wait goal pub stop 1s");
                }

                RCLCPP_DEBUG(node_->get_logger(), "FSM: WAIT_GOAL");

                if (timing) {
                    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                                    std::chrono::steady_clock::now() - t_start
                    )
                                    .count();
                    if (dt < 1.0 && use_control_output_) {
                        geometry_msgs::msg::Twist cmd;
                        cmd.linear.x = 0;
                        cmd.linear.y = 0;
                        cmd.angular.z = default_wz;
                        cmd_vel_pub_->publish(cmd);
                    } else {
                        timing = false;
                    }
                }
            }

            break;

            case ReplanFSM::REPLAN: {
                Eigen::Vector2f goal_pos(
                    current_goal_.pose.position.x,
                    current_goal_.pose.position.y
                );
                auto current = getCurrentPose();
                Eigen::Vector2f current_pos(current.pose.position.x, current.pose.position.y);
                double dist_to_goal = (goal_pos - current_pos).norm();

                if (dist_to_goal < 0.5) {
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Goal reached (dist=%.2f m), waiting for new goal",
                        dist_to_goal
                    );
                    change_to_wait_ = true;
                    replan_fsm_.state_ = ReplanFSM::WAIT_GOAL;
                    break;
                }
                removeOldTraj();
                removeOldPath();
                if (!checkSafeTraj(current_traj_)) {
                    have_traj_ = false;
                    searchOnce(goal_pos);
                    break;
                }
                auto unsafe_points = checkSafePath(current_raw_path_);
                if (dynamic_replan_) {
                    have_traj_ = false;
                    searchOnce(goal_pos);
                } else {
                    if (!unsafe_points.empty()) {
                        have_traj_ = false;
                        localReplan(unsafe_points, goal_pos);
                    } else {
                        unsafe_points = checkSafePath(current_traj_.toPointVector(0.05), 0.05, 3.0);
                        if (!unsafe_points.empty()
                            && current_raw_path_.size() > (1 / rose_map_->esdf_->esdf_->voxel_size))
                        {
                            have_traj_ = false;
                            resampleAndOpt(current_raw_path_);
                        }
                    }
                }

                break;
            }

            case ReplanFSM::SEARCH_PATH: {
                Eigen::Vector2f goal_w(
                    current_goal_.pose.position.x,
                    current_goal_.pose.position.y
                );
                RCLCPP_INFO(node_->get_logger(), "New goal received, starting global search");
                replan_fsm_.state_ = ReplanFSM::REPLAN;
                have_traj_ = false;
                searchOnce(goal_w);

                break;
            }
        }
    }
    void removeOldTraj() {
        auto current = getCurrentPose();
        double t_cur =
            current_traj_.getTimeByPos({ current.pose.position.x, current.pose.position.y }, 0.5);
        if (t_cur < 0) {
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
            return;
        }
        t_cur -= 0.0;
        if (t_cur < 0)
            t_cur = 0;
        current_traj_.truncateBeforeTime(t_cur);
    }
    void removeOldPath() {
        if (current_raw_path_.empty())
            return;

        auto current = getCurrentPose();

        int best_target_index = -1;
        double best_dist2 = std::numeric_limits<double>::infinity();

        for (int i = 0; i < static_cast<int>(current_raw_path_.size()); ++i) {
            double dx = current_raw_path_[i].x() - current.pose.position.x;
            double dy = current_raw_path_[i].y() - current.pose.position.y;
            double dist2 = dx * dx + dy * dy;

            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                best_target_index = i;
            }
        }
        if (best_target_index > 0) {
            current_raw_path_.erase(
                current_raw_path_.begin(),
                current_raw_path_.begin() + best_target_index
            );
        }
    }

    double orientationToYaw(const geometry_msgs::msg::Quaternion& q) noexcept {
        // Get armor yaw
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        // Make yaw change continuous (-pi~pi to -inf~inf)
        yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
        last_yaw_ = yaw;
        return yaw;
    }
    control::State getNowState() {
        control::State now_state;
        auto current = getCurrentPose();
        now_state.pos.x() = current.pose.position.x;
        now_state.pos.y() = current.pose.position.y;
        Eigen::Vector2d start_v = getCurrentVel();
        now_state.vel.x() = start_v.x();
        now_state.vel.y() = start_v.y();
        double now_yaw = orientationToYaw(current.pose.orientation);
        now_state.yaw = now_yaw;
        return now_state;
    }
    void mpcCallback() {
        if (replan_fsm_.state_ != ReplanFSM::REPLAN || current_traj_.getPieceNum() < 1) {
            return;
        }
        if (!have_traj_ && use_control_output_) {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = default_wz;
            cmd_vel_pub_->publish(cmd);
            return;
        }
        mpc_->setTrajectory(current_traj_);

        auto now_state = getNowState();
        mpc_->setCurrent(now_state);
        mpc_->solve();
        auto output = mpc_->getOutput();
        if (use_control_output_) {
            double yaw = now_state.yaw;
            double vx_world = output.vel.x();
            double vy_world = output.vel.y();
            double vx_body = std::cos(yaw) * vx_world + std::sin(yaw) * vy_world;
            double vy_body = -std::sin(yaw) * vx_world + std::cos(yaw) * vy_world;
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = vx_body;
            cmd.linear.y = vy_body;

            cmd.angular.z = default_wz;
            cmd_vel_pub_->publish(cmd);
            std_msgs::msg::Float64 cmd_norm;
            cmd_norm.data = std::hypot(vx_world, vy_world);
            cmd_vel_norm_pub_->publish(cmd_norm);
        }
        visualization_msgs::msg::Marker vel_marker;

        vel_marker.header.frame_id = target_frame_;
        vel_marker.header.stamp = node_->get_clock()->now();
        output.fillVelocityArrow(vel_marker, current_odom_);

        vel_marker_pub_->publish(vel_marker);

        nav_msgs::msg::Path predict_path;
        predict_path.header.frame_id = target_frame_;
        predict_path.header.stamp = rclcpp::Clock().now();
        output.fillPath(predict_path, current_odom_);
        predict_path_pub_->publish(predict_path);
    }
    std::vector<int> checkSafePath(const std::vector<Eigen::Vector2d>& path) {
        std::vector<int> unsafe_points;

        if (path.size() < 2)
            return unsafe_points;

        if (!rose_map_ || !rose_map_->esdf_ || !rose_map_->esdf_->esdf_)
            return unsafe_points;

        const auto& esdf = rose_map_->esdf_;

        const double step = std::min(esdf->esdf_->voxel_size * 0.5, parameters_.robot_radius * 0.5);

        if (step <= 1e-6)
            return unsafe_points;

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const Eigen::Vector2d p0 = path[i];
            const Eigen::Vector2d p1 = path[i + 1];

            const double seg_len = (p1 - p0).norm();
            const int n = std::max(1, (int)std::ceil(seg_len / step));

            bool unsafe = false;

            for (int k = 0; k <= n; ++k) {
                double alpha = (double)k / n;
                Eigen::Vector2d p = p0 + alpha * (p1 - p0);

                auto key = esdf->worldToKey(p.cast<float>());
                int idx = esdf->keyToIndex(key);

                if (idx < 0)
                    continue;

                if (esdf->getEsdf(idx) < parameters_.robot_radius) {
                    unsafe = true;
                    break;
                }
            }

            if (unsafe)
                unsafe_points.push_back(i);
        }

        return unsafe_points;
    }

    std::vector<int>
    checkSafePath(const std::vector<Eigen::Vector2d>& path, double sample_dt, double horizon) {
        std::vector<Eigen::Vector2d> path_cut = path;

        int target_size = static_cast<int>(horizon / sample_dt);
        if (target_size < 0) {
            target_size = 0;
        }

        if (static_cast<int>(path_cut.size()) > target_size) {
            path_cut.resize(target_size);
        }

        return checkSafePath(path_cut);
    }

    bool checkSafeTraj(const TrajType& traj) {
        auto sampled = traj.toPointVector(parameters_.resampler_params_.dt);
        double total_length = 0.0;
        for (int i = 0; i < sampled.size() - 1; ++i) {
            total_length += TrajectorySampler2D::segmentLength(sampled[i], sampled[i + 1]);
        }
        if (total_length > 100) {
            RCLCPP_WARN(node_->get_logger(), "Traj too long, skipping.");
            return false;
        }

        return true;
    }
    void localReplan(const std::vector<int>& unsafe_points, const Eigen::Vector2f& goal_w) {
        if (current_raw_path_.size() < 2 || current_traj_.getTotalDuration() < 1.0) {
            RCLCPP_WARN(node_->get_logger(), "Raw path too short for local replan.");
            return;
        }
        auto current = getCurrentPose();
        const int Num = static_cast<int>(current_raw_path_.size());
        Eigen::Vector2d start_w(current.pose.position.x, current.pose.position.y);

        int local_end_idx = unsafe_points.empty()
            ? findHeadPointIndex(start_w, current_raw_path_)
            : unsafe_points.back() + (1 / rose_map_->esdf_->esdf_->voxel_size);

        local_end_idx = std::clamp(local_end_idx, 0, Num - 1);

        int next_idx = local_end_idx + 1;
        if (next_idx >= Num) {
            RCLCPP_WARN(node_->get_logger(), "Local replan end is last point, skipping.");
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
            return;
        }

        std::vector<Eigen::Vector2d> path_after(
            current_raw_path_.begin() + next_idx,
            current_raw_path_.end()
        );

        PathSearch::Path local_path;
        SearchState search_state = SearchState::NO_PATH;

        try {
            search_state = path_search_->search(
                start_w.cast<float>(),
                current_raw_path_[next_idx].cast<float>(),
                local_path
            );
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Local A* exception: %s", e.what());
            search_state = SearchState::NO_PATH;
        }

        if (search_state != SearchState::SUCCESS || local_path.size() < 2) {
            RCLCPP_WARN(node_->get_logger(), "Local replan failed, fallback to global.");
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
            return;
        }

        const double stitch_dist =
            TrajectorySampler2D::segmentLength(local_path.back(), path_after.front());
        if (stitch_dist > 1.0) {
            Eigen::Vector2d mid = 0.5 * (local_path.back() + path_after.front());
            local_path.emplace_back(mid);
            RCLCPP_INFO(node_->get_logger(), "Inserted stitch midpoint for smooth connection.");
        }

        std::vector<Eigen::Vector2d> new_raw_path;
        new_raw_path.reserve(local_path.size() + path_after.size());

        for (const auto& p: local_path) {
            new_raw_path.emplace_back(p.x(), p.y());
        }
        new_raw_path.insert(new_raw_path.end(), path_after.begin(), path_after.end());

        std::vector<Eigen::Vector2d> filtered;
        filtered.reserve(new_raw_path.size());
        filtered.push_back(new_raw_path.front());

        for (size_t i = 1; i < new_raw_path.size(); i++) {
            double d = TrajectorySampler2D::segmentLength(filtered.back(), new_raw_path[i]);
            if (d > 0.05) {
                filtered.push_back(new_raw_path[i]);
            }
        }

        if (filtered.size() < 2) {
            RCLCPP_WARN(node_->get_logger(), "Filtered path invalid, aborting replan.");
            return;
        }
        auto backup = current_raw_path_;
        current_raw_path_.swap(filtered);
        bool opt_ok = false;
        try {
            resampleAndOpt(current_raw_path_);
            opt_ok = true;
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "Trajectory optimization crashed! Reverting path.");
        }

        if (!opt_ok) {
            current_raw_path_ = backup;
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
            return;
        }
    }

    int findHeadPointIndex(
        const Eigen::Vector2d& start_w,
        const std::vector<Eigen::Vector2d>& current_path
    ) {
        if (current_path.empty())
            return -1;

        constexpr double TARGET_DIST = 2.0;
        constexpr double TOLERANCE = 0.5;
        constexpr double TARGET_DIST2 = TARGET_DIST * TARGET_DIST;

        int best_index = 0;
        double best_diff2 = std::numeric_limits<double>::infinity();
        int best_target_index = -1;

        for (int i = 0; i < (int)current_path.size(); ++i) {
            double dx = current_path[i].x() - start_w.x();
            double dy = current_path[i].y() - start_w.y();
            double dist2 = dx * dx + dy * dy;
            double diff2 = std::abs(dist2 - TARGET_DIST2);
            if (diff2 < best_diff2) {
                best_diff2 = diff2;
                best_index = i;
                best_target_index = i;
            }

            double dist = std::sqrt(dist2);
            if (dist > TARGET_DIST - TOLERANCE && dist < TARGET_DIST + TOLERANCE) {
                return i;
            }
        }

        return best_target_index != -1 ? best_target_index : best_index;
    }

    void searchOnce(const Eigen::Vector2f& goal_w) {
        auto current = getCurrentPose();
        Eigen::Vector2d start_w(current.pose.position.x, current.pose.position.y);

        current_raw_path_.clear();

        RCLCPP_INFO(
            node_->get_logger(),
            "Global planning | Start: [%.2f, %.2f] → Goal: [%.2f, %.2f]",
            start_w.x(),
            start_w.y(),
            goal_w.x(),
            goal_w.y()
        );

        using Clock = std::chrono::system_clock;
        using Ms = std::chrono::milliseconds;
        auto t0_total = Clock::now();
        auto t1_search_start = Clock::now();

        PathSearch::Path path;
        SearchState search_state = SearchState::NO_PATH;

        try {
            search_state = path_search_->search(start_w.cast<float>(), goal_w, path);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "A* search exception: %s", e.what());
        }

        auto t1_search_end = Clock::now();
        auto search_ms = std::chrono::duration_cast<Ms>(t1_search_end - t1_search_start).count();

        if (search_state == SearchState::SUCCESS) {
            auto t2_opt_start = Clock::now();
            current_raw_path_ = path;
            resampleAndOpt(current_raw_path_);
            auto t2_opt_end = Clock::now();

            auto opt_ms = std::chrono::duration_cast<Ms>(t2_opt_end - t2_opt_start).count();
            auto total_ms = std::chrono::duration_cast<Ms>(t2_opt_end - t0_total).count();

            RCLCPP_INFO(
                node_->get_logger(),
                "Planning success | A*: %ld ms | Opt: %ld ms | Total: %ld ms | Raw: %zu pts | Opt: %u pts",
                search_ms,
                opt_ms,
                total_ms,
                current_raw_path_.size(),
                current_traj_.getPieceNum()
            );

        } else if (search_state == SearchState::NO_PATH) {
            RCLCPP_WARN(
                node_->get_logger(),
                "No path found by A*, waiting for new goal or map update"
            );
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;

        } else if (search_state == SearchState::TIMEOUT) {
            RCLCPP_WARN(node_->get_logger(), "A* search timeout, abandoning current goal");
            replan_fsm_.state_ = ReplanFSM::WAIT_GOAL;
            change_to_wait_ = true;
        }
    }

    void resampleAndOpt(const std::vector<Eigen::Vector2d>& path) {
        start_time_ = node_->now();
        nav_msgs::msg::Path raw_path_msg;
        raw_path_msg.header.stamp = node_->now();
        raw_path_msg.header.frame_id = target_frame_;

        nav_msgs::msg::Path opt_path_msg;
        opt_path_msg.header.stamp = node_->now();
        opt_path_msg.header.frame_id = target_frame_;

        Eigen::Vector2d start_v = getCurrentVel();
        auto traj = traj_sampler_->sampleTrapezoid(path, start_v);
        // Publish raw path
        if (raw_path_pub_->get_subscription_count() > 0) {
            for (int i = 0; i < (int)traj.size(); i = i + 3) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header = raw_path_msg.header;
                pose_msg.pose.position.x = traj[i].p.x();
                pose_msg.pose.position.y = traj[i].p.y();
                pose_msg.pose.position.z = getCurrentPose().pose.position.z;

                double yaw = std::atan2(traj[i].v.y(), traj[i].v.x());
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                q.normalize();
                pose_msg.pose.orientation.x = q.x();
                pose_msg.pose.orientation.y = q.y();
                pose_msg.pose.orientation.z = q.z();
                pose_msg.pose.orientation.w = q.w();

                raw_path_msg.poses.push_back(pose_msg);
            }
            raw_path_pub_->publish(raw_path_msg);
        }
        traj_opt_->setSampledPath(traj, parameters_.resampler_params_.dt, getNowState());

        traj_opt_->optimize();
        auto opt_traj = traj_opt_->getTrajectory();
        if (opt_traj.getPieceNum() > 1 && opt_traj.getPieceNum() < 1000) {
            current_traj_ = opt_traj;
            have_traj_ = true;
        }

        // Publish optimized path + marker spheres
        if ((opt_path_pub_->get_subscription_count() > 0
             || opt_marker_pub_->get_subscription_count() > 0)
            && opt_traj.getPieceNum() > 1)
        {
            double totalDur = opt_traj.getTotalDuration();
            double dt = parameters_.resampler_params_.dt;
            int sampleNum = static_cast<int>(totalDur / dt) + 2;

            double t_cur = 0.0;
            opt_path_msg.poses.clear();

            visualization_msgs::msg::Marker marker;
            marker.header = opt_path_msg.header;
            marker.ns = "opt_traj";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.scale.x = 0.2; // 球半径（直径）
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.r = 0.1f;
            marker.color.g = 0.1f;
            marker.color.b = 0.9f;
            marker.color.a = 1.0f;

            marker.points.clear();

            for (int i = 0; i < sampleNum; ++i) {
                Eigen::VectorXd pos = opt_traj.getPos(t_cur);
                Eigen::VectorXd vel = opt_traj.getVel(t_cur);
                if (pos.size() < 2 || vel.size() < 2)
                    break;

                double yaw =
                    std::hypot(vel.x(), vel.y()) > 1e-3 ? std::atan2(vel.y(), vel.x()) : 0.0;
                geometry_msgs::msg::PoseStamped p;
                p.header = opt_path_msg.header;
                p.pose.position.x = pos.x();
                p.pose.position.y = pos.y();
                p.pose.position.z = getCurrentPose().pose.position.z;

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                q.normalize();
                p.pose.orientation = tf2::toMsg(q);

                opt_path_msg.poses.push_back(p);
                geometry_msgs::msg::Point mp;
                mp.x = pos.x();
                mp.y = pos.y();
                mp.z = p.pose.position.z;
                marker.points.push_back(mp);

                t_cur = std::min(t_cur + dt, totalDur);
            }

            // 发布
            opt_path_pub_->publish(opt_path_msg);
            opt_marker_pub_->publish(marker);
        }
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto T = tf_->getTransform(target_frame_, msg->header.frame_id, msg->header.stamp);
        if (!T.has_value()) {
            return;
        }
        Eigen::Vector4f p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, 1.0f);
        p = T.value() * p;
        geometry_msgs::msg::PoseStamped goal_out = *msg;
        goal_out.header.frame_id = target_frame_;
        goal_out.pose.position.x = p.x();
        goal_out.pose.position.y = p.y();
        goal_out.pose.position.z = p.z();

        RCLCPP_INFO(
            node_->get_logger(),
            "Received PoseStamped goal (target_frame) [%.2f, %.2f]",
            goal_out.pose.position.x,
            goal_out.pose.position.y
        );

        current_goal_ = goal_out;
        replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
    }

    void goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        auto T = tf_->getTransform(target_frame_, msg->header.frame_id, msg->header.stamp);
        Eigen::Vector4f p(msg->point.x, msg->point.y, msg->point.z, 1.0f);
        p = T.value() * p;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.header.frame_id = target_frame_;
        pose.header.stamp = msg->header.stamp;
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = p.z();
        pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(
            node_->get_logger(),
            "Received PointStamped goal (target_frame) [%.2f, %.2f]",
            pose.pose.position.x,
            pose.pose.position.y
        );

        current_goal_ = pose;
        replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rose_map_->odomCallback(msg);

        const auto& odom_in = *msg;

        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();

        auto T_opt =
            tf_->getTransform(target_frame_, odom_in.header.frame_id, odom_in.header.stamp);

        if (T_opt.has_value()) {
            T = *T_opt;
        } else {
            RCLCPP_WARN(node_->get_logger(), "[TF] odomCallback transform failed → identity");
            return;
        }

        Eigen::Vector3f p(
            odom_in.pose.pose.position.x,
            odom_in.pose.pose.position.y,
            odom_in.pose.pose.position.z
        );

        p = T * p;
        Eigen::Quaternionf q_in(
            odom_in.pose.pose.orientation.w,
            odom_in.pose.pose.orientation.x,
            odom_in.pose.pose.orientation.y,
            odom_in.pose.pose.orientation.z
        );

        Eigen::Quaternionf q_out(T.rotation() * q_in);
        q_out.normalize();
        geometry_msgs::msg::PoseStamped pose_out;
        pose_out.header = odom_in.header;
        pose_out.header.frame_id = target_frame_;

        pose_out.pose.position.x = p.x();
        pose_out.pose.position.y = p.y();
        pose_out.pose.position.z = p.z();
        pose_out.pose.orientation.x = q_out.x();
        pose_out.pose.orientation.y = q_out.y();
        pose_out.pose.orientation.z = q_out.z();
        pose_out.pose.orientation.w = q_out.w();
        current_pose_ = pose_out;
        Eigen::Matrix3f R = T.rotation();

        Eigen::Vector3f vlin(
            odom_in.twist.twist.linear.x,
            odom_in.twist.twist.linear.y,
            odom_in.twist.twist.linear.z
        );

        Eigen::Vector3f vang(
            odom_in.twist.twist.angular.x,
            odom_in.twist.twist.angular.y,
            odom_in.twist.twist.angular.z
        );

        vlin = R * vlin;
        vang = R * vang;

        nav_msgs::msg::Odometry odom_out = odom_in;
        odom_out.header.frame_id = target_frame_;
        odom_out.pose.pose = pose_out.pose;

        odom_out.twist.twist.linear.x = vlin.x();
        odom_out.twist.twist.linear.y = vlin.y();
        odom_out.twist.twist.linear.z = vlin.z();

        odom_out.twist.twist.angular.x = vang.x();
        odom_out.twist.twist.angular.y = vang.y();
        odom_out.twist.twist.angular.z = vang.z();

        current_odom_ = odom_out;
    }

    Parameters parameters_;
    rclcpp::Node* node_;
    TF::Ptr tf_;
    SearchType::Ptr path_search_;
    rose_map::RoseMap::Ptr rose_map_;
    TrajectoryOpt::Ptr traj_opt_;
    bool dynamic_replan_ = false;
    geometry_msgs::msg::PoseStamped current_pose_;
    double last_yaw_;
    bool use_control_output_ = false;
    nav_msgs::msg::Odometry current_odom_;
    geometry_msgs::msg::PoseStamped current_goal_;
    rclcpp::Time start_time_;
    ReplanFSM replan_fsm_;
    bool change_to_wait_ = true;
    std::thread timer_thread_;
    std::thread control_timer_thread_;
    std::atomic<bool> running_ { true };
    TrajectorySampler2D::Ptr traj_sampler_;
    MpcType::Ptr mpc_;
    bool have_traj_ = false;
    TrajType current_traj_;
    std::vector<Eigen::Vector2d> current_raw_path_;
    std::string target_frame_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_vel_norm_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr opt_marker_pub_;
};

RosePlanner::RosePlanner(rclcpp::Node& node) {
    _impl = std::make_unique<Impl>(node);
}
RosePlanner::~RosePlanner() {
    _impl.reset();
}
} // namespace rose_planner