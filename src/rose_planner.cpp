#include "rose_planner.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "parameters.hpp"
#include "path_search/a*.hpp"
#include "replan_fsm.hpp"
#include "rose_map/rose_map.hpp"
#include "trajectory_optimize/trajectory_opt.hpp"
#include "trajectory_optimize/trajectory_sampler.hpp"
#include <angles.h>
// #include <mpc_control/acado_mpc.hpp>
#include "mpc_control/osqp_mpc.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace rose_planner {
struct RosePlanner::Impl {
public:
    using SearchType = AStar;
    ~Impl() {
        running_ = false;
        if (timer_thread_.joinable())
            timer_thread_.join();
        if (control_timer_thread_.joinable())
            control_timer_thread_.join();
    }

    Impl(rclcpp::Node& node): tf_buffer_(node.get_clock()) {
        node_ = &node;
        parameters_.load(node);

        rose_map_ = std::make_shared<rose_map::RoseMap>(node);
        path_search_ = SearchType::create(rose_map_, parameters_);
        traj_opt_ = TrajectoryOpt::create(rose_map_, parameters_);
        // mpc_ = AcadoMpc::create(parameters_);
        mpc_ = OsqpMpc::create(parameters_);
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
                if (cost < 10)
                    std::this_thread::sleep_for(std::chrono::milliseconds(10 - cost));
            }
        });
        use_control_output_ = node.declare_parameter<bool>("use_control_output", false);
        replan_fsm_.state_ = ReplanFSM::INIT;
        RCLCPP_INFO(node_->get_logger(), "RosePlanner initialized");
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
                    if (dt < 1.0) {
                        geometry_msgs::msg::Twist cmd;
                        cmd.linear.x = 0;
                        cmd.linear.y = 0;
                        cmd.angular.z = 0;
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
                Eigen::Vector2f current_pos(current_pose_.position.x, current_pose_.position.y);
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

                auto unsafe_points = checkSafePath(
                    current_traj_.toPointVector(parameters_.path_search_params_.resampler.dt)
                );

                localReplan(unsafe_points, goal_pos);
                // searchOnce(goal_pos);
                break;
            }

            case ReplanFSM::SEARCH_PATH: {
                Eigen::Vector2f goal_w(
                    current_goal_.pose.position.x,
                    current_goal_.pose.position.y
                );
                RCLCPP_INFO(node_->get_logger(), "New goal received, starting global search");
                replan_fsm_.state_ = ReplanFSM::REPLAN;
                searchOnce(goal_w);

                break;
            }
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

    void mpcCallback() {
        if (replan_fsm_.state_ != ReplanFSM::REPLAN || current_traj_.getPieceNum() < 1
            || !have_traj_) {
            return;
        }
        mpc_->setTrajectory(current_traj_);
        Eigen::Vector2d start_v(
            current_odom_.twist.twist.linear.x,
            current_odom_.twist.twist.linear.y
        );
        MPCState now_state;
        now_state.x = current_pose_.position.x;
        now_state.y = current_pose_.position.y;
        now_state.vx = start_v.x();
        now_state.vy = start_v.y();
        mpc_->setCurrent(now_state);
        mpc_->solve();
        if (use_control_output_) {
            Eigen::VectorXd output = mpc_->getOutput();

            double yaw = orientationToYaw(current_pose_.orientation);

            double vx_world = output(0);
            double vy_world = output(1);

            double vx_body = std::cos(yaw) * vx_world + std::sin(yaw) * vy_world;
            double vy_body = -std::sin(yaw) * vx_world + std::cos(yaw) * vy_world;

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = vx_body;
            cmd.linear.y = vy_body;
            cmd.angular.z = 3.14;
            cmd_vel_pub_->publish(cmd);
        }

        nav_msgs::msg::Path predict_path;
        predict_path.header.frame_id = target_frame_;
        predict_path.header.stamp = rclcpp::Clock().now();
        geometry_msgs::msg::PoseStamped pose_msg;
        for (int i = 0; i < mpc_->T_; ++i) {
            pose_msg.header = predict_path.header;
            pose_msg.pose.position.x = mpc_->xopt_[i].x;
            pose_msg.pose.position.y = mpc_->xopt_[i].y;
            pose_msg.pose.position.z = current_pose_.position.z;
            double yaw = std::hypot(mpc_->xopt_[i].vx, mpc_->xopt_[i].vy) > 1e-3
                ? std::atan2(mpc_->xopt_[i].vy, mpc_->xopt_[i].vx)
                : 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            q.normalize();
            pose_msg.pose.orientation = tf2::toMsg(q);
            predict_path.poses.push_back(pose_msg);
        }
        predict_path_pub_->publish(predict_path);
    }

    std::vector<int> checkSafePath(const std::vector<Eigen::Vector2d>& path) {
        std::vector<int> unsafe_points;
        for (int i = 0; i < path.size(); ++i) {
            auto key = rose_map_->worldToKey2D(path[i].cast<float>());
            int idx = rose_map_->key2DToIndex2D(key);
            if (idx < 0)
                continue;
            if (rose_map_->esdf_[idx] < parameters_.robot_radius) {
                unsafe_points.push_back(i);
            }
        }
        return unsafe_points;
    }

    void localReplan(const std::vector<int>& unsafe_points, const Eigen::Vector2f& goal_w) {
        if (current_raw_path_.size() < 2) {
            RCLCPP_WARN(node_->get_logger(), "Raw path too short for local replan.");
            replan_fsm_.state_ = ReplanFSM::STATE::WAIT_GOAL;
            change_to_wait_ = true;
            return;
        }

        const int Num = static_cast<int>(current_raw_path_.size());
        Eigen::Vector2d start_w(current_pose_.position.x, current_pose_.position.y);

        int local_end_idx = unsafe_points.empty()
            ? findClosestPointIndex(start_w, current_raw_path_)
            : unsafe_points.back();

        local_end_idx = std::clamp(local_end_idx, 0, Num - 1);

        int next_idx = local_end_idx + 1;
        if (next_idx >= Num) {
            RCLCPP_WARN(node_->get_logger(), "Local replan end is last point, skipping.");
            replan_fsm_.state_ = ReplanFSM::STATE::WAIT_GOAL;
            change_to_wait_ = true;
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

        const double stitch_dist = segmentLength(local_path.back(), path_after.front());
        if (stitch_dist > 1.0) { // 若跳变过大，插值一个过渡点
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
            double d = segmentLength(filtered.back(), new_raw_path[i]);
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

        RCLCPP_INFO(
            node_->get_logger(),
            "Local replan succeeded (%zu new pts), optimizing trajectory.",
            local_path.size()
        );
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

    int findClosestPointIndex(
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
        Eigen::Vector2d start_w(current_pose_.position.x, current_pose_.position.y);

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

        Eigen::Vector2d start_v(
            current_odom_.twist.twist.linear.x,
            current_odom_.twist.twist.linear.y
        );
        start_v = Eigen::Vector2d::Zero();
        auto traj = sampleTrajectoryTrapezoid(
            path,
            parameters_.path_search_params_.resampler.acc,
            parameters_.path_search_params_.resampler.max_vel,
            parameters_.path_search_params_.resampler.dt,
            start_v
        );

        traj_opt_->setSampledPath(
            traj,
            parameters_.path_search_params_.resampler.dt,
            start_v.cast<double>()
        );

        traj_opt_->optimize();
        auto opt_traj = traj_opt_->getTrajectory();
        current_traj_ = opt_traj;
        have_traj_ = true;
        // Publish raw path
        if (raw_path_pub_->get_subscription_count() > 0) {
            for (int i = 0; i < (int)traj.size(); i = i + 3) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header = raw_path_msg.header;
                pose_msg.pose.position.x = traj[i].p.x();
                pose_msg.pose.position.y = traj[i].p.y();
                pose_msg.pose.position.z = current_pose_.position.z;

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

        // Publish optimized path
        if (opt_path_pub_->get_subscription_count() > 0 && opt_traj.getPieceNum() > 1) {
            double totalDur = opt_traj.getTotalDuration();
            double dt = parameters_.path_search_params_.resampler.dt;
            int sampleNum = static_cast<int>(totalDur / dt) + 2;

            double t_cur = 0.0;
            opt_path_msg.poses.clear();

            for (int i = 0; i < sampleNum; ++i) {
                Eigen::VectorXd pos = opt_traj.getPos(t_cur);
                Eigen::VectorXd vel = opt_traj.getVel(t_cur);
                if (pos.size() < 2 || vel.size() < 2)
                    break; // 保护

                double yaw =
                    std::hypot(vel.x(), vel.y()) > 1e-3 ? std::atan2(vel.y(), vel.x()) : 0.0;

                geometry_msgs::msg::PoseStamped p;
                p.header = opt_path_msg.header;
                p.pose.position.x = pos.x();
                p.pose.position.y = pos.y();
                p.pose.position.z = current_pose_.position.z;

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                q.normalize();
                p.pose.orientation = tf2::toMsg(q);

                opt_path_msg.poses.push_back(p);
                t_cur = std::min(t_cur + dt, totalDur);
            }

            opt_path_pub_->publish(opt_path_msg);
        }
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        try {
            auto tf =
                tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp);
            T = tf2ToEigen(tf);
        } catch (...) {
            RCLCPP_WARN(node_->get_logger(), "[TF] goalPoseCallback transform failed → identity");
        }

        Eigen::Vector4f p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, 1.0f);
        p = T * p;
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
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        try {
            auto tf =
                tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp);
            T = tf2ToEigen(tf);
        } catch (...) {
            RCLCPP_WARN(node_->get_logger(), "[TF] goalPointCallback transform failed → identity");
        }
        Eigen::Vector4f p(msg->point.x, msg->point.y, msg->point.z, 1.0f);
        p = T * p;
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
        const auto& odom_in = *msg;

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        tf2::Transform tf2_T = tf2::Transform::getIdentity();
        tf2::Quaternion q_in;

        try {
            auto tf = tf_buffer_.lookupTransform(
                target_frame_,
                odom_in.header.frame_id,
                odom_in.header.stamp
            );
            T = tf2ToEigen(tf);
            tf2::fromMsg(tf.transform, tf2_T);

            tf2::fromMsg(odom_in.pose.pose.orientation, q_in);
            q_in = tf2_T.getRotation() * q_in;
            q_in.normalize();
        } catch (...) {
            RCLCPP_WARN(node_->get_logger(), "[TF] odomCallback transform failed → identity");
            tf2::fromMsg(odom_in.pose.pose.orientation, q_in);
        }

        Eigen::Vector4f p(
            odom_in.pose.pose.position.x,
            odom_in.pose.pose.position.y,
            odom_in.pose.pose.position.z,
            1.0f
        );
        p = T * p;

        geometry_msgs::msg::Pose pose_out;
        pose_out.position.x = p.x();
        pose_out.position.y = p.y();
        pose_out.position.z = p.z();
        pose_out.orientation = tf2::toMsg(q_in);

        current_pose_ = pose_out;

        tf2::Vector3 vlin(
            odom_in.twist.twist.linear.x,
            odom_in.twist.twist.linear.y,
            odom_in.twist.twist.linear.z
        );
        vlin = tf2_T.getBasis() * vlin;

        tf2::Vector3 vang(
            odom_in.twist.twist.angular.x,
            odom_in.twist.twist.angular.y,
            odom_in.twist.twist.angular.z
        );
        vang = tf2_T.getBasis() * vang;

        nav_msgs::msg::Odometry odom_out = odom_in;
        odom_out.header.frame_id = target_frame_;
        odom_out.pose.pose.position.x = p.x();
        odom_out.pose.pose.position.y = p.y();
        odom_out.pose.pose.position.z = p.z();
        odom_out.pose.pose.orientation = tf2::toMsg(q_in);

        odom_out.twist.twist.linear.x = vlin.x();
        odom_out.twist.twist.linear.y = vlin.y();
        odom_out.twist.twist.linear.z = vlin.z();
        odom_out.twist.twist.angular.x = vang.x();
        odom_out.twist.twist.angular.y = vang.y();
        odom_out.twist.twist.angular.z = vang.z();

        current_odom_ = odom_out;

        rose_map_->setOrigin(Eigen::Vector3f(p.x(), p.y(), p.z()));
    }

    Eigen::Matrix4f tf2ToEigen(const geometry_msgs::msg::TransformStamped& tf) {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        const auto& t = tf.transform.translation;
        const auto& q = tf.transform.rotation;
        Eigen::Quaternionf Q(q.w, q.x, q.y, q.z);
        T.block<3, 3>(0, 0) = Q.toRotationMatrix();
        T(0, 3) = t.x;
        T(1, 3) = t.y;
        T(2, 3) = t.z;
        return T;
    }

    Parameters parameters_;
    rclcpp::Node* node_;

    SearchType::Ptr path_search_;
    rose_map::RoseMap::Ptr rose_map_;
    TrajectoryOpt::Ptr traj_opt_;

    geometry_msgs::msg::Pose current_pose_;
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
    // AcadoMpc::Ptr mpc_;
    OsqpMpc::Ptr mpc_;
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
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_ { tf_buffer_ };
};

RosePlanner::RosePlanner(rclcpp::Node& node) {
    _impl = std::make_unique<Impl>(node);
}
RosePlanner::~RosePlanner() {
    _impl.reset();
}
} // namespace rose_planner