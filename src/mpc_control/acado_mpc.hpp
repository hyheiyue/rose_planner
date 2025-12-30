// #pragma once

// #include "../parameters.hpp"
// #include "acado_auxiliary_functions.h"
// #include "acado_common.h"
// #define NX ACADO_NX /* Number of differential state variables.  */
// #define NXA ACADO_NXA /* Number of algebraic variables. */
// #define NU ACADO_NU /* Number of control inputs. */
// #define NOD ACADO_NOD /* Number of online data values. */

// #define NY ACADO_NY /* Number of measurements/references on nodes 0..N - 1. */
// #define NYN ACADO_NYN /* Number of measurements/references on node N. */

// #define N ACADO_N /* Number of intervals in the horizon. */

// #define NUM_STEPS 20 /* Number of real-time iterations. */
// #define VERBOSE 1 /* Show iterations: 1, silent: 0.  */

// #define Ts 0.1 // sampling time
// #define Lf 1.0

// namespace rose_planner {
// class AcadoMpc {
// public:
//     using Ptr = std::shared_ptr<AcadoMpc>;
//     AcadoMpc(Parameters params): params_(params) {
//         weight_p_ = params.mpc_params.weight_p;
//         weight_yaw_ = params.mpc_params.weight_yaw;
//         weight_v_ = params.mpc_params.weight_v;
//         weight_w_ = params.mpc_params.weight_w;
//         control_output_ = initAcado();
//     }
//     static Ptr create(Parameters params) {
//         return std::make_shared<AcadoMpc>(params);
//     }
//     std::vector<std::vector<double>> initAcado() {
//         acado_initializeSolver();

//         /* Initialize the states and controls. */
//         for (int i = 0; i < NX * (N + 1); ++i)
//             acadoVariables.x[i] = 0.0;
//         for (int i = 0; i < NU * N; ++i)
//             acadoVariables.u[i] = 0.0;

//         /* Initialize the measurements/reference. */
//         for (int i = 0; i < NY * N; ++i)
//             acadoVariables.y[i] = 0.0;
//         for (int i = 0; i < NYN; ++i)
//             acadoVariables.yN[i] = 0.0;

//         acado_preparationStep();

//         // 创建用于存储控制输出的向量
//         std::vector<double> control_output_vx;
//         std::vector<double> control_output_vy;
//         std::vector<double> control_output_w;

//         // 提取控制输出
//         for (int i = 0; i < ACADO_N; ++i) {
//             // 有三个控制输出：vx, vy, w
//             control_output_vx.push_back(acadoVariables.u[i * ACADO_NU + 0]);
//             control_output_vy.push_back(acadoVariables.u[i * ACADO_NU + 1]);
//             control_output_w.push_back(acadoVariables.u[i * ACADO_NU + 2]);
//         }

//         for (int i = 0; i < N; i++) {
//             // Setup diagonal entries
//             acadoVariables.W[NY * NY * i + (NY + 1) * 0] = weight_p_;
//             acadoVariables.W[NY * NY * i + (NY + 1) * 1] = weight_p_;
//             acadoVariables.W[NY * NY * i + (NY + 1) * 2] = weight_yaw_;
//             acadoVariables.W[NY * NY * i + (NY + 1) * 3] = weight_v_;
//             acadoVariables.W[NY * NY * i + (NY + 1) * 4] = weight_v_;
//             acadoVariables.W[NY * NY * i + (NY + 1) * 5] = weight_w_;
//         }
//         acadoVariables.WN[(NYN + 1) * 0] = weight_p_;
//         acadoVariables.WN[(NYN + 1) * 1] = weight_p_;
//         acadoVariables.WN[(NYN + 1) * 2] = weight_yaw_;

//         // 返回包含三个控制输出向量的向量
//         return { control_output_vx, control_output_vy, control_output_w };
//     }
//     std::vector<std::vector<double>>
//     solve(std::vector<double> states, std::vector<double> desired_state) {
//         /* Some temporary variables. */
//         int i, iter;
//         acado_timer t;

//         /* Initialize the states and controls. */
//         for (i = 0; i < NX * (N + 1); ++i) {
//             acadoVariables.x[i] = (real_t)states[i];
//         }
//         for (i = 0; i < NX; ++i) {
//             acadoVariables.x0[i] = (real_t)states[i];
//         }

//         /* Initialize the measurements/reference. */
//         for (i = 0; i < NY * N; ++i) {
//             acadoVariables.y[i] = (real_t)desired_state[i];
//         }
//         for (i = 0; i < NYN; ++i) {
//             acadoVariables.yN[i] = (real_t)desired_state[NY * (N - 1) + i];
//         }

//         // /* Prepare first step */
//         acado_preparationStep();

//         /* Get the time before start of the loop. */
//         acado_tic(&t);

//         /* The "real-time iterations" loop. */
//         for (iter = 0; iter < NUM_STEPS; ++iter) {
//             /* Perform the feedback step. */
//             acado_feedbackStep();
//             acado_preparationStep();
//         }

//         /* Read the elapsed time. */
//         real_t te = acado_toc(&t);

//         // 提取控制输出
//         std::vector<double> control_output_vx;
//         std::vector<double> control_output_vy;
//         std::vector<double> control_output_w;
//         real_t* u = acado_getVariablesU();
//         for (int i = 0; i < ACADO_N; ++i) {
//             control_output_vx.push_back((double)u[i * ACADO_NU + 0]);
//             control_output_vy.push_back((double)u[i * ACADO_NU + 1]);
//             control_output_w.push_back((double)u[i * ACADO_NU + 2]);
//         }

//         // 返回第一个时间步的控制输出
//         return { control_output_vx, control_output_vy, control_output_w };
//     }
//     std::vector<double>
//     update_states(std::vector<double> state, double vx_cmd, double vy_cmd, double w_cmd) {
//         // based on kinematic model
//         double x0 = state[0];
//         double y0 = state[1];
//         double q0 = state[2];
//         double vx = vx_cmd;
//         double vy = vy_cmd;
//         double w0 = w_cmd;

//         double x1 = x0 + (vx * cos(q0) - vy * sin(q0)) * Ts;
//         double y1 = y0 + (vx * sin(q0) + vy * cos(q0)) * Ts;

//         double q1 = q0 + w0 * Ts;
//         return { x1, y1, q1 };
//     }
//     std::vector<double> motion_prediction(
//         const std::vector<double>& cur_states,
//         const std::vector<std::vector<double>>& prev_u
//     ) {
//         std::vector<double> old_vx_cmd = prev_u[0];
//         std::vector<double> old_vy_cmd = prev_u[1];
//         std::vector<double> old_w_cmd = prev_u[2];

//         std::vector<std::vector<double>> predicted_states;
//         predicted_states.push_back(cur_states);

//         for (int i = 0; i < N; i++) {
//             std::vector<double> cur_state = predicted_states[i];
//             std::vector<double> next_state =
//                 update_states(cur_state, old_vx_cmd[i], old_vy_cmd[i], old_w_cmd[i]);
//             predicted_states.push_back(next_state);
//         }

//         std::vector<double> result;
//         for (int i = 0; i < (ACADO_N + 1); ++i) {
//             for (int j = 0; j < NX; ++j) {
//                 result.push_back(predicted_states[i][j]);
//             }
//         }
//         return result;
//     }

//     Parameters params_;
//     double weight_p_, weight_yaw_, weight_v_, weight_w_;
//     std::vector<std::vector<double>> control_output_;
// };

// } // namespace rose_planner
/* void mpcCallback() {
        if (replan_fsm_.state_ != ReplanFSM::REPLAN) {
            return;
        }
        const double Ts_local = Ts; // 0.1s
        const int N_horizon = N;
        const double totalDur = current_traj_.getTotalDuration();

        std::vector<Eigen::Vector2d> pts = current_traj_.toPointVector(Ts_local);
        if (pts.size() < 2) {
            return;
        }

        Eigen::Vector2d robot_pos(current_pose_.position.x, current_pose_.position.y);
        double current_yaw = orientationToYaw(current_pose_.orientation);

        int closest_index = 0;
        double min_dist = 1e9;
        for (int i = 0; i < (int)pts.size(); ++i) {
            double dist = (pts[i] - robot_pos).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_index = i;
            }
        }
        double t0 = closest_index * Ts_local;
        t0 = std::clamp(t0, 0.0, totalDur);

        std::vector<double> ref_states;
        ref_states.reserve(6 * N_horizon);
        double t_cur = t0;
        for (int i = 0; i < N_horizon; ++i) {
            if (t_cur > totalDur)
                t_cur = totalDur;

            Eigen::VectorXd pos = current_traj_.getPos(t_cur);
            Eigen::VectorXd vel = current_traj_.getVel(t_cur);
            Eigen::VectorXd acc = current_traj_.getAcc(t_cur);
            double yaw = atan2(vel.y(), vel.x());
            yaw = angles::shortest_angular_distance(current_yaw, yaw) + current_yaw;
            double yawdot = (vel.x() * acc.y() - vel.y() * acc.x())
                / (vel.x() * vel.x() + vel.y() * vel.y() + 1e-6);

            ref_states.push_back(pos.x());
            ref_states.push_back(pos.y());
            ref_states.push_back(yaw);
            ref_states.push_back(vel.x());
            ref_states.push_back(vel.y());
            ref_states.push_back(yawdot);

            t_cur += Ts_local;
            t_cur = std::clamp(t_cur, 0.0, totalDur);
        }

        std::vector<double> cur_vec = { robot_pos.x(), robot_pos.y(), current_yaw };
        auto predicted = mpc_->motion_prediction(cur_vec, mpc_->control_output_);

        mpc_->control_output_ = mpc_->solve(predicted, ref_states);
        if (use_control_output_) {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = mpc_->control_output_[0][0];
            cmd.linear.y = mpc_->control_output_[1][0];
            cmd.angular.z = mpc_->control_output_[2][0];
            cmd_vel_pub_->publish(cmd);
        }

        nav_msgs::msg::Path predict_path;
        predict_path.header.frame_id = target_frame_;
        predict_path.header.stamp = rclcpp::Clock().now();

        geometry_msgs::msg::PoseStamped pose_msg;
        for (int i = 0; i < ACADO_N; ++i) {
            pose_msg.pose.position.x = acadoVariables.x[NX * i + 0];
            pose_msg.pose.position.y = acadoVariables.x[NX * i + 1];
            predict_path.poses.push_back(pose_msg);
        }
        predict_path_pub_->publish(predict_path);
    }
*/
