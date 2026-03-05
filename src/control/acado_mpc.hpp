// #pragma once

// #include "../parameters.hpp"
// #include "acado_auxiliary_functions.h"
// #include "acado_common.h"
// #include "angles.h"
// #include "common.hpp"
// #include "type.hpp"
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

// namespace rose_planner::control {

// class AcadoMpc {
// public:
//     using Ptr = std::shared_ptr<AcadoMpc>;
//     AcadoMpc(Parameters params): params_(params) {
//         // weight_p_ = params.mpc_params.weight_p;
//         // weight_yaw_ = params.mpc_params.weight_yaw;
//         // weight_v_ = params.mpc_params.weight_v;
//         // weight_w_ = params.mpc_params.weight_w;
//         weight_p_ = 100;
//         weight_yaw_ = 200;
//         weight_v_ = 1;
//         weight_w_ = 1;
//         delay_time_ = 0.0;
//         control_output_ = initAcado();
//     }
//     static Ptr create(Parameters params) {
//         return std::make_shared<AcadoMpc>(params);
//     }
//     Output getOutput() {
//         return output_;
//     }
//     void getRefPoints(const int T_in, double dt_in) {
//         P_.clear();
//         P_.reserve(T_in);

//         double t_cur = traj_.getTimeByPos(now_state_w_.pos, 0.5);
//         if (t_cur < 0.0)
//             t_cur = 0.0;
//         double t = t_cur + dt_in + delay_time_;

//         Eigen::Vector2d pos_end = Eigen::Vector2d::Zero();
//         Eigen::Vector2d vel_end = Eigen::Vector2d::Zero();
//         Eigen::Vector2d acc_end = Eigen::Vector2d::Zero();
//         Eigen::Vector2d now_pos = now_state_w_.pos;
//         double yaw_end = 0.0;
//         double w_end = 0.0;
//         Eigen::Vector2d prev_pos = traj_.getPos(t);
//         for (int i = 0; i < T_in; ++i) {
//             TrajPoint tp;

//             if (t <= traj_duration_) {
//                 Eigen::Vector2d curr_pos = traj_.getPos(t);
//                 Eigen::Vector2d vec = curr_pos - now_pos;

//                 if (vec.norm() < 0.3) {
//                     prev_pos = curr_pos;
//                     t += dt_in;
//                     i--;
//                     continue;
//                 }
//                 tp.pos = traj_.getPos(t);
//                 tp.vel = traj_.getVel(t);
//                 tp.acc = traj_.getAcc(t);
//                 double yaw = traj_.getYaw(t);
//                 yaw = yaw_end + angles::shortest_angular_distance(yaw_end, yaw);
//                 tp.yaw = yaw;
//                 tp.w = traj_.getYawDot(t);

//                 pos_end = tp.pos;
//                 vel_end = tp.vel;
//                 acc_end = tp.acc;
//                 yaw_end = tp.yaw;
//                 w_end = tp.w;
//             } else {
//                 tp.pos = pos_end;
//                 tp.vel = Eigen::Vector2d::Zero();
//                 tp.acc = Eigen::Vector2d::Zero();
//                 tp.yaw = yaw_end;
//                 tp.w = w_end;
//             }

//             P_.push_back(tp);
//             t += dt_in;
//         }
//     }
//     void setCurrent(const State& c) {
//         now_state_w_ = c;
//     }
//     void setTrajectory(const TrajType& traj) {
//         if (traj.getPieceNum() > 1 && traj.getTotalDuration() > 0.01) {
//             traj_ = traj;
//             traj_duration_ = traj.getTotalDuration();
//         }
//     }
//     void solve() {
//         getRefPoints(N, Ts);
//         std::vector<double> ref_states;
//         ref_states.reserve(6 * N);
//         for (int i = 0; i < N; ++i) {
//             ref_states.push_back(P_[i].pos.x());
//             ref_states.push_back(P_[i].pos.y());
//             ref_states.push_back(P_[i].yaw);
//             ref_states.push_back(P_[i].vel.x());
//             ref_states.push_back(P_[i].vel.y());
//             ref_states.push_back(P_[i].w);
//         }
//         std::vector<double> cur_vec = {
//             now_state_w_.pos.x(),
//             now_state_w_.pos.y(),
//             now_state_w_.yaw,
//         };
//         auto predicted = motion_prediction(cur_vec, control_output_);
//         control_output_ = solve(predicted, ref_states);
//         output_.vel.x() = control_output_[0][0];
//         output_.vel.y() = control_output_[1][0];
//         output_.w = control_output_[2][0];
//         output_.pred_states.clear();
//         for (int i = 0; i < N; ++i) {
//             State state;
//             state.pos.x() = acadoVariables.x[i * NX + 0];
//             state.pos.y() = acadoVariables.x[i * NX + 1];
//             output_.pred_states.push_back(state);
//         }
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
//     Output output_;
//     Parameters params_;
//     TrajType traj_;
//     double traj_duration_;
//     double weight_p_, weight_yaw_, weight_v_, weight_w_;
//     std::vector<std::vector<double>> control_output_;
//     std::vector<TrajPoint> P_;
//     State now_state_w_;
//     double delay_time_;
// };

// } // namespace rose_planner::control