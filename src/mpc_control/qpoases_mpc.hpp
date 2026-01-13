// #pragma once
// #include "../parameters.hpp"
// #include "../trajectory_optimize/trajectory.hpp"
// #include "angles.h"
// #include "common.hpp"
// #include "mpc_common.hpp"
// #include "qpOASES.hpp"
// #include <Eigen/Dense>
// #include <chrono>
// #include <cmath>
// #include <iostream>
// #include <memory>
// #include <vector>

// namespace rose_planner {

// class QpOASESMpc {
// public:
//     using Ptr = std::shared_ptr<QpOASESMpc>;

//     QpOASESMpc(const Parameters& params) {
//         dt_ = params.mpc_params.dt;
//         max_iter_ = params.mpc_params.max_iter;
//         T_ = params.mpc_params.predict_steps;
//         max_speed_ = params.mpc_params.max_speed;
//         min_speed_ = params.mpc_params.min_speed;
//         max_accel_ = params.mpc_params.max_accel;
//         delay_num_ = params.mpc_params.delay_num;
//         Q_ = params.mpc_params.Q;
//         R_ = params.mpc_params.R;
//         Rd_ = params.mpc_params.Rd;
//         max_cv_ = max_accel_ * dt_;

//         xref_.setZero(4, 500);
//         dref_.setZero(2, 500);
//         output_.setZero(2, 500);
//         last_output_ = output_;

//         for (int i = 0; i < delay_num_; i++)
//             output_buff_.emplace_back(Eigen::Vector2d::Zero());
//     }

//     static Ptr create(const Parameters& params) {
//         return std::make_shared<QpOASESMpc>(params);
//     }

//     void setTrajectory(const TrajType& traj) {
//         if (traj.getPieceNum() > 1 && traj.getTotalDuration() > 0.01) {
//             traj_ = traj;
//             traj_duration_ = traj.getTotalDuration();
//         }
//     }

//     void setCurrent(const MPCState& c) {
//         now_state_ = c;
//     }

//     Eigen::Vector2d getOutput() {
//         Eigen::Vector2d cmd = Eigen::Vector2d::Zero();
//         cmd[0] = output_(0, delay_num_);
//         cmd[1] = output_(1, delay_num_);
//         return cmd;
//     }

//     void solve() {
//         P_.clear();
//         getRefPoints(T_, dt_);
//         for (int i = 0; i < T_; i++) {
//             xref_(0, i) = P_[i].x;
//             xref_(1, i) = P_[i].y;
//             xref_(2, i) = P_[i].vx;
//             xref_(3, i) = P_[i].vy;
//             dref_(0, i) = P_[i].vx;
//             dref_(1, i) = P_[i].vy;
//         }
//         getCmd();
//     }

//     void getCmd() {
//         for (int i = 0; i < max_iter_; i++) {
//             last_output_ = output_;
//             solveMPCV();
//             double du = 0;
//             for (int c = 0; c < output_.cols(); c++) {
//                 du += std::fabs(output_(0, c) - last_output_(0, c));
//                 du += std::fabs(output_(1, c) - last_output_(1, c));
//             }
//             if (du < 1e-4)
//                 break;
//         }
//         predictMotion(xopt_);
//         if (delay_num_ > 0) {
//             if (!output_buff_.empty())
//                 output_buff_.erase(output_buff_.begin());
//             output_buff_.push_back({ output_(0, delay_num_), output_(1, delay_num_) });
//         }
//     }
//     void getRefPoints(const int T_in, double dt_in) {
//         P_.clear();
//         getRefPointsInternal(T_in, dt_in);
//     }

//     void getRefPointsInternal(const int T_in, double dt_in) {
//         P_.clear();

//         double t_cur = traj_.getTimeByPos({ now_state_.x, now_state_.y }, 0.5);
//         if (t_cur < 0)
//             t_cur = 0;

//         Eigen::Vector2d pos_end = traj_.getPos(traj_duration_ - 0.01);

//         TrajPoint tp;
//         int j = 0;

//         for (double t = t_cur + dt_in; j < T_in; j++, t += dt_in) {
//             if (t <= traj_duration_) {
//                 Eigen::Vector2d pos = traj_.getPos(t);
//                 Eigen::Vector2d vel = traj_.getVel(t);
//                 Eigen::Vector2d acc = traj_.getAcc(t);
//                 tp.x = pos.x();
//                 tp.y = pos.y();
//                 tp.vx = vel.x();
//                 tp.vy = vel.y();
//                 tp.ax = acc.x();
//                 tp.ay = acc.y();
//             } else {
//                 tp.x = pos_end.x();
//                 tp.y = pos_end.y();
//                 tp.vx = 0;
//                 tp.vy = 0;
//                 tp.ax = 0;
//                 tp.ay = 0;
//             }

//             P_.push_back(tp);
//         }
//     }

//     void solveMPCV() {
//         const int steps = T_ - delay_num_;
//         const int dimx = 4 * steps;
//         const int dimu = 2 * steps;
//         const int nV = dimx + dimu;
//         const int nC = 4 + 2 * steps + 2 * (steps - 1) + 2 * (steps - 1);

//         // 构造 H（对角）
//         Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nV, nV);
//         for (int i = 0; i < steps; i++) {
//             int xi = 4 * i;
//             H(xi, xi) = 2 * Q_[0];
//             H(xi + 1, xi + 1) = 2 * Q_[1];
//             H(xi + 2, xi + 2) = 2 * Q_[2];
//             H(xi + 3, xi + 3) = 2 * Q_[3];
//         }
//         for (int i = 0; i < steps; i++) {
//             int ui = dimx + 2 * i;
//             H(ui, ui) = 2 * R_[0];
//             H(ui + 1, ui + 1) = 2 * R_[1];
//         }

//         // 构造梯度 g
//         Eigen::VectorXd g = Eigen::VectorXd::Zero(nV);
//         for (int i = 0; i < steps; i++) {
//             int xi = 4 * i;
//             g[xi] = -2 * Q_[0] * xref_(0, i + delay_num_);
//             g[xi + 1] = -2 * Q_[1] * xref_(1, i + delay_num_);
//             g[xi + 2] = -2 * Q_[2] * xref_(2, i + delay_num_);
//             g[xi + 3] = -2 * Q_[3] * xref_(3, i + delay_num_);
//         }

//         // 控制目标梯度
//         for (int i = 0; i < steps; i++) {
//             int ui = dimx + 2 * i;
//             g[ui] = -2 * R_[0] * dref_(0, i);
//             g[ui + 1] = -2 * R_[1] * dref_(1, i);
//         }

//         // 构造约束矩阵 A (密集)
//         Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nC, nV);
//         int row = 0;

//         // 初始状态约束 (x0 = now)
//         A(row + 0, 0) = 1;
//         A(row + 1, 1) = 1;
//         A(row + 2, 2) = 1;
//         A(row + 3, 3) = 1;
//         row += 4;

//         // 速度约束
//         for (int i = 0; i < steps; i++) {
//             int ui = dimx + 2 * i;
//             A(row + 2 * i, ui) = 1;
//             A(row + 2 * i + 1, ui + 1) = 1;
//         }
//         row += 2 * steps;

//         // 控制变化约束 Δu
//         for (int i = 1; i < steps; i++) {
//             int u_now = dimx + 2 * i;
//             int u_last = dimx + 2 * (i - 1);
//             A(row + 2 * (i - 1), u_now) = 1;
//             A(row + 2 * (i - 1), u_last) = -1;
//             A(row + 2 * (i - 1) + 1, u_now + 1) = 1;
//             A(row + 2 * (i - 1) + 1, u_last + 1) = -1;
//         }
//         row += 2 * (steps - 1);

//         // 加速度约束
//         const double inv_dt = 1.0 / dt_;
//         for (int i = 1; i < steps; i++) {
//             int u_now = dimx + 2 * i;
//             int u_last = dimx + 2 * (i - 1);
//             A(row + 2 * (i - 1), u_now) = inv_dt;
//             A(row + 2 * (i - 1), u_last) = -inv_dt;
//             A(row + 2 * (i - 1) + 1, u_now + 1) = inv_dt;
//             A(row + 2 * (i - 1) + 1, u_last + 1) = -inv_dt;
//         }

//         // 构造 QProblem 输入
//         qpOASES::real_t* H_qp = H.data();
//         qpOASES::real_t* g_qp = g.data();
//         qpOASES::real_t* A_qp = A.data();

//         // 变量上下界
//         std::vector<qpOASES::real_t> lb(nV, -max_speed_);
//         std::vector<qpOASES::real_t> ub(nV, max_speed_);

//         // 约束上下界
//         std::vector<qpOASES::real_t> lbA(nC, -1e10);
//         std::vector<qpOASES::real_t> ubA(nC, 1e10);

//         // 初始状态等式约束 bound
//         lbA[0] = ubA[0] = now_state_.x;
//         lbA[1] = ubA[1] = now_state_.y;
//         lbA[2] = ubA[2] = now_state_.vx;
//         lbA[3] = ubA[3] = now_state_.vy;

//         int nWSR = 200;

//         if (!solver_initialized_) {
//             solver_ = std::make_unique<qpOASES::QProblem>(nV, nC);
//             qpOASES::Options opt;
//             opt.setToMPC();
//             opt.printLevel = qpOASES::PL_NONE;
//             solver_->setOptions(opt);
//             solver_->init(H_qp, g_qp, A_qp, lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);
//             solver_initialized_ = true;
//         } else {
//             solver_->init(H_qp, g_qp, A_qp, lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);
//         }

//         // 读取解
//         std::vector<qpOASES::real_t> sol(nV);
//         solver_->getPrimalSolution(sol.data());

//         // 写回控制量
//         for (int i = 0; i < steps; i++) {
//             int ui = dimx + 2 * i;
//             output_(0, i + delay_num_) = sol[ui];
//             output_(1, i + delay_num_) = sol[ui + 1];
//         }
//     }

//     void predictMotionInternal() {
//         xbar_[0] = now_state_;
//         MPCState t = now_state_;
//         for (int i = 1; i <= T_; i++) {
//             t.x += output_(0, i - 1) * dt_;
//             t.y += output_(1, i - 1) * dt_;
//             t.vx = output_(0, i - 1);
//             t.vy = output_(1, i - 1);
//             xbar_[i] = t;
//         }
//     }

//     void predictMotion(MPCState* b) {
//         predictMotionInternal();
//         for (int i = 0; i <= T_; i++)
//             b[i] = xbar_[i];
//     }

//     double dt_;
//     int T_, max_iter_, delay_num_;
//     double max_speed_, min_speed_, max_cv_, max_accel_, traj_duration_;
//     std::vector<double> Q_, R_, Rd_;
//     TrajType traj_;
//     MPCState now_state_, xbar_[501], xopt_[501];
//     Eigen::MatrixXd xref_, dref_, output_, last_output_;
//     std::vector<TrajPoint> P_;
//     std::vector<Eigen::Vector2d> output_buff_;
//     bool solver_initialized_ = false;
//     std::unique_ptr<qpOASES::QProblem> solver_;
// };

// } // namespace rose_planner
