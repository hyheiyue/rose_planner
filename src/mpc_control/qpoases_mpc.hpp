// #pragma once
// #include "../parameters.hpp"
// #include "../trajectory_optimize/trajectory.hpp"
// #include "angles.h"
// #include "common.hpp"
// #include "mpc_common.hpp"
// #include "qpOASES.hpp"
// #include "rose_map/rose_map.hpp"
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

//     QpOASESMpc(rose_map::RoseMap::Ptr rose_map, const Parameters& params) {
//         rose_map_ = rose_map;
//         params_ = params;
//         fps_ = params.mpc_params.fps;
//         dt_ = params.mpc_params.dt;
//         max_iter_ = params.mpc_params.max_iter;
//         T_ = params.mpc_params.predict_steps;
//         max_speed_ = params.mpc_params.max_speed;
//         min_speed_ = params.mpc_params.min_speed;
//         max_accel_ = params.mpc_params.max_accel;
//         Q_ = params.mpc_params.Q;
//         R_ = params.mpc_params.R;
//         Rd_ = params.mpc_params.Rd;
//         max_cv_ = max_accel_ * dt_;
//         delay_time_ = params.mpc_params.delay_time;
//         xref_ = Eigen::MatrixXd::Zero(4, 500);
//         dref_ = Eigen::MatrixXd::Zero(2, 500);
//         output_ = Eigen::MatrixXd::Zero(2, 500);
//         last_output_ = output_;
//         last_final_output_ = output_;
//         xbar_.resize(T_ + 1);
//         xopt_.resize(T_ + 1);
//     }

//     static Ptr create(rose_map::RoseMap::Ptr rose_map, const Parameters& params) {
//         return std::make_shared<QpOASESMpc>(rose_map, params);
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
//         Eigen::Vector2d cmd(output_(0, 0), output_(1, 0));

//         return cmd;
//     }

//     void solve() {
//         P_.clear();
//         getRefPoints(T_, dt_);

//         for (int i = 0; i < T_; i++) {
//             xref_(0, i) = P_[i].pos.x();
//             xref_(1, i) = P_[i].pos.y();
//             xref_(2, i) = P_[i].vel.x();
//             xref_(3, i) = P_[i].vel.y();

//             dref_(0, i) = P_[i].vel.x();
//             dref_(1, i) = P_[i].vel.y();
//         }

//         getCmd();
//     }

//     void getRefPoints(const int T_in, double dt_in) {
//         P_.clear();
//         getRefPointsInternal(T_in, dt_in);
//     }

//     double getEsdf(const Eigen::Vector2d& pos) {
//         double dist = rose_map::ESDF::kInf;
//         rose_map::VoxelKey2D key = rose_map_->worldToKey2D(pos.cast<float>());
//         int idx = rose_map_->key2DToIndex2D(key);
//         if (idx >= 0) {
//             dist = rose_map_->esdf_[idx];
//         }
//         return dist;
//     }
//     void getRefPointsInternal(const int T_in, double dt_in) {
//         P_.clear();

//         double t_cur = traj_.getTimeByPos(now_state_.pos, 0.5);
//         if (t_cur < 0)
//             t_cur = 0;

//         Eigen::Vector2d pos_end = Eigen::Vector2d::Zero();
//         Eigen::Vector2d vel_end = Eigen::Vector2d::Zero();

//         const double smoothing_tau = 0.20; // 时间常数，越大越平滑（秒）
//         const double min_speed_local = std::max(0.01, min_speed_);
//         std::vector<TrajPoint> raw;
//         std::vector<double> dts;
//         raw.reserve(T_in);
//         dts.reserve(T_in);
//         double t = t_cur + dt_in + delay_time_;
//         for (int j = 0; j < T_in; ++j) {
//             double dt_eff = dt_in;

//             TrajPoint rp;
//             if (t <= traj_duration_) {
//                 Eigen::Vector2d pos = traj_.getPos(t);
//                 Eigen::Vector2d vel = traj_.getVel(t);
//                 Eigen::Vector2d acc = traj_.getAcc(t);
//                 Eigen::Vector2d vel_raw = vel;
//                 double dist = getEsdf(pos);
//                 if (vel.norm() > 1e-8) {
//                     const double R = params_.robot_radius * 2.0;
//                     if (dist < R) {
//                         vel = vel * (dist / R);
//                     }
//                     // 保证最小速度
//                     double vel_norm = vel.norm();
//                     if (vel_norm < min_speed_local && vel_raw.norm() > 1e-8) {
//                         vel = vel.normalized() * min_speed_local;
//                     }
//                     // 防止 vel_raw 为 0 导致除0
//                     double s = (vel_raw.norm() > 1e-8) ? (vel.norm() / vel_raw.norm()) : 1.0;
//                     dt_eff = dt_in * s;
//                     acc *= s;
//                 } else {
//                     // vel 近似为0，保持 acc 为 0，dt_eff 不变
//                     acc.setZero();
//                 }
//                 rp.pos = pos;
//                 rp.vel = vel;
//                 rp.acc = acc;
//                 pos_end = pos;
//                 vel_end = vel;
//             } else {
//                 rp.pos = pos_end;
//                 double decay = std::max(0.0, 1.0 - (t - traj_duration_));
//                 Eigen::Vector2d vel = vel_end * decay;
//                 rp.vel = vel;
//                 rp.acc = Eigen::Vector2d::Zero();
//             }

//             raw.push_back(rp);
//             dts.push_back(dt_eff);
//             t += dt_eff;
//         }

//         const int N = (int)raw.size();
//         std::vector<Eigen::Vector2d> v_fwd(N), v_both(N);

//         if (N == 0)
//             return;

//         // forward pass
//         v_fwd[0] = raw[0].vel;
//         for (int i = 1; i < N; ++i) {
//             double alpha = std::exp(-dts[i - 1] / smoothing_tau);
//             v_fwd[i] = alpha * v_fwd[i - 1] + (1.0 - alpha) * raw[i].vel;
//         }

//         // backward pass (reduce相位)
//         v_both[N - 1] = v_fwd[N - 1];
//         for (int i = N - 2; i >= 0; --i) {
//             double alpha = std::exp(-dts[i] / smoothing_tau);
//             v_both[i] = alpha * v_both[i + 1] + (1.0 - alpha) * v_fwd[i];
//         }

//         for (int i = 0; i < N - 1; ++i) {
//             Eigen::Vector2d dv = v_both[i + 1] - v_both[i];
//             double max_dv_norm = max_accel_ * dts[i];
//             double dv_norm = dv.norm();
//             if (dv_norm > max_dv_norm && dv_norm > 1e-12) {
//                 v_both[i + 1] = v_both[i] + dv * (max_dv_norm / dv_norm);
//             }
//         }
//         for (int i = N - 2; i >= 0; --i) {
//             Eigen::Vector2d dv = v_both[i + 1] - v_both[i];
//             double max_dv_norm = max_accel_ * dts[i];
//             double dv_norm = dv.norm();
//             if (dv_norm > max_dv_norm && dv_norm > 1e-12) {
//                 v_both[i] = v_both[i + 1] - dv * (max_dv_norm / dv_norm);
//             }
//         }
//         P_.reserve(N);
//         for (int i = 0; i < N; ++i) {
//             TrajPoint tp;
//             tp.pos = raw[i].pos;
//             tp.vel = v_both[i];
//             tp.acc = raw[i].acc;
//             // compute acceleration: use backward difference if possible, else forward
//             if (i == 0) {
//                 // use available raw acc as initial guess, but prefer difference to next if exists
//                 if (N > 1 && dts[0] > 1e-12) {
//                     tp.acc.x() = (v_both[1].x() - v_both[0].x()) / dts[0];
//                     tp.acc.y() = (v_both[1].y() - v_both[0].y()) / dts[0];
//                 } else {
//                     tp.acc = raw[0].acc;
//                 }
//             } else {
//                 double dt_here = dts[i - 1] > 1e-12 ? dts[i - 1] : dts.back();
//                 tp.acc.x() = (v_both[i].x() - v_both[i - 1].x()) / dt_here;
//                 tp.acc.y() = (v_both[i].y() - v_both[i - 1].y()) / dt_here;
//             }

//             P_.push_back(tp);
//         }
//     }
//     void stateTransOmni(MPCState& s, double vx_cmd, double vy_cmd) {
//         s.pos.x() += vx_cmd * dt_;
//         s.pos.y() += vy_cmd * dt_;
//         s.vel.x() = vx_cmd;
//         s.vel.y() = vy_cmd;
//     }

//     void predictMotionInternal() {
//         xbar_[0] = now_state_;
//         MPCState temp = now_state_;

//         for (int i = 1; i <= T_; i++) {
//             stateTransOmni(temp, output_(0, i - 1), output_(1, i - 1));
//             xbar_[i] = temp;
//         }
//     }
//     void getLinearModel(const MPCState&) {
//         // 状态4维 [x, y, vx, vy]
//         // 控制2维 [vx_cmd, vy_cmd]
//         A_ = Eigen::MatrixXd::Identity(4, 4);
//         B_ = Eigen::MatrixXd::Zero(4, 2);
//         C_ = Eigen::VectorXd::Zero(4);

//         // 控制直接影响位置和速度
//         B_(0, 0) = dt_; // x += vx_cmd * dt
//         B_(1, 1) = dt_; // y += vy_cmd * dt
//         B_(2, 0) = 1; // vx = vx_cmd
//         B_(3, 1) = 1; // vy = vy_cmd
//     }
//     void predictMotion() {
//         predictMotionInternal();
//     }

//     void predictMotion(std::vector<MPCState>& b) {
//         for (int i = 0; i <= T_; i++)
//             b[i] = xbar_[i];
//     }
//     void solveMPCV() {
//         const int steps = T_;
//         const int nx = 4;
//         const int nu = 2;

//         const int dimx = nx * steps;
//         const int dimu = nu * steps;
//         const int nV = dimx + dimu;

//         // 约束：
//         // 1) 初始状态 4
//         // 2) 动力学约束 4*(steps-1)
//         const int nC = 4 + 4 * (steps - 1);

//         Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nV, nV);
//         Eigen::VectorXd g = Eigen::VectorXd::Zero(nV);

//         // 状态代价
//         for (int k = 0; k < steps; k++) {
//             int xi = nx * k;
//             for (int i = 0; i < nx; i++) {
//                 H(xi + i, xi + i) = Q_[i];
//                 g[xi + i] = -Q_[i] * xref_(i, k);
//             }
//         }

//         // 控制代价（速度跟踪）
//         for (int k = 0; k < steps; k++) {
//             int ui = dimx + nu * k;
//             H(ui + 0, ui + 0) = R_[0];
//             H(ui + 1, ui + 1) = R_[1];
//             g[ui + 0] = -R_[0] * dref_(0, k);
//             g[ui + 1] = -R_[1] * dref_(1, k);
//         }

//         Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nC, nV);
//         Eigen::VectorXd lbA = Eigen::VectorXd::Zero(nC);
//         Eigen::VectorXd ubA = Eigen::VectorXd::Zero(nC);

//         int row = 0;

//         // ---- 初始状态 ----
//         Eigen::Vector4d x0;
//         x0 << now_state_.pos.x(), now_state_.pos.y(), now_state_.vel.x(), now_state_.vel.y();

//         for (int i = 0; i < 4; i++) {
//             A(row, i) = 1.0;
//             lbA[row] = ubA[row] = x0[i];
//             row++;
//         }

//         // ---- 动力学约束（与你 stateTransOmni 完全一致）----
//         for (int k = 0; k < steps - 1; k++) {
//             int xk = nx * k;
//             int xkp1 = nx * (k + 1);
//             int uk = dimx + nu * k;

//             // px_{k+1} = px_k + vx_cmd * dt
//             A(row, xkp1 + 0) = 1;
//             A(row, xk + 0) = -1;
//             A(row, uk + 0) = -dt_;
//             lbA[row] = ubA[row] = 0;
//             row++;

//             // py
//             A(row, xkp1 + 1) = 1;
//             A(row, xk + 1) = -1;
//             A(row, uk + 1) = -dt_;
//             lbA[row] = ubA[row] = 0;
//             row++;

//             // vx_{k+1} = vx_cmd
//             A(row, xkp1 + 2) = 1;
//             A(row, uk + 0) = -1;
//             lbA[row] = ubA[row] = 0;
//             row++;

//             // vy_{k+1} = vy_cmd
//             A(row, xkp1 + 3) = 1;
//             A(row, uk + 1) = -1;
//             lbA[row] = ubA[row] = 0;
//             row++;
//         }

//         Eigen::VectorXd lb = Eigen::VectorXd::Constant(nV, -1e10);
//         Eigen::VectorXd ub = Eigen::VectorXd::Constant(nV, 1e10);

//         for (int k = 0; k < steps; k++) {
//             int xi = nx * k;
//             int ui = dimx + nu * k;

//             // 速度状态限幅
//             lb[xi + 2] = -max_speed_;
//             ub[xi + 2] = max_speed_;
//             lb[xi + 3] = -max_speed_;
//             ub[xi + 3] = max_speed_;

//             // 控制量 = 速度指令
//             lb[ui + 0] = -max_speed_;
//             ub[ui + 0] = max_speed_;
//             lb[ui + 1] = -max_speed_;
//             ub[ui + 1] = max_speed_;
//         }

//         int nWSR = 100;

//         if (!solver_initialized_) {
//             solver_ = std::make_unique<qpOASES::QProblem>(nV, nC);
//             qpOASES::Options opt;
//             opt.setToMPC();
//             opt.printLevel = qpOASES::PL_NONE;
//             solver_->setOptions(opt);
//             solver_initialized_ = true;
//         }

//         solver_->init(
//             H.data(),
//             g.data(),
//             A.data(),
//             lb.data(),
//             ub.data(),
//             lbA.data(),
//             ubA.data(),
//             nWSR
//         );

//         Eigen::VectorXd sol(nV);
//         solver_->getPrimalSolution(sol.data());

//         for (int k = 0; k < steps; k++) {
//             int ui = dimx + nu * k;
//             output_(0, k) = sol[ui + 0];
//             output_(1, k) = sol[ui + 1];
//         }
//     }

//     void getCmd() {
//         const double max_time = 1.0 / static_cast<double>(fps_);
//         const auto t_start = std::chrono::steady_clock::now();
//         for (int iter = 0; iter < max_iter_; ++iter) {
//             const auto t_now = std::chrono::steady_clock::now();
//             const double elapsed = std::chrono::duration<double>(t_now - t_start).count();

//             if (elapsed >= max_time) {
//                 break;
//             }

//             predictMotionInternal();
//             last_output_ = output_;
//             solveMPCV();

//             double du = 0.0;
//             for (int c = 0; c < output_.cols(); ++c) {
//                 du += std::fabs(output_(0, c) - last_output_(0, c));
//                 du += std::fabs(output_(1, c) - last_output_(1, c));
//             }

//             if (du < 1e-4) {
//                 break;
//             }
//         }
//         last_final_output_ = output_;
//         predictMotion(xopt_);
//     }
//     Parameters params_;
//     int fps_;
//     double dt_;
//     int T_, max_iter_;
//     double delay_time_;
//     double max_speed_, min_speed_, max_cv_, max_accel_, traj_duration_;
//     std::vector<double> Q_, R_, Rd_;
//     TrajType traj_;
//     MPCState now_state_;
//     std::vector<MPCState> xbar_, xopt_;
//     Eigen::MatrixXd xref_, dref_, output_, last_output_;
//     Eigen::MatrixXd last_final_output_;
//     Eigen::MatrixXd A_, B_;
//     Eigen::VectorXd C_;
//     std::vector<TrajPoint> P_;
//     bool solver_initialized_ = false;
//     std::unique_ptr<qpOASES::QProblem> solver_;
//     rose_map::RoseMap::Ptr rose_map_;
// };

// } // namespace rose_planner
