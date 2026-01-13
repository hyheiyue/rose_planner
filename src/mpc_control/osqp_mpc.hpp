#pragma once
#include "../parameters.hpp"
#include "../trajectory_optimize/trajectory.hpp"
//clang-format off
#include "OsqpEigen/OsqpEigen.h"
//clang-format on
#include "angles.h"
#include "common.hpp"
#include "mpc_common.hpp"
#include "rose_map/rose_map.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
namespace rose_planner {

class OsqpMpc {
public:
    using Ptr = std::shared_ptr<OsqpMpc>;

    OsqpMpc(rose_map::RoseMap::Ptr rose_map, const Parameters& params) {
        rose_map_ = rose_map;
        params_ = params;
        fps_ = params.mpc_params.fps;
        dt_ = params.mpc_params.dt;
        max_iter_ = params.mpc_params.max_iter;
        T_ = params.mpc_params.predict_steps;
        max_speed_ = params.mpc_params.max_speed;
        min_speed_ = params.mpc_params.min_speed;
        max_accel_ = params.mpc_params.max_accel;
        delay_num_ = params.mpc_params.delay_num;
        Q_ = params.mpc_params.Q;
        R_ = params.mpc_params.R;
        Rd_ = params.mpc_params.Rd;
        max_cv_ = max_accel_ * dt_;

        xref_ = Eigen::MatrixXd::Zero(4, 500);
        dref_ = Eigen::MatrixXd::Zero(2, 500);
        output_ = Eigen::MatrixXd::Zero(2, 500);
        last_output_ = output_;
        last_final_output_ = output_;
        xbar_.resize(T_ + 1);
        xopt_.resize(T_ + 1);
        for (int i = 0; i < delay_num_; i++)
            output_buff_.emplace_back(Eigen::Vector2d::Zero());
    }

    static Ptr create(rose_map::RoseMap::Ptr rose_map, const Parameters& params) {
        return std::make_shared<OsqpMpc>(rose_map, params);
    }

    void setTrajectory(const TrajType& traj) {
        if (traj.getPieceNum() > 1 && traj.getTotalDuration() > 0.01) {
            traj_ = traj;
            traj_duration_ = traj.getTotalDuration();
        }
    }

    void setCurrent(const MPCState& c) {
        now_state_ = c;
    }

    Eigen::Vector2d getOutput() {
        Eigen::Vector2d cmd = Eigen::Vector2d::Zero();

        if (delay_num_ < 0 || delay_num_ >= output_.cols()) {
            std::cerr << "[MPC] Warning: delay_num out of range\n";
            return cmd;
        }

        cmd[0] = output_(0, delay_num_);
        cmd[1] = output_(1, delay_num_);
        return cmd;
    }

    void solve() {
        P_.clear();
        getRefPoints(T_, dt_);

        for (int i = 0; i < T_; i++) {
            xref_(0, i) = P_[i].pos.x();
            xref_(1, i) = P_[i].pos.y();
            xref_(2, i) = P_[i].vel.x();
            xref_(3, i) = P_[i].vel.y();

            dref_(0, i) = P_[i].vel.x();
            dref_(1, i) = P_[i].vel.y();
        }

        getCmd();
    }

    void getRefPoints(const int T_in, double dt_in) {
        P_.clear();
        getRefPointsInternal(T_in, dt_in);
    }

private:
    double getEsdf(const Eigen::Vector2d& pos) {
        double dist = rose_map::ESDF::kInf;
        rose_map::VoxelKey2D key = rose_map_->worldToKey2D(pos.cast<float>());
        int idx = rose_map_->key2DToIndex2D(key);
        if (idx >= 0) {
            dist = rose_map_->esdf_[idx];
        }
        return dist;
    }
    void getRefPointsInternal(const int T_in, double dt_in) {
        P_.clear();

        double t_cur = traj_.getTimeByPos({ now_state_.x, now_state_.y }, 0.5);
        if (t_cur < 0)
            t_cur = 0;

        Eigen::Vector2d pos_end = Eigen::Vector2d::Zero();
        Eigen::Vector2d vel_end = Eigen::Vector2d::Zero();

        const double smoothing_tau = 0.20; // 时间常数，越大越平滑（秒）
        const double min_speed_local = std::max(0.01, min_speed_);
        std::vector<TrajPoint> raw;
        std::vector<double> dts;
        raw.reserve(T_in);
        dts.reserve(T_in);
        double t = t_cur;
        for (int j = 0; j < T_in; ++j) {
            double dt_eff = dt_in;

            TrajPoint rp;
            if (t <= traj_duration_) {
                Eigen::Vector2d pos = traj_.getPos(t);
                Eigen::Vector2d vel = traj_.getVel(t);
                Eigen::Vector2d acc = traj_.getAcc(t);
                Eigen::Vector2d vel_raw = vel;
                double dist = getEsdf(pos);
                if (vel.norm() > 1e-8) {
                    const double R = params_.robot_radius * 2.0;
                    if (dist < R) {
                        vel = vel * (dist / R);
                    }
                    // 保证最小速度
                    double vel_norm = vel.norm();
                    if (vel_norm < min_speed_local && vel_raw.norm() > 1e-8) {
                        vel = vel.normalized() * min_speed_local;
                    }
                    // 防止 vel_raw 为 0 导致除0
                    double s = (vel_raw.norm() > 1e-8) ? (vel.norm() / vel_raw.norm()) : 1.0;
                    dt_eff = dt_in * s;
                    acc *= s;
                } else {
                    // vel 近似为0，保持 acc 为 0，dt_eff 不变
                    acc.setZero();
                }
                rp.pos = pos;
                rp.vel = vel;
                rp.acc = acc;
                pos_end = pos;
                vel_end = vel;
            } else {
                rp.pos = pos_end;
                double decay = std::max(0.0, 1.0 - (t - traj_duration_));
                Eigen::Vector2d vel = vel_end * decay;
                rp.vel = vel;
                rp.acc = Eigen::Vector2d::Zero();
            }

            raw.push_back(rp);
            dts.push_back(dt_eff);
            t += dt_eff;
        }

        const int N = (int)raw.size();
        std::vector<Eigen::Vector2d> v_fwd(N), v_both(N);

        if (N == 0)
            return;

        // forward pass
        v_fwd[0] = raw[0].vel;
        for (int i = 1; i < N; ++i) {
            double alpha = std::exp(-dts[i - 1] / smoothing_tau);
            v_fwd[i] = alpha * v_fwd[i - 1] + (1.0 - alpha) * raw[i].vel;
        }

        // backward pass (reduce相位)
        v_both[N - 1] = v_fwd[N - 1];
        for (int i = N - 2; i >= 0; --i) {
            double alpha = std::exp(-dts[i] / smoothing_tau);
            v_both[i] = alpha * v_both[i + 1] + (1.0 - alpha) * v_fwd[i];
        }

        for (int i = 0; i < N - 1; ++i) {
            Eigen::Vector2d dv = v_both[i + 1] - v_both[i];
            double max_dv_norm = max_accel_ * dts[i];
            double dv_norm = dv.norm();
            if (dv_norm > max_dv_norm && dv_norm > 1e-12) {
                v_both[i + 1] = v_both[i] + dv * (max_dv_norm / dv_norm);
            }
        }
        for (int i = N - 2; i >= 0; --i) {
            Eigen::Vector2d dv = v_both[i + 1] - v_both[i];
            double max_dv_norm = max_accel_ * dts[i];
            double dv_norm = dv.norm();
            if (dv_norm > max_dv_norm && dv_norm > 1e-12) {
                v_both[i] = v_both[i + 1] - dv * (max_dv_norm / dv_norm);
            }
        }
        P_.reserve(N);
        for (int i = 0; i < N; ++i) {
            TrajPoint tp;
            tp.pos = raw[i].pos;
            tp.vel = v_both[i];
            tp.acc = raw[i].acc;
            // compute acceleration: use backward difference if possible, else forward
            if (i == 0) {
                // use available raw acc as initial guess, but prefer difference to next if exists
                if (N > 1 && dts[0] > 1e-12) {
                    tp.acc.x() = (v_both[1].x() - v_both[0].x()) / dts[0];
                    tp.acc.y() = (v_both[1].y() - v_both[0].y()) / dts[0];
                } else {
                    tp.acc = raw[0].acc;
                }
            } else {
                double dt_here = dts[i - 1] > 1e-12 ? dts[i - 1] : dts.back();
                tp.acc.x() = (v_both[i].x() - v_both[i - 1].x()) / dt_here;
                tp.acc.y() = (v_both[i].y() - v_both[i - 1].y()) / dt_here;
            }

            P_.push_back(tp);
        }
    }

    void stateTransOmni(MPCState& s, double vx_cmd, double vy_cmd) {
        s.x += vx_cmd * dt_;
        s.y += vy_cmd * dt_;
        s.vx = vx_cmd;
        s.vy = vy_cmd;
    }

    void predictMotionInternal() {
        xbar_[0] = now_state_;
        MPCState temp = now_state_;

        for (int i = 1; i <= T_; i++) {
            stateTransOmni(temp, output_(0, i - 1), output_(1, i - 1));
            xbar_[i] = temp;
        }
    }
    void getLinearModel(const MPCState&) {
        // 状态4维 [x, y, vx, vy]
        // 控制2维 [vx_cmd, vy_cmd]
        A_ = Eigen::MatrixXd::Identity(4, 4);
        B_ = Eigen::MatrixXd::Zero(4, 2);
        C_ = Eigen::VectorXd::Zero(4);

        // 控制直接影响位置和速度
        B_(0, 0) = dt_; // x += vx_cmd * dt
        B_(1, 1) = dt_; // y += vy_cmd * dt
        B_(2, 0) = 1; // vx = vx_cmd
        B_(3, 1) = 1; // vy = vy_cmd
    }
    void predictMotion() {
        predictMotionInternal();
    }

    void predictMotion(std::vector<MPCState>& b) {
        for (int i = 0; i <= T_; i++)
            b[i] = xbar_[i];
    }

public:
    void solveMPCV() {
        const int steps = T_ - delay_num_;
        if (steps <= 0) {
            std::cerr << "[MPC] Not enough steps to solve.\n";
            return;
        }

        const int dimx = 4 * steps;
        const int dimu = 2 * steps;
        const int nx = dimx + dimu;

        // 生成梯度
        qp_gradient_ = Eigen::VectorXd::Zero(nx);

        // 状态梯度
        for (int i = 0; i < steps; i++) {
            int xi = 4 * i;
            qp_gradient_[xi] = -2 * Q_[0] * xref_(0, i + delay_num_);
            qp_gradient_[xi + 1] = -2 * Q_[1] * xref_(1, i + delay_num_);
            qp_gradient_[xi + 2] = -2 * Q_[2] * xref_(2, i + delay_num_);
            qp_gradient_[xi + 3] = -2 * Q_[3] * xref_(3, i + delay_num_);
        }

        // 控制梯度
        for (int i = 0; i < steps; i++) {
            int ui = dimx + 2 * i;
            qp_gradient_[ui] = -2 * R_[0] * dref_(0, i);
            qp_gradient_[ui + 1] = -2 * R_[1] * dref_(1, i);
        }

        // 生成 Hessian（仅对角，范围严格控制在 nx 内）
        std::vector<Eigen::Triplet<double>> H_trip;
        H_trip.reserve(nx);

        // 状态部分对角
        for (int i = 0; i < steps; i++) {
            int xi = 4 * i;
            H_trip.emplace_back(xi, xi, 2 * Q_[0]);
            H_trip.emplace_back(xi + 1, xi + 1, 2 * Q_[1]);
            H_trip.emplace_back(xi + 2, xi + 2, 2 * Q_[2]);
            H_trip.emplace_back(xi + 3, xi + 3, 2 * Q_[3]);
        }

        // 控制部分对角
        for (int i = 0; i < steps; i++) {
            int ui = dimx + 2 * i;
            H_trip.emplace_back(ui, ui, 2 * R_[0]);
            H_trip.emplace_back(ui + 1, ui + 1, 2 * R_[1]);
        }

        qp_hessian_.resize(nx, nx);
        qp_hessian_.setFromTriplets(H_trip.begin(), H_trip.end());
        qp_hessian_.makeCompressed();

        // 生成约束 Bound
        const int dyn_rows = 4 * steps;
        const int speed_rows = 2 * steps;
        const int cv_rows = 2 * (steps - 1);
        const int acc_rows = 2 * (steps - 1);
        const int nc = dyn_rows + speed_rows + cv_rows + acc_rows;

        qp_lowerBound_.resize(nc);
        qp_upperBound_.resize(nc);
        qp_lowerBound_.setConstant(-1e10);
        qp_upperBound_.setConstant(1e10);

        // 初始状态 bound
        Eigen::Vector4d x0;
        Eigen::Vector2d start_v(now_state_.vx, now_state_.vy);
        //if (start_v.norm() < min_speed_) {
        // auto output = getOutput();
        // start_v = output.normalized() * min_speed_;
        //}
        x0 << now_state_.x, now_state_.y, start_v.x(), start_v.y();

        qp_lowerBound_.segment<4>(0) = x0;
        qp_upperBound_.segment<4>(0) = x0;

        // 动力学约束 = 0
        for (int i = 1; i < steps; i++) {
            int r = 4 * i;
            for (int k = 0; k < 4; k++) {
                qp_lowerBound_[r + k] = 0;
                qp_upperBound_[r + k] = 0;
            }
        }

        // 速度约束
        int offset = dyn_rows;
        for (int i = 0; i < steps; i++) {
            int r = offset + 2 * i;
            qp_lowerBound_[r] = -max_speed_;
            qp_upperBound_[r] = max_speed_;
            qp_lowerBound_[r + 1] = -max_speed_;
            qp_upperBound_[r + 1] = max_speed_;
        }

        // 控制变化约束
        offset += speed_rows;
        for (int i = 1; i < steps; i++) {
            int r = offset + 2 * (i - 1);
            qp_lowerBound_[r] = -max_cv_;
            qp_upperBound_[r] = max_cv_;
            qp_lowerBound_[r + 1] = -max_cv_;
            qp_upperBound_[r + 1] = max_cv_;
        }

        // 加速度约束
        offset += cv_rows;
        for (int i = 1; i < steps; i++) {
            int r = offset + 2 * (i - 1);
            qp_lowerBound_[r] = -max_accel_;
            qp_upperBound_[r] = max_accel_;
            qp_lowerBound_[r + 1] = -max_accel_;
            qp_upperBound_[r + 1] = max_accel_;
        }

        // 生成 A 结构（仅第一次）
        if (!solver_initialized_) {
            Eigen::SparseMatrix<double> A;
            std::vector<Eigen::Triplet<double>> A_trip;

            // 初始状态约束
            for (int i = 0; i < 4; i++)
                A_trip.emplace_back(i, i, 1.0);

            // 动力学约束
            for (int i = 1; i < steps; i++) {
                int r = 4 * i;
                int last = 4 * (i - 1);
                int ucol = dimx + 2 * (i - 1);

                A_trip.emplace_back(r, r, 1.0);
                A_trip.emplace_back(r, last, -1.0);
                A_trip.emplace_back(r, ucol, -dt_);

                A_trip.emplace_back(r + 1, r + 1, 1.0);
                A_trip.emplace_back(r + 1, last + 1, -1.0);
                A_trip.emplace_back(r + 1, ucol + 1, -dt_);

                A_trip.emplace_back(r + 2, r + 2, 1.0);
                A_trip.emplace_back(r + 2, ucol, -1.0);

                A_trip.emplace_back(r + 3, r + 3, 1.0);
                A_trip.emplace_back(r + 3, ucol + 1, -1.0);
            }

            // 速度约束
            offset = dyn_rows;
            for (int i = 0; i < steps; i++) {
                int r = offset + 2 * i;
                int c = dimx + 2 * i;
                A_trip.emplace_back(r, c, 1.0);
                A_trip.emplace_back(r + 1, c + 1, 1.0);
            }

            // 控制变化约束
            offset += speed_rows;
            for (int i = 1; i < steps; i++) {
                int r = offset + 2 * (i - 1);
                int u_now = dimx + 2 * i;
                int u_last = dimx + 2 * (i - 1);
                A_trip.emplace_back(r, u_now, 1.0);
                A_trip.emplace_back(r, u_last, -1.0);
                A_trip.emplace_back(r + 1, u_now + 1, 1.0);
                A_trip.emplace_back(r + 1, u_last + 1, -1.0);
            }

            // 加速度约束
            offset += cv_rows;
            const double inv_dt = 1.0 / dt_;
            for (int i = 1; i < steps; i++) {
                int r = offset + 2 * (i - 1);
                int u_now = dimx + 2 * i;
                int u_last = dimx + 2 * (i - 1);
                A_trip.emplace_back(r, u_now, inv_dt);
                A_trip.emplace_back(r, u_last, -inv_dt);
                A_trip.emplace_back(r + 1, u_now + 1, inv_dt);
                A_trip.emplace_back(r + 1, u_last + 1, -inv_dt);
            }

            A.resize(nc, nx);
            A.setFromTriplets(A_trip.begin(), A_trip.end());
            A.makeCompressed();
            A_cache_ = A;

            solver_.settings()->setWarmStart(true);
            solver_.settings()->setAdaptiveRho(true);
            solver_.settings()->setRho(0.05);
            solver_.settings()->setMaxIteration(2000);
            solver_.settings()->setTimeLimit(0.02);
            solver_.settings()->setAbsoluteTolerance(1e-4);
            solver_.settings()->setRelativeTolerance(1e-4);
            solver_.settings()->setVerbosity(0);

            solver_.data()->setNumberOfVariables(nx);
            solver_.data()->setNumberOfConstraints(nc);
            solver_.data()->setLinearConstraintsMatrix(A_cache_);
            solver_.data()->setHessianMatrix(qp_hessian_);
            solver_.data()->setGradient(qp_gradient_);
            solver_.data()->setLowerBound(qp_lowerBound_);
            solver_.data()->setUpperBound(qp_upperBound_);

            if (!solver_.initSolver()) {
                std::cerr << "[OSQP] initSolver failed! QP contains NaN/Inf!\n";
                return;
            }
            solver_initialized_ = true;
        }

        bool hessian_bad = false;
        for (int k = 0; k < qp_hessian_.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(qp_hessian_, k); it; ++it) {
                if (!std::isfinite(it.value())) {
                    hessian_bad = true;
                    break;
                }
            }
            if (hessian_bad)
                break;
        }

        if (hessian_bad || !qp_gradient_.allFinite()) {
            std::cerr << "[QP ERROR] Contains NaN/Inf! Aborting solve.\n";
            return;
        }

        solver_.updateHessianMatrix(qp_hessian_);
        solver_.updateGradient(qp_gradient_);
        solver_.updateBounds(qp_lowerBound_, qp_upperBound_);
        solver_.solveProblem();

        Eigen::VectorXd sol = solver_.getSolution();
        if (!sol.allFinite()) {
            std::cerr << "[OSQP] Solution is NaN/Inf!\n";
            return;
        }
        for (int i = 0; i < steps; i++) {
            int ui = dimx + 2 * i;
            output_(0, i + delay_num_) = sol[ui];
            output_(1, i + delay_num_) = sol[ui + 1];
        }
    }

    void getCmd() {
        const double max_time = 1.0 / static_cast<double>(fps_);
        const auto t_start = std::chrono::steady_clock::now();
        for (int iter = 0; iter < max_iter_; ++iter) {
            const auto t_now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(t_now - t_start).count();

            if (elapsed >= max_time) {
                break;
            }

            predictMotionInternal();
            last_output_ = output_;
            solveMPCV();

            double du = 0.0;
            for (int c = 0; c < output_.cols(); ++c) {
                du += std::fabs(output_(0, c) - last_output_(0, c));
                du += std::fabs(output_(1, c) - last_output_(1, c));
            }

            if (du < 1e-4) {
                break;
            }
        }
        last_final_output_ = output_;
        predictMotion(xopt_);

        if (delay_num_ > 0) {
            if (!output_buff_.empty())
                output_buff_.erase(output_buff_.begin());
            output_buff_.push_back({ output_(0, delay_num_), output_(1, delay_num_) });
        }
    }
    int fps_;
    double dt_;
    int T_;
    int max_iter_;
    int delay_num_;
    double max_speed_;
    double min_speed_;
    double max_cv_;
    double max_accel_;
    double traj_duration_;
    Parameters params_;
    std::vector<double> Q_, R_, Rd_;
    TrajType traj_;
    MPCState now_state_;
    std::vector<MPCState> xbar_;
    std::vector<MPCState> xopt_;
    Eigen::MatrixXd A_, B_;
    Eigen::VectorXd C_;
    Eigen::MatrixXd xref_;
    Eigen::MatrixXd dref_;
    Eigen::MatrixXd output_;
    Eigen::MatrixXd last_output_;
    Eigen::MatrixXd last_final_output_;
    std::vector<Eigen::Vector2d> output_buff_;
    std::vector<TrajPoint> P_;
    OsqpEigen::Solver solver_;
    Eigen::VectorXd qp_gradient_;
    Eigen::VectorXd qp_lowerBound_;
    Eigen::VectorXd qp_upperBound_;
    Eigen::SparseMatrix<double> qp_hessian_;
    Eigen::SparseMatrix<double> A_cache_;
    int qp_nx_ = 0;
    int qp_nc_ = 0;
    bool solver_initialized_ = false;
    int A_rows_cached_ = 0;
    int A_cols_cached_ = 0;
    rose_map::RoseMap::Ptr rose_map_;
};

} // namespace rose_planner
