#pragma once
#include "../parameters.hpp"
#include "../trajectory_optimize/trajectory.hpp"
//clang-format off
#include "OsqpEigen/OsqpEigen.h"
//clang-format on
#include "angles.h"
#include "common.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
namespace rose_planner {

class MPCState {
public:
    double x = 0;
    double y = 0;
    double vx = 0;
    double vy = 0;
};

class TrajPoint {
public:
    double x = 0;
    double y = 0;
    double vx = 0;
    double vy = 0;
    double ax = 0;
    double ay = 0;
};

class OsqpMpc {
public:
    using Ptr = std::shared_ptr<OsqpMpc>;

    OsqpMpc(const Parameters& params) {
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

        for (int i = 0; i < delay_num_; i++)
            output_buff_.emplace_back(Eigen::Vector2d::Zero());
    }

    static Ptr create(const Parameters& params) {
        return std::make_shared<OsqpMpc>(params);
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
        auto t0 = std::chrono::system_clock::now();

        P_.clear();
        getRefPoints(T_, dt_);

        for (int i = 0; i < T_; i++) {
            xref_(0, i) = P_[i].x;
            xref_(1, i) = P_[i].y;
            xref_(2, i) = P_[i].vx;
            xref_(3, i) = P_[i].vy;

            dref_(0, i) = P_[i].vx;
            dref_(1, i) = P_[i].vy;
        }

        getCmd();

        auto t1 = std::chrono::system_clock::now();
        // std::cout << "[MPC] solve time: "
        //           << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
        //           << "ms\n";
    }

    void getRefPoints(const int T_in, double dt_in) {
        P_.clear();
        getRefPointsInternal(T_in, dt_in);
    }

private:
    void getRefPointsInternal(const int T_in, double dt_in) {
        P_.clear();

        double t_cur = traj_.getTimeByPos({ now_state_.x, now_state_.y }, 0.5);
        if (t_cur < 0)
            t_cur = 0;

        Eigen::Vector2d pos_end = traj_.getPos(traj_duration_ - 0.01);

        TrajPoint tp;
        int j = 0;

        for (double t = t_cur + dt_in; j < T_in; j++, t += dt_in) {
            if (t <= traj_duration_) {
                Eigen::Vector2d pos = traj_.getPos(t);
                Eigen::Vector2d vel = traj_.getVel(t);
                Eigen::Vector2d acc = traj_.getAcc(t);
                tp.x = pos.x();
                tp.y = pos.y();
                tp.vx = vel.x();
                tp.vy = vel.y();
                tp.ax = acc.x();
                tp.ay = acc.y();
            } else {
                tp.x = pos_end.x();
                tp.y = pos_end.y();
                tp.vx = 0;
                tp.vy = 0;
                tp.ax = 0;
                tp.ay = 0;
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

    void predictMotion(MPCState* b) {
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
        for (int i = 0; i < 4; i++) {
            double v = (&now_state_.x)[i];
            qp_lowerBound_[i] = v;
            qp_upperBound_[i] = v;
        }

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

        // 更新求解
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

        // 结果写回
        for (int i = 0; i < steps; i++) {
            int ui = dimx + 2 * i;
            output_(0, i + delay_num_) = sol[ui];
            output_(1, i + delay_num_) = sol[ui + 1];
        }
    }

    void getCmd() {
        for (int i = 0; i < max_iter_; i++) {
            predictMotionInternal();
            last_output_ = output_;
            solveMPCV();

            double du = 0;
            for (int c = 0; c < output_.cols(); c++) {
                du += std::fabs(output_(0, c) - last_output_(0, c));
                du += std::fabs(output_(1, c) - last_output_(1, c));
            }
            if (du < 1e-4)
                break;
        }
        predictMotion(xopt_);
        if (delay_num_ > 0) {
            if (!output_buff_.empty())
                output_buff_.erase(output_buff_.begin());
            output_buff_.push_back({ output_(0, delay_num_), output_(1, delay_num_) });
        }
    }

    double dt_;
    int T_;
    int max_iter_;
    int delay_num_;
    double max_speed_;
    double min_speed_;
    double max_cv_;
    double max_accel_;
    double traj_duration_;

    std::vector<double> Q_, R_, Rd_;
    TrajType traj_;
    MPCState now_state_;
    MPCState xbar_[501];
    MPCState xopt_[501];
    Eigen::MatrixXd A_, B_;
    Eigen::VectorXd C_;
    Eigen::MatrixXd xref_;
    Eigen::MatrixXd dref_;
    Eigen::MatrixXd output_;
    Eigen::MatrixXd last_output_;
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
};

} // namespace rose_planner
