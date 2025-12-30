#pragma once
#include "../parameters.hpp"
#include "../trajectory_optimize/trajectory.hpp"
#include "angles.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
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
        dt = params.mpc_params.dt;
        max_iter = params.mpc_params.max_iter;
        T = params.mpc_params.predict_steps;
        max_speed = params.mpc_params.max_speed;
        min_speed = params.mpc_params.min_speed;
        max_accel = params.mpc_params.max_accel;
        delay_num = params.mpc_params.delay_num;
        Q = params.mpc_params.Q;
        R = params.mpc_params.R;
        Rd = params.mpc_params.Rd;
        max_cv = max_accel * dt;

        xref = Eigen::MatrixXd::Zero(4, 500);
        dref = Eigen::MatrixXd::Zero(2, 500);
        output = Eigen::MatrixXd::Zero(2, 500);
        last_output = output;

        for (int i = 0; i < delay_num; i++)
            output_buff.emplace_back(Eigen::Vector2d::Zero());
    }

    static Ptr create(const Parameters& params) {
        return std::make_shared<OsqpMpc>(params);
    }

    void setTrajectory(const Trajectory<3, 2>& traj) {
        traj_ = traj;
        traj_duration = traj.getTotalDuration();
    }

    void setCurrent(const MPCState& c) {
        now_state = c;
    }

    Eigen::Vector2d getOutput() {
        Eigen::Vector2d cmd = Eigen::Vector2d::Zero();

        if (delay_num < 0 || delay_num >= output.cols()) {
            std::cerr << "[MPC] Warning: delay_num out of range\n";
            return cmd;
        }

        cmd[0] = output(0, delay_num);
        cmd[1] = output(1, delay_num);
        return cmd;
    }

    void solve() {
        auto t0 = std::chrono::system_clock::now();

        P.clear();
        getRefPoints(T, dt);

        for (int i = 0; i < T; i++) {
            xref(0, i) = P[i].x;
            xref(1, i) = P[i].y;
            xref(2, i) = P[i].vx;
            xref(3, i) = P[i].vy;

            dref(0, i) = P[i].vx;
            dref(1, i) = P[i].vy;
        }

        getCmd();

        auto t1 = std::chrono::system_clock::now();
        // std::cout << "[MPC] solve time: "
        //           << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
        //           << "ms\n";
    }

    void getRefPoints(const int T_in, double dt_in) {
        P.clear();
        getRefPointsInternal(T_in, dt_in);
    }

private:
    void getRefPointsInternal(const int T_in, double dt_in) {
        P.clear();

        double t_cur = traj_.getTimeByPos({ now_state.x, now_state.y }, 0.5);
        if (t_cur < 0)
            t_cur = 0;

        Eigen::Vector2d pos_end = traj_.getPos(traj_duration - 0.01);

        TrajPoint tp;
        int j = 0;

        for (double t = t_cur + dt_in; j < T_in; j++, t += dt_in) {
            if (t <= traj_duration) {
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

            P.push_back(tp);
        }
    }

    void stateTransOmni(MPCState& s, double vx_cmd, double vy_cmd) {
        s.x += vx_cmd * dt;
        s.y += vy_cmd * dt;
        s.vx = vx_cmd;
        s.vy = vy_cmd;
    }

    void predictMotionInternal() {
        xbar[0] = now_state;
        MPCState temp = now_state;

        for (int i = 1; i <= T; i++) {
            stateTransOmni(temp, output(0, i - 1), output(1, i - 1));
            xbar[i] = temp;
        }
    }
    void getLinearModel(const MPCState&) {
        // 状态4维 [x, y, vx, vy]
        // 控制2维 [vx_cmd, vy_cmd]
        A = Eigen::MatrixXd::Identity(4, 4);
        B = Eigen::MatrixXd::Zero(4, 2);
        C = Eigen::VectorXd::Zero(4);

        // 控制直接影响位置和速度
        B(0, 0) = dt; // x += vx_cmd * dt
        B(1, 1) = dt; // y += vy_cmd * dt
        B(2, 0) = 1; // vx = vx_cmd
        B(3, 1) = 1; // vy = vy_cmd
    }
    void predictMotion() {
        predictMotionInternal();
    }

    void predictMotion(MPCState* b) {
        for (int i = 0; i <= T; i++)
            b[i] = xbar[i];
    }

public:
    void solveMPCV() {
        const int steps = T - delay_num;
        if (steps <= 0) {
            std::cerr << "[MPC] Not enough steps to solve.\n";
            return;
        }

        const int dimx = 4 * steps;
        const int dimu = 2 * steps;
        const int nx = dimx + dimu;

        // 1. 代价梯度
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
        for (int i = 0; i < steps; i++) {
            int xi = 4 * i;
            gradient[xi] = -2 * Q[0] * xref(0, i + delay_num);
            gradient[xi + 1] = -2 * Q[1] * xref(1, i + delay_num);
            gradient[xi + 2] = -2 * Q[2] * xref(2, i + delay_num);
            gradient[xi + 3] = -2 * Q[3] * xref(3, i + delay_num);
        }
        for (int i = 0; i < steps; i++) {
            int ui = dimx + 2 * i;
            gradient[ui] = -2 * Rd[0] * dref(0, i);
            gradient[ui + 1] = -2 * Rd[1] * dref(1, i);
        }

        // 2. Hessian（对角矩阵）
        Eigen::SparseMatrix<double> hessian(nx, nx);
        for (int i = 0; i < dimx; i++) {
            hessian.insert(i, i) = 2 * Q[i % 4];
        }
        for (int i = 0; i < steps; i++) {
            int ui = dimx + 2 * i;
            hessian.insert(ui, ui) = 2 * R[0]; // vx_cmd 权重
            hessian.insert(ui + 1, ui + 1) = 2 * R[1]; // vy_cmd 权重
        }

        // 3. 约束矩阵 + 上下界（重新计算约束行数）
        const int dyn_rows = 4 * steps;
        const int speed_rows = 2 * steps;
        const int delta_rows = 2 * (steps - 1);
        const int accel_rows = 2 * (steps - 1);
        const int nc = dyn_rows + speed_rows + delta_rows + accel_rows;

        Eigen::SparseMatrix<double> linearMatrix(nc, nx);
        Eigen::VectorXd lowerBound = Eigen::VectorXd::Zero(nc);
        Eigen::VectorXd upperBound = Eigen::VectorXd::Zero(nc);

        // 4. 初始状态约束
        for (int i = 0; i < 4; i++) {
            linearMatrix.insert(i, i) = 1;
            lowerBound[i] = upperBound[i] = (&now_state.x)[i];
        }

        // 5. 动力学一致性约束
        for (int i = 1; i < steps; i++) {
            int row = 4 * i;
            int last = 4 * (i - 1);
            int ucol = dimx + 2 * (i - 1);

            linearMatrix.insert(row, row) = 1;
            linearMatrix.insert(row, last) = -1;
            linearMatrix.insert(row, ucol) = -dt;

            linearMatrix.insert(row + 1, row + 1) = 1;
            linearMatrix.insert(row + 1, last + 1) = -1;
            linearMatrix.insert(row + 1, ucol + 1) = -dt;

            linearMatrix.insert(row + 2, row + 2) = 1;
            linearMatrix.insert(row + 2, ucol) = -1;

            linearMatrix.insert(row + 3, row + 3) = 1;
            linearMatrix.insert(row + 3, ucol + 1) = -1;

            lowerBound[row] = upperBound[row] = 0;
            lowerBound[row + 1] = upperBound[row + 1] = 0;
            lowerBound[row + 2] = upperBound[row + 2] = 0;
            lowerBound[row + 3] = upperBound[row + 3] = 0;
        }

        // 6. 速度限制约束（√）
        int offset = dyn_rows;
        for (int i = 0; i < steps; i++) {
            int row = offset + 2 * i;
            int col = dimx + 2 * i;
            linearMatrix.insert(row, col) = 1;
            linearMatrix.insert(row + 1, col + 1) = 1;
            lowerBound[row] = lowerBound[row + 1] = -max_speed;
            upperBound[row] = upperBound[row + 1] = max_speed;
        }

        // 7. 控制增量约束 |u(k)-u(k-1)| ≤ max_cv（√）
        offset += speed_rows;
        for (int i = 1; i < steps; i++) {
            int row = offset + 2 * (i - 1);
            int u_now = dimx + 2 * i;
            int u_last = dimx + 2 * (i - 1);

            linearMatrix.insert(row, u_now) = 1;
            linearMatrix.insert(row, u_last) = -1;
            linearMatrix.insert(row + 1, u_now + 1) = 1;
            linearMatrix.insert(row + 1, u_last + 1) = -1;

            lowerBound[row] = lowerBound[row + 1] = -max_cv;
            upperBound[row] = upperBound[row + 1] = max_cv;
        }

        // 8. 加速度约束 |Δv/dt| ≤ max_accel（√）
        offset += delta_rows;
        for (int i = 1; i < steps; i++) {
            int row = offset + 2 * (i - 1);
            int u_now = dimx + 2 * i;
            int u_last = dimx + 2 * (i - 1);

            linearMatrix.insert(row, u_now) = 1.0 / dt;
            linearMatrix.insert(row, u_last) = -1.0 / dt;
            linearMatrix.insert(row + 1, u_now + 1) = 1.0 / dt;
            linearMatrix.insert(row + 1, u_last + 1) = -1.0 / dt;

            lowerBound[row] = lowerBound[row + 1] = -max_accel;
            upperBound[row] = upperBound[row + 1] = max_accel;
        }

        // 9. OSQP 求解（不改）
        hessian.makeCompressed();
        linearMatrix.makeCompressed();

        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);

        solver.data()->setNumberOfVariables(nx);
        solver.data()->setNumberOfConstraints(nc);
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setGradient(gradient);
        solver.data()->setLinearConstraintsMatrix(linearMatrix);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);

        solver.initSolver();
        solver.solve();

        Eigen::VectorXd sol = solver.getSolution();

        // 10. 写回控制输出（保持你原逻辑）
        for (int i = 0; i < delay_num && i < output_buff.size(); i++) {
            output.col(i) = output_buff[i];
        }
        for (int i = 0; i < steps; i++) {
            int ui = dimx + 2 * i;
            output(0, i + delay_num) = sol[ui];
            output(1, i + delay_num) = sol[ui + 1];
        }
    }

    void getCmd() {
        for (int i = 0; i < max_iter; i++) {
            predictMotionInternal();
            last_output = output;
            solveMPCV();

            double du = 0;
            for (int c = 0; c < output.cols(); c++) {
                du += std::fabs(output(0, c) - last_output(0, c));
                du += std::fabs(output(1, c) - last_output(1, c));
            }
            if (du < 1e-4)
                break;
        }
        predictMotion(xopt);
        if (delay_num > 0) {
            if (!output_buff.empty())
                output_buff.erase(output_buff.begin());
            output_buff.push_back({ output(0, delay_num), output(1, delay_num) });
        }
    }

    double dt;
    int T;
    int max_iter;
    int delay_num;
    double max_speed;
    double min_speed;
    double max_cv;
    double max_accel;
    double traj_duration;

    std::vector<double> Q, R, Rd;
    Trajectory<3, 2> traj_;
    MPCState now_state;
    MPCState xbar[501];
    MPCState xopt[501];
    Eigen::MatrixXd A, B;
    Eigen::VectorXd C;
    Eigen::MatrixXd xref;
    Eigen::MatrixXd dref;
    Eigen::MatrixXd output;
    Eigen::MatrixXd last_output;
    std::vector<Eigen::Vector2d> output_buff;
    std::vector<TrajPoint> P;
};

} // namespace rose_planner
