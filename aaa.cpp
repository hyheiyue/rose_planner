#pragma once
#include "../parameters.hpp"
#include "../trajectory_optimize/trajectory.hpp"
#include "angles.h"
#include <OsqpEigen/OsqpEigen.h>
#include <osqp/osqp.h>

namespace rose_planner {
class MPCState {
public:
    double x = 0;
    double y = 0;
    double v = 0;
    double theta = 0;
    double w = 0;
};

class TrajPoint {
public:
    double x = 0;
    double y = 0;
    double v = 0;
    double a = 0;
    double theta = 0;
    double w = 0;
};

class OsqpMpc {
public:
    using Ptr = std::shared_ptr<OsqpMpc>;
    OsqpMpc(Parameters params) {
        dt = params.mpc_params.dt;
        max_iter = params.mpc_params.max_iter;
        T = params.mpc_params.predict_steps;
        max_omega = params.mpc_params.max_omega;
        max_domega = params.mpc_params.max_domega;
        max_speed = params.mpc_params.max_speed;
        min_speed = params.mpc_params.min_speed;
        max_accel = params.mpc_params.max_accel;
        delay_num = params.mpc_params.delay_num;
        Q = params.mpc_params.Q;
        R = params.mpc_params.R;
        Rd = params.mpc_params.Rd;
        max_comega = max_domega * dt;
        max_cv = max_accel * dt;
        xref = Eigen::Matrix<double, 4, 500>::Zero(4, 500);
        last_output = output = dref = Eigen::Matrix<double, 2, 500>::Zero(2, 500);
        for (int i = 0; i < delay_num; i++)
            output_buff.push_back(Eigen::Vector2d::Zero());
    }
    static Ptr create(Parameters params) {
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
        Eigen::Vector2d _output = Eigen::Vector2d::Zero();

        if (delay_num < 0 || delay_num >= output.cols()) {
            std::cerr << "[MPC] ⚠ delay_num out of range: " << delay_num
                      << " (cols=" << output.cols() << ")" << std::endl;
            return _output;
        }

        _output.x() = output(0, delay_num);
        _output.y() = output(1, delay_num);
        return _output;
    }

    void solve() {
        auto start = std::chrono::system_clock::now();
        getRefPoints(T, dt);
        for (int i = 0; i < T; i++) {
            xref(0, i) = P[i].x;
            xref(1, i) = P[i].y;
            xref(3, i) = P[i].theta;
            dref(0, i) = P[i].v;
            dref(1, i) = P[i].w;
        }
        smooth_yaw();
        getCmd();
        auto end = std::chrono::system_clock::now();
        std::cout << "solve time: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << "ms" << std::endl;
    }

    void getRefPoints(const int T, double dt) {
        auto start = std::chrono::system_clock::now();
        TrajPoint tp;
        int j = 0;
        double t_cur = traj_.getTimeByPos({ now_state.x, now_state.y }, 0.5);
        auto end = std::chrono::system_clock::now();
        if (t_cur < 0) {
            t_cur = 0;
        }
        for (double temp_t = t_cur + dt; j < T; j++, temp_t += dt) {
            Eigen::Vector2d curP = traj_.getPos(temp_t);
            Eigen::Vector2d curV = traj_.getVel(temp_t);
            Eigen::Vector2d curA = traj_.getAcc(temp_t);
            double yaw = traj_.getYaw(temp_t);
            tp.v = curV.norm();
            tp.x = curP.x();
            tp.y = curP.y();
            tp.a = curA.norm();
            tp.theta = yaw;
            normlize_theta(tp.theta);
            tp.w = traj_.getYawDot(temp_t);
            P.push_back(tp);
        }
    }
    void smooth_yaw(void) {
        double dyaw = xref(3, 0) - now_state.theta;

        while (dyaw >= M_PI / 2) {
            xref(3, 0) -= M_PI * 2;
            dyaw = xref(3, 0) - now_state.theta;
        }
        while (dyaw <= -M_PI / 2) {
            xref(3, 0) += M_PI * 2;
            dyaw = xref(3, 0) - now_state.theta;
        }

        for (int i = 0; i < T - 1; i++) {
            dyaw = xref(3, i + 1) - xref(3, i);
            while (dyaw >= M_PI / 2) {
                xref(3, i + 1) -= M_PI * 2;
                dyaw = xref(3, i + 1) - xref(3, i);
            }
            while (dyaw <= -M_PI / 2) {
                xref(3, i + 1) += M_PI * 2;
                dyaw = xref(3, i + 1) - xref(3, i);
            }
        }
    }
    void predictMotion(MPCState* b) {
        b[0] = xbar[0];

        Eigen::MatrixXd Ax;
        Eigen::MatrixXd Bx;
        Eigen::MatrixXd Cx;
        Eigen::MatrixXd xnext;
        MPCState temp = xbar[0];

        for (int i = 1; i < T + 1; i++) {
            Bx = Eigen::Matrix<double, 3, 2>::Zero();
            Bx(0, 0) = cos(xbar[i - 1].theta) * dt;
            Bx(1, 0) = sin(xbar[i - 1].theta) * dt;
            Bx(2, 1) = dt;

            Ax = Eigen::Matrix3d::Identity();
            Ax(0, 2) = -Bx(1, 0) * xbar[i - 1].v;
            Ax(1, 2) = Bx(0, 0) * xbar[i - 1].v;

            Cx = Eigen::Vector3d::Zero();
            Cx(0) = -Ax(0, 2) * xbar[i - 1].theta;
            Cx(1) = -Ax(1, 2) * xbar[i - 1].theta;
            xnext = Ax * Eigen::Vector3d(temp.x, temp.y, temp.theta)
                + Bx * Eigen::Vector2d(output(0, i - 1), output(1, i - 1)) + Cx;
            temp.x = xnext(0);
            temp.y = xnext(1);
            temp.theta = xnext(2);
            b[i] = temp;
        }
    }
    void stateTrans(MPCState& s, double a, double yaw_dot) {
        if (yaw_dot >= max_omega) {
            yaw_dot = max_omega;
        } else if (yaw_dot <= -max_omega) {
            yaw_dot = -max_omega;
        }
        if (s.v >= max_speed) {
            s.v = max_speed;
        } else if (s.v <= min_speed) {
            s.v = min_speed;
        }

        s.x = s.x + a * cos(s.theta) * dt;
        s.y = s.y + a * sin(s.theta) * dt;
        s.theta = s.theta + yaw_dot * dt;
        s.v = a;
    }
    void predictMotion(void) {
        xbar[0] = now_state;

        MPCState temp = now_state;
        for (int i = 1; i < T + 1; i++) {
            stateTrans(temp, output(0, i - 1), output(1, i - 1));
            xbar[i] = temp;
        }
    }
    void solveMPCV(void) {
        const int dimx = 3 * (T - delay_num);
        const int dimu = 2 * (T - delay_num);
        const int nx = dimx + dimu;

        Eigen::SparseMatrix<double> hessian;
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
        Eigen::SparseMatrix<double> linearMatrix;
        Eigen::VectorXd lowerBound;
        Eigen::VectorXd upperBound;

        // first-order
        for (int i = 0, j = delay_num, k = 0; i < dimx; i += 3, j++, k += 2) {
            gradient[i] = -2 * Q[0] * xref(0, j);
            gradient[i + 1] = -2 * Q[1] * xref(1, j);
            gradient[i + 2] = -2 * Q[3] * xref(3, j);
            gradient[dimx + k] = -2 * Q[2] * dref(0, j);
        }

        // second-order
        const int nnzQ = nx + dimu - 2;
        int irowQ[nnzQ];
        int jcolQ[nnzQ];
        double dQ[nnzQ];
        for (int i = 0; i < nx; i++) {
            irowQ[i] = jcolQ[i] = i;
        }
        for (int i = nx; i < nnzQ; i++) {
            irowQ[i] = i - dimu + 2;
            jcolQ[i] = i - dimu;
        }
        for (int i = 0; i < dimx; i += 3) {
            dQ[i] = Q[0] * 2.0;
            dQ[i + 1] = Q[1] * 2.0;
            dQ[i + 2] = Q[3] * 2.0;
        }
        dQ[dimx] = dQ[nx - 2] = (R[0] + Rd[0] + Q[2]) * 2.0;
        dQ[dimx + 1] = dQ[nx - 1] = (R[1] + Rd[1]) * 2.0;
        for (int i = dimx + 2; i < nx - 2; i += 2) {
            dQ[i] = 2 * (R[0] + 2 * Rd[0] + Q[2]);
            dQ[i + 1] = 2 * (R[1] + 2 * Rd[1]);
        }
        for (int i = nx; i < nnzQ; i += 2) {
            dQ[i] = -Rd[0] * 2.0;
            dQ[i + 1] = -Rd[1] * 2.0;
        }
        hessian.resize(nx, nx);
        Eigen::MatrixXd QQ(nx, nx);
        for (int i = 0; i < nx; i++) {
            hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
        }
        for (int i = nx; i < nnzQ; i++) {
            hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
            hessian.insert(jcolQ[i], irowQ[i]) = dQ[i];
        }

        // equality constraints
        MPCState temp = xbar[delay_num];
        getLinearModel(temp);
        int my = dimx;
        double b[my];
        const int nnzA = 11 * (T - delay_num) - 5;
        int irowA[nnzA];
        int jcolA[nnzA];
        double dA[nnzA];
        Eigen::Vector3d temp_vec(temp.x, temp.y, temp.theta);
        Eigen::Vector3d temp_b = A * temp_vec + C;

        for (int i = 0; i < dimx; i++) {
            irowA[i] = jcolA[i] = i;
            dA[i] = 1;
        }
        b[0] = temp_b[0];
        b[1] = temp_b[1];
        b[2] = temp_b[2];
        irowA[dimx] = 0;
        jcolA[dimx] = dimx;
        dA[dimx] = -B(0, 0);
        irowA[dimx + 1] = 1;
        jcolA[dimx + 1] = dimx;
        dA[dimx + 1] = -B(1, 0);
        irowA[dimx + 2] = 2;
        jcolA[dimx + 2] = dimx + 1;
        dA[dimx + 2] = -B(2, 1);
        int ABidx = 8 * (T - delay_num) - 8;
        int ABbegin = dimx + 3;
        for (int i = 0, j = 1; i < ABidx; i += 8, j++) {
            getLinearModel(xbar[j + delay_num]);
            for (int k = 0; k < 3; k++) {
                b[3 * j + k] = C[k];
                irowA[ABbegin + i + k] = 3 * j + k;
                jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 3;
                dA[ABbegin + i + k] = -A(k, k);
            }
            irowA[ABbegin + i + 3] = 3 * j;
            jcolA[ABbegin + i + 3] = 3 * j - 1;
            dA[ABbegin + i + 3] = -A(0, 2);

            irowA[ABbegin + i + 4] = 3 * j + 1;
            jcolA[ABbegin + i + 4] = 3 * j - 1;
            dA[ABbegin + i + 4] = -A(1, 2);

            irowA[ABbegin + i + 5] = 3 * j;
            jcolA[ABbegin + i + 5] = dimx + 2 * j;
            dA[ABbegin + i + 5] = -B(0, 0);

            irowA[ABbegin + i + 6] = 3 * j + 1;
            jcolA[ABbegin + i + 6] = dimx + 2 * j;
            dA[ABbegin + i + 6] = -B(1, 0);

            irowA[ABbegin + i + 7] = 3 * j + 2;
            jcolA[ABbegin + i + 7] = dimx + 2 * j + 1;
            dA[ABbegin + i + 7] = -B(2, 1);
        }

        // iequality constraints
        const int mz = 2 * (T - delay_num) - 2;
        const int nnzC = 2 * dimu - 4;
        int irowC[nnzC];
        int jcolC[nnzC];
        double dC[nnzC];
        for (int i = 0, k = 0; i < mz; i += 2, k += 4) {
            irowC[k] = i;
            jcolC[k] = dimx + i;
            dC[k] = -1.0;

            irowC[k + 1] = i;
            jcolC[k + 1] = jcolC[k] + 2;
            dC[k + 1] = 1.0;

            irowC[k + 2] = i + 1;
            jcolC[k + 2] = dimx + 1 + i;
            dC[k + 2] = -1.0;

            irowC[k + 3] = i + 1;
            jcolC[k + 3] = jcolC[k + 2] + 2;
            dC[k + 3] = 1.0;
        }

        // xlimits and all
        int mx = dimu;
        int nc = mx + my + mz;
        lowerBound.resize(nc);
        upperBound.resize(nc);
        linearMatrix.resize(nc, nx);
        for (int i = 0; i < mx; i += 2) {
            lowerBound[i] = -max_speed;
            lowerBound[i + 1] = -max_omega;
            upperBound[i] = max_speed;
            upperBound[i + 1] = max_omega;
            linearMatrix.insert(i, dimx + i) = 1;
            linearMatrix.insert(i + 1, dimx + i + 1) = 1;
        }

        for (int i = 0; i < nnzA; i++) {
            linearMatrix.insert(irowA[i] + mx, jcolA[i]) = dA[i];
        }

        for (int i = 0; i < my; i++) {
            lowerBound[mx + i] = upperBound[mx + i] = b[i];
        }

        for (int i = 0; i < nnzC; i++) {
            linearMatrix.insert(irowC[i] + mx + my, jcolC[i]) = dC[i];
        }

        for (int i = 0; i < mz; i += 2) {
            lowerBound[mx + my + i] = -max_cv;
            upperBound[mx + my + i] = max_cv;
            lowerBound[mx + my + i + 1] = -max_comega;
            upperBound[mx + my + i + 1] = max_comega;
        }
        // instantiate the solver
        OsqpEigen::Solver solver;

        // settings
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.settings()->setAbsoluteTolerance(1e-6);
        solver.settings()->setMaxIteration(30000);
        solver.settings()->setRelativeTolerance(1e-6);
        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(nx);
        solver.data()->setNumberOfConstraints(nc);
        if (!solver.data()->setHessianMatrix(hessian))
            return;
        if (!solver.data()->setGradient(gradient))
            return;
        if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
            return;
        if (!solver.data()->setLowerBound(lowerBound))
            return;
        if (!solver.data()->setUpperBound(upperBound))
            return;

        // instantiate the solver
        if (!solver.initSolver())
            return;

        // controller input and QPSolution vector
        Eigen::VectorXd QPSolution;

        // solve the QP problem
        if (!solver.solve())
            return;

        // get the controller input
        QPSolution = solver.getSolution();
        // ROS_INFO("Solution: v0=%f     omega0=%f", QPSolution[dimx], QPSolution[dimx+1]);
        for (int i = 0; i < delay_num; i++) {
            output.col(i) = output_buff[i];
            // output(0, i) = output_buff[i][0];
            // output(1, i) = output_buff[i][1];
        }
        for (int i = 0, j = 0; i < dimu; i += 2, j++) {
            output(0, j + delay_num) = QPSolution[dimx + i];
            output(1, j + delay_num) = QPSolution[dimx + i + 1];
        }
    }

    void getCmd(void) {
        int iter;
        for (iter = 0; iter < max_iter; iter++) {
            predictMotion();
            last_output = output;
            solveMPCV();
            double du = 0;
            for (int i = 0; i < output.cols(); i++) {
                du = du + fabs(output(0, i) - last_output(0, i))
                    + fabs(output(1, i) - last_output(1, i));
            }
        }
        predictMotion(xopt);
        if (delay_num > 0) {
            output_buff.erase(output_buff.begin());
            output_buff.push_back(Eigen::Vector2d(output(0, delay_num), output(1, delay_num)));
        }
    }
    void getLinearModel(const MPCState& s) {
        B = Eigen::Matrix<double, 3, 2>::Zero();
        B(0, 0) = cos(s.theta) * dt;
        B(1, 0) = sin(s.theta) * dt;
        B(2, 1) = dt;

        A = Eigen::Matrix3d::Identity();
        A(0, 2) = -B(1, 0) * s.v;
        A(1, 2) = B(0, 0) * s.v;

        C = Eigen::Vector3d::Zero();
        C(0) = -A(0, 2) * s.theta;
        C(1) = -A(1, 2) * s.theta;
    }
    void normlize_theta(double& th) {
        while (th > M_PI)
            th -= 2 * M_PI;
        while (th < -M_PI)
            th += 2 * M_PI;
    }
    Trajectory<3, 2> traj_;
    MPCState now_state;

    int max_iter;
    int delay_num;
    double traj_duration;
    // Time step of MPC prediction
    double dt;
    // Number of time steps of MPC prediction
    int T;
    std::vector<double> Q;
    std::vector<double> R;
    std::vector<double> Rd;
    double max_omega;
    double max_domega;
    double max_comega;
    // Maximum speed, minimum speed, maximum change in speed per step, acceleration
    double max_speed;
    double min_speed;
    double max_cv;
    double max_accel;
    // MPC linear state transition function matrix
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd C;

    // MPC state matrix
    MPCState xbar[500];
    // Reference state
    Eigen::MatrixXd xref;
    // Reference input
    Eigen::MatrixXd dref;
    // Output buffer
    Eigen::MatrixXd output;
    // Store the last output as the initial value for MPC
    Eigen::MatrixXd last_output;
    // MPC calculation results
    std::vector<Eigen::Vector2d> output_buff;
    std::vector<TrajPoint> P;
    MPCState xopt[500];
};
} // namespace rose_planner
#pragma once
#include "../parameters.hpp"
#include "../trajectory_optimize/trajectory.hpp"
#include "angles.h"
#include <OsqpEigen/OsqpEigen.h>
#include <osqp/osqp.h>

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
    OsqpMpc(Parameters params) {
        dt = params.mpc_params.dt;
        max_iter = params.mpc_params.max_iter;
        T = params.mpc_params.predict_steps;
        max_omega = params.mpc_params.max_omega;
        max_domega = params.mpc_params.max_domega;
        max_speed = params.mpc_params.max_speed;
        min_speed = params.mpc_params.min_speed;
        max_accel = params.mpc_params.max_accel;
        delay_num = params.mpc_params.delay_num;
        Q = params.mpc_params.Q;
        R = params.mpc_params.R;
        Rd = params.mpc_params.Rd;
        max_comega = max_domega * dt;
        max_cv = max_accel * dt;
        xref = Eigen::Matrix<double, 4, 500>::Zero(4, 500);
        last_output = output = dref = Eigen::Matrix<double, 2, 500>::Zero(2, 500);
        for (int i = 0; i < delay_num; i++)
            output_buff.push_back(Eigen::Vector2d::Zero());
    }
    static Ptr create(Parameters params) {
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
            std::cerr << "[MPC] ⚠ delay_num out of range: " << delay_num
                      << " (cols=" << output.cols() << ")" << std::endl;
            return cmd;
        }

        // 这里取出的现在是全向底盘的速度指令 vx, vy（世界坐标系）
        cmd[0] = output(0, delay_num); // vx
        cmd[1] = output(1, delay_num); // vy

        return cmd;
    }

    void solve() {
        auto start = std::chrono::system_clock::now();

        getRefPoints(T, dt);

        for (int i = 0; i < T; i++) {
            // 位置参考
            xref(0, i) = P[i].x;
            xref(1, i) = P[i].y;

            // 速度参考（全向 → 直接取 vx, vy）
            xref(2, i) = P[i].vx;
            xref(3, i) = P[i].vy;

            // 控制输入参考（MPC QP 里会用）
            dref(0, i) = P[i].vx;
            dref(1, i) = P[i].vy;
        }

        getCmd(); // 迭代求解 MPC 速度指令

        auto end = std::chrono::system_clock::now();
        std::cout << "solve time: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << "ms" << std::endl;
    }

    void getRefPoints(const int T, double dt) {
        auto start = std::chrono::system_clock::now();
        TrajPoint tp;
        int j = 0;
        double t_cur = traj_.getTimeByPos({ now_state.x, now_state.y }, 0.5);
        auto end = std::chrono::system_clock::now();
        if (t_cur < 0) {
            t_cur = 0;
        }
        for (double temp_t = t_cur + dt; j < T; j++, temp_t += dt) {
            Eigen::Vector2d curP = traj_.getPos(temp_t);
            Eigen::Vector2d curV = traj_.getVel(temp_t);
            Eigen::Vector2d curA = traj_.getAcc(temp_t);
            double yaw = traj_.getYaw(temp_t);
            tp.vx = curV.x();
            tp.vy = curV.y();
            tp.x = curP.x();
            tp.y = curP.y();
            tp.ax = curA.x();
            tp.ay = curA.y();
            P.push_back(tp);
        }
    }

    void stateTransOmni(MPCState& s, double vx_cmd, double vy_cmd) {
        // 速度直接积分
        s.x = s.x + vx_cmd * dt;
        s.y = s.y + vy_cmd * dt;
        s.vx = vx_cmd;
        s.vy = vy_cmd;
    }

    void predictMotion() {
        xbar[0] = now_state;
        MPCState temp = now_state;
        for (int i = 1; i < T + 1; i++) {
            double vx_in = output(0, i - 1);
            double vy_in = output(1, i - 1);
            stateTransOmni(temp, vx_in, vy_in);
            xbar[i] = temp;
        }
    }

    void predictMotion(MPCState* b) {
        b[0] = xbar[0];
        MPCState temp = xbar[0];

        for (int i = 1; i < T + 1; i++) {
            double vx_in = output(0, i - 1);
            double vy_in = output(1, i - 1);
            stateTransOmni(temp, vx_in, vy_in);
            b[i] = temp;
        }
    }

    void solveMPCV(void) {
        const int dimx = 4 * (T - delay_num); // ⭐ 状态4维
        const int dimu = 2 * (T - delay_num); // 控制2维
        const int nx = dimx + dimu;

        Eigen::SparseMatrix<double> hessian;
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
        Eigen::SparseMatrix<double> linearMatrix;
        Eigen::VectorXd lowerBound, upperBound;

        // -------- 代价梯度（tracking x, y, vx, vy） --------
        for (int i = 0, j = delay_num, k = 0; i < dimx; i += 4, j++, k += 2) {
            gradient[i] = -2 * Q[0] * xref(0, j); // x
            gradient[i + 1] = -2 * Q[1] * xref(1, j); // y
            gradient[i + 2] = -2 * Q[2] * xref(2, j); // vx_ref
            gradient[i + 3] = -2 * Q[3] * xref(3, j); // vy_ref
        }

        // -------- Hessian 对角权重 --------
        hessian.resize(nx, nx);
        for (int i = 0; i < dimx; i += 4) {
            hessian.insert(i, i) = 2 * Q[0];
            hessian.insert(i + 1, i + 1) = 2 * Q[1];
            hessian.insert(i + 2, i + 2) = 2 * Q[2];
            hessian.insert(i + 3, i + 3) = 2 * Q[3];
        }
        for (int i = dimx; i < nx; i++) {
            hessian.insert(i, i) = 2 * R[0]; // 控制输入正则
        }

        // -------- 等式约束 (x,y,vx,vy 预测一致性) --------
        int my = dimx;
        std::vector<double> b_eq(my, 0);
        int nnzA = 5 * (T - delay_num); // ⭐ 每步5个约束非零项

        std::vector<int> irowA(nnzA), jcolA(nnzA);
        std::vector<double> dA(nnzA);

        // 初始状态约束
        for (int i = 0; i < dimx; i++) {
            irowA[i] = jcolA[i] = i;
            dA[i] = 1;
        }
        b_eq[0] = xbar[delay_num].x;
        b_eq[1] = xbar[delay_num].y;
        b_eq[2] = xbar[delay_num].vx;
        b_eq[3] = xbar[delay_num].vy;

        // 之后每步动力学约束
        int idx = dimx;
        for (int step = 0; step < T - delay_num; step++) {
            int row = 4 * (step + 1);
            int u_col = dimx + 2 * step;

            // x_next = x + vx*dt
            irowA[idx] = row;
            jcolA[idx] = row - 4;
            dA[idx++] = -1;
            linearMatrix.insert(row, u_col) = dt;
            b_eq[row] = 0;

            // y_next = y + vy*dt
            irowA[idx] = row + 1;
            jcolA[idx] = row - 3;
            dA[idx++] = -1;
            linearMatrix.insert(row + 1, u_col + 1) = dt;

            // vx = vx_cmd
            irowA[idx] = row + 2;
            jcolA[idx] = row - 2;
            dA[idx++] = -1;
            linearMatrix.insert(row + 2, u_col) = 1;

            // vy = vy_cmd
            irowA[idx] = row + 3;
            jcolA[idx] = row - 1;
            dA[idx++] = -1;
            linearMatrix.insert(row + 3, u_col + 1) = 1;
        }

        // 写入线性矩阵和上下界
        int nc = dimu + my;
        lowerBound.resize(nc);
        upperBound.resize(nc);
        linearMatrix.resize(nc, nx);

        for (int i = 0; i < my; i++) {
            lowerBound[dimu + i] = upperBound[dimu + i] = b_eq[i];
            for (int k = 0; k < nnzA; k++) {
                linearMatrix.insert(irowA[k] + dimu, jcolA[k]) = dA[k];
            }
        }

        // -------- vx, vy 速度约束 --------
        for (int i = 0; i < dimu; i += 2) {
            lowerBound[i] = -max_speed;
            lowerBound[i + 1] = -max_speed;
            upperBound[i] = max_speed;
            upperBound[i + 1] = max_speed;
            linearMatrix.insert(i, dimx + i) = 1;
            linearMatrix.insert(i + 1, dimx + i + 1) = 1;
        }

        // -------- 求解 QP --------
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

        // 还原delay缓存并写入output
        for (int i = 0; i < delay_num; i++)
            output.col(i) = output_buff[i];

        for (int i = 0, j = 0; i < dimu; i += 2, j++) {
            output(0, j + delay_num) = sol[dimx + i];
            output(1, j + delay_num) = sol[dimx + i + 1];
        }
    }

    void getCmd(void) {
        int iter;
        for (iter = 0; iter < max_iter; iter++) {
            predictMotion();
            last_output = output;
            solveMPCV();
            double du = 0;
            for (int i = 0; i < output.cols(); i++) {
                du = du + fabs(output(0, i) - last_output(0, i))
                    + fabs(output(1, i) - last_output(1, i));
            }
        }
        predictMotion(xopt);
        if (delay_num > 0) {
            output_buff.erase(output_buff.begin());
            output_buff.push_back(Eigen::Vector2d(output(0, delay_num), output(1, delay_num)));
        }
    }
    void getLinearModel(const MPCState& s) {
        // 全向模型不需要cos/sin方向耦合
        A = Eigen::Matrix4d::Identity();

        // 控制输入直接作用于速度
        B = Eigen::Matrix<double, 4, 2>::Zero();
        B(0, 0) = dt; // x += vx_cmd*dt
        B(1, 1) = dt; // y += vy_cmd*dt
        B(2, 0) = 1; // vx = vx_cmd
        B(3, 1) = 1; // vy = vy_cmd

        // 这个模型是纯线性的，无偏置项
        C = Eigen::Vector4d::Zero();
    }

    void normlize_theta(double& th) {
        while (th > M_PI)
            th -= 2 * M_PI;
        while (th < -M_PI)
            th += 2 * M_PI;
    }
    Trajectory<3, 2> traj_;
    MPCState now_state;

    int max_iter;
    int delay_num;
    double traj_duration;
    // Time step of MPC prediction
    double dt;
    // Number of time steps of MPC prediction
    int T;
    std::vector<double> Q;
    std::vector<double> R;
    std::vector<double> Rd;
    double max_omega;
    double max_domega;
    double max_comega;
    // Maximum speed, minimum speed, maximum change in speed per step, acceleration
    double max_speed;
    double min_speed;
    double max_cv;
    double max_accel;
    // MPC linear state transition function matrix
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd C;

    // MPC state matrix
    MPCState xbar[500];
    // Reference state
    Eigen::MatrixXd xref;
    // Reference input
    Eigen::MatrixXd dref;
    // Output buffer
    Eigen::MatrixXd output;
    // Store the last output as the initial value for MPC
    Eigen::MatrixXd last_output;
    // MPC calculation results
    std::vector<Eigen::Vector2d> output_buff;
    std::vector<TrajPoint> P;
    MPCState xopt[500];
};
} // namespace rose_planner
