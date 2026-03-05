#pragma once
#include "../parameters.hpp"
#include "../trajectory_optimize/trajectory.hpp"
//clang-format off
#include "OsqpEigen/OsqpEigen.h"
//clang-format on
#include "angles.h"
#include "common.hpp"
#include "rose_map/rose_map.hpp"
#include "type.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
namespace rose_planner {
namespace control {

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
            blind_radius_ = params.mpc_params.blind_radius;
            max_speed_ = params.mpc_params.max_speed;
            max_accel_ = params.mpc_params.max_accel;
            Q_ = params.mpc_params.Q;
            R_ = params.mpc_params.R;
            Rd_ = params.mpc_params.Rd;
            max_cv_ = max_accel_ * dt_;
            delay_time_ = params.mpc_params.delay_time;
            xref_ = Eigen::MatrixXd::Zero(4, 500);
            dref_ = Eigen::MatrixXd::Zero(2, 500);
            output_ = Eigen::MatrixXd::Zero(2, 500);
            last_output_ = output_;
            last_final_output_ = output_;
            xbar_.resize(T_ + 1);
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

        void setCurrent(const State& c) {
            now_state_w_ = c;
        }
        Eigen::Vector2d transformPointToBody(const Eigen::Vector2d& pos_w) {
            Eigen::Vector2d dp = pos_w - now_state_w_.pos;

            double yaw = now_state_w_.yaw;
            Eigen::Matrix2d R;

            R << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);

            return R.transpose() * dp;
        }
        Eigen::Vector2d tansformVecToBody(const Eigen::Vector2d& vec_w) {
            Eigen::Vector2d dp = vec_w;

            double yaw = now_state_w_.yaw;
            Eigen::Matrix2d R;

            R << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);

            return R.transpose() * dp;
        }

        Output getOutput() {
            Eigen::Vector2d cmd(output_(0, 0), output_(1, 0));
            Output o;
            o.vel = cmd;
            o.pred_states = xbar_;
            return o;
        }

        void solve() {
            P_.clear();
            if (!getRefPoints(T_, dt_)) {
                output_ = Eigen::MatrixXd::Zero(2, 500);
                xbar_.clear();
                xbar_.resize(T_ + 1);
                std::cout << "getRefPoints failed" << std::endl;
                return;
            }

            for (int i = 0; i < P_.size(); i++) {
                xref_(0, i) = P_[i].pos.x();
                xref_(1, i) = P_[i].pos.y();
                xref_(2, i) = P_[i].vel.x();
                xref_(3, i) = P_[i].vel.y();
                dref_(0, i) = P_[i].vel.x();
                dref_(1, i) = P_[i].vel.y();
            }

            getCmd();
        }

        bool getRefPoints(const int T_in, double dt_in) {
            P_.clear();
            P_.reserve(T_in);

            double t_cur = traj_.getTimeByPos(now_state_w_.pos, 0.5);
            if (t_cur < 0.0)
                t_cur = 0.0;
            double t = t_cur + dt_in + delay_time_;

            Eigen::Vector2d pos_end = Eigen::Vector2d::Zero();
            Eigen::Vector2d vel_end = Eigen::Vector2d::Zero();
            Eigen::Vector2d acc_end = Eigen::Vector2d::Zero();
            Eigen::Vector2d now_pos = now_state_w_.pos;
            bool has_ref = false;
            double yaw_end = 0.0;
            double w_end = 0.0;
            Eigen::Vector2d prev_pos = traj_.getPos(t);
            for (int i = 0; i < T_in; ++i) {
                TrajPoint tp;

                if (t <= traj_duration_) {
                    Eigen::Vector2d curr_pos = traj_.getPos(t);
                    Eigen::Vector2d vec = curr_pos - now_pos;

                    if (vec.norm() < blind_radius_) {
                        prev_pos = curr_pos;
                        t += dt_in;
                        i--;
                        continue;
                    }
                    has_ref = true;
                    tp.pos = traj_.getPos(t);
                    tp.vel = traj_.getVel(t);
                    tp.acc = traj_.getAcc(t);
                    double yaw = traj_.getYaw(t);
                    yaw = yaw_end + angles::shortest_angular_distance(yaw_end, yaw);
                    tp.yaw = yaw;
                    tp.w = traj_.getYawDot(t);

                    pos_end = tp.pos;
                    vel_end = tp.vel;
                    acc_end = tp.acc;
                    yaw_end = tp.yaw;
                    w_end = tp.w;
                } else {
                    tp.pos = pos_end;
                    tp.vel = Eigen::Vector2d::Zero();
                    tp.acc = Eigen::Vector2d::Zero();
                    tp.yaw = yaw_end;
                    tp.w = w_end;
                }

                P_.push_back(tp);
                t += dt_in;
            }
            return has_ref;
        }
        double getEsdf(const Eigen::Vector2d& pos) {
            return rose_map_->esdf_->getEsdf(pos.cast<float>());
        }

        void stateTransOmni(State& s, double vx_cmd, double vy_cmd) {
            s.pos.x() += vx_cmd * dt_;
            s.pos.y() += vy_cmd * dt_;
            s.vel.x() = vx_cmd;
            s.vel.y() = vy_cmd;
        }

        void predictMotion() {
            xbar_[0] = now_state_w_;
            State temp = now_state_w_;

            for (int i = 1; i <= T_; i++) {
                stateTransOmni(temp, output_(0, i - 1), output_(1, i - 1));
                xbar_[i] = temp;
            }
        }

        void solveMPCV() {
            const int N = T_;
            if (N <= 1)
                return;

            const int dimx = 4 * N;
            const int dimu = 2 * N;
            const int nx = dimx + dimu;

            qp_gradient_.setZero(nx);
            std::vector<Eigen::Triplet<double>> H_trip;
            H_trip.reserve(nx * 6);

            for (int i = 0; i < N; ++i) {
                int xi = 4 * i;

                H_trip.emplace_back(xi + 0, xi + 0, 2.0 * Q_[0]);
                H_trip.emplace_back(xi + 1, xi + 1, 2.0 * Q_[1]);
                H_trip.emplace_back(xi + 2, xi + 2, 2.0 * Q_[2]);
                H_trip.emplace_back(xi + 3, xi + 3, 2.0 * Q_[3]);

                qp_gradient_[xi + 0] = -2.0 * Q_[0] * xref_(0, i);
                qp_gradient_[xi + 1] = -2.0 * Q_[1] * xref_(1, i);
                qp_gradient_[xi + 2] = -2.0 * Q_[2] * xref_(2, i);
                qp_gradient_[xi + 3] = -2.0 * Q_[3] * xref_(3, i);
            }

            for (int i = 0; i < N; ++i) {
                int ui = dimx + 2 * i;

                double w0 = R_[0];
                double w1 = R_[1];

                if (i == 0 || i == N - 1) {
                    w0 += Rd_[0];
                    w1 += Rd_[1];
                } else {
                    w0 += 2.0 * Rd_[0];
                    w1 += 2.0 * Rd_[1];
                }

                H_trip.emplace_back(ui, ui, 2.0 * w0);
                H_trip.emplace_back(ui + 1, ui + 1, 2.0 * w1);

                qp_gradient_[ui] = -2.0 * R_[0] * dref_(0, i);
                qp_gradient_[ui + 1] = -2.0 * R_[1] * dref_(1, i);
            }

            for (int i = 1; i < N; ++i) {
                int ui = dimx + 2 * i;
                int up = dimx + 2 * (i - 1);

                H_trip.emplace_back(ui, up, -2.0 * Rd_[0]);
                H_trip.emplace_back(up, ui, -2.0 * Rd_[0]);

                H_trip.emplace_back(ui + 1, up + 1, -2.0 * Rd_[1]);
                H_trip.emplace_back(up + 1, ui + 1, -2.0 * Rd_[1]);
            }

            qp_hessian_.resize(nx, nx);
            qp_hessian_.setFromTriplets(H_trip.begin(), H_trip.end());
            qp_hessian_.makeCompressed();

            const int dyn_rows = 4 * N;
            const int nc = dyn_rows;

            std::vector<Eigen::Triplet<double>> A_trip;
            A_trip.reserve(N * 20);

            Eigen::Vector4d x0;
            x0 << now_state_w_.pos.x(), now_state_w_.pos.y(), now_state_w_.vel.x(),
                now_state_w_.vel.y();

            for (int i = 0; i < 4; ++i)
                A_trip.emplace_back(i, i, 1.0);

            qp_lowerBound_.resize(nc);
            qp_upperBound_.resize(nc);

            qp_lowerBound_.segment<4>(0) = x0;
            qp_upperBound_.segment<4>(0) = x0;

            for (int k = 0; k < N - 1; ++k) {
                int row = 4 * (k + 1);
                int xk = 4 * k;
                int xk1 = 4 * (k + 1);
                int uk = dimx + 2 * k;

                // px
                A_trip.emplace_back(row + 0, xk1 + 0, 1.0);
                A_trip.emplace_back(row + 0, xk + 0, -1.0);
                A_trip.emplace_back(row + 0, xk + 2, -dt_);

                // py
                A_trip.emplace_back(row + 1, xk1 + 1, 1.0);
                A_trip.emplace_back(row + 1, xk + 1, -1.0);
                A_trip.emplace_back(row + 1, xk + 3, -dt_);

                // vx
                A_trip.emplace_back(row + 2, xk1 + 2, 1.0);
                A_trip.emplace_back(row + 2, uk + 0, -1.0);

                // vy
                A_trip.emplace_back(row + 3, xk1 + 3, 1.0);
                A_trip.emplace_back(row + 3, uk + 1, -1.0);

                qp_lowerBound_.segment<4>(row).setZero();
                qp_upperBound_.segment<4>(row).setZero();
            }

            Eigen::SparseMatrix<double> A(nc, nx);
            A.setFromTriplets(A_trip.begin(), A_trip.end());
            A.makeCompressed();

            if (!solver_initialized_) {
                solver_.settings()->setWarmStart(true);
                solver_.settings()->setVerbosity(false);

                solver_.data()->setNumberOfVariables(nx);
                solver_.data()->setNumberOfConstraints(nc);
                solver_.data()->setHessianMatrix(qp_hessian_);
                solver_.data()->setLinearConstraintsMatrix(A);
                solver_.data()->setGradient(qp_gradient_);
                solver_.data()->setLowerBound(qp_lowerBound_);
                solver_.data()->setUpperBound(qp_upperBound_);

                if (!solver_.initSolver()) {
                    std::cerr << "OSQP init failed\n";
                    return;
                }

                solver_initialized_ = true;
            } else {
                solver_.updateHessianMatrix(qp_hessian_);
                solver_.updateGradient(qp_gradient_);
                solver_.updateBounds(qp_lowerBound_, qp_upperBound_);
                solver_.updateLinearConstraintsMatrix(A);
            }

            if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
                return;

            Eigen::VectorXd sol = solver_.getSolution();

            for (int i = 0; i < N; ++i) {
                int ui = dimx + 2 * i;
                output_(0, i) = sol[ui];
                output_(1, i) = sol[ui + 1];
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

                predictMotion();
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
        }

        int fps_;
        double dt_;
        int T_;
        int max_iter_;
        double delay_time_;
        double max_speed_;
        double max_cv_;
        double max_accel_;
        double blind_radius_;
        double traj_duration_;
        Parameters params_;
        std::vector<double> Q_, R_, Rd_;
        TrajType traj_;
        State now_state_w_;
        std::vector<State> xbar_;
        Eigen::MatrixXd A_, B_;
        Eigen::VectorXd C_;
        Eigen::MatrixXd xref_;
        Eigen::MatrixXd dref_;
        Eigen::MatrixXd output_;
        Eigen::MatrixXd last_output_;
        Eigen::MatrixXd last_final_output_;
        Eigen::VectorXd last_solution_;
        bool has_last_solution_ = false;

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
} // namespace control
} // namespace rose_planner