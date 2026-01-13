#pragma once
#include "../common.hpp"
#include "../parameters.hpp"
#include "cubic_spline.hpp"
#include "flatness.hpp"
#include "lbfgs.hpp"
#include "minco.hpp"
#include "rose_map/rose_map.hpp"

#include <algorithm>
#include <cfloat>
#include <memory>

namespace rose_planner {

class TrajectoryOpt {
public:
    using Ptr = std::shared_ptr<TrajectoryOpt>;
    TrajectoryOpt(rose_map::RoseMap::Ptr rose_map, Parameters params):
        rose_map_(rose_map),
        params_(params) {
        double base_scale = params_.opt_params.base_scale;
        w_obs = params_.opt_params.obstacle_weight * base_scale;
        w_smooth = params_.opt_params.smooth_weight * base_scale;
        w_time = params_.opt_params.time_weight * base_scale;
    }
    static Ptr create(rose_map::RoseMap::Ptr rose_map, Parameters params) {
        return std::make_shared<TrajectoryOpt>(rose_map, params);
    }

    void setSampledPath(
        const std::vector<SampleTrajectoryPoint>& sampled,
        double sample_dt,
        Eigen::Vector2d init_v = Eigen::Vector2d(0, 0)
    ) {
        if (sampled.size() < 5) {
            ctx_.skip = true;
            return;
        }
        ctx_.path.clear();
        for (const auto& pt: sampled) {
            ctx_.path.push_back(pt.p.cast<double>());
        }
        ctx_.head_pos = ctx_.path[0];
        ctx_.tail_pos = ctx_.path[ctx_.path.size() - 1];
        ctx_.pieceNum = ctx_.path.size() - 1;
        ctx_.sample_dt = sample_dt;
        ctx_.init_obs = true;
        Eigen::Matrix<double, 2, 3> headState;
        init_v = sampled[1].v.cast<double>();
        // init_v = init_v.normalized() * std::max(init_v.norm(), 1.0);
        headState << ctx_.head_pos, init_v, Eigen::Vector2d::Zero();
        Eigen::Matrix<double, 2, 3> tailState;
        tailState << ctx_.tail_pos, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero();
        minco_.setConditions(
            headState,
            tailState,
            ctx_.pieceNum,
            Eigen::Vector2d(w_smooth, w_smooth)
        );
    }
    template<typename EIGENVEC>
    inline void RealT2VirtualT(const Eigen::VectorXd& RT, EIGENVEC& VT) {
        const int sizeT = RT.size();
        VT.resize(sizeT);
        for (int i = 0; i < sizeT; ++i) {
            VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0) : (1.0 - sqrt(2.0 / RT(i) - 1.0));
        }
    }

    template<typename EIGENVEC>
    inline void VirtualT2RealT(const EIGENVEC& VT, Eigen::VectorXd& RT) {
        const int sizeTau = VT.size();
        RT.resize(sizeTau);
        for (int i = 0; i < sizeTau; ++i) {
            RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                                : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
        }
    }
    template<typename EIGENVEC>
    inline void static backwardGradT(
        const Eigen::VectorXd& tau,
        const Eigen::VectorXd& gradT,
        EIGENVEC& gradTau
    ) {
        const int sizetau = tau.size();
        gradTau.resize(sizetau);
        double gradrt2vt;
        for (int i = 0; i < sizetau; i++) {
            if (tau(i) > 0) {
                gradrt2vt = tau(i) + 1.0;
            } else {
                double denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                gradrt2vt = (1.0 - tau(i)) / (denSqrt * denSqrt);
            }
            gradTau(i) = gradT(i) * gradrt2vt;
        }
        return;
    }
    void optimize() {
        if (ctx_.skip) {
            ctx_.skip = false;
            ctx_.path.clear();
            return;
        }

        const int pieceNum = ctx_.pieceNum;
        const int ctrlNum = pieceNum - 1;

        Eigen::VectorXd x(3 * pieceNum - 1);

        for (int i = 0; i < ctrlNum; ++i) {
            x(i) = ctx_.path[i + 1].x();
            x(i + ctrlNum) = ctx_.path[i + 1].y();
        }

        ctx_.inTimes.resize(pieceNum);
        for (int i = 0; i < pieceNum; ++i) {
            ctx_.inTimes(i) = ctx_.sample_dt;
        }

        Eigen::VectorXd virtualT(pieceNum);
        RealT2VirtualT(ctx_.inTimes, virtualT);
        const int timeOffset = 2 * ctrlNum;
        x.segment(timeOffset, pieceNum) = virtualT;
        double minCost = 0.0;
        lbfgs_params_.mem_size = 256;
        lbfgs_params_.past = 20;
        lbfgs_params_.min_step = 1e-32;
        lbfgs_params_.g_epsilon = 2.0e-7;
        lbfgs_params_.delta = 2e-7;
        lbfgs_params_.max_iterations = 4000;
        lbfgs_params_.max_linesearch = 32;
        lbfgs_params_.f_dec_coeff = 1e-4;
        lbfgs_params_.s_curv_coeff = 0.9;

        int ret = lbfgs::lbfgs_optimize(
            x,
            minCost,
            &TrajectoryOpt::cost,
            nullptr,
            nullptr,
            this,
            lbfgs_params_
        );

        if (ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH) {
            // std::cout << "[Smooth Optimize] OK: " << lbfgs::lbfgs_strerror(ret) << std::endl;
            for (int i = 0; i < ctrlNum; ++i) {
                ctx_.path[i + 1].x() = x(i);
                ctx_.path[i + 1].y() = x(i + ctrlNum);
            }
            Eigen::Matrix2Xd inPs(2, ctrlNum);
            inPs.row(0) = x.head(ctrlNum).transpose();
            inPs.row(1) = x.segment(ctrlNum, ctrlNum).transpose();

            virtualT = x.segment(timeOffset, pieceNum);

            VirtualT2RealT(virtualT, ctx_.inTimes);

            minco_.setParameters(inPs, ctx_.inTimes);
            minco_.getTrajectory(finalTraj_);
        } else {
            std::cout << "[Smooth Optimize] FAIL: " << lbfgs::lbfgs_strerror(ret) << std::endl;
        }
    }

    static double cost(void* ptr, const Eigen::VectorXd& x, Eigen::VectorXd& g) {
        if (!ptr || !x.allFinite() || x.norm() > 1e3) {
            if (g.size() > 0)
                g.setZero();
            return 1e6;
        }

        auto* instance = static_cast<TrajectoryOpt*>(ptr);
        if (!instance) {
            if (g.size() > 0)
                g.setZero();
            return 1e6;
        }

        const int pieceNum = instance->ctx_.pieceNum;
        const int points_num = pieceNum - 1;

        double cost_val = 0.0;
        int idx = 0;
        Eigen::Matrix2Xd inPs(2, points_num);
        inPs.row(0) = x.segment(idx, points_num).transpose();
        idx += points_num;
        inPs.row(1) = x.segment(idx, points_num).transpose();
        idx += points_num;
        Eigen::VectorXd t = x.segment(idx, pieceNum);
        idx += pieceNum;

        Eigen::Matrix2Xd gradp = Eigen::Matrix2Xd::Zero(2, points_num);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(pieceNum);

        instance->VirtualT2RealT(t, instance->ctx_.inTimes);

        instance->minco_.setParameters(inPs, instance->ctx_.inTimes);

        double energy = 0.0;
        Eigen::Matrix2Xd energy_grad = Eigen::Matrix2Xd::Zero(2, points_num);
        Eigen::VectorXd energyT_grad = Eigen::VectorXd::Zero(pieceNum);

        Eigen::Matrix2Xd gradByPoints;
        Eigen::VectorXd gradByTimes;
        Eigen::MatrixX2d partialGradByCoeffs;
        Eigen::VectorXd partialGradByTimes;
        instance->minco_.getEnergyPartialGradByCoeffs(partialGradByCoeffs);
        instance->minco_.getEnergyPartialGradByTimes(partialGradByTimes);
        instance->minco_.getEnergy(energy);
        instance->minco_
            .propogateGrad(partialGradByCoeffs, partialGradByTimes, energy_grad, energyT_grad);

        if (!std::isfinite(energy))
            energy = 0.0;

        Eigen::Vector2d gradByTailStateS;
        double gradByTailState;
        instance->minco_.propogateArcYawLenghGrad(
            partialGradByCoeffs,
            partialGradByTimes,
            gradByPoints,
            gradByTimes,
            gradByTailStateS
        );
        gradByTimes += energyT_grad;
        cost_val += energy;
        gradp += energy_grad;
        gradp += gradByPoints;
        cost_val += instance->attachPenaltyFunctional(inPs, gradp);
        cost_val += instance->w_time * instance->ctx_.inTimes.sum();
        Eigen::VectorXd rhotimes;
        rhotimes.resize(gradByTimes.size());
        gradByTimes += instance->w_time * rhotimes.setOnes();
        instance->backwardGradT(t, gradByTimes, gradt);
        g.setZero();
        g.segment(0, points_num) = gradp.row(0).transpose();
        g.segment(points_num, points_num) = gradp.row(1).transpose();
        g.segment(2 * points_num, pieceNum) = gradt;

        return cost_val;
    }
    static inline bool smoothedL1(const double& x, const double& mu, double& f, double& df) {
        if (x < 0.0) {
            return false;
        } else if (x > mu) {
            f = x - 0.5 * mu;
            df = 1.0;
            return true;
        } else {
            const double xdmu = x / mu;
            const double sqrxdmu = xdmu * xdmu;
            const double mumxd2 = mu - 0.5 * x;
            f = mumxd2 * sqrxdmu * xdmu;
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
            return true;
        }
    }
    static inline double kahanSum(double& sum, double& c, const double& val) {
        double y = val - c;
        double t = sum + y;
        c = (t - sum) - y;
        sum = t;
        return sum;
    }
    double attachPenaltyFunctional(const Eigen::Matrix2Xd& inPs, Eigen::Matrix2Xd& gradp) {
        const int N = inPs.cols();
        if (N < 2)
            return 0.0;

        double cost_val = 0.0;
        double c_cost = 0.0;

        for (int i = 0; i < N - 1; i++) {
            double nearest_cost = 0.0;
            const Eigen::Vector2d& p0 = inPs.col(i);
            const Eigen::Vector2d& p1 = inPs.col(i + 1);

            // 1. Obstacle cost
            Eigen::Vector2d obs_grad = obstacleTerm(p0, nearest_cost);
            kahanSum(cost_val, c_cost, nearest_cost);
            gradp.col(i).noalias() += obs_grad;
        }

        return cost_val;
    }

    Eigen::Vector2d obstacleTerm(const Eigen::Vector2d& xcur, double& nearest_cost) {
        nearest_cost = 0.0;
        Eigen::Vector2d grad = Eigen::Vector2d::Zero();

        const double R = 1.0;
        const double mu = 0.4;

        double d;
        Eigen::Vector2d g;
        if (!sampleEsdfAndGrad(xcur, d, g) || !std::isfinite(d)) {
            return grad;
        }

        if (d > R) {
            return grad;
        }

        double penetration = R - d;
        double cost_s1 = 0.0, dcost_s1 = 0.0;
        if (!smoothedL1(penetration, mu, cost_s1, dcost_s1)) {
            return grad;
        }

        double safe_margin = params_.robot_radius * 2;
        double w_safe = 1.0;
        if (d < safe_margin) {
            w_safe += 4.0 * (safe_margin - d) / safe_margin;
        }

        nearest_cost = w_obs * cost_s1 * w_safe;
        Eigen::Vector2d dir = (g.norm() > 1e-6) ? g.normalized() : Eigen::Vector2d::Zero();
        grad = w_obs * dcost_s1 * w_safe * (-dir);

        if (!grad.allFinite()) {
            grad.setZero();
        }

        return grad;
    }

    bool sampleEsdfAndGrad(const Eigen::Vector2d& pos, double& out_dist, Eigen::Vector2d& out_grad)
        const {
        if (!rose_map_)
            return false;
        const double vs = rose_map_->acc_map_info_.voxel_size_;
        if (vs <= 0.0)
            return false;
        const double R = 1.0;
        Eigen::Vector2f pos2f(pos.x(), pos.y());
        auto key = rose_map_->worldToKey2D(pos2f);
        if (rose_map_->key2DToIndex2D(key) < 0)
            return false;

        // 采样时截断距离避免突变
        auto read = [&](int dx, int dy) {
            rose_map::VoxelKey2D k { key.x + dx, key.y + dy };
            int idx = rose_map_->key2DToIndex2D(k);
            if (idx < 0)
                return R + 1.0;
            double d = rose_map_->esdf_[idx];
            return std::min(std::isfinite(d) ? d : R + 1.0, R + 1.0);
        };

        double x0 = read(-1, 0), x1 = read(0, 0), x2 = read(1, 0);
        double x3 = read(-1, 1), x4 = read(0, 1), x5 = read(1, 1);
        double y0 = read(0, -1), y1 = read(0, 0), y2 = read(0, 1);
        double y3 = read(1, -1), y4 = read(1, 0), y5 = read(1, 1);

        auto fitQuad = [&](double p0, double p1, double p2, double f) {
            double a = 0.5 * (p2 + p0 - 2 * p1);
            double b = 0.5 * (p2 - p0);
            double c = p1;
            double v = a * f * f + b * f + c;
            double gq = 2 * a * f + b;
            return std::pair<double, double> { v, gq };
        };

        double fx = (pos2f.x() - (key.x + 0.5) * vs) / vs;
        double fy = (pos2f.y() - (key.y + 0.5) * vs) / vs;
        fx = std::clamp(fx, -0.5, 0.5);
        fy = std::clamp(fy, -0.5, 0.5);

        auto r0 = fitQuad(x0, x1, x2, fx);
        auto r1 = fitQuad(x3, x4, x5, fx);
        double valx = r0.first * (1.0 - (fy + 0.5)) + r1.first * (fy + 0.5);
        double gradx = r0.second * (1.0 - (fy + 0.5)) + r1.second * (fy + 0.5);

        auto s0 = fitQuad(y0, y1, y2, fy);
        auto s1 = fitQuad(y3, y4, y5, fy);
        double valy = s0.first * (1.0 - (fx + 0.5)) + s1.first * (fx + 0.5);
        double grady = s0.second * (1.0 - (fx + 0.5)) + s1.second * (fx + 0.5);

        out_dist = 0.5 * (valx + valy);
        out_grad = Eigen::Vector2d(gradx, grady);

        return out_dist < R;
    }

    TrajType getTrajectory() {
        return finalTraj_;
    }
    std::vector<Eigen::Vector2d> getPath() {
        return ctx_.path;
    }

    struct Ctx {
        std::vector<Eigen::Vector2d> path;
        Eigen::Vector2d head_pos;
        Eigen::Vector2d tail_pos;
        Eigen::VectorXd inTimes;
        int pieceNum = 0;
        double sample_dt = 0.0;
        bool init_obs = true;
        bool skip = false;
    } ctx_;

    rose_map::RoseMap::Ptr rose_map_;
    Parameters params_;
    minco::MINCO_S3NU minco_;
    TrajType finalTraj_;
    lbfgs::lbfgs_parameter_t lbfgs_params_;
    double w_obs = 1.0;
    double w_smooth = 1.0;
    double w_time = 1.0;
};

} // namespace rose_planner
