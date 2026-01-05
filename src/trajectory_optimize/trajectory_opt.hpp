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
        init_v = (ctx_.path[1] - ctx_.path[0]).normalized()*init_v.norm();
        headState << ctx_.head_pos, init_v, Eigen::Vector2d::Zero();
        Eigen::Matrix<double, 2, 3> tailState;
        tailState << ctx_.tail_pos, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero();
        minco_.setConditions(
            headState,
            tailState,
            ctx_.pieceNum,
            Eigen::Vector2d(w_smooth, w_smooth)
        );
        minco_.getTrajectory(finalTraj_);
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
            ctx_.inTimes(i) = ctx_.sample_dt + 1e-3 * (i % 2);
        }

        Eigen::VectorXd virtualT(pieceNum);
        RealT2VirtualT(ctx_.inTimes, virtualT);
        const int timeOffset = 2 * ctrlNum;
        x.segment(timeOffset, pieceNum) = virtualT;
        double minCost = 0.0;
        lbfgs_params_.mem_size = 256;
        lbfgs_params_.past = 20;
        lbfgs_params_.min_step = 1e-16;
        lbfgs_params_.g_epsilon = 2.0e-5;
        lbfgs_params_.delta = 2e-5;
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
            //std::cout << "[Smooth Optimize] OK: " << lbfgs::lbfgs_strerror(ret) << std::endl;
        } else {
            std::cout << "[Smooth Optimize] FAIL: " << lbfgs::lbfgs_strerror(ret) << std::endl;
        }
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
        ctx_.pathInPs.resize(2, ctrlNum);
        ctx_.pathInPs.row(0) = inPs.row(0);
        ctx_.pathInPs.row(1) = inPs.row(1);
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
        // gradByTimes += energyT_grad;
        cost_val += energy;
        gradp += energy_grad;
        gradp += gradByPoints;
        for (int i = 0; i < points_num; ++i) {
            double nearest_cost = 0.0;
            Eigen::Vector2d x_cur = inPs.col(i);
            Eigen::Vector2d obs_grad = instance->obstacleTerm(x_cur, nearest_cost);
            cost_val += nearest_cost;
            gradp.col(i) += obs_grad;
        }
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

        double safe_margin = params_.robot_radius + 0.1;
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
        Eigen::Matrix2Xd pathInPs;
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
// static inline void attachPenaltyFunctional(
//     const Eigen::VectorXd& T,
//     const Eigen::MatrixX2d&
//         coeffs, // coeffs 仍是6×3分块，但只用前2列(x,y)，第3列yaw不再参与平坦映射
//     const Eigen::VectorXi& hIdx,
//     const PolyhedraH& hPolys,
//     const double& smoothFactor,
//     const int& integralResolution,
//     const Eigen::VectorXd& magnitudeBounds,
//     const Eigen::VectorXd& penaltyWeights,
//     flatness::FlatnessMap2D&
//         flatMap, // 你之后可以换成2D版本，但这里直接当成数学工具用，不再输出3D姿态
//     double& cost,
//     Eigen::VectorXd& gradT,
//     Eigen::MatrixX2d& gradC
// ) {
//     const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
//     const double omgSqrMax =
//         magnitudeBounds(1) * magnitudeBounds(1); // 这里 omg 变成 yaw rate，依然可以用它做上界
//     const double thrustMin = magnitudeBounds(2);
//     const double thrustMax = magnitudeBounds(3);

//     const double weightPos = penaltyWeights(0);
//     const double weightVel = penaltyWeights(1);
//     const double weightOmg = penaltyWeights(2);
//     const double weightThrust = penaltyWeights(3);

//     Eigen::Vector2d pos, vel, acc;
//     Eigen::Vector2d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
//     double totalGradPsi, totalGradPsiD;
//     double thr;
//     double yaw_rate; // 2D 角速度只保留 yaw
//     double gradThr;
//     double gradYawRate;
//     Eigen::Vector2d gradPos, gradVel;
//     double step, alpha;
//     double s1, s2, s3, s4, s5;
//     Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
//     Eigen::Vector2d outerNormal2D;
//     int K, L;
//     double violaPos, violaVel, violaYawRate, violaThrust;
//     double violaPosPenaD, violaVelPenaD, violaYawRatePenaD, violaThrustPenaD;
//     double violaPosPena, violaVelPena, violaYawRatePena, violaThrustPena;
//     double node, pena;

//     const int pieceNum = T.size();
//     const double integralFrac = 1.0 / integralResolution;

//     for (int i = 0; i < pieceNum; i++) {
//         const Eigen::Matrix<double, 6, 3>& c = coeffs.block<6, 3>(i * 6, 0);
//         step = T(i) * integralFrac;

//         for (int j = 0; j <= integralResolution; j++) {
//             s1 = j * step;
//             s2 = s1 * s1;
//             s3 = s2 * s1;
//             s4 = s2 * s2;
//             s5 = s4 * s1;

//             beta0 << 1.0, s1, s2, s3, s4, s5;
//             beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
//             beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
//             beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;

//             // 只计算 2D pos/vel/acc
//             pos = c.block<2, 3>(0, 0).transpose() * beta0; // 取 (x,y) 多项式系数
//             vel = c.block<2, 3>(0, 0).transpose() * beta1;
//             acc = c.block<2, 3>(0, 0).transpose() * beta2;
//             double jer2D_x = (c.block<2, 3>(0, 0).transpose() * beta3)(0);
//             double jer2D_y = (c.block<2, 3>(0, 0).transpose() * beta3)(1);
//             thr = massClamp(acc.norm(), thrustMin, thrustMax);
//             yaw_rate = dpsi;

//             violaVel = vel.squaredNorm() - velSqrMax;
//             violaYawRate = yaw_rate * yaw_rate - omgSqrMax;
//             violaThrust =
//                 (thr < thrustMin ? thrustMin - thr : (thr > thrustMax ? thr - thrustMax : 0.0));

//             gradThr = 0.0;
//             gradYawRate = 0.0;
//             gradPos.setZero(), gradVel.setZero();
//             pena = 0.0;

//             L = hIdx(i);
//             K = hPolys[L].rows();
//             for (int k = 0; k < K; k++) {
//                 outerNormal2D = hPolys[L].block<1, 2>(k, 0);
//                 violaPos = outerNormal2D.dot(pos) + hPolys[L](k, 2);
//                 if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
//                     gradPos += weightPos * violaPosPenaD * outerNormal2D;
//                     pena += weightPos * violaPosPena;
//                 }
//             }

//             if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
//                 gradVel += weightVel * violaVelPenaD * 2.0 * vel;
//                 pena += weightVel * violaVelPena;
//             }

//             if (smoothedL1(violaYawRate, smoothFactor, violaYawRatePena, violaYawRatePenaD)) {
//                 gradYawRate += weightOmg * violaYawRatePenaD * 2.0 * yaw_rate;
//                 pena += weightOmg * violaYawRatePena;
//             }

//             if (smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD)) {
//                 gradThr += weightThrust * violaThrustPenaD * 2.0 * (thr - thrustMean);
//                 pena += weightThrust * violaThrustPena;
//             }

//             // 2D backward 传播
//             flatMap.backward(
//                 gradPos,
//                 gradVel,
//                 gradThr,
//                 psi,
//                 gradYawRate,
//                 totalGradPos,
//                 totalGradVel,
//                 totalGradAcc,
//                 totalGradJer,
//                 totalGradPsi,
//                 totalGradPsiD
//             );

//             node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
//             alpha = j * integralFrac;

//             gradC.block<6, 2>(i * 6, 0) +=
//                 (beta0 * totalGradPos.transpose() + beta1 * totalGradVel.transpose()
//                  + beta2 * totalGradAcc.transpose() + beta3 * totalGradJer.transpose())
//                 * node * step;

//             gradT(i) += (totalGradPos.dot(vel) + totalGradVel.dot(acc)
//                          + totalGradAcc.dot(Eigen::Vector2d(jer2D_x, jer2D_y)))
//                     * alpha * node * step
//                 + node * integralFrac * pena;

//             cost += node * step * pena;
//         }
//     }

//     return;
// }