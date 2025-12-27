#pragma once
#include "common.hpp"
#include "cubic_spline.hpp"
#include "lbfgs.hpp"
#include "parameters.hpp"
#include "rose_map/rose_map.hpp"

#include <algorithm>
#include <cfloat>
#include <limits>
#include <memory>

namespace rose_planner {

class TrajectoryOpt {
public:
    using Ptr = std::shared_ptr<TrajectoryOpt>;
    TrajectoryOpt(rose_map::RoseMap::Ptr rose_map, Parameters params):
        rose_map_(rose_map),
        params_(params) {
        auto robot_size = params_.path_search_params_.robot_size;
        robot_radius_ =
            0.5f * std::sqrt(robot_size.x() * robot_size.x() + robot_size.y() * robot_size.y());
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
        cubSpline_.setConditions(ctx_.head_pos, init_v, ctx_.tail_pos, ctx_.pieceNum);
    }
    void optimize() {
        if (ctx_.skip) {
            ctx_.skip = false;
            ctx_.path.clear();
            return;
        }
        Eigen::VectorXd x(ctx_.pieceNum * 2 - 2);

        for (int i = 0; i < ctx_.pieceNum - 1; i++) { // 控制点
            x(i) = ctx_.path[i + 1].x();
            x(i + ctx_.pieceNum - 1) = ctx_.path[i + 1].y();
        }

        double minCost = 0.0;
        lbfgs_params_.mem_size = 64; // 32
        lbfgs_params_.past = 5;
        lbfgs_params_.min_step = 1.0e-32;
        lbfgs_params_.g_epsilon = 2.0e-5;
        lbfgs_params_.delta = 2e-5; // 3e-4
        lbfgs_params_.max_linesearch = 32; // 32
        lbfgs_params_.f_dec_coeff = 1.0e-4;
        lbfgs_params_.s_curv_coeff = 0.9;
        int ret =
            lbfgs::lbfgs_optimize(x, minCost, &TrajectoryOpt::cost, nullptr, this, lbfgs_params_);
        if (ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH) {
            if (ret > 0) {
                std::cout << "[Smooth Optimize] Optimization Success: " << lbfgs::lbfgs_stderr(ret)
                          << std::endl;
            } else if (ret == 0) {
                std::cout << "[Smooth Optimize] Optimization STOP: " << lbfgs::lbfgs_stderr(ret)
                          << std::endl;
            } else {
                std::cout
                    << "[Smooth Optimize] Optimization reaches the maximum number of evaluations: "
                    << lbfgs::lbfgs_stderr(ret) << std::endl;
            }

            for (int i = 0; i < ctx_.pieceNum - 1; i++) {
                ctx_.path[i + 1].x() = x(i);
                ctx_.path[i + 1].y() = x(i + ctx_.pieceNum - 1);
            }
            ctx_.pathInPs.resize(2, ctx_.pieceNum - 1);
            ctx_.pathInPs.row(0) = x.head(ctx_.pieceNum - 1);
            ctx_.pathInPs.row(1) = x.segment(ctx_.pieceNum - 1, ctx_.pieceNum - 1);
        } else {
            ctx_.path.clear();
            std::cout << "[Smooth Optimize] Optimization Failed: " << lbfgs::lbfgs_stderr(ret);
        }
    }


    static double cost(void* ptr, const Eigen::VectorXd& x, Eigen::VectorXd& g) {
        auto instance = reinterpret_cast<TrajectoryOpt*>(ptr);
        const int points_num = instance->ctx_.pieceNum - 1;
        Eigen::Matrix2Xd grad;
        grad.resize(2, points_num);
        grad.setZero();
        double cost = 0.0;
        Eigen::Matrix2Xd inPs;
        inPs.resize(2, points_num);
        inPs.row(0) = x.head(points_num);
        inPs.row(1) = x.tail(points_num);
        Eigen::VectorXd inTimes;
        inTimes.resize(instance->ctx_.pieceNum);
        for (int i = 0; i < instance->ctx_.pieceNum; i++) {
            inTimes(i) = instance->ctx_.sample_dt;
        }
        instance->cubSpline_.setInnerPoints(inPs, inTimes);

        // Smooth 能量项
        double energy;
        Eigen::Matrix2Xd energy_grad;
        Eigen::VectorXd energyT_grad, energyTau_grad;
        energy_grad.resize(2, points_num);
        energy_grad.setZero();

        energyT_grad.resize(instance->ctx_.pieceNum);
        energyT_grad.setZero();
        energyTau_grad.resize(instance->ctx_.pieceNum);
        energyTau_grad.setZero();

        instance->cubSpline_.getEnergy(energy); // 能量损失cost和梯度
        instance->cubSpline_.getGradSmooth(energy_grad, energyT_grad);
        cost += energy;
        grad += energy_grad;

        // 障碍势项：直接用 ESDF 采样 + 梯度
        Eigen::Matrix2Xd potential_grad;
        potential_grad.resize(2, points_num);
        potential_grad.setZero();
        for (int i = 0; i < points_num; ++i) {
            double nearest_cost = 0.0;
            Eigen::Vector2d x_cur = inPs.col(i);
            Eigen::Vector2d obstacleGrad = instance->obstacleTerm(i, x_cur, nearest_cost);

            potential_grad.col(i) = obstacleGrad;
            cost += nearest_cost;
        }
        grad += potential_grad;

        g.setZero();
        // 控制点和时间梯度
        g.head(points_num) = grad.row(0).transpose();
        g.tail(points_num) = grad.row(1).transpose();

        return cost;
    }


    Eigen::Vector2d obstacleTerm(int /*idx*/, Eigen::Vector2d xcur, double& nearest_cost) {
        nearest_cost = 0.0;
        Eigen::Vector2d gradient(0.0, 0.0);

        const double R = 1.0; /// 影响半径（可调）
        double esdf_dist = 0.0;
        Eigen::Vector2d esdf_grad(0.0, 0.0);

        // 从 ESDF 插值并得到梯度（世界坐标）
        bool ok = sampleEsdfAndGrad(xcur, esdf_dist, esdf_grad);
        if (!ok) {
            // 无效采样（超出地图或其它原因），认为安全
            return gradient;
        }

        // 如果距离大于影响半径，则不计代价
        if (!std::isfinite(esdf_dist) || esdf_dist > R) {
            return gradient;
        }

        // 代价：二次穿透代价 (R - d)^2
        double penetration = (R - esdf_dist);
        nearest_cost = static_cast<double>(wObstacle) * penetration * penetration;

        // d/dx [ w * (R - d(x))^2 ] = 2 * w * (R - d(x)) * (- grad d(x))
        // 注意：esdf_grad 指向距离增大的方向（∇d）
        gradient = static_cast<double>(wObstacle) * 2.0 * penetration * (-esdf_grad);

        return gradient;
    }

    bool sampleEsdfAndGrad(const Eigen::Vector2d &pos, double &out_dist, Eigen::Vector2d &out_grad) const {
        if (!rose_map_) return false;

        const double vs = static_cast<double>(rose_map_->acc_map_info_.voxel_size_);
        if (vs <= 0.0) return false;

        // 得到包含 pos 的格子 key 与该格中心的世界坐标
        Eigen::Vector2f pos2f(static_cast<float>(pos.x()), static_cast<float>(pos.y()));
        rose_map::VoxelKey2D key = rose_map_->worldToKey2D(pos2f);
        Eigen::Vector3f _center = rose_map_->key2DToWorld(key);
        Eigen::Vector2f center = _center.head<2>();
        // 计算 pos 在该格子内的相对偏移 fx, fy ∈ [0,1]
        double fx = (pos.x() - static_cast<double>(center.x())) / vs + 0.5;
        double fy = (pos.y() - static_cast<double>(center.y())) / vs + 0.5;
        fx = std::min(std::max(fx, 0.0), 1.0);
        fy = std::min(std::max(fy, 0.0), 1.0);

        // 四个角格子键
        rose_map::VoxelKey2D k00 = key;
        rose_map::VoxelKey2D k10 = { key.x + 1, key.y };
        rose_map::VoxelKey2D k01 = { key.x, key.y + 1 };
        rose_map::VoxelKey2D k11 = { key.x + 1, key.y + 1 };

        auto readEsdf = [&](const rose_map::VoxelKey2D &k)->double {
            int idx = rose_map_->key2DToIndex2D(k);
            if (idx < 0) {
                // out of bounds -> treat as very large distance (safe)
                return 1e3;
            }
            return static_cast<double>(rose_map_->esdf_[idx]);
        };

        double v00 = readEsdf(k00);
        double v10 = readEsdf(k10);
        double v01 = readEsdf(k01);
        double v11 = readEsdf(k11);

        // 双线性插值
        double v0 = v00 * (1.0 - fx) + v10 * fx;
        double v1 = v01 * (1.0 - fx) + v11 * fx;
        double val = v0 * (1.0 - fy) + v1 * fy;

        // 插值对世界坐标的偏导数
        double dv_dx = ((v10 - v00) * (1.0 - fy) + (v11 - v01) * fy) / vs;
        double dv_dy = ((v01 - v00) * (1.0 - fx) + (v11 - v10) * fx) / vs;

        out_dist = val;
        out_grad.x() = dv_dx;
        out_grad.y() = dv_dy;

        return true;
    }

    std::vector<Eigen::Vector2d> getTrajectory() {
        return ctx_.path;
    }

    struct Ctx {
        std::vector<Eigen::Vector2d> path;
        Eigen::Vector2d head_pos;
        Eigen::Vector2d tail_pos;
        Eigen::Matrix2Xd pathInPs;
        int pieceNum = 0;
        double sample_dt = 0.0;
        bool init_obs = true;
        bool skip = false;
    } ctx_;

    rose_map::RoseMap::Ptr rose_map_;
    Parameters params_;
    CubicSpline cubSpline_;

    lbfgs::lbfgs_parameter lbfgs_params_;
    float robot_radius_;
    float wObstacle = 1.0;
};

} // namespace rose_planner
