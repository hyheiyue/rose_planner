#pragma once
#include "common.hpp"
#include "cubic_spline.hpp"
#include "lbfgs.hpp"
#include "parameters.hpp"
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
        w_obs = params_.opt_params.obstacle_weight;
        w_smooth = params_.opt_params.smooth_weight;
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
            if (ret > 0) {
                // std::cout << "[Smooth Optimize] Optimization Success: "
                //           << lbfgs::lbfgs_strerror(ret) << std::endl;
            } else if (ret == 0) {
                std::cout << "[Smooth Optimize] Optimization STOP: " << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            } else {
                std::cout
                    << "[Smooth Optimize] Optimization reaches the maximum number of evaluations: "
                    << lbfgs::lbfgs_strerror(ret) << std::endl;
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
            std::cout << "[Smooth Optimize] Optimization Failed: " << lbfgs::lbfgs_strerror(ret);
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
        energy_grad *= instance->w_smooth;
        energy = energy * instance->w_smooth;
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
        double weight_min_safe = 1.0;
        if (esdf_dist < params_.robot_radius) {
            weight_min_safe *= (1 + (params_.robot_radius - esdf_dist));
        }
        nearest_cost = w_obs * penetration * penetration * weight_min_safe;

        // d/dx [ w * (R - d(x))^2 ] = 2 * w * (R - d(x)) * (- grad d(x))
        // 注意：esdf_grad 指向距离增大的方向（∇d）
        gradient = w_obs * 2.0 * penetration * (-esdf_grad) * weight_min_safe;

        return gradient;
    }

    bool sampleEsdfAndGrad(const Eigen::Vector2d& pos, double& out_dist, Eigen::Vector2d& out_grad)
        const {
        if (!rose_map_)
            return false;
        const float vs = rose_map_->acc_map_info_.voxel_size_;
        if (!(vs > 0.f))
            return false;

        // 转为 key
        Eigen::Vector2f pos2f(pos.x(), pos.y());
        auto key = rose_map_->worldToKey2D(pos2f);
        if (rose_map_->key2DToIndex2D(key) < 0)
            return false;

        auto read = [&](int dx, int dy) {
            rose_map::VoxelKey2D k { key.x + dx, key.y + dy };
            int idx = rose_map_->key2DToIndex2D(k);
            if (idx < 0)
                return 1000.0;
            double d = rose_map_->esdf_[idx];
            return std::isfinite(d) ? d : 1000.0;
        };

        // 取 x 方向 3 点（y 固定在 key.y, key.y+1 两行形成 2×3）
        double x0 = read(-1, 0), x1 = read(0, 0), x2 = read(1, 0);
        double x3 = read(-1, 1), x4 = read(0, 1), x5 = read(1, 1);

        // 取 y 方向 3 点（x 固定在 key.x, key.x+1 两列形成 3×2，但我们分别对两列拟合）
        double y0 = read(0, -1), y1 = read(0, 0), y2 = read(0, 1);
        double y3 = read(1, -1), y4 = read(1, 0), y5 = read(1, 1);

        // 先对 x 方向两行分别二次拟合，再在 fy 方向线性插值两个二次函数的结果
        auto fitQuad = [&](double p0, double p1, double p2, double f) {
            // 解二次系数 a,b,c 使得 f(0)=p1, f(-1)=p0, f(1)=p2
            double a = 0.5f * (p2 + p0 - 2.f * p1);
            double b = 0.5f * (p2 - p0);
            double c = p1;
            // 计算插值值
            double v = a * (f * f) + b * f + c;
            // 计算导数
            double g = 2.f * a * f + b;
            return std::pair<double, double> { v, g };
        };

        double fx = (pos2f.x() - (key.x + 0.5f) * vs) / vs; // 相对 cell 中心偏移 [-0.5,0.5]
        fx = std::clamp(fx, -0.5, 0.5);
        double fy = (pos2f.y() - (key.y + 0.5f) * vs) / vs;
        fy = std::clamp(fy, -0.5, 0.5);

        auto r0 = fitQuad(x0, x1, x2, fx);
        auto r1 = fitQuad(x3, x4, x5, fx);
        double valx = r0.first * (1.f - (fy + 0.5f)) + r1.first * (fy + 0.5f);
        double gradx = r0.second * (1.f - (fy + 0.5f)) + r1.second * (fy + 0.5f);

        // 同理 y 方向：两列分别二次拟合，再在 fx 方向线性插值
        auto s0 = fitQuad(y0, y1, y2, fy);
        auto s1 = fitQuad(y3, y4, y5, fy);
        double valy = s0.first * (1.f - (fx + 0.5f)) + s1.first * (fx + 0.5f);
        double grady = s0.second * (1.f - (fx + 0.5f)) + s1.second * (fx + 0.5f);

        // 最终距离取 x/y 拟合的均衡融合（也可以只用距离场值，梯度用 (gradx, grady)）
        out_dist = 0.5f * (valx + valy);
        out_grad.x() = gradx;
        out_grad.y() = grady;

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

    lbfgs::lbfgs_parameter_t lbfgs_params_;
    double w_obs = 1.0;
    double w_smooth = 1.0;
};

} // namespace rose_planner
