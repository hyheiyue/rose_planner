#pragma once
#include "../common.hpp"
#include "../parameters.hpp"
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
        w_obs = params_.opt_params.obstacle_weight;
        w_smooth = params_.opt_params.smooth_weight;
        w_time = params_.opt_params.time_weight;
    }
    static Ptr create(rose_map::RoseMap::Ptr rose_map, Parameters params) {
        return std::make_shared<TrajectoryOpt>(rose_map, params);
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
        ctx_.Innerpoints.resize(2, sampled.size());

        // 只遍历 0 到 sampled.size()-2，避免列越界
        for (int i = 0; i < (int)sampled.size(); i++) {
            ctx_.Innerpoints.col(i) = sampled[i].p.cast<double>();
        }
        ctx_.pieceNum = ctx_.path.size() + 1;
        ctx_.pieceTime.resize(ctx_.pieceNum);
        ctx_.pieceTime.setOnes();
        ctx_.pieceTime *= sample_dt;
        ctx_.headPV << ctx_.path.front(), init_v;
        ctx_.tailPV << ctx_.path.back(), Eigen::Vector2d(0, 0);
        ctx_.Minco.setConditions(ctx_.headPV, ctx_.tailPV, ctx_.pieceNum);
        ctx_.Minco.setParameters(ctx_.Innerpoints, ctx_.pieceTime);
        ctx_.Minco.getTrajectory(ctx_.final_traj);
        ctx_.sample_dt = sample_dt;
        ctx_.init_obs = true;
    }
    void optimize() {
        if (ctx_.skip) {
            ctx_.skip = false;
            ctx_.path.clear();
            return;
        }

        const int segNum = ctx_.pieceNum;
        const int innerCols = segNum - 1;
        const int varNum = 2 * innerCols + 1 + segNum; // P + tailV + tau

        if (innerCols < 1) {
            std::cout << "[optimize] invalid inner columns\n";
            return;
        }

        // 构造优化变量向量 x
        Eigen::VectorXd x(varNum);
        int offset = 0;

        // 1. 拷贝 P 变量（内部控制点）
        x.segment(offset, 2 * innerCols) =
            Eigen::Map<const Eigen::VectorXd>(ctx_.Innerpoints.data(), 2 * innerCols);
        offset += 2 * innerCols;

        // 2. tail 速度变量
        x[offset++] = ctx_.tailPV(1, 0);

        // 3. 虚拟时间 tau 变量
        Eigen::VectorXd tauVirtual(segNum);
        RealT2VirtualT(ctx_.pieceTime, tauVirtual);
        x.segment(offset, segNum) = tauVirtual;
        offset += segNum;

        // L-BFGS 参数（已去除重复赋值，集中管理）
        lbfgs_params_.mem_size = 256;
        lbfgs_params_.past = 3;
        lbfgs_params_.min_step = 1e-32;
        lbfgs_params_.g_epsilon = 2e-5;
        lbfgs_params_.delta = 5.0e-4;
        lbfgs_params_.max_iterations = 5000;
        lbfgs_params_.max_linesearch = 32;
        lbfgs_params_.f_dec_coeff = 1e-4;
        lbfgs_params_.s_curv_coeff = 0.9;

        double minCost = 0.0;
        int ret = lbfgs::lbfgs_optimize(
            x,
            minCost,
            &TrajectoryOpt::cost,
            nullptr,
            nullptr,
            this,
            lbfgs_params_
        );

        if (ret < 0 && ret != lbfgs::LBFGSERR_MAXIMUMLINESEARCH) {
            std::cout << "[Smooth Optimize] Optimization Failed: " << lbfgs::lbfgs_strerror(ret)
                      << std::endl;
            ctx_.path.clear();
            return;
        }

        std::cout << "[Smooth Optimize] Result: " << lbfgs::lbfgs_strerror(ret) << std::endl;

        // 解析优化结果
        offset = 0;
        // 1. 取回 P
        ctx_.Innerpoints = Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>>(
            x.data() + offset,
            2,
            innerCols
        );
        offset += 2 * innerCols;
        ctx_.tailPV(1, 0) = x[offset++];
        Eigen::Map<const Eigen::VectorXd> tauMapped(x.data() + offset, segNum);
        VirtualT2RealT(tauMapped, ctx_.pieceTime);
        offset += segNum;

        ctx_.Minco.setTConditions(ctx_.tailPV);
        ctx_.Minco.setParameters(ctx_.Innerpoints, ctx_.pieceTime);
        ctx_.Minco.getTrajectory(optimizer_traj_);
    }

    static double cost(void* ptr, const Eigen::VectorXd& x, Eigen::VectorXd& g) {
        if (!ptr)
            return 1e9;
        if (x.norm() > 1e4)
            return 1e9;

        auto* obj = reinterpret_cast<TrajectoryOpt*>(ptr);
        auto& ctx = obj->ctx_;

        const int segNum = ctx.pieceNum;
        const int innerCols = segNum - 1;
        const int pVarSize = 2 * innerCols;

        if (innerCols < 1)
            return 1e9;
        if (x.size() < pVarSize + 1 + segNum)
            return 1e9;
        if (g.size() != pVarSize + 1 + segNum)
            return 1e9;

        int offset = 0;
        // map state vars from x
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> Pmat(
            x.data() + offset,
            2,
            innerCols
        );
        offset += pVarSize;

        double tailV = x[offset];
        offset++;

        Eigen::Map<const Eigen::VectorXd> tauVirtual(x.data() + offset, segNum);
        offset += segNum;

        // zero gradient vector first
        g.setZero();

        // create maps into g for writing gradients
        offset = 0;
        Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>> gradP(g.data() + offset, 2, innerCols);
        offset += pVarSize;

        // grad for tailV (one scalar) will be g[pVarSize]
        double& gradTailV = g[pVarSize];
        offset++; // move past tailV

        Eigen::Map<Eigen::VectorXd> gradTauVirtual(g.data() + offset, segNum);
        // offset += segNum; // not needed further
        ctx.Innerpoints = Pmat;
        ctx.tailPV(1, 0) = tailV;
        obj->VirtualT2RealT(tauVirtual, ctx.pieceTime);

        ctx.Minco.setTConditions(ctx.tailPV);
        ctx.Minco.setParameters(ctx.Innerpoints, ctx.pieceTime);

        // get energy and gradients from Minco
        double smoothCost = 0.0;
        ctx.Minco.getEnergy(smoothCost);
        Eigen::Matrix<double, 2, Eigen::Dynamic> energy_grad(2, innerCols);
        Eigen::VectorXd energyT_grad(segNum);
        energy_grad.setZero();
        energyT_grad.setZero();
        ctx.Minco.getGradSmooth(energy_grad, energyT_grad);

        // scale cost and gradients consistently
        smoothCost *= obj->w_smooth;
        energy_grad *= obj->w_smooth;
        energyT_grad *= obj->w_smooth;

        // obstacle term cost + grad
        double obsCost = 0.0;
        Eigen::Matrix<double, 2, Eigen::Dynamic> gradObs = Eigen::Matrix2Xd::Zero(2, innerCols);
        for (int i = 0; i < innerCols; ++i) {
            double c = 0.0;
            auto gp = obj->obstacleTerm(Pmat.col(i), c);
            obsCost += c;
            gradObs.col(i) = gp;
        }

        // time regularization: add to cost and to energyT_grad
        double timeRegCost = obj->w_time * ctx.pieceTime.sum();
        energyT_grad.array() += obj->w_time; // add w_time to each tau gradient

        double totalCost = smoothCost + obsCost + timeRegCost;

        // fill gradients (gradP, gradTauVirtual already mapped into g)
        gradP = energy_grad + gradObs;

        // convert energyT_grad (w.r.t real T) to gradTauVirtual (w.r.t tauVirtual)
        // assume backwardGradT does that mapping and writes into gradTauVirtual
        backwardGradT(tauVirtual, energyT_grad, gradTauVirtual);

        return totalCost;
    }

    Eigen::Vector2d obstacleTerm(const Eigen::Vector2d& xcur, double& nearest_cost) {
        nearest_cost = 0.0;
        Eigen::Vector2d grad(0.0, 0.0);

        const double R = 1.0; // 影响半径
        double d = 0.0;
        Eigen::Vector2d g(0.0, 0.0);

        if (!sampleEsdfAndGrad(xcur, d, g) || !std::isfinite(d))
            return grad; // 采样失败视为安全

        if (d > R)
            return grad; // 超出影响范围无代价

        // 计算穿透量
        double penetration = R - d;

        // 安全距离权重增强（线性而非平方，避免梯度爆炸）
        double safe_margin = params_.robot_radius + 0.1;
        double w_safe = 1.0;
        if (d < safe_margin) {
            w_safe += 4.0 * (safe_margin - d) / safe_margin; // 归一化增强
        }

        // 代价使用 Huber-like 平滑截断，避免不稳定
        double cost_core = penetration * penetration;
        double huber_delta = 0.6;
        if (penetration > huber_delta) {
            cost_core = 2 * huber_delta * penetration - huber_delta * huber_delta;
        }

        nearest_cost = w_obs * cost_core * w_safe;

        // 梯度公式：∇cost = w * cost'(penetration) * (-∇d)
        double grad_core_coeff = 2.0 * penetration;
        if (penetration > huber_delta) {
            grad_core_coeff = 2.0 * huber_delta;
        }

        double coeff = w_obs * grad_core_coeff * w_safe;

        grad = coeff * (-g);

        return grad;
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

    Trajectory<3, 2> getTrajectory() {
        return optimizer_traj_;
    }

    struct Ctx {
        std::vector<Eigen::Vector2d> path;
        Eigen::Matrix2Xd pathInPs;
        Eigen::Matrix<double, 2, 2> headPV;
        Eigen::Matrix<double, 2, 2> tailPV;
        Eigen::MatrixXd Innerpoints;
        Eigen::VectorXd pieceTime;
        minco::MINCO_S2NU Minco;
        Trajectory<3, 2> final_traj;
        int pieceNum = 0;
        double sample_dt = 0.0;
        bool init_obs = true;
        bool skip = false;
    } ctx_;

    rose_map::RoseMap::Ptr rose_map_;
    Parameters params_;
    lbfgs::lbfgs_parameter_t lbfgs_params_;
    double w_obs = 1.0;
    double w_smooth = 1.0;
    double w_time = 1.0;
    Trajectory<3, 2> optimizer_traj_;
};

} // namespace rose_planner
