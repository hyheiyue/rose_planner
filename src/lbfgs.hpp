#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace lbfgs {
struct lbfgs_parameter {
    /**
         * 近似逆 hessian 矩阵的修正次数,即为LBFGS的窗口长度
         *该参数控制有限内存的大小，默认值为8 小于 3 的值不建议
         */
    int mem_size = 8;
    /**
         * Epsilon 用于梯度收敛测试。 不要在不光滑的情况下使用
         * 将其设置为 0.0 并对 非平滑函数 使用基于过去增量的测试
         * 终止于||g(x)||_inf / max(1, ||x||_inf) < g_epsilon,
         * 实际上，该值应大于 1.0e-6，因为L-BFGS不会直接减少一阶残差，它仍然需要函数值
         */
    double g_epsilon = 1.0e-4;
    /**
         * 基于增量的收敛测试的距离，此参数确定迭代中的距离，以计算成本函数的下降率
         * 如果该参数的值为零，则库不执行基于增量的收敛测试。 默认值为 3
         */

    int past = 3;
    /**
         * 收敛测试的Delta，该参数决定了cost function的最小下降率
         * 当出现以下情况时，库将停止迭代满足条件   |f' - f| / max(1, |f|) < delta
         * 其中 f'是过去迭代之前的成本值，f是当前迭代的成本值
         */
    double delta = 1.0e-5;
    int max_iterations = 0; //最大迭代次数，如果等于0 ，则为直到收敛或出现错误
    /**
         * 线搜索的最大尝试次数，该参数控制函数和梯度评估的数量
         * 线搜索例程的每次迭代。 默认值为 64
         */
    int max_linesearch = 64;
    /**
         * 线搜索例程的最小与最大步长， 默认值为 1.0e-20  1.0e+20
         */
    double min_step = 1.0e-10;
    double max_step = 1.0e+10;
    /**
         * 控制线搜索例程精度的参数，保证充分下降条件的参数c1
         * 默认值为 1.0e-4。 这个参数应该大一些大于零且小于 1.0
         */
    double f_dec_coeff = 1.0e-4;
    /**
         * 控制线搜索例程精度的参数,保证曲率条件的参数c2， 默认值为 0.9
         * 将此参数设置为较小的值可能会有利，典型的小值为0.1，这个参数应该是大于f_dec_coeff参数并小于1.0
         */
    double s_curv_coeff = 0.9;
    /**
         * 确保非凸函数全局收敛的参数，默认值为 1.0e-6
         * 执行Cautious-LBFGS
         */
    double cautious_factor = 1.0e-6;
    double machine_prec = 1.0e-16; // 浮点值的机器精度。 默认值为 1.0e-16
};

enum {
    /**LBFGS收敛*/
    LBFGS_CONVERGENCE = 0,
    /**L-BFGS满足停止标准*/
    LBFGS_STOP,
    /**迭代已被回调取消*/
    LBFGS_CANCELED,
    /**未知错误*/
    LBFGSERR_UNKNOWNERROR = -1024,
    /**指定的变量数量无效*/
    LBFGSERR_INVALID_N,
    /**指定的参数 lbfgs_parameter_t::mem_size 无效*/
    LBFGSERR_INVALID_MEMSIZE,
    /**指定的参数 lbfgs_parameter_t::g_epsilon 无效*/
    LBFGSERR_INVALID_GEPSILON,
    /** Invalid parameter lbfgs_parameter_t::past specified. */
    LBFGSERR_INVALID_TESTPERIOD,
    /** Invalid parameter lbfgs_parameter_t::delta specified. */
    LBFGSERR_INVALID_DELTA,
    /** Invalid parameter lbfgs_parameter_t::min_step specified. */
    LBFGSERR_INVALID_MINSTEP,
    /** Invalid parameter lbfgs_parameter_t::max_step specified. */
    LBFGSERR_INVALID_MAXSTEP,
    /** Invalid parameter lbfgs_parameter_t::f_dec_coeff specified. */
    LBFGSERR_INVALID_FDECCOEFF,
    /** Invalid parameter lbfgs_parameter_t::s_curv_coeff specified. */
    LBFGSERR_INVALID_SCURVCOEFF,
    /** Invalid parameter lbfgs_parameter_t::machine_prec specified. */
    LBFGSERR_INVALID_MACHINEPREC,
    /** Invalid parameter lbfgs_parameter_t::max_linesearch specified. */
    LBFGSERR_INVALID_MAXLINESEARCH,
    /**函数值变为NaN or Inf. */
    LBFGSERR_INVALID_FUNCVAL,
    /**线搜索步长变得小于 lbfgs_parameter_t::min_step*/
    LBFGSERR_MINIMUMSTEP,
    /**线搜索步长变得大于 lbfgs_parameter_t::max_step*/
    LBFGSERR_MAXIMUMSTEP,
    /**线搜索达到最大值，不满足假设或无法达到精度*/
    LBFGSERR_MAXIMUMLINESEARCH,
    /**算法例程达到最大迭代次数*/
    LBFGSERR_MAXIMUMITERATION,
    /**相对搜索间隔宽度至少为 lbfgs_parameter_t::machine_prec*/
    LBFGSERR_WIDTHTOOSMALL,
    /**发生逻辑错误（无效的line search步骤）*/
    LBFGSERR_INVALIDPARAMETERS,
    /**当前搜索方向增加成本函数值*/
    LBFGSERR_INCREASEGRADIENT,
};

/**
     * 定义回调函数指针，提供cost function和梯度评估
     * x 变量的当前值     g 梯度向量。 回调函数必须计算当前变量的梯度值
     * instance 客户端发送给 lbfgs_optimize() 函数的用户数据
     * retval double 当前变量的成本函数值
     */
typedef double (*lbfgs_evaluate_t)(void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g);
/**
      * 回调接口来监控最小化过程的进度, 此功能，客户端程序可以存储或显示当前进度.如果不使用，则设置为nullptr即可
      * fx 成本函数的当前值     step 用于本次迭代的线搜索步骤       ls 本次迭代调用的评估次数
      * k 迭代次数      x,g 变量的当前值与梯度值
      * retval int 清零以继续最小化过程.返回一个非零值将取消最小化过程
      */
typedef int (*lbfgs_progress_t
)(void* instance,
  const Eigen::VectorXd& x,
  const Eigen::VectorXd& g,
  const double fx,
  const double step,
  const int k,
  const int ls);

/**
     * Callback data struct
     */
struct callback_data_t {
    void* instance = nullptr;
    lbfgs_evaluate_t proc_evaluate = nullptr;
    lbfgs_progress_t proc_progress = nullptr;
};

/**
     * 该函数执行线搜索来查找同时满足 Armijo条件和弱 Wolfe条件的点
     * @param x
     * @param f
     * @param g
     * @param stp
     * @param d
     * @param xp
     * @param gp
     * @param stpmin
     * @param stpmax
     * @param cd
     * @param param
     * @return
     */

inline int line_search_wolfe_condition(
    Eigen::VectorXd& x,
    double& f,
    Eigen::VectorXd& g,
    double& stp,
    const Eigen::VectorXd& d,
    const Eigen::VectorXd& xp,
    const Eigen::VectorXd& gp,
    const double stpmin,
    const double stpmax,
    const callback_data_t& cd,
    const lbfgs_parameter& param
) {
    int iter_num = 0;
    bool touched = false; // 参看迭代进度
    if (!(stp > 0.0)) {
        std::cout << "stp: " << stp << std::endl;
        return LBFGSERR_INVALIDPARAMETERS;
    }

    double f_k0 = f;
    double f_k1 = f;

    auto S_aplha_right = param.f_dec_coeff * gp.dot(d); // c1 * d * grad
    auto C_alpha_right = param.s_curv_coeff * gp.dot(d); // c2* d * grad

    double left = 0.0, right = stpmax;
    while (true) {
        x = xp + stp * d; // 步进进行线搜索 stp对应wolfe条件中的alpha
        //            std::cout<<"xp: "<<xp<<" stp * d: "<<stp * d<<std::endl;
        //            std::cout<<" stp: "<<stp<<std::endl;
        f_k1 = cd.proc_evaluate(cd.instance, x, g); // 评估步进后的梯度与cost function
        iter_num++;

        if (f_k1 - f_k0 > stp * S_aplha_right) { // 充分下降条件判断失败
            right = stp;
        } else if (g.dot(d) < C_alpha_right) { // 曲率下降条件判定失败
            left = stp;
        } else {
            f = f_k1;
            return iter_num;
        }

        bool bracket = right < stpmax ? true : false; // 判断线搜索的有效性
        if (param.max_linesearch <= iter_num) {
            return LBFGSERR_MAXIMUMLINESEARCH;
        }
        if (bracket && (right - left) < param.machine_prec * right) {
            return LBFGSERR_WIDTHTOOSMALL;
        }

        if (right < stpmax) {
            stp = 0.5 * (left + right);
            //                std::cout<<" left: "<<left<<" right: "<<right<<std::endl;
        } else {
            stp *= 2.0;
        }

        if (stp < stpmin) {
            return LBFGSERR_MINIMUMSTEP;
        }
        if (stp > stpmax) {
            if (touched) {
                return LBFGSERR_MAXIMUMSTEP;
            } else {
                touched = true;
                stp = stpmax;
            }
        }
    }
}

/**
     * LBFGS，启动！
     * @param x
     * @param f
     * @param proc_evaluate
     * @param proc_progress
     * @param instance
     * @param param
     * @return
     */
inline int lbfgs_optimize(
    Eigen::VectorXd& x,
    double& f,
    lbfgs_evaluate_t proc_evaluate,
    lbfgs_progress_t proc_progress,
    void* instance,
    const lbfgs_parameter& param
) {
    int ret, i, j, k, ls, end, bound;
    double step, step_min, step_max, fx, ys, yy;
    double gnorm_inf, xnorm_inf, beta, rate, cau;

    // 近似逆hessian矩阵的修正次数与问题的复杂度 时间复杂度为O(mu)
    const int n = x.size();
    const int m = param.mem_size;
    /* 准备中间变量 */
    Eigen::VectorXd xp(n); // 分别记录xk+1,xk,gk,gk+1
    Eigen::VectorXd g(n);
    Eigen::VectorXd gp(n);
    Eigen::VectorXd d(n);
    Eigen::VectorXd pf(std::max(1, param.past));
    Eigen::VectorXd lm_alpha = Eigen::VectorXd::Zero(m);
    Eigen::MatrixXd lm_s = Eigen::MatrixXd::Zero(n, m);
    Eigen::MatrixXd lm_y = Eigen::MatrixXd::Zero(n, m);
    Eigen::VectorXd lm_ys = Eigen::VectorXd::Zero(m);

    /* Construct a callback data. */
    callback_data_t cd;
    cd.instance = instance;
    cd.proc_evaluate = proc_evaluate;
    cd.proc_progress = proc_progress;
    fx = cd.proc_evaluate(cd.instance, x, g); // get梯度和损失函数值
    pf(0) = fx;
    d = -g; // 搜索方向，我们假定初始的海森矩阵是I矩阵
    gnorm_inf = g.cwiseAbs().maxCoeff(); // 得到x和g的无穷范数
    xnorm_inf = x.cwiseAbs().maxCoeff();
    if (gnorm_inf / std::max(1.0, xnorm_inf) < param.g_epsilon) /// 梯度判断条件
    {
        /* The initial guess is already a stationary point. */
        ret = LBFGS_CONVERGENCE;
    } else {
        step = 1.0 / d.norm();

        k = 1;
        end = 0;
        bound = 0;

        while (true) {
            //                std::cout<<" lbfgs_optimize: "<<std::endl;
            xp = x;
            gp = g;

            step_max = param.max_step;
            step_min = param.min_step;
            ls = line_search_wolfe_condition(
                x,
                fx,
                g,
                step,
                d,
                xp,
                gp,
                step_min,
                step_max,
                cd,
                param
            ); // 注意线搜索时x,g就已经得到了更新

            if (ls < 0) { // 异常的线搜索处理情况，直接返回异常
                x = xp;
                g = gp;
                ret = ls;
                //std::cout << "d: " << d << std::endl;
                break;
            }

            if (cd.proc_progress) {
                if (cd.proc_progress(cd.instance, x, g, fx, step, k, ls)) {
                    ret = LBFGS_CANCELED;
                    break;
                }
            }

            // 收敛条件判断
            gnorm_inf = g.cwiseAbs().maxCoeff();
            xnorm_inf = x.cwiseAbs().maxCoeff();

            //                std::cout<<"gnorm: "<<gnorm_inf / std::max(1.0, xnorm_inf)<<" gnorm_inf: "<<gnorm_inf <<" xnorm_inf: "<<xnorm_inf<<std::endl;
            if (gnorm_inf / std::max(1.0, xnorm_inf) < param.g_epsilon) {
                /* Convergence. */
                ret = LBFGS_CONVERGENCE;
                break;
            }

            if (0 < param.past) // 只保留过去past个数字用于计算非光滑条件下的收敛条件
            {
                if (param.past <= k) // 注意如果迭代次数，也就是有效数据小于past的话不进行收敛性判断
                {
                    rate = std::fabs(pf(k % param.past) - fx) / std::max(1.0, std::fabs(fx));

                    if (rate < param.delta) { // 收敛性判断
                        ret = LBFGS_STOP;
                        break;
                    }
                }
                pf(k % param.past) = fx;
            }
            if (param.max_iterations != 0 && param.max_iterations <= k) // TODO 最easy的方法
            { // 达到了最大收敛次数
                ret = LBFGSERR_MAXIMUMITERATION;
                break;
            }
            ++k;

            lm_s.col(end) = x - xp; // s_{k+1} = x_{k+1} - x_{k} = \step * d_{k}. cost变化
            lm_y.col(end) = g - gp; // y_{k+1} = g_{k+1} - g_{k}.  cost梯度变化

            ys = lm_y.col(end).dot(lm_s.col(end)); // 1/rho
            yy = lm_y.col(end).squaredNorm();
            lm_ys(end) = ys;

            d = -g;

            // 进行cautious-LBFGS更新，更新条件：delta_{g}.dot(delta_{x}) >  cautious_factor * delta_{x}^2 * gp.norm()
            cau = lm_s.col(end).squaredNorm() * gp.norm() * param.cautious_factor;
            if (ys > cau) { // 进行LBFGS的步长更新
                ++bound;
                // m：窗口长度
                bound = m < bound ? m : bound;
                end = (end + 1) % m;

                j = end;

                for (i = 0; i < bound; ++i) {
                    // k-1, k-2, ..., k-m
                    j = (j + m - 1) % m;
                    lm_alpha(j) = lm_s.col(j).dot(d) / lm_ys(j);
                    d += (-lm_alpha(j)) * lm_y.col(j);
                }

                d *= ys / yy; // d = d/rho * yy

                for (i = 0; i < bound; ++i) {
                    beta = lm_y.col(j).dot(d) / lm_ys(j);
                    d += (lm_alpha(j) - beta) * lm_s.col(j);
                    // k-m, k-m-1, ..., k-1 (注意与上面的区别)
                    j = (j + 1) % m;
                }
            }
            /* The search direction d is ready. We try step = 1 first. */
            step = 2;
        }
    }

    /* Return the final value of the cost function. */
    f = fx;
    return ret;
}

/**
     * 输入的错误码来输出错误信息
     * @param err
     * @return
     */
inline const char* lbfgs_stderr(const int err) {
    switch (err) {
        case LBFGS_CONVERGENCE:
            return "Success: reached convergence (g_epsilon).";

        case LBFGS_STOP:
            return "Success: met stopping criteria (past f decrease less than delta).";

        case LBFGS_CANCELED:
            return "The iteration has been canceled by the monitor callback.";

        case LBFGSERR_UNKNOWNERROR:
            return "Unknown error.";

        case LBFGSERR_INVALID_N:
            return "Invalid number of variables specified.";

        case LBFGSERR_INVALID_MEMSIZE:
            return "Invalid parameter lbfgs_parameter_t::mem_size specified.";

        case LBFGSERR_INVALID_GEPSILON:
            return "Invalid parameter lbfgs_parameter_t::g_epsilon specified.";

        case LBFGSERR_INVALID_TESTPERIOD:
            return "Invalid parameter lbfgs_parameter_t::past specified.";

        case LBFGSERR_INVALID_DELTA:
            return "Invalid parameter lbfgs_parameter_t::delta specified.";

        case LBFGSERR_INVALID_MINSTEP:
            return "Invalid parameter lbfgs_parameter_t::min_step specified.";

        case LBFGSERR_INVALID_MAXSTEP:
            return "Invalid parameter lbfgs_parameter_t::max_step specified.";

        case LBFGSERR_INVALID_FDECCOEFF:
            return "Invalid parameter lbfgs_parameter_t::f_dec_coeff specified.";

        case LBFGSERR_INVALID_SCURVCOEFF:
            return "Invalid parameter lbfgs_parameter_t::s_curv_coeff specified.";

        case LBFGSERR_INVALID_MACHINEPREC:
            return "Invalid parameter lbfgs_parameter_t::machine_prec specified.";

        case LBFGSERR_INVALID_MAXLINESEARCH:
            return "Invalid parameter lbfgs_parameter_t::max_linesearch specified.";

        case LBFGSERR_INVALID_FUNCVAL:
            return "The function value became NaN or Inf.";

        case LBFGSERR_MINIMUMSTEP:
            return "The line-search step became smaller than lbfgs_parameter_t::min_step.";

        case LBFGSERR_MAXIMUMSTEP:
            return "The line-search step became larger than lbfgs_parameter_t::max_step.";

        case LBFGSERR_MAXIMUMLINESEARCH:
            return "Line search reaches the maximum try number, assumptions not satisfied or precision not achievable.";

        case LBFGSERR_MAXIMUMITERATION:
            return "The algorithm routine reaches the maximum number of iterations.";

        case LBFGSERR_WIDTHTOOSMALL:
            return "Relative search interval width is at least lbfgs_parameter_t::machine_prec.";

        case LBFGSERR_INVALIDPARAMETERS:
            return "A logic error (negative line-search step) occurred.";

        case LBFGSERR_INCREASEGRADIENT:
            return "The current search direction increases the cost function value.";

        default:
            return "(unknown)";
    }
}

} // namespace lbfgs
