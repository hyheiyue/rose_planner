#pragma once

#include "../parameters.hpp"
#include "acado_auxiliary_functions.h"
#include "acado_common.h"
#define NX ACADO_NX /* Number of differential state variables.  */
#define NXA ACADO_NXA /* Number of algebraic variables. */
#define NU ACADO_NU /* Number of control inputs. */
#define NOD ACADO_NOD /* Number of online data values. */

#define NY ACADO_NY /* Number of measurements/references on nodes 0..N - 1. */
#define NYN ACADO_NYN /* Number of measurements/references on node N. */

#define N ACADO_N /* Number of intervals in the horizon. */

#define NUM_STEPS 20 /* Number of real-time iterations. */
#define VERBOSE 1 /* Show iterations: 1, silent: 0.  */

#define Ts 0.1 // sampling time
#define Lf 1.0

namespace rose_planner {
class AcadoMpc {
public:
    using Ptr = std::shared_ptr<AcadoMpc>;
    AcadoMpc(Parameters params): params_(params) {
        weight_p_ = params.mpc_params.weight_p;
        weight_yaw_ = params.mpc_params.weight_yaw;
        weight_v_ = params.mpc_params.weight_v;
        weight_w_ = params.mpc_params.weight_w;
        control_output_ = initAcado();
    }
    static Ptr create(Parameters params) {
        return std::make_shared<AcadoMpc>(params);
    }
    std::vector<std::vector<double>> initAcado() {
        acado_initializeSolver();

        /* Initialize the states and controls. */
        for (int i = 0; i < NX * (N + 1); ++i)
            acadoVariables.x[i] = 0.0;
        for (int i = 0; i < NU * N; ++i)
            acadoVariables.u[i] = 0.0;

        /* Initialize the measurements/reference. */
        for (int i = 0; i < NY * N; ++i)
            acadoVariables.y[i] = 0.0;
        for (int i = 0; i < NYN; ++i)
            acadoVariables.yN[i] = 0.0;

        acado_preparationStep();

        // 创建用于存储控制输出的向量
        std::vector<double> control_output_vx;
        std::vector<double> control_output_vy;
        std::vector<double> control_output_w;

        // 提取控制输出
        for (int i = 0; i < ACADO_N; ++i) {
            // 有三个控制输出：vx, vy, w
            control_output_vx.push_back(acadoVariables.u[i * ACADO_NU + 0]);
            control_output_vy.push_back(acadoVariables.u[i * ACADO_NU + 1]);
            control_output_w.push_back(acadoVariables.u[i * ACADO_NU + 2]);
        }

        for (int i = 0; i < N; i++) {
            // Setup diagonal entries
            acadoVariables.W[NY * NY * i + (NY + 1) * 0] = weight_p_;
            acadoVariables.W[NY * NY * i + (NY + 1) * 1] = weight_p_;
            acadoVariables.W[NY * NY * i + (NY + 1) * 2] = weight_yaw_;
            acadoVariables.W[NY * NY * i + (NY + 1) * 3] = weight_v_;
            acadoVariables.W[NY * NY * i + (NY + 1) * 4] = weight_v_;
            acadoVariables.W[NY * NY * i + (NY + 1) * 5] = weight_w_;
        }
        acadoVariables.WN[(NYN + 1) * 0] = weight_p_;
        acadoVariables.WN[(NYN + 1) * 1] = weight_p_;
        acadoVariables.WN[(NYN + 1) * 2] = weight_yaw_;

        // 返回包含三个控制输出向量的向量
        return { control_output_vx, control_output_vy, control_output_w };
    }
    std::vector<std::vector<double>>
    solve(std::vector<double> states, std::vector<double> desired_state) {
        /* Some temporary variables. */
        int i, iter;
        acado_timer t;

        /* Initialize the states and controls. */
        for (i = 0; i < NX * (N + 1); ++i) {
            acadoVariables.x[i] = (real_t)states[i];
        }
        for (i = 0; i < NX; ++i) {
            acadoVariables.x0[i] = (real_t)states[i];
        }

        /* Initialize the measurements/reference. */
        for (i = 0; i < NY * N; ++i) {
            acadoVariables.y[i] = (real_t)desired_state[i];
        }
        for (i = 0; i < NYN; ++i) {
            acadoVariables.yN[i] = (real_t)desired_state[NY * (N - 1) + i];
        }

        // /* Prepare first step */
        acado_preparationStep();

        /* Get the time before start of the loop. */
        acado_tic(&t);

        /* The "real-time iterations" loop. */
        for (iter = 0; iter < NUM_STEPS; ++iter) {
            /* Perform the feedback step. */
            acado_feedbackStep();
            acado_preparationStep();
        }

        /* Read the elapsed time. */
        real_t te = acado_toc(&t);

        // 提取控制输出
        std::vector<double> control_output_vx;
        std::vector<double> control_output_vy;
        std::vector<double> control_output_w;
        real_t* u = acado_getVariablesU();
        for (int i = 0; i < ACADO_N; ++i) {
            control_output_vx.push_back((double)u[i * ACADO_NU + 0]);
            control_output_vy.push_back((double)u[i * ACADO_NU + 1]);
            control_output_w.push_back((double)u[i * ACADO_NU + 2]);
        }

        // 返回第一个时间步的控制输出
        return { control_output_vx, control_output_vy, control_output_w };
    }
    Parameters params_;
    double weight_p_, weight_yaw_, weight_v_, weight_w_;
    std::vector<std::vector<double>> control_output_;
};

} // namespace rose_planner