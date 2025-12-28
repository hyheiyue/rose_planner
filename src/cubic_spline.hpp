#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <cfloat>
#include <cmath>
namespace rose_planner {
class BandedSystem {
public:
    // The size of A, as well as the lower/upper
    // banded width p/q are needed
    inline void create(const int& n, const int& p, const int& q) {
        // In case of re-creating before destroying
        destroy();
        N = n;
        lowerBw = p;
        upperBw = q;
        int actualSize = N * (lowerBw + upperBw + 1);
        ptrData = new double[actualSize];
        std::fill_n(ptrData, actualSize, 0.0);
        return;
    }

    inline void destroy() {
        if (ptrData != nullptr) {
            delete[] ptrData;
            ptrData = nullptr;
        }
        return;
    }

private:
    int N;
    int lowerBw;
    int upperBw;
    // Compulsory nullptr initialization here
    double* ptrData = nullptr;

public:
    // Reset the matrix to zero
    inline void reset(void) {
        std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
        return;
    }

    // The band matrix is stored as suggested in "Matrix Computation"
    inline const double& operator()(const int& i, const int& j) const {
        return ptrData[(i - j + upperBw) * N + j];
    }

    inline double& operator()(const int& i, const int& j) {
        return ptrData[(i - j + upperBw) * N + j];
    }

    // This function conducts banded LU factorization in place
    // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
    inline void factorizeLU() {
        int iM, jM;
        double cVl;
        for (int k = 0; k <= N - 2; k++) {
            iM = std::min(k + lowerBw, N - 1);
            cVl = operator()(k, k);
            for (int i = k + 1; i <= iM; i++) {
                if (operator()(i, k) != 0.0) {
                    operator()(i, k) /= cVl;
                }
            }
            jM = std::min(k + upperBw, N - 1);
            for (int j = k + 1; j <= jM; j++) {
                cVl = operator()(k, j);
                if (cVl != 0.0) {
                    for (int i = k + 1; i <= iM; i++) {
                        if (operator()(i, k) != 0.0) {
                            operator()(i, j) -= operator()(i, k) * cVl;
                        }
                    }
                }
            }
        }
        return;
    }

    // This function solves Ax=b, then stores x in b
    // The input b is required to be N*m, i.e.,
    // m vectors to be solved.
    template<typename EIGENMAT>
    inline void solve(EIGENMAT& b) const {
        int iM;
        for (int j = 0; j <= N - 1; j++) {
            iM = std::min(j + lowerBw, N - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(i, j) != 0.0) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
        for (int j = N - 1; j >= 0; j--) {
            b.row(j) /= operator()(j, j);
            iM = std::max(0, j - upperBw);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(i, j) != 0.0) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
        return;
    }

    // This function solves ATx=b, then stores x in b
    // The input b is required to be N*m, i.e.,
    // m vectors to be solved.
    template<typename EIGENMAT>
    inline void solveAdj(EIGENMAT& b) const {
        int iM;
        for (int j = 0; j <= N - 1; j++) {
            b.row(j) /= operator()(j, j);
            iM = std::min(j + upperBw, N - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(j, i) != 0.0) {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
        for (int j = N - 1; j >= 0; j--) {
            iM = std::max(0, j - lowerBw);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(j, i) != 0.0) {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
        return;
    }
};

class CubicSpline {
public:
    CubicSpline() = default;
    ~CubicSpline() {
        A.destroy();
    }

private:
    int N;
    Eigen::Vector2d headP;
    Eigen::Vector2d tailP;
    Eigen::Vector2d headV;
    BandedSystem A;
    Eigen::VectorXd T1; // 时间向量
    Eigen::MatrixX2d b;
    Eigen::MatrixX2d gradc;
    Eigen::VectorXd gradt;
    Eigen::MatrixXd b_solve;
    Eigen::MatrixX2d gradcur;
    Eigen::Matrix2Xd coeff;

public:
    inline void setConditions(
        const Eigen::Vector2d& headPos,
        const Eigen::Vector2d& headVel,
        const Eigen::Vector2d& tailPos,
        const int& pieceNum
    ) {
        N = pieceNum;
        headP = headPos;
        tailP = tailPos;
        headV = headVel;
        // 带状矩阵的存储：https://zhuanlan.zhihu.com/p/400460201
        A.create(4 * N, 4, 4); ///
        b.resize(4 * N, 2);
        T1.resize(N);
        gradc.resize(4 * N, 2);
        gradcur.resize(4 * N, 2);
        gradt.resize(N);

        return;
    }

    inline void
    setInnerPoints(const Eigen::Ref<const Eigen::Matrix2Xd>& CPs, const Eigen::VectorXd& ts) {
        A.reset();
        b.setZero();
        T1 = ts;

        A(0, 0) = 1;
        A(1, 1) = 1;
        b.row(0) = headP;
        b.row(1) = headV;

        for (int i = 0; i < N - 1; ++i) { // 样条差值求解的稀疏系数矩阵
            A(4 * i + 2, 4 * i + 2) = 2.0;
            A(4 * i + 2, 4 * i + 3) = 6.0 * T1(i);
            A(4 * i + 2, 4 * i + 6) = -2.0;

            A(4 * i + 3, 4 * i) = 1.0;
            A(4 * i + 3, 4 * i + 1) = 1.0 * T1(i);
            A(4 * i + 3, 4 * i + 2) = 1.0 * pow(T1(i), 2);
            A(4 * i + 3, 4 * i + 3) = 1.0 * pow(T1(i), 3);

            A(4 * i + 4, 4 * i) = 1.0;
            A(4 * i + 4, 4 * i + 1) = 1.0 * T1(i);
            A(4 * i + 4, 4 * i + 2) = 1.0 * pow(T1(i), 2);
            A(4 * i + 4, 4 * i + 3) = 1.0 * pow(T1(i), 3);
            A(4 * i + 4, 4 * i + 4) = -1.0;

            A(4 * i + 5, 4 * i + 1) = 1.0;
            A(4 * i + 5, 4 * i + 2) = 2.0 * T1(i);
            A(4 * i + 5, 4 * i + 3) = 3.0 * pow(T1(i), 2);
            A(4 * i + 5, 4 * i + 5) = -1.0;

            b.row(4 * i + 3) = CPs.col(i).transpose();
        }
        A(4 * N - 2, 4 * N - 4) = 1.0;
        A(4 * N - 2, 4 * N - 3) = 1.0 * T1(N - 1);
        A(4 * N - 2, 4 * N - 2) = 1.0 * pow(T1(N - 1), 2);
        A(4 * N - 2, 4 * N - 1) = 1.0 * pow(T1(N - 1), 3);
        A(4 * N - 1, 4 * N - 3) = 1.0;
        A(4 * N - 1, 4 * N - 2) = 2.0 * T1(N - 1);
        A(4 * N - 1, 4 * N - 1) = 3.0 * pow(T1(N - 1), 2);

        b.row(4 * N - 2) = tailP;

        A.factorizeLU();

        A.solve(b);

        return;
    }

    inline void getEnergy(double& energy) {
        energy = 0.0;
        for (int i = 0; i < N; i++) {
            energy += 4.0 * b.row(4 * i + 2).squaredNorm() * T1(i)
                + 12.0 * b.row(4 * i + 2).dot(b.row(4 * i + 3)) * pow(T1(i), 2)
                + 12.0 * b.row(4 * i + 3).squaredNorm() * pow(T1(i), 3);
        }
        return;
    }

    inline void getAcc(Eigen::Matrix2Xd& accleration) {
        for (int i = 0; i < N; i++) {
            accleration.col(i) = 2.0 * b.row(4 * i + 2) + 6 * b.row(4 * i + 3) * T1(i);
        }
        return;
    }

    inline void getCurvature(double& curvature) {
        /// 计算曲率，这里计算出来的导数需要绝对值，因此我们直接平方使用
        curvature = 0.0;
        for (int i = 0; i < N; i++) {
            Eigen::Vector2d vel =
                (3.0 * b.row(4 * i + 3) + 2.0 * b.row(4 * i + 2) + b.row(4 * i + 1));
            Eigen::Vector2d acc = (6.0 * b.row(4 * i + 3) + 2.0 * b.row(4 * i + 2));
            double cur_temp = (vel.x() * acc.y() - vel.y() * acc.x()) / (pow(vel.norm(), 3) + 1e-5);

            double w_temp = (vel.x() * acc.y() - vel.y() * acc.x()) / (pow(vel.norm(), 2) + 1e-5);
            if (w_temp > 0.3) {
                curvature += pow((w_temp - 0.3), 4);
            } else if (w_temp < -0.3) {
                curvature += pow((w_temp + 0.3), 4);
            }
        }
    }

    inline const Eigen::MatrixX2d& getCoeffs(void) const {
        return b;
    }

    inline void getEnergyGradCoeff() {
        gradc.setZero();
        for (int i = 0; i < N; i++) {
            gradc.row(4 * i + 3) =
                12.0 * b.row(4 * i + 2) * pow(T1(i), 2) + 24.0 * b.row(4 * i + 3) * pow(T1(i), 3);
            gradc.row(4 * i + 2) =
                8.0 * b.row(4 * i + 2) * T1(i) + 12.0 * b.row(4 * i + 3) * pow(T1(i), 2);
        }
        return;
    }

    inline void getEnergyGradTimes() {
        gradt.setZero();
        for (int i = 0; i < N; i++) {
            gradt(i) = 4.0 * b.row(4 * i + 2).squaredNorm()
                + 24.0 * b.row(4 * i + 2).dot(b.row(4 * i + 3)) * T1(i)
                + 36.0 * b.row(4 * i + 3).squaredNorm() * pow(T1(i), 2);
        }
        return;
    }

    inline void getCurvatureGradfCoeff() {
        /// 这里gradcur * matrix_A = gradx, 这里的matrix_A其实就是一个常数矩阵
        gradcur.setZero();
        Eigen::MatrixXd vel_coeff(1, 4);
        Eigen::MatrixXd acc_coeff(1, 4);

        vel_coeff << 0, 1, 2, 3;
        acc_coeff << 0, 0, 2, 6;

        for (int i = 0; i < N; i++) {
            Eigen::Vector2d vel =
                (3.0 * b.row(4 * i + 3) + 2.0 * b.row(4 * i + 2) + b.row(4 * i + 1));
            Eigen::Vector2d acc = (6.0 * b.row(4 * i + 3) + 2.0 * b.row(4 * i + 2));
            Eigen::Vector2d pos =
                (b.row(4 * i + 3) + b.row(4 * i + 2) + b.row(4 * i + 1) + b.row(4 * i + 0));

            double k_temp =
                3 * (vel.x() * acc.y() - vel.y() * acc.x()) / (pow(vel.norm(), 5) + 1e-5);
            Eigen::MatrixXd temp1(2, 1);
            temp1(0, 0) =
                (acc.y() * (pow(vel.y(), 2) - pow(vel.x(), 2)) + 2 * vel.x() * vel.y() * acc.x())
                / (pow(vel.norm(), 4) + 1e-5);
            temp1(1, 0) =
                (acc.x() * (pow(vel.y(), 2) - pow(vel.x(), 2)) - 2 * vel.x() * vel.y() * acc.y())
                / (pow(vel.norm(), 4) + 1e-5);

            Eigen::MatrixXd temp2(2, 1);
            temp2(0, 0) = -vel.y() / (pow(vel.norm(), 2) + 1e-5);
            temp2(1, 0) = vel.x() / (pow(vel.norm(), 2) + 1e-5);
            double w_temp = (vel.x() * acc.y() - vel.y() * acc.x()) / (pow(vel.norm(), 2) + 1e-5);

            if (w_temp > 0.3) {
                gradcur.block<4, 2>(4 * i, 0) = 4 * pow((w_temp - 0.3), 3)
                    * (temp1 * vel_coeff + temp2 * acc_coeff).transpose();

            } else if (w_temp < -0.3) {
                gradcur.block<4, 2>(4 * i, 0) = 4 * pow((w_temp + 0.3), 3)
                    * (temp1 * vel_coeff + temp2 * acc_coeff).transpose();
            }
        }
    }

    inline void getGradSmooth(Eigen::Matrix2Xd& gradByPoints, Eigen::VectorXd& gradByTimes) {
        getEnergyGradCoeff();
        getEnergyGradTimes();
        A.solveAdj(gradc);
        for (int i = 0; i < N - 1; i++) {
            gradByPoints.col(i) = gradc.row(4 * i + 3).transpose();
        }

        Eigen::Matrix<double, 4, 2> B1;
        Eigen::Matrix<double, 2, 2> B2;

        for (int i = 0; i < N - 1; i++) {
            B1.row(0) = -6.0 * b.row(i * 4 + 3);
            B1.row(1) =
                -(b.row(i * 4 + 1) + 2.0 * T1(i) * b.row(i * 4 + 2)
                  + 3.0 * pow(T1(i), 2) * b.row(i * 4 + 3));
            B1.row(2) = B1.row(1);
            B1.row(3) = -(2.0 * b.row(i * 4 + 2) + 6.0 * T1(i) * b.row(i * 4 + 3));
            gradByTimes(i) = B1.cwiseProduct(gradc.block<4, 2>(4 * i + 2, 0)).sum();
        }

        B2.row(0) =
            -(b.row(4 * N - 3) + 2.0 * T1(N - 1) * b.row(4 * N - 2)
              + 3.0 * pow(T1(N - 2), 2) * b.row(4 * N - 1));

        B2.row(1) = -(2.0 * b.row(4 * N - 2) + 6.0 * T1(N - 1) * b.row(4 * N - 1));
        gradByTimes(N - 1) = B2.cwiseProduct(gradc.block<2, 2>(4 * N - 2, 0)).sum();

        gradByTimes += gradt;
    }

    inline void getGradCurvature(Eigen::Matrix2Xd& gradByPoints) {
        /// 计算曲率导数
        getCurvatureGradfCoeff();
        A.solveAdj(gradcur);
        for (int i = 0; i < N - 1; i++) {
            gradByPoints.col(i) = gradcur.row(4 * i + 3).transpose();
        }
    }
};

} // namespace rose_planner
