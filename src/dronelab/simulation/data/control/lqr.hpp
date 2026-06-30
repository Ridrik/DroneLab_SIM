#ifndef DRONELAB_SIMULATION_DATA_CONTROL_LQR_HPP
#define DRONELAB_SIMULATION_DATA_CONTROL_LQR_HPP

#include <cstddef>
#include <cstdint>
#include <expected>
#include <utility>

#include <Eigen/Dense>

namespace dronelab::sim::msg {

enum class DareError : std::uint8_t {
    kSingularInversion,     // (R + B'PB) was not invertible at some iteration
    kDiverged,              // P became non-finite (NaN/Inf) — likely (A,B) not
                            // stabilizable, or Q/R not valid weighting matrices
    kMaxIterationsExceeded  // Loop ran to completion without satisfying tolerance
};

[[nodiscard]] constexpr std::string_view dareErrorToString(DareError err) noexcept {
    switch (err) {
        case DareError::kSingularInversion:
            return "R + B'PB was singular or near-singular during DARE iteration";
        case DareError::kDiverged:
            return "P diverged (non-finite) during DARE iteration; check that "
                   "(A,B) is stabilizable and Q, R are valid weight matrices";
        case DareError::kMaxIterationsExceeded:
            return "DARE iteration did not converge within maxIterations";
    }
    std::unreachable();
}

template <std::size_t N, std::size_t U>
struct LQRSolver {
    using StateMatrix = Eigen::Matrix<double, N, N>;
    using ControlMatrix = Eigen::Matrix<double, N, U>;
    using GainMatrix = Eigen::Matrix<double, U, N>;
    using InputMatrix = Eigen::Matrix<double, U, U>;

    [[nodiscard]] std::expected<void, DareError> computeStaticGain(
        const StateMatrix& A, const ControlMatrix& B, const StateMatrix& Q,
        const Eigen::Matrix<double, U, U>& R, double tolerance = 1e-9,
        std::size_t maxIterations = 10000) {
        // 1. Solve the Algebraic Riccati Equation for P
        // 2. K = R^-1 * B.transpose() * P
        auto expectK = solveDARE(A, B, Q, R, tolerance, maxIterations);
        if (!expectK.has_value()) {
            return std::unexpected(expectK.error());
        }
        K = std::move(expectK).value();
        return {};
    }

    auto computeCommand(const Eigen::Vector<double, N>& error) const {
        return -K * error;
    }

    GainMatrix K{GainMatrix::Zero()};

   private:
    [[nodiscard]] static StateMatrix symmetrize(const StateMatrix& M) {
        return 0.5 * (M + M.transpose());
    }

    [[nodiscard]] static std::expected<GainMatrix, DareError> solveDARE(
        const StateMatrix& Ad, const ControlMatrix& Bd, const StateMatrix& Q,
        const Eigen::Matrix<double, U, U>& R, double tolerance = 1e-9,
        std::size_t maxIterations = 10000) {
        // 1st version. Review to be made.

        Eigen::Matrix<double, N, N> P = Q;  // Initial guess
        Eigen::Matrix<double, N, N> pNext;

        for (std::size_t i{0}; i < maxIterations; ++i) {
            // Pre-calculate common terms for performance
            StateMatrix atP = Ad.transpose() * P;
            Eigen::Matrix<double, U, N> btP = Bd.transpose() * P;

            // The "Inversion" part (R + B'PB)
            InputMatrix temp = R + btP * Bd;
            Eigen::FullPivLU<Eigen::Matrix<double, U, U>> lu(temp);
            if (!lu.isInvertible()) {
                return std::unexpected(DareError::kSingularInversion);
            }

            // Update P using the Discrete Riccati Equation
            pNext = LQRSolver::symmetrize(atP * Ad - (atP * Bd) * temp.inverse() * (btP * Ad) + Q);

            if (!pNext.allFinite()) {
                return std::unexpected{DareError::kDiverged};
            }

            const double delta = (pNext - P).norm();
            if (delta < tolerance * (1.0 + P.norm())) {
                P = pNext;
                const InputMatrix finalTemp = R + Bd.transpose() * P * Bd;
                const Eigen::FullPivLU<InputMatrix> finalLu(finalTemp);
                if (!finalLu.isInvertible()) {
                    return std::unexpected(DareError::kSingularInversion);
                }
                return finalLu.inverse() * (Bd.transpose() * P * Ad);
            }
            P = pNext;
        }

        return std::unexpected(DareError::kMaxIterationsExceeded);
    }
};

}  // namespace dronelab::sim::msg

#endif  // DRONELAB_SIMULATION_DATA_CONTROL_LQR_HPP