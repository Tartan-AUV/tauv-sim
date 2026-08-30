/**
 * @file util.cpp
 * @brief Implements math conversion and inertia helper routines for tauv_sim.
 */

#include "tauv_sim/util.h"

#include "Eigen/Eigenvalues"

/**
 * @brief Converts Stonefish matrix storage into Eigen matrix storage.
 */
Eigen::Matrix3d sf_to_eigen_matrix(const sf::Matrix3& m) {
    Eigen::Matrix3d e;

    auto row = m.getRow(0);
    e(0, 0) = row[0];
    e(0, 1) = row[1];
    e(0, 2) = row[2];
    row = m.getRow(1);
    e(1, 0) = row[0];
    e(1, 1) = row[1];
    e(1, 2) = row[2];
    row = m.getRow(2);
    e(2, 0) = row[0];
    e(2, 1) = row[1];
    e(2, 2) = row[2];

    return e;
}

/**
 * @brief Converts Eigen matrix storage into Stonefish matrix storage.
 */
sf::Matrix3 eigen_to_sf_matrix(const Eigen::Matrix3d& m) {
    sf::Matrix3 r;
    r[0][0] = m(0, 0);
    r[0][1] = m(0, 1);
    r[0][2] = m(0, 2);
    r[1][0] = m(1, 0);
    r[1][1] = m(1, 1);
    r[1][2] = m(1, 2);
    r[2][0] = m(2, 0);
    r[2][1] = m(2, 1);
    r[2][2] = m(2, 2);
    return r;
}

/**
 * @brief Computes principal inertia axes and moments for Stonefish rigid-body setup.
 */
std::pair<sf::Transform, sf::Vector3> get_sf_inertia(const config::osprey::InertialBuoyancy& cfg,
                                                     const sf::Matrix3 body_R_cad) {
    auto hull_inertia_COM_B_sf = body_R_cad * cfg.hull_inertia_COM_C * body_R_cad.transpose();
    auto I_B = sf_to_eigen_matrix(hull_inertia_COM_B_sf);

    // Symmetrize
    I_B = 0.5 * (I_B + I_B.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(I_B);
    auto eigenvectors = es.eigenvectors();
    auto eigenvalues = es.eigenvalues();

    // Ensure determinant is +1 (valid rotation)
    if (eigenvectors.determinant() < 0.0) {
        eigenvectors.col(2) *= -1.0;
    }

    auto I_CG = sf::Vector3{eigenvalues(0), eigenvalues(1), eigenvalues(2)};

    // Eigen matrices are accessed (row, col)
    sf::Matrix3 body_R_CG(
        eigenvectors(0,0), eigenvectors(0,1), eigenvectors(0,2),
        eigenvectors(1,0), eigenvectors(1,1), eigenvectors(1,2),
        eigenvectors(2,0), eigenvectors(2,1), eigenvectors(2,2)
    );

    // Safety net: Fallback to Identity if the matrix is still somehow corrupt
    if (body_R_CG[0].x() == 0 && body_R_CG[1].y() == 0 && body_R_CG[2].z() == 0) {
        body_R_CG.setIdentity();
    }

    auto body_T_CG = sf::Transform{body_R_CG, body_R_cad * cfg.t_hull_com_C};

    return {body_T_CG, I_CG};
}
