#ifndef GRAVITYCOMPENSATOR_H
#define GRAVITYCOMPENSATOR_H

#include <Eigen/Dense>
#include <tuple>

using namespace Eigen;
using namespace std;

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skew_mat;
    skew_mat << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
    return skew_mat;
}

class GravityCompensator {
public:
  /**
   *
   * @param gravity_earth - gravity vector w.r.t. base frame
   * @param m_load - mass of the external load
   * @param force_offset - constant force offset
   * @param torque_offset - constant torque offset
   * @param EE_x_Cload -- center of gravity of the external load w.r.t. EE frame
   */
  GravityCompensator
  (
    const Vector3d &gravity_earth = {0.292529, -0.089269, -9.80523},
    const double &m_load = 0.232912, 
    const Vector3d &force_offset= {-1.35642, 1.84318, -6.33185},
    const Vector3d &torque_offset = {0.00557076, -0.0556243, -0.097491},
    const Vector3d &EE_x_Cload = {-3.504e-05, 0.00716195, 0.0100285}
  )
      : gravity_earth_(gravity_earth), force_offset_(force_offset),
        torque_offset_(torque_offset), EE_x_Cload_(EE_x_Cload),
        m_load_(m_load) {}

  /**
   *
   * @param force_measured - input force
   * @param torque_measured  - input torque
   * @param O_R_E  - rotation of the end effector in the robot base frame
   * @param force_compensated  - output force unbiased and gravity compensated
   * @param torque_compensated  - output torque unbiased and gravity compensated
   */
  void apply(const Vector3d &force_measured, const Vector3d &torque_measured,
             const Matrix3d &O_R_E, Vector3d &force_compensated,
             Vector3d &torque_compensated) const noexcept {
    Vector3d tmp = -m_load_ * O_R_E.transpose() * gravity_earth_;
    force_compensated = force_measured - force_offset_ + tmp;
    torque_compensated =
        torque_measured - torque_offset_ + skew(EE_x_Cload_) * tmp;
  }

private:
  const Vector3d gravity_earth_, force_offset_, torque_offset_, EE_x_Cload_;
  const double m_load_;
};

#endif