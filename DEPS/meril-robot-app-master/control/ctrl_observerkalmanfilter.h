/*
 * Developer : Roel van Mil (roel.van.mil@vectioneer.com)
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER.
 */

#ifndef MCX_CTRL_OBSERVER_EKF_H
#define MCX_CTRL_OBSERVER_EKF_H

#include <cmath>
#include <mcx/core.h>
#include <mcx/math.h>

namespace control {

/**
 * @template<typename T>
 * @class mcx::control::ObserverKalmanFilter for Torque Estimation
 *
 * Implements an Extended Kalman Filter (EKF)-based observer for torque
 * estimation. Dynamical model is based on a mass-spring-damper system:
 *
 *     I * θ'' + b * θ' + k * θ = τ (input torque)
 *
 * where:
 * - I: Inertia
 * - b: Damping coefficient
 * - k: Stiffness of the spring
 * - θ: Joint position
 * - θ': Joint velocity
 * - θ'': Joint acceleration
 *
 * Goal: Estimate smoother torque τ_filtered using noisy position (θ) and torque
 * (τ) readings.
 */
template <unsigned int NUMBER_OF_STATES = 3, unsigned int NUMBER_OF_INPUTS = 1,
          unsigned int NUMBER_OF_MEASUREMENTS = 3>
class ObserverKalmanFilter final : public mcx::container::Module {
public:
  using StateVector =      mcx::math::Matrix<NUMBER_OF_STATES, 1>; // [θ, θ', τ_filtered]
  using InputVector =      mcx::math::Matrix<NUMBER_OF_INPUTS, 1>; // Input torque (τ_commanded)
  using MeasurementVector =      mcx::math::Matrix<NUMBER_OF_MEASUREMENTS,                        1>; // Measured [θ_measured, τ_measured]

  ObserverKalmanFilter() = default;
  ~ObserverKalmanFilter() override = default;

  /**
   * @brief Set model parameters for dynamics.
   * @param inertia - Inertia (I)
   * @param damping - Damping coefficient (b)
   * @param stiffness - Stiffness (k)
   */
  void setModelParameters(double inertia, double damping, double stiffness) {
    inertia_ = inertia;
    damping_ = damping;
    stiffness_ = stiffness;
  }

  /**
   * @brief Set model parameters for inertia.
   * @param inertia - Inertia (I)
   */
  void setModelInertia(double inertia) {
    inertia_ = inertia;
  }

  /**
   * @brief Sets measurement noise covariance matrix (R).
   */
  void setMeasurementNoiseCovariance(
      const mcx::math::Matrix<NUMBER_OF_MEASUREMENTS, NUMBER_OF_MEASUREMENTS> &R) {
    R_ = R;
  }

  /**
   * @brief Sets process noise covariance matrix (Q).
   */
  void setProcessNoiseCovariance(
      const mcx::math::Matrix<NUMBER_OF_STATES, NUMBER_OF_STATES> &Q) {
    Q_ = Q;
  }

  /**
   * @brief Sets the initial state vector.
   */
  void setInitialState(const StateVector &initialState) {
    stateVector_ = initialState;
    stateVectorInit_ = initialState;
  }

  double computeFrictionForce(double velocity) const {
    // Compute non-linear friction
    double viscous = viscousFrictionCoefficient_ * velocity; // Viscous friction
    double coulomb = coulombFrictionCoefficient_ * std::copysign(1.0, velocity); // Sign function
    return viscous + coulomb;
  }


  /**
   * @brief Predict the next state using the system dynamics.
   */
  void predictState(double dt) {
    // Friction term (non-linear friction model)
    double velocity = stateVector_(1, 0); // θ' (velocity)
    double frictionForce = computeFrictionForce(velocity); // Non-linear friction model

    // State prediction: f(x, u)
    StateVector f;
    f(0, 0) = stateVector_(1, 0); // θ' (velocity)
    f(1, 0) = (1 / inertia_) * (inputVector_(0, 0)   // τ_commanded
               - frictionForce     // Friction force
              - damping_ * stateVector_(1, 0) // Viscous damping
              - stiffness_ * stateVector_(0, 0)); // θ'' (acceleration)
    f(2, 0) = 0; // τ_filtered remains constant during prediction step

    stateVector_ += f * dt;

    // Covariance prediction: P = A * P * A^T + Q
    mcx::math::Matrix<NUMBER_OF_STATES, NUMBER_OF_STATES> A; // Jacobian of f(x, u)
    A(0, 0) = 0;
    A(0, 1) = 1;
    A(0, 2) = 0;
    A(1, 0) = -stiffness_ / inertia_;     // ∂θ''/∂θ
    A(1, 1) = -damping_ / inertia_ + viscousFrictionCoefficient_ / inertia_; // ∂θ''/∂θ' (this is the jacobian)
    A(1, 2) = 1 / inertia_;               // ∂θ''/∂τ_filtered
    A(2, 0) = 0;
    A(2, 1) = 0;
    A(2, 2) = 1;

//    P_ = A.dot(P_).dot(A.transpose()) + Q_;
    auto P = A.dot(P_).dot(A.transpose()) + Q_;
    if (!P.isNaN()){
      P_ = P;
    }
  }

  /**
   * @brief Update state and covariance using measurements.
   */
  void updateWithMeasurements() {
    // Measurement prediction: h(x)
    MeasurementVector h;
    h(0, 0) = stateVector_(0, 0); // θ (position)
    h(1, 0) = stateVector_(1, 0); // θ' (velocity)
    h(2, 0) = stateVector_(2, 0); // τ_filtered (torque)

    // Jacobian H of h(x) w.r.t. state vector
    mcx::math::Matrix<NUMBER_OF_MEASUREMENTS, NUMBER_OF_STATES> H;

    H(0, 0) = 1; // θ (position)
    H(0, 1) = 0;
    H(0, 2) = 0;

    H(1, 0) = 0;
    H(1, 1) = 1; // θ' (velocity)
    H(1, 2) = 0;

    H(2, 0) = 0;
    H(2, 1) = 0;
    H(2, 2) = 1; // τ_filtered (torque)

    // Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
    //    auto S = H.dot(P_).dot(H.transpose()) + R_; // Innovation covariance
    //    auto K = P_.dot(H.transpose()).dot(S.inv());

    mcx::math::Matrix<NUMBER_OF_MEASUREMENTS, NUMBER_OF_MEASUREMENTS> S =
        H.dot(P_).dot(H.transpose()) + R_; // Innovation covariance
    mcx::math::Matrix<NUMBER_OF_STATES, NUMBER_OF_MEASUREMENTS> K =
        P_.dot(H.transpose()).dot(S.inv()); // Kalman Gain

    // State update: x = x + K * (y - h(x))
    stateVector_ += K.dot(measurementVector_ - h);

    // Covariance update: P = (I - K * H) * P
    auto P = (Ip_ - K.dot(H)).dot(P_);
    if (!P.isNaN()){
      P_ = P;
    }
  }

  /**
   * @brief Set input torque.
   */
  void setInput(double value) { inputVector_(0, 0) = value; }

  /**
   * @brief Set input torque.
   */
  void setDisable(bool value) { disable_ = value; }

  /**
   * @brief Set measurement vector [θ_measured, τ_measured].
   */
  void setMeasurementVector(const MeasurementVector &measurements) {
    measurementVector_ = measurements;
  }

  /**
   * @brief Get the estimated state vector.
   * State vector: [θ, θ', τ_filtered]
   */
  const StateVector &getStateVector() const { return stateVector_; }

private:
  void create_(const char *name,
               mcx::parameter_server::Parameter *parameterServer,
               uint64_t dtMicroS) override {}

  bool initPhase1_() override {
    using namespace mcx::parameter_server;
    addParameter("inputVector", ParameterType::INPUT, inputVector_.data(),                 inputVector_.size());
    addParameter("measurementVector", ParameterType::INPUT,                 measurementVector_.data(), measurementVector_.size());

    addParameter("stateVector", ParameterType::OUTPUT, stateVector_.data(),                 stateVector_.size());

    addParameter("covarianceMatrix", ParameterType::OUTPUT, P_.data(),                 P_.size());
    addParameter("processNoiseCovariance", ParameterType::PARAMETER, Q_.data(),                 Q_.size());
    addParameter("measurementNoiseCovariance", ParameterType::PARAMETER,                 R_.data(), R_.size());
    addParameter("stateVectorInit", ParameterType::INPUT,                 stateVectorInit_.data(), stateVectorInit_.size());

    addParameter("enable", ParameterType::PARAMETER, &enable_);
    addParameter("disable", ParameterType::INPUT, &disable_);
    addParameter("isEnabled", ParameterType::OUTPUT, &isEnabled_);

    addParameter("model/inertia", ParameterType::PARAMETER, &inertia_);
    addParameter("model/damping", ParameterType::PARAMETER, &damping_);
    addParameter("model/stiffness", ParameterType::PARAMETER, &stiffness_);
    addParameter("model/viscousFrictionCoefficient", ParameterType::PARAMETER, &viscousFrictionCoefficient_);
    addParameter("model/coulombFrictionCoefficient_", ParameterType::PARAMETER, &coulombFrictionCoefficient_);
    addParameter("frequencyDivider", ParameterType::PARAMETER,
                 &frequencyDivider_);

    // write parameters
    P_.identity();
    Q_.identity();
    R_.identity();
    return true;
  }

  bool initPhase2_() override {
    Ip_.identity();
    return true;
  }

  bool startOp_() override { return true; }

  bool stopOp_() override { return true; }
  /**
   * @brief Process a single iteration of the EKF (prediction + update).
   */
  bool iterateOp_(const mcx::container::TaskTime &systemTime,
                  mcx::container::UserTime *userTime) override {
    const double dt = getDtSec();
    isEnabled_ = enable_ && !disable_;

    if (isEnabled_) {
      // Step 1: Predict
      predictState(dt);

      // Step 2: Update (only runs at 100 Hz)
      if (++counter_ % frequencyDivider_ ==
          0) { // Execute every 10th iteration (100 Hz)
        updateWithMeasurements();
        counter_ = 0; // Reset the counter to prevent overflow
      }
    }
    return true;
  }

private:
  unsigned int counter_ = 0;           // frequency divider
  unsigned int frequencyDivider_ = 1;  // frequency divider

  bool enable_{false};
  bool disable_{false};
  bool isEnabled_{false};

  double inertia_ = 1.0;               // Inertia
  double damping_ = 10.0;              // Damping coefficient
  double stiffness_ = 0.0;             // Stiffness coefficient
  double viscousFrictionCoefficient_ = 0.10;             // viscous friction coefficient
  double coulombFrictionCoefficient_ = 0.10;             // coulomb friction coefficient

  mcx::math::Matrix<NUMBER_OF_STATES, NUMBER_OF_STATES> Ip_; // Identity states
  mcx::math::Matrix<NUMBER_OF_STATES, NUMBER_OF_STATES> P_; // Covariance matrix
  mcx::math::Matrix<NUMBER_OF_STATES, NUMBER_OF_STATES> Q_; // Process noise covariance
  mcx::math::Matrix<NUMBER_OF_MEASUREMENTS, NUMBER_OF_MEASUREMENTS> R_; // Measurement noise covariance

  StateVector stateVector_;
  StateVector stateVectorInit_;
  InputVector inputVector_;
  MeasurementVector measurementVector_;
};

} // namespace control

#endif /* MCX_CTRL_OBSERVER_EKF_H */