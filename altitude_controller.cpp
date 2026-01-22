#include "altitude_controller.hpp"
#include <iostream>
// Error = Target Altitude - Current Altitude
// Proportional (P): Error * K_p
// Integral (I): Accumulated Error * K_i
// Derivative (D): (Current error - Prev error) * K_d
static const float32_t DELTA_TIME = 0.01f;

// Implement this formula every 10ms
AltitudeController::AltitudeController(float32_t proportional_gain, float32_t integral_gain, float32_t derivative_gain)
    : proportional_gain_(proportional_gain),
    integral_gain_(integral_gain),
    derivative_gain_(derivative_gain),
    error_(0.0f),
    accumulated_error_(0.0f),
    previous_error_(10.0f)
    {}

AltitudeController::Status AltitudeController::updateAltitude(const float32_t target_altitude, float32_t current_altitude, float32_t &output_thrust) {
    if (target_altitude < 0.0f || current_altitude < 0.0f) {
        return AltitudeController::Status::ERROR_INVALID_INPUT;
    }

    // Update error variables
    error_ = target_altitude - current_altitude;
    accumulated_error_ += (error_ * DELTA_TIME);
    // Proportional
    float32_t p_term = error_ * proportional_gain_;
    // Integral
    float32_t i_term = accumulated_error_ * integral_gain_;
    // Derivative
    float32_t d_term = ((error_ - previous_error_) / (DELTA_TIME)) * derivative_gain_;
    // Total = (P + I + D)
    output_thrust = p_term + i_term + d_term;
    // Applying clamping to output_thrust as it cannot be < 0.0f or > 1.0f
    if (output_thrust > 1.0f) {
        output_thrust = 1.0f;
    } else if (output_thrust < 0.0f){
        output_thrust = 0.0f;
    }

    if (accumulated_error_ > 10.0f) {
        accumulated_error_ = 0.0f;
    } else if (accumulated_error_ < -0.05f) {
        accumulated_error_ = 0.0f;
    }

    // Update state
    previous_error_ = error_;

    return AltitudeController::Status::SUCCESS;
}

float32_t AltitudeController::getError() {
    return error_;
}