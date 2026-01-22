#include "drone_simulator.hpp"

Drone::Drone(float32_t altitude_m, float32_t velocity_mps):
altitude_m_(altitude_m),
velocity_mps_(velocity_mps)
{}

Drone::Status Drone::update(float32_t thrust, float32_t delta_time) {
    // Need to add clamping to altitude and velocity 
    // New velocity = old velocity + (a * dt)
    // New altitude = old altitude + (velocity * dt)
    if (thrust < 0.0f) {
        return Drone::Status::ERROR_INVALID_INPUT;
    }

    // Calculate total acceleration
    float32_t total_acceleration = (thrust * MAX_THRUST_ACCEL) - GRAVITY;
    // Calculate new velocity and altitude
    float32_t new_velocity_mps = velocity_mps_ + (total_acceleration * delta_time);
    float32_t new_altitude_m = altitude_m_ + (new_velocity_mps * delta_time);
    
    altitude_m_ = new_altitude_m;
    velocity_mps_ = new_velocity_mps;

    // Check that the new measurements are logical
    if (altitude_m_ < 0.0f) {
        altitude_m_ = 0.0f;
        velocity_mps_ = 0.0f;
    }
    
    return Drone::Status::SUCCESS;
}

float32_t Drone::getAltitude() const{
    return altitude_m_;
}

float32_t Drone::getVelocity() const{
    return velocity_mps_;
}