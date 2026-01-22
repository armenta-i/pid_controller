#include "drone_simulator.hpp"
#include "altitude_controller.hpp"
#include <iostream>

typedef float float32_t;
typedef int int32_t;

// TODO: Modify values in altitude controller until ALT hovers 
// around 10 instead of +-.5
int main() {
    // const float32_t MAX_SIM_TIME = 5.0f;
    const float32_t DELTA_TIME = 0.01f;
    // Create Done and AltitudeController
    Drone new_drone = Drone(0.0f, 0.0f);
    AltitudeController altitude_controller =
     AltitudeController(0.15f, 0.05f, 0.1f);

    float32_t curr_altitude;
    float32_t curr_velocity;
    float32_t curr_error;
    float32_t output_thrust;

    for (int32_t i = 0; i < 10000; i++) {
        // Calculate the values first
        altitude_controller.updateAltitude(10.0f, curr_altitude, output_thrust);
        new_drone.update(output_thrust, DELTA_TIME);

        // Get updated values
        curr_altitude = new_drone.getAltitude();
        curr_velocity = new_drone.getVelocity();
        curr_error = altitude_controller.getError();

        // Print information
        std::cout << "Tick: " << i << " | "
        << "Alt: " << curr_altitude << " | "
        << "Vel: " << curr_velocity << " | "
        << "Thrust: " << output_thrust << " | "
        << "Error: " <<  curr_error << " | " <<
        std::endl;    }
}