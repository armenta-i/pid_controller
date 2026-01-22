#ifndef DRONE_SIMULATOR_H
#define DRONE_SIMULATOR_H

typedef float float32_t;

/**
 * Drone Class Header
 */
class Drone {
    public:
        /**
         * This enum will be used as a return value for functions to signal what happened
         * when the functions was called.
         */
        enum class Status {
            SUCCESS,
            ERROR_INVALID_INPUT,
            ERROR_MATH_FAILURE 
        };

        /**
         * @brief Constructor class for Drone.
         * @param altitude_m Current altitude of drone.
         * @param velocity_mps Current velocity of drone.
         */
        explicit Drone(float32_t altitude_m, float32_t velocity_mps);

        /**
         * @brief Will update the values of the drone and calculate the new_velocity of it.
         * @param thrust Value obtained from pid, indicates how much to add to drone.
         * @param delta_time Value to calculate the new values for drone.
         */
        Status update(float32_t thrust, float32_t delta_time);
        
        /**
         * @brief Gets the altitude value from the object
         */
        float32_t getAltitude() const;

        /**
         * @brief Gets the velocity value from the object
         */
        float32_t getVelocity() const;

    private:
        float32_t altitude_m_;
        float32_t velocity_mps_;
        static constexpr float32_t GRAVITY = 9.81f;
        static constexpr float32_t MAX_THRUST_ACCEL = 20.0f;
};

#endif