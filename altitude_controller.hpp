#ifndef ALTITUDE_CONTROLLER_H
#define ALTITUDE_CONTROLLER_H
typedef float float32_t;

// Error = Target Altitude - Current Altitude
// Proportional (P): Error * K_p
// Integral (I): Total Error over Time * K_i
// Derivative (D): (Current error - Prev error) * K_d

/**
 * Altitude Controller Class Skeleton
 */
class AltitudeController
{
    public:
    /**
     * An enum type
     * This enum will be used as the return type for updateAltitude to show what the outcome of the variable is
     */
    // We use class to force Status::enum
    enum class Status {
        SUCCESS, /** */
        ERROR_INVALID_INPUT,
        ERROR_MATH_FAILURE
    };

    /**
     * @brief Constructor for AltitudeController class.
     * @param proportional_gain The calculate proportional gain value.
     * @param integral_gain The calculate integral gain value.
     * @param derivative_gain The calculate derivative gain value.
     */
    explicit AltitudeController(float32_t proportional_gain, float32_t integral_gain, float32_t derivative_gain);
    
    // Function that takes target_altitude & current_alitude
    // Need reference to output_trust to send back result, as we are only returning enum.
    /**
     * @brief Calculates motor thrust based on altitude error.
     * @param target_altitude The desired height in meters.
     * @param current_altitude The current sensor reading in meters.
     * @param output_thrust [out] The calculated thrust value (0.0 to 1.0).
     * @return Status::SUCCESS if calculation is valid.
     */
    Status updateAltitude(const float32_t target_altitude, float32_t current_altitude, float32_t &output_thrust);
    float32_t getError();

    private:
        float32_t proportional_gain_;
        float32_t integral_gain_;
        float32_t derivative_gain_;
        float32_t error_;
        float32_t accumulated_error_;
        float32_t previous_error_;
};

#endif