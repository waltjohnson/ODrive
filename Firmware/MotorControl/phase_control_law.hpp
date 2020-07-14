#ifndef __PHASE_CONTROL_LAW_HPP
#define __PHASE_CONTROL_LAW_HPP

#include <autogen/interfaces.hpp>
#include <variant>

template<size_t N_PHASES>
class PhaseControlLaw {
public:
    using Result = std::variant<std::array<float, N_PHASES>, ODriveIntf::MotorIntf::Error>;

    /**
     * @brief Called when this controller becomes the active controller.
     */
    virtual void reset() = 0;

    /**
     * @brief Informs the control law about a new set of measurements.
     *
     * This function gets called in a high priority interrupt context and should
     * run fast.
     *
     * Beware that all inputs can be NAN.
     *
     * @param vbus_voltage: The most recently measured DC link voltage. NAN if
     *        the measurement is not available or valid for some reason.
     * @param currents: The most recently measured (or inferred) phase currents
     *        in Amps. Any of the values can be NAN if the measurement is not
     *        available or valid for some reason.
     * @param input_timestamp: The timestamp (in HCLK ticks) corresponding to
     *        the vbus_voltage and current measurement.
     */
    virtual ODriveIntf::MotorIntf::Error on_measurement(float vbus_voltage,
        std::array<float, N_PHASES> currents, uint32_t input_timestamp) = 0;

    /**
     * @brief Shall calculate the PWM timings for the specified target time.
     *
     * This function gets called in a high priority interrupt context and should
     * run fast.
     *
     * Beware that this function can be called before a call to on_measurement().
     * 
     * @param output_timestamp: The timestamp (in HCLK ticks) corresponding to
     *        the middle of the time span during which the output will be
     *        active.
     * 
     * @returns: This function shall return an array of PWM timings, each item
     *           corresponding to one phase. Each of the PWM values must lie in
     *           0.0f...1.0f.
     *           If the function returns an error the motor gets disarmed with
     *           one exception: If the controller never returned valid PWM
     *           timings since it became active then it is allowed to return
     *           ERROR_CONTROLLER_INITIALIZING without triggering a motor disarm.
     *           In this phase the PWMs will not yet be truly active.
     */
    virtual Result get_output(uint32_t output_timestamp) = 0;
};

class AlphaBetaFrameController : public PhaseControlLaw<3> {
private:
    ODriveIntf::MotorIntf::Error on_measurement(float vbus_voltage,
        std::array<float, 3> currents, uint32_t input_timestamp) final;

    Result get_output(uint32_t output_timestamp) final;

protected:
    virtual ODriveIntf::MotorIntf::Error on_measurement(
            float vbus_voltage, float Ialpha, float Ibeta, uint32_t input_timestamp) = 0;

    virtual std::variant<std::tuple<float, float>, ODriveIntf::MotorIntf::Error> get_alpha_beta_output(
            uint32_t output_timestamp) = 0;
};

#endif // __PHASE_CONTROL_LAW_HPP