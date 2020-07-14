#ifndef __MOTOR_HPP
#define __MOTOR_HPP

class Axis; // declared in axis.hpp
class Motor;

#include <board.h>
#include <autogen/interfaces.hpp>
#include "foc.hpp"

enum TimingLog_t {
    TIMING_LOG_UPDATE_START,
    TIMING_LOG_CURRENT_MEAS,
    TIMING_LOG_DC_CAL,
    TIMING_LOG_CTRL_DONE,
    TIMING_LOG_NUM_SLOTS
};

class Motor : public ODriveIntf::MotorIntf {
public:
    struct Iph_ABC_t {
        float phA;
        float phB;
        float phC;
    };

    // NOTE: for gimbal motors, all units of Nm are instead V.
    // example: vel_gain is [V/(turn/s)] instead of [Nm/(turn/s)]
    // example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
    struct Config_t {
        bool pre_calibrated = false; // can be set to true to indicate that all values here are valid
        int32_t pole_pairs = 7;
        float calibration_current = 10.0f;    // [A]
        float resistance_calib_max_voltage = 2.0f; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
        float phase_inductance = 0.0f;        // to be set by measure_phase_inductance
        float phase_resistance = 0.0f;        // to be set by measure_phase_resistance
        float torque_constant = 0.04f;         // [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor
        MotorType motor_type = MOTOR_TYPE_HIGH_CURRENT;
        // Read out max_allowed_current to see max supported value for current_lim.
        // float current_lim = 70.0f; //[A]
        float current_lim = 10.0f;          //[A]
        float current_lim_margin = 8.0f;    // Maximum violation of current_lim
        float torque_lim = std::numeric_limits<float>::infinity();           //[Nm]. 
        // Value used to compute shunt amplifier gains
        float requested_current_range = 60.0f; // [A]
        float current_control_bandwidth = 1000.0f;  // [rad/s]
        float inverter_temp_limit_lower = 100;
        float inverter_temp_limit_upper = 120;
        bool R_wL_FF_enable = false; // Enable feedforwards for R*I and w*L*I terms
        bool bEMF_FF_enable = false; // Enable feedforward for bEMF

        float I_bus_hard_min = -INFINITY;
        float I_bus_hard_max = INFINITY;
        float I_leak_max = 0.1f;

        float dc_calib_tau = 0.2f;

        // custom property setters
        Motor* parent = nullptr;
        void set_pre_calibrated(bool value) {
            pre_calibrated = value;
            parent->is_calibrated_ = parent->is_calibrated_ || parent->config_.pre_calibrated;
        }
        void set_phase_inductance(float value) { phase_inductance = value; parent->update_current_controller_gains(); }
        void set_phase_resistance(float value) { phase_resistance = value; parent->update_current_controller_gains(); }
        void set_current_control_bandwidth(float value) { current_control_bandwidth = value; parent->update_current_controller_gains(); }
    };

    Motor(TIM_HandleTypeDef* timer,
         uint8_t current_sensor_mask,
         float shunt_conductance,
         TGateDriver& gate_driver,
         TOpAmp& opamp);

    bool arm(PhaseControlLaw<3>* control_law);
    void apply_pwm_timings(uint16_t timings[3], bool tentative);
    bool disarm(bool* was_armed = nullptr);
    bool apply_config();
    bool setup();

    void update_current_controller_gains();
    void disarm_with_error(Error error);
    bool do_checks(uint32_t timestamp);
    float effective_current_lim();
    float max_available_torque();
    void log_timing(TimingLog_t log_idx);
    float phase_current_from_adcval(uint32_t ADCValue);
    bool measure_phase_resistance(float test_current, float max_voltage);
    bool measure_phase_inductance(float test_voltage);
    bool run_calibration();
    void tim_update_cb(uint32_t adc_a, uint32_t adc_b, uint32_t adc_c);

    // hardware config
    TIM_HandleTypeDef* const timer_;
    const uint8_t current_sensor_mask_;
    const float shunt_conductance_;
    TGateDriver& gate_driver_;
    TOpAmp& opamp_;

    Config_t config_;
    Axis* axis_ = nullptr; // set by Axis constructor

//private:

    uint32_t last_update_timestamp_ = 0;
    bool counting_down_ = false;
    uint16_t last_cpu_time_ = 0;
    int timing_log_index_ = 0;
    struct {
        uint16_t& operator[](size_t idx) { return content[idx]; }
        uint16_t& get(size_t idx) { return content[idx]; }
        uint16_t content[TIMING_LOG_NUM_SLOTS];
    } timing_log_;

    // variables exposed on protocol
    Error error_ = ERROR_NONE;
    // Do not write to this variable directly!
    // It is for exclusive use by the safety_critical_... functions.
    bool is_armed_ = false;
    bool is_calibrated_ = config_.pre_calibrated;
    Iph_ABC_t current_meas_ = {NAN, NAN, NAN};
    Iph_ABC_t DC_calib_ = {0.0f, 0.0f, 0.0f};
    float dc_calib_running_since_ = 0.0f; // current sensor calibration needs some time to settle
    float I_leak_ = NAN; // close to zero if only two current sensors are available
    float I_bus_ = 0.0f; // this motors contribution to the bus current
    bool current_meas_valid_ = false; // if false, the measured current values must not be used for control
    float phase_current_rev_gain_ = 0.0f; // Reverse gain for ADC to Amps (to be set by DRV8301_setup)
    FieldOrientedController current_control_;
    float effective_current_lim_ = 10.0f; // [A]
    float max_allowed_current_ = 0.0f; // [A] set in setup()
    float max_dc_calib_ = 0.0f; // [A] set in setup()
    
    PhaseControlLaw<3>* control_law_;
};


#endif // __MOTOR_HPP
