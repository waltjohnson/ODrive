
#include "foc.hpp"
#include <board.h>

Motor::Error AlphaBetaFrameController::on_measurement(
            float vbus_voltage, std::array<float, 3> currents,
            uint32_t input_timestamp) {
    // Clarke transform
    float Ialpha = currents[0];
    float Ibeta = one_by_sqrt3 * (currents[1] - currents[2]);
    return on_measurement(vbus_voltage, Ialpha, Ibeta, input_timestamp);
}

Motor::Error AlphaBetaFrameController::get_output(
            uint32_t output_timestamp, float (&pwm_timings)[3], float* ibus) {
    float mod_alpha = NAN;
    float mod_beta = NAN;

    Motor::Error status = get_alpha_beta_output(output_timestamp, &mod_alpha, &mod_beta, ibus);
    
    if (status != Motor::ERROR_NONE) {
        return status;
    } else if (std::isnan(mod_alpha) || std::isnan(mod_alpha)) {
        return Motor::ERROR_MODULATION_IS_NAN;
    } else if (SVM(mod_alpha, mod_beta, &pwm_timings[0], &pwm_timings[1], &pwm_timings[2]) != 0) {
        return Motor::ERROR_MODULATION_MAGNITUDE;
    }

    return Motor::ERROR_NONE;
}

void FieldOrientedController::reset() {
    v_current_control_integral_d_ = 0.0f;
    v_current_control_integral_q_ = 0.0f;
    mod_to_V_ = NAN;
    mod_d_ = NAN;
    mod_q_ = NAN;
    ibus_ = NAN;
}

Motor::Error FieldOrientedController::on_measurement(
            float vbus_voltage, float Ialpha, float Ibeta,
            uint32_t input_timestamp) {
    const uint32_t max_delay_us = 1000; // maximum age of the input data
    mod_d_ = NAN;
    mod_q_ = NAN;

    // Ensure that input data is not from the future and that it's not too old
    if ((int32_t)(input_timestamp - timestamp_) < 0) {
        return Motor::ERROR_BAD_TIMING;
    } else if (input_timestamp - timestamp_ > (uint32_t)((uint64_t)max_delay_us * (uint64_t)TIM_1_8_CLOCK_HZ / 1000000ULL)) {
        return Motor::ERROR_BAD_TIMING;
    }

    Ialpha_measured_ = Ialpha;
    Ibeta_measured_ = Ibeta;

    // Fetch member variables into local variables to make the optimizer's life easier.
    float Vd = Vd_setpoint_;
    float Vq = Vq_setpoint_;
    float Id_setpoint = Id_setpoint_;
    float Iq_setpoint = Iq_setpoint_;
    float phase = phase_;
    float phase_vel = phase_vel_;

    if (std::isnan(phase) || std::isnan(phase_vel)) {
        return Motor::ERROR_UNKNOWN_PHASE;
    }

    // Park transform
    float I_phase = phase + phase_vel * ((float)(input_timestamp - timestamp_) / (float)TIM_1_8_CLOCK_HZ);
    float c_I = our_arm_cos_f32(I_phase);
    float s_I = our_arm_sin_f32(I_phase);
    float Id = c_I * Ialpha + s_I * Ibeta;
    float Iq = c_I * Ibeta - s_I * Ialpha;
    Iq_measured_ += I_measured_report_filter_k_ * (Iq - Iq_measured_);
    Id_measured_ += I_measured_report_filter_k_ * (Id - Id_measured_);

    // Current error
    float Ierr_d = Id_setpoint - Id;
    float Ierr_q = Iq_setpoint - Iq;


    if (enable_current_control_) {
        // Check for current sense saturation
        if (std::isnan(Ierr_d) || std::isnan(Ierr_q)) {
            return Motor::ERROR_UNKNOWN_CURRENT;
        }

        // Apply PI control (V{d,q}_setpoint act as feed-forward terms in this mode)
        Vd += v_current_control_integral_d_ + Ierr_d * p_gain_;
        Vq += v_current_control_integral_q_ + Ierr_q * p_gain_;
    }

    if (std::isnan(vbus_voltage)) {
        return Motor::ERROR_UNKNOWN_VBUS_VOLTAGE;
    }

    mod_to_V_ = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V_;
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

    if (enable_current_control_) {
        // Vector modulation saturation, lock integrator if saturated
        // TODO make maximum modulation configurable
        float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
        if (mod_scalefactor < 1.0f) {
            mod_d *= mod_scalefactor;
            mod_q *= mod_scalefactor;
            // TODO make decayfactor configurable
            v_current_control_integral_d_ *= 0.99f;
            v_current_control_integral_q_ *= 0.99f;
        } else {
            v_current_control_integral_d_ += Ierr_d * (i_gain_ * current_meas_period);
            v_current_control_integral_q_ += Ierr_q * (i_gain_ * current_meas_period);
        }
    }

    mod_d_ = mod_d;
    mod_q_ = mod_q;
    ibus_ = mod_d * Id + mod_q * Iq;
    return Motor::ERROR_NONE;
}

ODriveIntf::MotorIntf::Error FieldOrientedController::get_alpha_beta_output(
        uint32_t output_timestamp, float* mod_alpha, float* mod_beta, float* ibus) {
    if (std::isnan(mod_d_) || std::isnan(mod_q_)) {
        return Motor::ERROR_CONTROLLER_INITIALIZING;
    }

    // Inverse park transform
    float pwm_phase = phase_ + phase_vel_ * ((float)(output_timestamp - timestamp_) / (float)TIM_1_8_CLOCK_HZ);
    float c_p = our_arm_cos_f32(pwm_phase);
    float s_p = our_arm_sin_f32(pwm_phase);
    float mod_alpha_temp = c_p * mod_d_ - s_p * mod_q_;
    float mod_beta_temp = c_p * mod_q_ + s_p * mod_d_;

    // Report final applied voltage in stationary frame (for sensorless estimator)
    final_v_alpha_ = mod_to_V_ * mod_alpha_temp;
    final_v_beta_ = mod_to_V_ * mod_beta_temp;

    *mod_alpha = mod_alpha_temp;
    *mod_beta = mod_beta_temp;
    *ibus = ibus_;
    return Motor::ERROR_NONE;
}

void FieldOrientedController::update(uint32_t timestamp) {
    CRITICAL_SECTION() {
        timestamp_ = timestamp;
        enable_current_control_ = enable_current_control_src_;
        Id_setpoint_ = Id_setpoint_src_ ? *Id_setpoint_src_ : NAN;
        Iq_setpoint_ = Iq_setpoint_src_ ? *Iq_setpoint_src_ : NAN;
        Vd_setpoint_ = Vd_setpoint_src_ ? *Vd_setpoint_src_ : NAN;
        Vq_setpoint_ = Vq_setpoint_src_ ? *Vq_setpoint_src_ : NAN;
        phase_ = phase_src_ ? *phase_src_ : NAN;
        phase_vel_ = phase_vel_src_ ? *phase_vel_src_ : NAN;
    }
}
