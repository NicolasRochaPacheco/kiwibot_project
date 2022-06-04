/*! @package pid_controller
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "motion_control/pid_controller.hpp"

PIDController::PIDController() {}

float PIDController::ThrottlePID(float ref_vx, float cur_vx, double dt)
{
    /********************************************
     * DEFINE YOUR AMAZING PID CONTROLLER
     * Find Documentation here:
     * https://www.elprocus.com/the-working-of-a-pid-controller/
     ********************************************/

    // Check if the controller is used or not
    if (m_throttle_ctrl) {
        // Check if reference velocity is zero
        if (ref_vx == 0) {
            // Reset the integral error
            m_vx_int_error = 0.0;
            // Return zero
            return 0.0;
        }
        // Define the error variable
        float vx_err = ref_vx - cur_vx;
        // Update integral error
        m_vx_int_error += vx_err;
        // Calculate the differential error
        float diff_err = (vx_err - m_vx_prop_ek1) / dt;
        // Update the previous error value
        m_vx_prop_ek1 = vx_err;
        // Calculate the control signal value
        float throttle_pid_sig = m_kp_thr * vx_err + m_ki_thr * m_vx_int_error + m_kd_thr * diff_err;
        // Check if the control signal value is higher than maximum linear speed
        if (throttle_pid_sig > m_max_linear_spd){
            // Return maximum linear speed in control signal
            return m_max_linear_spd;
        } else {
            // Return the control value
            return throttle_pid_sig;
        }

    }
    // Return reference value if controller is not used
    else {return ref_vx;}

    /********************************************
     * END CODE
     *********************************************/
}

float PIDController::SteeringPID(float ref_wz, float cur_wz, double dt)
{
    /********************************************
     * DEFINE YOUR AMAZING PID CONTROLLER
     * Find Documentation here:
     * https://www.elprocus.com/the-working-of-a-pid-controller/
     *
     * FeedForward:
     * https://zhuanlan.zhihu.com/p/382010500#:~:text=In%20many%20applications,dynamic%20models%20used.
     * "Combined FeedForward and Feedback Control"
     ********************************************/

    // Check if the controller is used or not
    if (m_steering_ctrl) {
        // Check if the velocity is zero
        if (ref_wz == 0.0) {
            // Reset the integral error
            m_wz_int_error = 0.0;
            // Return zero
            return 0.0;
        }
        // Define the error variable
        float wz_err = ref_wz - cur_wz;
        // Update the integral error
        m_wz_int_error += wz_err;
        // Calculate the differential error
        float wz_diff_err = (wz_err - m_wz_prop_ek1) / dt;
        // Update the previous error
        m_wz_prop_ek1 = wz_err;
        // Calculate the PID control signal
        float wz_pid_sig = m_kp_str * wz_err + m_kd_str * wz_diff_err + m_ki_str * m_wz_int_error;
        // Calculate the FF control signal
        float wz_ff_sig = m_kff_str * ref_wz; // TODO: Not sure of this part.
        // Return the sum of PID and FF control signals
        return wz_pid_sig + wz_ff_sig;
    }
    // If controller is not used, return reference value
    else{return ref_wz;}

    /********************************************
     * END CODE
     *  ********************************************/
}
