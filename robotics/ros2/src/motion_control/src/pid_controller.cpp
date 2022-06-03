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
        // Return the control value
        return m_kp_thr * vx_err + m_ki_thr * m_vx_int_error + m_kd_thr * diff_err;
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



    /********************************************
     * END CODE
     *  ********************************************/
}
