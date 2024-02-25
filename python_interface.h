extern "C" {
        #include "vehicle_and_lowlevel_control.h"

        void* CarModel_new();
        void CarModel_initialize(void* instance);
        void CarModel_delete(void* instance);

        void CarModel_step(void* instance,
                       real_T arg_tuning_param_key, real_T arg_tuning_param_value, real_T
              arg_Command_throttle, real_T arg_Command_steering, real_T
              arg_a_x_target0, real_T arg_a_x_target1, real_T arg_a_x_target2,
              real_T arg_delta_target0, real_T arg_delta_target1, real_T
              arg_delta_target2, real_T arg_yawrate_target0, real_T
              arg_yawrate_target1, real_T arg_yawrate_target2, real_T
              arg_vx_mpc0, real_T arg_vx_mpc1, real_T arg_vx_mpc2, real_T
              arg_vy_mpc0, real_T arg_vy_mpc1, real_T arg_vy_mpc2, real_T
              arg_F_x_target_FL_0, real_T arg_F_x_target_FL_1, real_T
              arg_F_x_target_FL_2, real_T arg_F_x_target_FR_0, real_T
              arg_F_x_target_FR_1, real_T arg_F_x_target_FR_2, real_T
              arg_F_x_target_RL_0, real_T arg_F_x_target_RL_1, real_T
              arg_F_x_target_RL_2, real_T arg_F_x_target_RR_0, real_T
              arg_F_x_target_RR_1, real_T arg_F_x_target_RR_2, real_T &arg_x,
              real_T &arg_y, real_T &arg_phi, real_T &arg_VE_vx, real_T
              &arg_VE_vy, real_T &arg_VE_r, real_T &arg_VE_ax, real_T &arg_VE_ay,
              real_T (&arg_Fz)[4], real_T &arg_steering_rad, real_T
              &arg_steering_rate, real_T &arg_steering_torque, real_T
              (&arg_wheel_speed)[4], real_T (&arg_wheel_speed_max)[4], real_T
              (&arg_wheel_speed_min)[4], real_T (&arg_wheel_torque)[4], real_T (
               &arg_wheel_torque_target)[4], real_T &arg_DV_DC, real_T
              &arg_DV_delta, real_T &arg_yaw_moment_target_);
}
