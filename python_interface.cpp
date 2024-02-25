extern "C" {
    #include "python_interface.h"
        void* CarModel_new() {
                return new simulink_codegen::CarModel();
        }

        void CarModel_initialize(void* instance) {
                static_cast<simulink_codegen::CarModel*>(instance)->initialize();
        }

        void CarModel_delete(void* instance) {
                delete static_cast<simulink_codegen::CarModel*>(instance);
        }

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
              &arg_DV_delta, real_T &arg_yaw_moment_target_) {
                        // Cast the void pointer back to the correct type
                        simulink_codegen::CarModel* carModel = static_cast<simulink_codegen::CarModel*>(instance);

                        // Call the member function
                        carModel->step(arg_tuning_param_key, arg_tuning_param_value,
              arg_Command_throttle, arg_Command_steering,
              arg_a_x_target0, arg_a_x_target1, arg_a_x_target2,
              arg_delta_target0, arg_delta_target1,
              arg_delta_target2, arg_yawrate_target0,
              arg_yawrate_target1, arg_yawrate_target2,
              arg_vx_mpc0, arg_vx_mpc1, arg_vx_mpc2,
              arg_vy_mpc0, arg_vy_mpc1, arg_vy_mpc2,
              arg_F_x_target_FL_0, arg_F_x_target_FL_1,
              arg_F_x_target_FL_2, arg_F_x_target_FR_0,
              arg_F_x_target_FR_1, arg_F_x_target_FR_2,
              arg_F_x_target_RL_0, arg_F_x_target_RL_1,
              arg_F_x_target_RL_2, arg_F_x_target_RR_0,
              arg_F_x_target_RR_1, arg_F_x_target_RR_2, arg_x,
              arg_y, arg_phi, arg_VE_vx,
              arg_VE_vy, arg_VE_r, arg_VE_ax, arg_VE_ay,
              (arg_Fz), arg_steering_rad,
              arg_steering_rate, arg_steering_torque,
              (arg_wheel_speed), (arg_wheel_speed_max),
              (arg_wheel_speed_min), (arg_wheel_torque), (
               arg_wheel_torque_target), arg_DV_DC,
              arg_DV_delta, arg_yaw_moment_target_);
        }
}
