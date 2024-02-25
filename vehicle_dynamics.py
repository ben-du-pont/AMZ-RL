import ctypes
from ctypes import c_double, c_int, Structure
import os
import numpy as np
# Define realT type used by the shared library
real_T = ctypes.c_double

class Vehicle:

    def __init__(self):
        
        # Add the current directory to LD_LIBRARY_PATH
        current_dir = os.getcwd()
        current_ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
        new_ld_library_path = os.path.abspath('.') + ':' + current_ld_library_path
        os.environ['LD_LIBRARY_PATH'] = new_ld_library_path

        # Construct the full path to the shared library
        libvehicle_path = os.path.join(current_dir, 'libvehicle_and_lowlevel_control.so')

        # Load the shared library
        self.lib = ctypes.CDLL(libvehicle_path)



        # Define CarModel model class
        CarModel_new = self.lib.CarModel_new
        CarModel_new.restype = ctypes.c_void_p

        # Define the initialize method of the CarModel class
        CarModel_initialize = self.lib.CarModel_initialize
        CarModel_initialize.argtypes = [ctypes.c_void_p]

        # Define the destructor of the CarModel class
        self.CarModel_delete = self.lib.CarModel_delete
        self.CarModel_delete.argtypes = [ctypes.c_void_p]
        
        # Create an instance of the car model and initialize it
        self.car_model_instance = CarModel_new()
        CarModel_initialize(self.car_model_instance)

        # Set state to 0
        self.state = {
            'x': 0.0, 'y': 0.0, 'yaw': 0.0,
            'v_x': 0.0, 'v_y': 0.0, 'r': 0.0,
            'a_x': 0.0, 'a_y': 0.0, 'steer': 0.0,
        }

        # Define the variables to be used in the step function
        self.arg_x = real_T()
        self.arg_y = real_T()
        self.arg_phi = real_T()

        self.arg_VE_vx, self.arg_VE_vy, self.arg_VE_r  = real_T(), real_T(), real_T()
        self.arg_VE_ax, self.arg_VE_ay = real_T(), real_T()

        self.arg_steering_rad, self.arg_steering_rate, self.arg_steering_torque = real_T(), real_T(), real_T()

        self.arg_DV_DC, self.arg_DV_delta, self.arg_yaw_moment_target = real_T(), real_T(), real_T()

        # Define the time step
        dt = 0.5

        self.step_func = self.lib.CarModel_step
        self.step_func.argtypes = [
            ctypes.c_void_p,  # instance pointer
            real_T, real_T,  # tuning_param_key, tuning_param_value
            real_T, real_T,  # Command_throttle, Command_steering
            real_T, real_T, real_T,  # a_x_target0, a_x_target1, a_x_target2
            real_T, real_T, real_T,  # delta_target0, delta_target1, delta_target2
            real_T, real_T, real_T,  # yawrate_target0, yawrate_target1, yawrate_target2
            real_T, real_T, real_T,  # vx_mpc0, vx_mpc1, vx_mpc2
            real_T, real_T, real_T,  # vy_mpc0, vy_mpc1, vy_mpc2
            real_T, real_T, real_T,  # F_x_target_FL_0, F_x_target_FL_1, F_x_target_FL_2
            real_T, real_T, real_T,  # F_x_target_FR_0, F_x_target_FR_1, F_x_target_FR_2
            real_T, real_T, real_T,  # F_x_target_RL_0, F_x_target_RL_1, F_x_target_RL_2
            real_T, real_T, real_T,  # F_x_target_RR_0, F_x_target_RR_1, F_x_target_RR_2
            ctypes.POINTER(real_T), ctypes.POINTER(real_T), ctypes.POINTER(real_T),  # &arg_x, &arg_y, &arg_phi
            ctypes.POINTER(real_T), ctypes.POINTER(real_T), ctypes.POINTER(real_T),  # &arg_VE_vx, &arg_VE_vy, &arg_VE_r
            ctypes.POINTER(real_T), ctypes.POINTER(real_T),  # &arg_VE_ax, &arg_VE_ay
            (real_T * 4),  # arg_Fz
            ctypes.POINTER(real_T), ctypes.POINTER(real_T), ctypes.POINTER(real_T),  # &arg_steering_rad, &arg_steering_rate, &arg_steering_torque
            (real_T * 4), (real_T * 4), (real_T * 4),  # arg_wheel_speed, arg_wheel_speed_max, arg_wheel_speed_min
            (real_T * 4), (real_T * 4),  # arg_wheel_torque, arg_wheel_torque_target
            ctypes.POINTER(real_T), ctypes.POINTER(real_T), ctypes.POINTER(real_T),  # &arg_DV_DC, &arg_DV_delta, &arg_yaw_moment_target
        ]

        self.step_func.restype = None 


    def update_dynamics(self, dt, input_command):
        
        throttle_input = input_command['throttle']
        steering_input = input_command['steering']

        for _ in range(int(dt/0.001)):
            self.step_func(self.car_model_instance,
                    real_T(0), real_T(0),  # example values for the first two parameters
                    real_T(0), real_T(steering_input),  # example values for the first two parameters
                    real_T(throttle_input), real_T(throttle_input),real_T(throttle_input),
                    real_T(0), real_T(0),real_T(0),
                    real_T(0), real_T(0),real_T(0), 
                    real_T(0), real_T(0),real_T(0),
                    real_T(0), real_T(0),real_T(0),
                    real_T(0), real_T(0),real_T(0),
                    real_T(0), real_T(0),real_T(0),
                    real_T(0), real_T(0),real_T(0),
                    real_T(0), real_T(0),real_T(0),
                    ctypes.byref(self.arg_x), ctypes.byref(self.arg_y), ctypes.byref(self.arg_phi),
                    ctypes.byref(self.arg_VE_vx), ctypes.byref(self.arg_VE_vy), ctypes.byref(self.arg_VE_r),
                    ctypes.byref(self.arg_VE_ax), ctypes.byref(self.arg_VE_ay),
                    (real_T * 4)(10.0, 10.0, 10.0, 10.0),
                    ctypes.byref(self.arg_steering_rad), ctypes.byref(self.arg_steering_rate), ctypes.byref(self.arg_steering_torque),
                    (real_T * 4)(0.0, 0.0, 0.0, 0.0), (real_T * 4)(0.0, 0.0, 0.0, 0.0), (real_T * 4)(0.0, 0.0, 0.0, 0.0),
                    (real_T * 4)(0.0, 0.0, 0.0, 0.0), (real_T * 4)(0.0, 0.0, 0.0, 0.0), (real_T * 4)(0.0, 0.0, 0.0, 0.0),
                    ctypes.byref(self.arg_DV_DC), ctypes.byref(self.arg_DV_delta), ctypes.byref(self.arg_yaw_moment_target)
                    )
        self.get_state()

    def get_state(self):
        self.state['x'] = self.arg_x.value
        self.state['y'] = self.arg_y.value
        self.state['yaw'] = self.arg_phi.value
        self.state['v_x'] = self.arg_VE_vx.value
        self.state['v_y'] = self.arg_VE_vy.value
        self.state['r'] = self.arg_VE_r.value
        self.state['a_x'] = self.arg_VE_ax.value
        self.state['a_y'] = self.arg_VE_ay.value
        self.state['steer'] = self.arg_steering_rad.value

        return self.state
    
    def reset(self, seed=None, options=None):
        self.state['x'] = 0
        self.state['y'] = 0
        self.state['yaw'] = 0
        self.state['v_x'] = 0
        self.state['v_y'] = 0
        self.state['r'] = 0
        self.state['a_x'] = 0
        self.state['a_y'] = 0
        self.state['steer'] = 0


        self.arg_x = real_T()
        self.arg_y = real_T()
        self.arg_phi = real_T()

        self.arg_VE_vx, self.arg_VE_vy, self.arg_VE_r  = real_T(), real_T(), real_T()
        self.arg_VE_ax, self.arg_VE_ay = real_T(), real_T()

        self.arg_steering_rad, self.arg_steering_rate, self.arg_steering_torque = real_T(), real_T(), real_T()

        self.arg_DV_DC, self.arg_DV_delta, self.arg_yaw_moment_target = real_T(), real_T(), real_T()

        observation = np.array([self.arg_VE_vx.value,self.arg_VE_vy.value], dtype=np.float32)  # Example observation for now

        return observation, {}  # Return an empty dict - no additional info

    def Destructor(self):
        self.CarModel_delete(self.car_model_instance)