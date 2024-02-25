import numpy as np
import matplotlib.pyplot as plt
# import pandas as pd
from dataclasses import dataclass

from math import radians
# IMPORTANT: NEED TO INSTALL ROS RO BE ABLE TO READ THE SIMULINK MODEL FOR NOW !
# sudo apt-get update
# sudo apt-get install libroscpp-dev

# Need access to the lib files :
# export LD_LIBRARY_PATH=~/Documents/AMZ/AMZ-RL/:$LD_LIBRARY_PATH

from shapely import LineString
from shapely.geometry import Polygon
from shapely.geometry import Point

from vehicle_dynamics import Vehicle
from path import Path
from controller import PurePursuitController, PIDController


import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, PillowWriter



        
def main():
    path = Path()
    vehicle = Vehicle()
    dt = 0.1  # Time step

    lookahead_distance = 5.0
    pp_controller = PurePursuitController(2.0, lookahead_distance)
    pid_controller = PIDController(2.0, 0.1, 0.1, dt)

    

    fig, ax = plt.subplots()
    ax.axis('equal')
    ax.set_xlim(-10, 50)
    ax.set_ylim(-20, 20)

    # Define car dimensions
    car_length = 2.0
    car_width = 1.0
    wheel_length = 0.4
    wheel_width = 0.15

    # Initialize car body and wheels
    car_body = patches.Rectangle((car_length/2, car_width/2), car_length, car_width, color='blue', angle=np.degrees(vehicle.state['yaw']))
    ax.add_patch(car_body)

    ax.plot(path.px, path.py, color='black', linewidth=0.2)

    # add recessed line
    x_recessed, y_recessed = path.track_params[5].xy
    ax.plot(x_recessed, y_recessed, color='green', linewidth=0.2)

    #add recessed line2
    x_recessed2, y_recessed2 = path.track_params[6].xy
    ax.plot(x_recessed2, y_recessed2, color='green', linewidth=0.2)

    # add self.blue_line
    x_blue, y_blue = path.track_params[1].xy
    ax.plot(x_blue, y_blue, color='blue')
    x_yellow, y_yellow = path.track_params[2].xy
    ax.plot(x_yellow, y_yellow, color='yellow')
    # plot path.start_finish_line
    ax.plot(path.track_params[4][:, 0], path.track_params[4][:, 1], color='orange')

    wheels = []
    for _ in range(4):
        wheel = patches.Rectangle((0, 0), wheel_length, wheel_width, color='black')
        ax.add_patch(wheel)
        wheels.append(wheel)
    # Initialize text for speed and steering angle
    speed_text = ax.text(0.5, 0.95, '', transform=ax.transAxes, ha='center')
    steering_text = ax.text(0.5, 0.90, '', transform=ax.transAxes, ha='center')

    waypoint, = ax.plot([], [], 'rx')
    waypoint_ahead, = ax.plot([], [], 'bo')

    def animate(i):
        current_time = i * dt

        state = vehicle.get_state()

        waypoint_idx = path.get_next_waypoint(state['x'], state['y'], path.track_params[3], path.track_params[0],path.track_params[1],path.track_params[2])
        waypoint_x, waypoint_y = path.track_params[0][path.track_params[0]['index'] == waypoint_idx]['x'][0], path.track_params[0][path.track_params[0]['index'] == waypoint_idx]['y'][0]
        waypoint.set_data((waypoint_x, waypoint_y))

        
        waypoint_ahead_idx = path.get_a_waypoint(state['x'], state['y'], path.track_params[3], path.track_params[0],path.track_params[1],path.track_params[2], lookahead_distance)
        waypoint_ahead_x, waypoint_ahead_y = path.track_params[0][path.track_params[0]['index'] == waypoint_ahead_idx]['x'][0], path.track_params[0][path.track_params[0]['index'] == waypoint_ahead_idx]['y'][0]
        waypoint_ahead.set_data((waypoint_ahead_x, waypoint_ahead_y))

        steering_input = 0.45 * np.sin(2 * np.pi * current_time *5)

        steering_input = pp_controller.calculate_steering_angle(vehicle.state['x'], vehicle.state['y'], vehicle.state['yaw'], (waypoint_ahead_x, waypoint_ahead_y))
        throttle_input = pid_controller.control(5, state['v_x'])          

        input_command = {'throttle': throttle_input, 'steering': steering_input}


        vehicle.update_dynamics(dt, input_command)
        state = vehicle.get_state()

        

        # Update car body
        car_center_x = state['x'] - car_length / 2 - car_length / 2 * np.cos(state['yaw'])
        car_center_y = state['y'] - car_width / 2 - car_width / 2 * np.sin(state['yaw'])
        corner_x = state['x'] - (car_length / 2 * np.cos(state['yaw']) - car_width / 2 * np.sin(state['yaw']))
        corner_y = state['y']  - (car_length / 2 * np.sin(state['yaw']) + car_width / 2 * np.cos(state['yaw']))

        car_body.set_xy((corner_x, corner_y))
        car_body.angle = np.degrees(state['yaw'])

        # Update speed and steering angle text
        speed_text.set_text(f'Speed: {state["v_x"]:.2f} m/s')
        steering_text.set_text(f'Steering: {state["steer"]:.2f}rad')

        # Set plot limits to follow the car
        view_margin = 10  # Margin around the car for the plot view
        ax.set_xlim(state['x'] - view_margin, state['x'] + view_margin)
        ax.set_ylim(state['y'] - view_margin, state['y'] + view_margin)
        # Activate the grid
        ax.grid(True)

        # Positions of the wheels relative to the car
        wheel_positions = [(car_length * 0.25 - wheel_length*0.5, car_width * 0.5 + car_width * 0.5 - wheel_width*0.5- car_width / 2, np.degrees(state['yaw'] + state['steer'])), 
                        (car_length * 0.25 - wheel_length*0.5, car_width * 0.5 -car_width * 0.5 - wheel_width*0.5- car_width / 2, np.degrees(state['yaw'] + state['steer'])),
                        (-car_length * 0.25 - wheel_length*0.5, car_width * 0.5 + car_width * 0.5 - wheel_width*0.5- car_width / 2, np.degrees(state['yaw'])),
                        (-car_length * 0.25 - wheel_length*0.5, car_width * 0.5 -car_width * 0.5 - wheel_width*0.5- car_width / 2, np.degrees(state['yaw']))]

        for wheel, (x_offset, y_offset, angle) in zip(wheels, wheel_positions):
            # Calculate the global position of each wheel
            wheel_x = state['x'] + x_offset * np.cos(state['yaw']) - y_offset * np.sin(state['yaw'])
            wheel_y = state['y'] + x_offset * np.sin(state['yaw']) + y_offset * np.cos(state['yaw'])
            wheel.set_xy((wheel_x, wheel_y))
            wheel.angle = angle

    # Creating the animation
    anim = FuncAnimation(fig, animate, frames=np.arange(0, 50, dt), interval=dt*1)

    plt.show()

    f = r"./animation.gif" 
    writergif = PillowWriter(fps=30) 
    anim.save(f, writer=writergif)


if __name__ == '__main__':
    main()




