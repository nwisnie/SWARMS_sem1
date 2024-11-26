import airsim
import airsimneurips
import time
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

#connect to server with drone controls
client = airsimneurips.MultirotorClient()
client.confirmConnection()

"""
To load the level first comment out the code below and run the exe file
Once the level is loaded comment out the simLoadLevel code and uncomment your code
Run the python script again and you should see the drone takeoff
"""

client.simLoadLevel('Soccer_Field_Easy')

client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")

# # Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name="drone_1").join()

moveList = [
            airsim.Vector3r(1,10.8,2),
            airsim.Vector3r(1,10.8,2),
            airsim.Vector3r(9.9,19.5,2),
            airsim.Vector3r(18.7,22.2,2),
            airsim.Vector3r(30,22,2),
            airsim.Vector3r(39,19.2,2),
            airsim.Vector3r(45.7,11.7,2),
            airsim.Vector3r(43.7,2.2,2),
            airsim.Vector3r(40.3,-4.8,2),
            airsim.Vector3r(30.7,-6.9,2),
            airsim.Vector3r(18.5,-7.9,2),
            airsim.Vector3r(9.5,-5.1,2),  
            airsim.Vector3r(0,-2,2)
            ]

# // sphere approach //

# curPos: position of drone (Vector3r)
# target: position of destination (Vector3r)
# radius: radius of circle (int)
def inRadius(curPos, target, radius):
    distance = ((target.x_val-curPos.x_val)**2 + (target.y_val-curPos.y_val)**2)**0.5
    return distance <= radius

client.simStartRace()

client.moveToPositionAsync(moveList[0].x_val, moveList[0].y_val, moveList[0].z_val, 9)

# velocities for kp graphs:
vel_num = 0
vel_list = []
pos_list = []
gate_vel_list = []

for i in range(len(moveList)-1):
    while True:
        curPos = client.getMultirotorState().kinematics_estimated.position
        curVel = client.getMultirotorState().kinematics_estimated.linear_velocity
        vel_num += 1
        vel_list.append([vel_num, (curVel.x_val**2+curVel.y_val**2)**0.5])
        pos_list.append([curPos.x_val,curPos.y_val])
        if inRadius(curPos, moveList[i], 4):
            gate_vel_list.append([vel_num, (curVel.x_val**2+curVel.y_val**2+curVel.z_val**2)**0.5])
            print(f"at gate {i}")
            break
        time.sleep(0.1) # sleep to not tank framerate

    client.moveToPositionAsync(moveList[i+1].x_val, moveList[i+1].y_val, moveList[i+1].z_val, 9)


# plot for kp

# putting values in np arrays for ease of use
plot_time, plot_vel = zip(*vel_list)
plot_time = np.array(plot_time)
plot_vel = np.array(plot_vel)
ideal_vel = np.array([5]*len(plot_time))

plot_xpos, plot_ypos = zip(*pos_list)
plot_xpos = np.array(plot_xpos)
plot_ypos = np.array(plot_ypos)

# base velocity plot
plt.figure()
plt.plot(plot_time,plot_vel)
plt.plot(plot_time,ideal_vel)
for i in gate_vel_list:
    plt.plot(i[0], i[1], marker='o', color='y', markersize=5) 
plt.xlabel('Time')
plt.ylabel('Vel')
plt.title('Vel Plot (only X and Y)')
plt.grid(True)
plt.show()

# base distance plot
plt.figure()
for i in moveList:
    plt.plot(i.x_val, i.y_val, marker='o', color='b', markersize=20)
plt.plot(plot_xpos,plot_ypos,color="y") 
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Plot of Position Relative to Gates')
plt.grid(True)
plt.show()





# old spline approach

# waypoints = []
# for i in moveList:
#     waypoints.append([i.x_val,i.y_val,i.z_val])

# waypoints = np.array(waypoints)
# t = np.linspace(0, 1, len(waypoints))

# spline_x = CubicSpline(t, waypoints[:, 0])
# spline_y = CubicSpline(t, waypoints[:, 1])
# spline_z = CubicSpline(t, waypoints[:, 2])

# num_points = 100  
# t_new = np.linspace(0, 1, num_points)

# path_x = spline_x(t_new)
# path_y = spline_y(t_new)
# path_z = spline_z(t_new)
# path = np.stack((path_x, path_y, path_z), axis=-1)

# moveList = []
# for i in path:
#     moveList.append(airsim.Vector3r(i[0],i[1],i[2]))
#     print([i[0],i[1],i[2]])

# print(path)

# original gains
# gains.kp_cross_track = 6.0  # cross-track error
# gains.kp_vel_cross_track = 0.0  # cross-track velocity error
# gains.kp_along_track = 0.0  # along-track error
# gains.kp_vel_along_track = 0.04  # along-track velocity error
# gains.kp_z_track = 2.5  # z-axis tracking error
# gains.kp_vel_z = 0.2  # z-axis velocity error
# gains.kp_yaw = 4.0  # yaw error