import airsim
import airsimneurips
import time
import numpy as np
from scipy.interpolate import CubicSpline, interp1d
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt

#connect to server with drone controls
client = airsimneurips.MultirotorClient()
client.confirmConnection()

"""
To load the level first comment out the code below and run the exe file
Once the level is loaded comment out the simLoadLevel code and uncomment your code
Run the python script again and you should see the drone takeoff
"""

# Soccer_Field_Easy, ZhangJiaJie_Medium, Qualifier_Tier_1
client.simLoadLevel('Qualifier_Tier_1')

client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")

# # Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name="drone_1").join()


# // gate coords //

# soccer_field_easy
# moveList = [airsim.Vector3r(0,6,2),
#             airsim.Vector3r(1.5999999046325684, 10.800000190734863, 2.0199999809265137),
#             airsim.Vector3r(8.887084007263184, 18.478761672973633, 2.0199999809265137),
#             airsim.Vector3r(18.74375343322754, 22.20650863647461, 2.0199999809265137),
#             airsim.Vector3r(30.04375457763672, 22.20648956298828, 2.0199999809265137),
#             airsim.Vector3r(39.04375457763672, 19.206478118896484, 2.0199999809265137),
#             airsim.Vector3r(45.74375534057617, 11.706478118896484, 2.0199999809265137),
#             airsim.Vector3r(45.74375534057617, 2.2064781188964844, 2.0199999809265137),
#             airsim.Vector3r(40.343753814697266, -4.793521404266357, 2.0199999809265137),
#             airsim.Vector3r(30.74375343322754, -7.893521785736084, 2.0199999809265137),
#             airsim.Vector3r(18.54375457763672, -7.893521785736084, 2.0199999809265137),
#             airsim.Vector3r(9.543754577636719, -5.093521595001221, 2.0199999809265137)]
#             # airsim.Vector3r(0.0, -2.0, 3.2200000286102295)]

# mountain
# moveList = [
#             airsim.Vector3r(-2.799999952316284, 7.699999809265137, 4.829999923706055),
#             airsim.Vector3r(-12.799999237060547, 2.799999952316284, 5.230000019073486),
#             airsim.Vector3r(-18.799999237060547, 2.700000047683716, 4.829999923706055),
#             airsim.Vector3r(-25.69999885559082, -1.0, 8.729999542236328),
#             airsim.Vector3r(-32.5, -11.699999809265137, 8.729999542236328),
#             airsim.Vector3r(-38.70000076293945, -29.399999618530273, 5.029999732971191),
#             airsim.Vector3r(-37.0, -45.89999771118164, -3.4700000286102295),
#             airsim.Vector3r(-30.799999237060547, -72.0999984741211, -13.569999694824219),
#             airsim.Vector3r(-25.399999618530273, -88.0, -22.469999313354492),
#             airsim.Vector3r(-24.899999618530273, -99.29999542236328, -22.369998931884766),
#             airsim.Vector3r(-38.5, -112.5999984741211, -22.369998931884766),
#             airsim.Vector3r(-58.69999694824219, -116.0, -22.369998931884766),
#             airsim.Vector3r(-61.19999694824219, -108.79999542236328, -22.369998931884766),
#             airsim.Vector3r(-56.0, -88.5999984741211, -22.369998931884766),
#             airsim.Vector3r(-32.795989990234375, -58.72628402709961, -22.369998931884766),
#             airsim.Vector3r(-17.30462074279785, -40.26478576660156, -22.369998931884766),
#             airsim.Vector3r(1.5008275508880615, -17.34554100036621, -17.56999969482422),
#             airsim.Vector3r(4.088334083557129, -2.6719322204589844, -11.469999313354492),
#             airsim.Vector3r(5.6859893798828125, 6.388282299041748, -11.469999313354492),
#             airsim.Vector3r(10.841537475585938, 8.78415298461914, -9.269999504089355),
#             airsim.Vector3r(15.76799201965332, 5.07656717300415, -5.569999694824219),
#             airsim.Vector3r(17.661367416381836, -3.4806108474731445, -1.8700000047683716),
#             airsim.Vector3r(10.874125480651855, -9.746925354003906, 1.4299999475479126),
#             airsim.Vector3r(5.950470447540283, -8.874055862426758, -0.17000000178813934),
#             airsim.Vector3r(3.804168224334717, -6.381180763244629, -1.3700000047683716)
#             ]

# qualifier_1
moveList = [airsim.Vector3r(10.388415336608887, 80.77406311035156, -43.57999801635742),
            airsim.Vector3r(18.11046600341797, 76.26078033447266, -43.57999801635742),
            airsim.Vector3r(25.433794021606445, 66.28687286376953, -43.57999801635742),
            airsim.Vector3r(30.065513610839844, 56.549530029296875, -43.57999801635742),
            airsim.Vector3r(32.30064392089844, 45.6310920715332, -43.87999725341797),
            airsim.Vector3r(26.503353118896484, 38.19984436035156, -43.37999725341797),
            airsim.Vector3r(3.264113664627075, 37.569061279296875, -43.57999801635742),
            airsim.Vector3r(-16.862957000732422, 45.41843795776367, -46.57999801635742),
            airsim.Vector3r(-15.493884086608887, 63.18687438964844, -52.07999801635742),
            airsim.Vector3r(-6.320737361907959, 78.21236419677734, -55.779998779296875),
            airsim.Vector3r(5.143640041351318, 82.38504791259766, -55.779998779296875),
            airsim.Vector3r(14.558510780334473, 84.4320297241211, -55.18000030517578),
            airsim.Vector3r(20.858510971069336, 87.83203125, -42.07999801635742), # added
            airsim.Vector3r(23.858510971069336, 82.83203125, -32.07999801635742),
            airsim.Vector3r(38.25851058959961, 78.13202667236328, -31.3799991607666),
            airsim.Vector3r(51.058509826660156, 52.13203048706055, -25.8799991607666),
            airsim.Vector3r(44.95851135253906, 38.932029724121094, -25.8799991607666),
            airsim.Vector3r(25.958515167236328, 26.33203125, -19.8799991607666),
            airsim.Vector3r(11.658514976501465, 26.33203125, -12.779999732971191),
            airsim.Vector3r(-10.141484260559082, 22.632030487060547, -6.37999963760376),
            airsim.Vector3r(-24.641483306884766, 9.132031440734863, 2.119999885559082),]

# qualifier_2
# moveList = [airsim.Vector3r(-3.7697627544403076, -73.76139831542969, -7.181804180145264), # added
#             airsim.Vector3r(-11.699999809265137, -72.9000015258789, -7.269999980926514),
#             airsim.Vector3r(-23.5, -72.29999542236328, -7.269999980926514),
#             airsim.Vector3r(-45.79999923706055, -79.29999542236328, -12.069999694824219),
#             airsim.Vector3r(-64.69999694824219, -82.0, -19.969999313354492),
#             airsim.Vector3r(-83.69999694824219, -82.0, -30.56999969482422),
#             airsim.Vector3r(-85.9000015258789, -66.29999542236328, -30.56999969482422),
#             airsim.Vector3r(-42.17225646972656, 20.308265686035156, -24.6733341217041),
#             airsim.Vector3r(-22.369873046875, 31.829633712768555, -18.154823303222656),
#             airsim.Vector3r(-11.601242065429688, 32.98910140991211, -12.158832550048828),
#             airsim.Vector3r(1.705966591835022, 33.913211822509766, -7.942454814910889),
#             airsim.Vector3r(8.486928939819336, 29.77781105041504, -3.672271728515625),
#             airsim.Vector3r(15.815702438354492, 23.04956817626953, 5.823721408843994),
#             airsim.Vector3r(12.275566101074219, 18.25774383544922, 15.606721878051758),
#             airsim.Vector3r(5.17556619644165, 15.057744026184082, 17.106721878051758)]

# qualifier_3 
# moveList = [airsim.Vector3r(58.25, -2, 13.11), # added
#             airsim.Vector3r(58.25, 4, 13.719999313354492), # added
#             airsim.Vector3r(57.099998474121094, 9.699999809265137, 13.719999313354492),
#             airsim.Vector3r(57.099998474121094, 27.5, 13.719999313354492),
#             airsim.Vector3r(57.099998474121094, 42.39999771118164, 12.319999694824219),
#             airsim.Vector3r(57.099998474121094, 54.29999923706055, 12.920000076293945),
#             airsim.Vector3r(48.94746017456055, 67.39400482177734, 13.02389907836914),
#             airsim.Vector3r(45.12211990356445, 80.97061920166016, 13.021943092346191),
#             airsim.Vector3r(36.00019836425781, 92.77997589111328, 13.118571281433105),
#             airsim.Vector3r(29.3155517578125, 100.70634460449219, 13.119451522827148),
#             airsim.Vector3r(23.299999237060547, 112.69999694824219, 13.15040111541748),
#             airsim.Vector3r(13.699999809265137, 125.5, 11.719999313354492),
#             airsim.Vector3r(-0.6884960532188416, 128.23724365234375, 7.819999694824219),
#             airsim.Vector3r(-16.625560760498047, 119.59599304199219, 2.919999837875366),
#             airsim.Vector3r(-41.83222198486328, 103.86845397949219, -8.679999351501465),
#             airsim.Vector3r(-48.599998474121094, 81.29999542236328, -10.179999351501465),
#             airsim.Vector3r(-50.0, 65.69999694824219, -15.979999542236328),
#             airsim.Vector3r(-47.0, 58.5, -15.979999542236328),
#             airsim.Vector3r(-41.5, 43.20000076293945, -14.979999542236328),
#             airsim.Vector3r(-41.5, 14.09999942779541, -14.979999542236328),
#             airsim.Vector3r(-33.70000076293945, -9.59999942779541, -8.479999542236328),
#             airsim.Vector3r(-13.395312309265137, -12.481369018554688, -10.579999923706055),
#             airsim.Vector3r(20.192157745361328, -4.337061882019043, -11.679999351501465),
#             airsim.Vector3r(44.355430603027344, -2.774184465408325, -16.68000030517578),
#             airsim.Vector3r(58.69259262084961, 0.23001037538051605, -20.979999542236328),
#             airsim.Vector3r(78.82240295410156, -2.2114856243133545, -24.3799991607666),
#             airsim.Vector3r(96.19999694824219, -9.199999809265137, -24.3799991607666),]





# // generate spline based on coords //

waypoints = []
for i in moveList:
    waypoints.append([i.x_val,i.y_val,i.z_val])

waypoints = np.array(waypoints)
t = np.linspace(0, 1, len(waypoints))

spline_x = CubicSpline(t, waypoints[:, 0])
spline_y = CubicSpline(t, waypoints[:, 1])
spline_z = CubicSpline(t, waypoints[:, 2])

# clamp z axis?

num_points = 300 
# t_new = np.linspace(0, 1, num_points)

# path_x = spline_x(t_new)
# path_y = spline_y(t_new)
# path_z = spline_z(t_new)
# path = np.stack((path_x, path_y, path_z), axis=-1)

dt = 0.01  # Small time step for numerical differentiation
t_fine = np.arange(0, 1, dt)
dx_dt = spline_x(t_fine, 1)
dy_dt = spline_y(t_fine, 1)
dz_dt = spline_z(t_fine, 1)

# Compute the differential arc length
d_length = np.sqrt(dx_dt**2 + dy_dt**2 + dz_dt**2)
arc_length = cumtrapz(d_length, t_fine, initial=0)

# Step 2: Interpolate to find evenly spaced points along arc length
total_length = arc_length[-1]
even_arc_lengths = np.linspace(0, total_length, num_points)
t_even = interp1d(arc_length, t_fine)(even_arc_lengths)

# Step 3: Evaluate splines at these new t values
path_x = spline_x(t_even)
path_y = spline_y(t_even)
path_z = spline_z(t_even)
path = np.stack((path_x, path_y, path_z), axis=-1)

moveList = []
for i in path:
    moveList.append(airsim.Vector3r(i[0],i[1],i[2]))
    # print([i[0],i[1],i[2]])

# print(path)
# print(moveList)





# LQR controller?

print("start here:")

client.simStartRace()

print(client.getMultirotorState().kinematics_estimated.position)

# values for PID graph
vel_num = 0
vel_list = []

# // PID VALUES //

kp = 8 # proportional gain was 8
ki = 6 # integral gain
kd = 0.07 # derivative gain
dt = 0.001 # sleep time for loop (ms)

prevError = np.array([0.0, 0.0, 0.0])
integral = np.array([0.0, 0.0, 0.0])

# // PID logic // 

for i in path:

    prevError = np.array([0.0, 0.0, 0.0])
    integral = np.array([0.0, 0.0, 0.0])

    while True:

        curVel = client.getMultirotorState().kinematics_estimated.linear_velocity
        vel_num += 1
        vel_list.append([vel_num, (curVel.x_val**2+curVel.y_val**2)**0.5])

        # get current position
        state = client.getMultirotorState()
        curPos = np.array([state.kinematics_estimated.position.x_val,
                           state.kinematics_estimated.position.y_val,
                           state.kinematics_estimated.position.z_val])

        # PID logic and calculations:
        error = np.array(i) - np.array(curPos)
        integral += error * dt # accumulation of error
        derivative = (error - prevError) / dt

        controlSignal = kp * error + ki * integral + kd * derivative
        prevError = error

        # print(f"error: {error}, integral: {integral}, derivative: {derivative}")
        # print(f"input: {controlSignal}, position: {curPos}, error: {error}")
        # print(f"vx: {controlSignal[0]}, vy: {controlSignal[1]}, vz: {controlSignal[2]}")
        # print("\n")

        # plug in PID values
        client.moveByVelocityAsync(controlSignal[0], controlSignal[1], controlSignal[2], dt) # .join()
        # print(controlSignal)

        # Check if drone is close enough to the waypoint (sphere detection)
        if np.linalg.norm(np.array(i) - np.array(curPos)) < 1:
            # print(f"here {error}")
            break
        
        time.sleep(dt)



plot_time, plot_vel = zip(*vel_list)
plot_time = np.array(plot_time)
plot_vel = np.array(plot_vel)
# ideal_vel = np.array([5]*len(plot_time))

plt.figure()
plt.plot(plot_time,plot_vel)
# plt.plot(plot_time,ideal_vel)
plt.xlabel('Time')
plt.ylabel('Vel')
plt.title('Vel Plot (only X and Y)')
plt.grid(True)
plt.show()