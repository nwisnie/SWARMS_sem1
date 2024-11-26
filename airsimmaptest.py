import airsim
import airsimneurips
import time
import numpy as np

#connect to server with drone controls
client = airsimneurips.MultirotorClient()
client.confirmConnection()

"""
To load the level first comment out the code below and run the exe file
Once the level is loaded comment out the simLoadLevel code and uncomment your code
Run the python script again and you should see the drone takeoff
"""

# maps: "Qualifier_Tier_1", "Soccer_Field_Easy", "Soccer_Field_Medium", "ZhangJiaJie_Medium", "Building99_Hard"

client.simLoadLevel('Qualifier_Tier_2')

client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")

# # Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name="drone_1").join()

# Code goes here to move the drone 

print(client.simListSceneObjects())

# qualifier_tier_1
# gates = ['Gate00', 'Gate01', 'Gate02', 'Gate03',
#          'Gate04', 'Gate05', 'Gate06', 'Gate07', 
#          'Gate08', 'Gate09', 'Gate10_21', 'Gate11_23', 
#          'Gate12', 'Gate13', 'Gate14', 'Gate15', 
#          'Gate16', 'Gate17', 'Gate18', 'Gate19']

# qualifier_tier_2
gates = ['Gate00', 'Gate01', 'Gate02', 'Gate03',
         'Gate04', 'Gate05', 'Gate06', 'Gate07', 
         'Gate08', 'Gate09', 'Gate10_10', 'Gate11_12', 
         'Gate12_14', 'Gate13_16']

# qualifier_tier_3
# gates = ['Gate00', 'Gate01', 'Gate02', 'Gate03', 
#          'Gate04', 'Gate05', 'Gate06', 'Gate07', 
#          'Gate08', 'Gate09', 'Gate10', 'Gate11', 
#          'Gate12', 'Gate13', 'Gate14', 'Gate15', 
#          'Gate16', 'Gate17', 'Gate18', 'Gate19', 
#          'Gate20', 'Gate21', 'Gate22', 'Gate23', 'Gate24']

# soccerfield_easy
# gates = ['Gate00', 'Gate01', 'Gate02', 
#          'Gate03', 'Gate04', 'Gate05', 
#          'Gate06', 'Gate07', 'Gate08', 
#          'Gate09', 'Gate10_21', 'Gate11_23', 
#          'StartBlock']

# ZhangJiaJie_Medium
# gates = [ 'Gate00', 'Gate01', 'Gate02', 'Gate03', 'Gate04',
#           'Gate05', 'Gate06', 'Gate07', 'Gate08', 'Gate09', 
#           'Gate10', 'Gate11', 'Gate12', 'Gate13', 'Gate14',
#           'Gate15', 'Gate16', 'Gate17', 'Gate18', 'Gate19', 
#           'Gate20', 'Gate21', 'Gate22', 'Gate23', 'Gate24' ]

# Soccer_Field_Medium
# gates = ['Gate00', 'Gate01', 'Gate02', 'Gate03', 'Gate04', 
#          'Gate05', 'Gate06', 'Gate07', 'Gate08', 'Gate09', 
#          'Gate10', 'Gate11', 'Gate12', 'Gate13', 'Gate14', 
#          'Gate15', 'Gate16', 'Gate17', 'Gate18', 'Gate19', 
#          'Gate20', 'Gate21', 'Gate22', 'Gate23', 'Gate24']

for obj in gates:
    print(f"airsim.Vector3r({client.simGetObjectPose(obj).position.x_val}, {client.simGetObjectPose(obj).position.y_val}, {client.simGetObjectPose(obj).position.z_val}),")






# move_list = [[0,2,2, 5,5], # actual coords in grapher.py file
#             [1.6,10.8,2, 5,5],
#             [8.9,18.5,2, 8,5],
#             [18.7,22.2,2, 8,5],
#             [30,22,2,2, 40,2],
#             [39,19.2,2, 8,5],
#             [45.7,11.7,2, 5,5],
#             [45.7,2.2,2, 8,5],
#             [40.3,-4.8,2, 8,5],
#             [30.7,-7.9,2, 8,5],
#             [18.5,-7.9,2, 8,5],
#             [9.5,-5.1,2, 6,5],  
#             [0,-2,3.2, 5,5]]

# move_list = [
#             airsim.Vector3r(0,2,2), # using Vector3r
#             airsim.Vector3r(1.6,10.8,2),
#             airsim.Vector3r(8.9,18.5,2),
#             airsim.Vector3r(18.7,22.2,2),
#             airsim.Vector3r(30,22,2,2),
#             airsim.Vector3r(39,19.2,2),
#             airsim.Vector3r(45.7,11.7,2),
#             airsim.Vector3r(45.7,2.2,2),
#             airsim.Vector3r(40.3,-4.8,2),
#             airsim.Vector3r(30.7,-7.9,2),
#             airsim.Vector3r(18.5,-7.9,2),
#             airsim.Vector3r(9.5,-5.1,2),  
#             airsim.Vector3r(0,-2,2)
#             ]

# client.moveToPositionAsync(0,0,3,5,5).join()

# client.simStartRace()

# # fix takeoff error

# # for i in move_list:
# #     x = i[0]
# #     y = i[1]
# #     z = i[2]
# #     vel = i[3]
# #     timeout = i[4]
# #     client.moveToPositionAsync(x,y,z,vel,timeout).join()

# # use spheres:

# def is_within_radius(current_position, target_position, radius):
#     distance = np.linalg.norm(np.array([current_position.x_val, current_position.y_val, current_position.z_val]) - 
#                               np.array([target_position.x_val, target_position.y_val, target_position.z_val]))
#     return distance <= radius

# def monitor_and_move(initial_target, second_target, radius):
#     # Start the first async call
#     client.moveToPositionAsync(initial_target.x_val, initial_target.y_val, initial_target.z_val, 5).join()

#     while True:
#         # Get the current position of the drone
#         current_position = client.getMultirotorState().kinematics_estimated.position

#         # Check if within the radius of the first target
#         if is_within_radius(current_position, initial_target, radius):
#             print("Within radius, triggering second moveToPositionAsync")
#             client.moveToPositionAsync(second_target.x_val, second_target.y_val, second_target.z_val, 5)
#             break

#         # Sleep briefly to avoid excessive CPU usage
#         time.sleep(0.1)

# # Initialize the AirSim client
# client = airsim.MultirotorClient()
# client.confirmConnection()

# # # Example usage
# # initial_target = airsim.Vector3r(10, 10, -10)  # Initial target position
# # second_target = airsim.Vector3r(20, 20, -10)   # Second target position
# # radius = 2.0  # Radius threshold to trigger the next move

# # Call the function to monitor and move

# for i in range(len(move_list)-1):
#     monitor_and_move(airsim.Vector3r(move_list[i][0],move_list[i][1],move_list[i][2]), 
#                      airsim.Vector3r(move_list[i+1][0],move_list[i+1][1],move_list[i+1][2]),
#                      2)

