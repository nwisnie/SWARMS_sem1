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

client.simLoadLevel('Soccer_Field_Medium')

client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")

# # Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name="drone_1").join()

# mountain
# moveList = [
#             # airsim.Vector3r(-2.799999952316284, 7.699999809265137, 4.829999923706055),
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

# soccer field
moveList = [
            #airsim.Vector3r(0.5, 9.699999809265137, 2.5199999809265137),
            airsim.Vector3r(0.5, 27.5, 2.5199999809265137),
            airsim.Vector3r(0.5, 42.39999771118164, 0.019999999552965164),
            airsim.Vector3r(0.5, 51.5, -3.179999828338623),
            airsim.Vector3r(5.199999809265137, 61.5, -3.179999828338623),
            airsim.Vector3r(10.800000190734863, 64.5999984741211, 0.41999998688697815),
            airsim.Vector3r(18.5, 65.5999984741211, 4.920000076293945),
            airsim.Vector3r(26.5, 65.9000015258789, 9.619999885559082),
            airsim.Vector3r(33.5, 65.9000015258789, 10.920000076293945),
            airsim.Vector3r(48.0, 63.29999923706055, 11.719999313354492),
            airsim.Vector3r(65.29999542236328, 60.39999771118164, 12.319999694824219),
            airsim.Vector3r(92.0999984741211, 73.9000015258789, 11.719999313354492),
            airsim.Vector3r(112.29999542236328, 104.5, 4.119999885559082),
            airsim.Vector3r(138.1999969482422, 87.0, 4.119999885559082),
            airsim.Vector3r(141.5, 49.79999923706055, 15.219999313354492),
            airsim.Vector3r(144.5, 42.599998474121094, 15.219999313354492),
            airsim.Vector3r(150.0, 27.299999237060547, 16.219999313354492),
            airsim.Vector3r(150.0, -1.7999999523162842, 16.219999313354492),
            airsim.Vector3r(150.0, -12.5, 16.219999313354492),
            airsim.Vector3r(131.8000030517578, -25.0, 15.319999694824219),
            airsim.Vector3r(84.9000015258789, -25.0, 15.319999694824219),
            airsim.Vector3r(75.9000015258789, -25.0, 9.220000267028809),
            airsim.Vector3r(58.599998474121094, -25.0, 4.920000076293945),
            airsim.Vector3r(39.20000076293945, -19.100000381469727, 1.5199999809265137),
            airsim.Vector3r(23.299999237060547, -9.199999809265137, 1.5199999809265137)]

# // sphere approach //

# curPos: position of drone (Vector3r)
# target: position of destination (Vector3r)
# radius: radius of circle (int)
def inRadius(curPos, target, radius):
    return np.linalg.norm(np.array([target.x_val,target.y_val,target.z_val]) - np.array([curPos.x_val,curPos.y_val,curPos.z_val])) < radius

client.simStartRace()
print(client.getMultirotorState().kinematics_estimated.position)

# client.moveToPositionAsync(-2.799999952316284,7.699999809265137,4.829999923706055, 2).join() # mountain
client.moveToPositionAsync(0.5, 9.699999809265137, 2.5199999809265137,2).join() # field

for i in moveList:
    print(i)
    client.moveToPositionAsync(i.x_val, i.y_val, i.z_val, 5).join()

# for i in range(len(moveList)-1):
#     while True:
#         curPos = client.getMultirotorState().kinematics_estimated.position
#         if inRadius(curPos, moveList[i], 1):
#             print(f"at gate {i}")
#             break
#         time.sleep(0.1) # sleep to not tank framerate

#     client.moveToPositionAsync(moveList[i+1].x_val, moveList[i+1].y_val, moveList[i+1].z_val, 4)