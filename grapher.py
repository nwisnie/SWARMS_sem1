import matplotlib.pyplot as plt

coord_list = [[0,2,2, 5,5], # try messing with speed values and such
            [1.6,10.8,2, 5,5],
            [8.9,18.5,2, 8,5],
            [18,24,2, 8,5],
            [30,22,2,2, 40,2],
            [39,19.2,2, 8,5],
            [45.7,11.7,2, 8,5],
            [45.7,2.2,2, 8,5],
            [40.3,-4.8,2, 8,5],
            [30.7,-7.9,2, 8,5],
            [18.5,-7.9,2, 8,5],
            [9.5,-5.1,2, 6,5],
            [0,-2,3.2, 5,5]]

new_coords = [[1.0, 10.8, 2.0]
,[1.8118570857525764, 11.836130038468212, 2.0]
,[2.6373216198161535, 12.82886196604641, 2.0]
,[3.477624769917288, 13.77744770708967, 2.0]
,[4.333997703782537, 14.681139185953075, 2.0]
,[5.207671589138458, 15.539188326991699, 2.0]
,[6.099877593711608, 16.350847054560624, 2.0]
,[7.011846885228545, 17.11536729301493, 2.0]
,[7.944810631415823, 17.832000966709696, 2.0]
,[8.9, 18.5, 2.0]
,[9.878646158707637, 19.11861631724092, 2.0]
,[10.881980275265285, 19.687101842787538, 2.0]
,[11.911233517399504, 20.20470850099493, 2.0]
,[12.967637052836855, 20.67068821621818, 2.0]
,[14.052422049303887, 21.08429291281236, 2.0]
,[15.16681967452716, 21.44477451513255, 2.0]
,[16.31206109623323, 21.751384947533836, 2.0]
,[17.489377482148658, 22.00337613437129, 2.0]
,[18.7, 22.2, 2.0]
,[19.94337995294226, 22.341393718631206, 2.0]
,[21.211849185844248, 22.43123546390083, 2.0]
,[22.49595967900321, 22.47408865930098, 2.0]
,[23.78626341271642, 22.474516728323728, 2.0]
,[25.073312367281122, 22.437083094461194, 2.0]
,[26.34765852299457, 22.366351181205456, 2.0]
,[27.599853860154035, 22.26688441204862, 2.0]
,[28.82045035905676, 22.14324621048277, 2.0]
,[30.000000000000007, 22.0, 2.0]
,[31.13151029838479, 21.83981429520271, 2.0]
,[32.21781091002715, 21.657777975134515, 2.0]
,[33.26418702584691, 21.447085009949316, 2.0]
,[34.27592383676387, 21.200929369801024, 2.0]
,[35.258306533697855, 20.912505024843544, 2.0]
,[36.21662030756864, 20.575005945230792, 2.0]
,[37.156150349296055, 20.18162610111668, 2.0]
,[38.08218184979991, 19.725559462655113, 2.0]
,[39.0, 19.2, 2.0]
,[39.9120603350001, 18.60017214582543, 2.0]
,[40.80949976663975, 17.929422182886206, 2.0]
,[41.68062555094249, 17.19312685645731, 2.0]
,[42.5137449439318, 16.39666291181373, 2.0]
,[43.2971652016312, 15.545407094230452, 2.0]
,[44.01919358006421, 14.644736148982462, 2.0]
,[44.66813733525435, 13.70002682134473, 2.0]
,[45.2323037232251, 12.71665585659225, 2.0]
,[45.7, 11.7, 2.0]
,[46.06236084446807, 10.655793417791866, 2.0]
,[46.32183062698037, 9.591199959987327, 2.0]
,[46.48368114075353, 8.513740897554765, 2.0]
,[46.55318417900414, 7.430937501462552, 2.0]
,[46.5356115349488, 6.350311042679066, 2.0]
,[46.43623500180413, 5.279382792172684, 2.0]
,[46.26032637278669, 4.225674020911798, 2.0]
,[46.013157441113115, 3.1967059998647764, 2.0]
,[45.699999999999996, 2.1999999999999895, 2.0]
,[45.32546473705908, 1.2420862817725256, 2.0]
,[44.89151791748266, 0.3255310635842131, 2.0]
,[44.39946470085821, -0.5480904466763872, 2.0]
,[43.85061024677317, -1.3772030411207457, 2.0]
,[43.246259714815, -2.1602315118603297, 2.0]
,[42.58771826457114, -2.8956006510065815, 2.0]
,[41.876291055629046, -3.5817352506709774, 2.0]
,[41.113283247576184, -4.217060102964961, 2.0]
,[40.29999999999999, -4.800000000000006, 2.0]
,[39.43742629783059, -5.32962551333193, 2.0]
,[38.5252664273688, -5.807590332294053, 2.0]
,[37.56290450025805, -6.2361939256640655, 2.0]
,[36.54972462814184, -6.617735762219639, 2.0]
,[35.48511092266361, -6.954515310738471, 2.0]
,[34.368447495466846, -7.248832039998234, 2.0]
,[33.19911845819501, -7.502985418776624, 2.0]
,[31.976507922491567, -7.719274915851317, 2.0]
,[30.7, -7.9, 2.0]
,[29.371469303442915, -8.046903844357034, 2.0]
,[28.002752449859685, -8.15950443948346, 2.0]
,[26.608176557368804, -8.236763480297004, 2.0]
,[25.20206874408881, -8.277642661715381, 2.0]
,[23.798756128138205, -8.281103678656315, 2.0]
,[22.412565827635497, -8.246108226037522, 2.0]
,[21.05782496069922, -8.171617998776727, 2.0]
,[19.748860645447877, -8.056594691791645, 2.0]
,[18.5, -7.9, 2.0]
,[17.321689629687135, -7.701689150392206, 2.0]
,[16.208854088693133, -7.465091498249474, 2.0]
,[15.152537418414843, -7.1945299309257065, 2.0]
,[14.143783660249131, -6.894327335774805, 2.0]
,[13.173636855592893, -6.56880660015068, 2.0]
,[12.233141045842979, -6.22229061140723, 2.0]
,[11.313340272396283, -5.859102256898366, 2.0]
,[10.40527857664966, -5.4835644239779855, 2.0]
,[9.5, -5.1, 2.0]
,[8.588548583844167, -4.712731872318311, 2.0]
,[7.661968369579024, -4.3260829282868185, 2.0]
,[6.711303398601464, -3.9443760552594362, 2.0]
,[5.72759771230834, -3.571934140590059, 2.0]
,[4.701895352096543, -3.213080071632601, 2.0]
,[3.62524035936294, -2.8721367357409613, 2.0]
,[2.4886767755043895, -2.5534270202690426, 2.0]
,[1.283248641917789, -2.2612738125707548, 2.0]]


for i in coord_list:
    plt.plot(i[0], i[1], marker='o', color='b', markersize=20)

for i in new_coords:
    plt.plot(i[0], i[1], marker='o', color='y', markersize=5) 

# mark start and end
plt.plot(coord_list[0][0], coord_list[0][1], marker='*', color='g', markersize=20)
plt.plot(coord_list[-1][0], coord_list[-1][1], marker='*', color='r', markersize=20)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Plot of Gates')
plt.grid(True)

plt.show()

# client dir:

# ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', 
# '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', 
# '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', 'armDisarm', 
# 'cancelLastTask', 'client', 'confirmConnection', 'enableApiControl', 'getBarometerData', 'getClientVersion', 'getDistanceSensorData', 
# 'getGpsData', 'getHomeGeoPoint', 'getImuData', 'getLidarData', 'getMagnetometerData', 'getMinRequiredClientVersion', 
# 'getMinRequiredServerVersion', 'getMultirotorState', 'getRotorStates', 'getServerVersion', 'getSettingsString', 'goHomeAsync', 
# 'hoverAsync', 'isApiControlEnabled', 'isRecording', 'landAsync', 'listVehicles', 'moveByAngleRatesThrottleAsync', 'moveByAngleRatesZAsync', 
# 'moveByAngleThrottleAsync', 'moveByAngleZAsync', 'moveByManualAsync', 'moveByMotorPWMsAsync', 'moveByRC', 'moveByRollPitchYawThrottleAsync', 
# 'moveByRollPitchYawZAsync', 'moveByRollPitchYawrateThrottleAsync', 'moveByRollPitchYawrateZAsync', 'moveByVelocityAsync', 
# 'moveByVelocityBodyFrameAsync', 'moveByVelocityZAsync', 'moveByVelocityZBodyFrameAsync', 'moveOnPathAsync', 'moveToGPSAsync', 
# 'moveToPositionAsync', 'moveToZAsync', 'ping', 'reset', 'rotateByYawRateAsync', 'rotateToYawAsync', 'setAngleLevelControllerGains', 
# 'setAngleRateControllerGains', 'setPositionControllerGains', 'setVelocityControllerGains', 'simAddDetectionFilterMeshName', 'simAddVehicle', 
# 'simClearDetectionMeshNames', 'simContinueForFrames', 'simContinueForTime', 'simCreateVoxelGrid', 'simDestroyObject', 'simEnableFocusPlane', 
# 'simEnableManualFocus', 'simEnableWeather', 'simFlushPersistentMarkers', 'simGetCameraInfo', 'simGetCollisionInfo', 'simGetCurrentFieldOfView', 
# 'simGetDetections', 'simGetDistortionParams', 'simGetFilmbackSettings', 'simGetFocalLength', 'simGetFocusAperture', 'simGetFocusDistance', 
# 'simGetGroundTruthEnvironment', 'simGetGroundTruthKinematics', 'simGetImage', 'simGetImages', 'simGetLensSettings', 'simGetLidarSegmentation', 
# 'simGetMeshPositionVertexBuffers', 'simGetObjectPose', 'simGetObjectScale', 'simGetPresetFilmbackSettings', 'simGetPresetLensSettings', 
# 'simGetSegmentationObjectID', 'simGetVehiclePose', 'simGetWorldExtents', 'simIsPause', 'simListAssets', 'simListSceneObjects', 'simLoadLevel', 
# 'simPause', 'simPlotArrows', 'simPlotLineList', 'simPlotLineStrip', 'simPlotPoints', 'simPlotStrings', 'simPlotTransforms', 'simPlotTransformsWithNames', 
# 'simPrintLogMessage', 'simRunConsoleCommand', 'simSetCameraFov', 'simSetCameraPose', 'simSetDetectionFilterRadius', 'simSetDistortionParam', 
# 'simSetDistortionParams', 'simSetFilmbackSettings', 'simSetFocalLength', 'simSetFocusAperture', 'simSetFocusDistance', 'simSetKinematics', 
# 'simSetLightIntensity', 'simSetObjectMaterial', 'simSetObjectMaterialFromTexture', 'simSetObjectPose', 'simSetObjectScale', 
# 'simSetPresetFilmbackSettings', 'simSetPresetLensSettings', 'simSetSegmentationObjectID', 'simSetTimeOfDay', 'simSetTraceLine', 
# 'simSetVehiclePose', 'simSetWeatherParameter', 'simSetWind', 'simSpawnObject', 'simSwapTextures', 'simTestLineOfSightBetweenPoints', 
# 'simTestLineOfSightToPoint', 'startRecording', 'stopRecording', 'takeoffAsync']