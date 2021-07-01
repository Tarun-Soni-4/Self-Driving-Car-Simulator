import time
from csv import QUOTE_ALL, writer
Log = list()
error_Log = list()
obj_data, LIDAR_data = None, None

def LIDAR_sensor(lidars):
    #In real life simulation this is the main feature required for the self driving car
    return None

def dist_sensor(dist_sensors, dist_sensor_names):
    obj_on_side = {}
    for j in range(len(dist_sensor_names)):
        if dist_sensors[dist_sensor_names[j]].getValue() < dist_sensors[dist_sensor_names[j]].getMaxValue():
            obj_on_side.update({dist_sensor_names[j]: dist_sensors[dist_sensor_names[j]].getValue()})
        else:
            obj_on_side.pop(dist_sensor_names[j], None)
    return obj_on_side

def cam_obj_rec(camera, string):
    obj_to_reduce = {"road", "building", "tree", "hotel"}
    cam_objects = camera.getRecognitionObjects()
    for i in range(0, len(cam_objects)):
        s1 = set(cam_objects[i].model.split())
        if not bool(s1.intersection(obj_to_reduce)):
            print("Model of object at " + string + "{} : {}".format(i, cam_objects[i].model))
            print(cam_objects[i].get_position_on_image())
            print(cam_objects[i].get_position())
            print(cam_objects[i].get_size_on_image())

def main(dist_sensor_names, lidars, dist_sensors, front_cams, back_camera):
    global obj_data, LIDAR_data
    start_time = time.time()
    Log.clear()
    error_Log.clear()
    try:
        obj_data = dist_sensor(dist_sensors, dist_sensor_names)
        LIDAR_data = None
        Log.append(str(time.time() - start_time))
    except Exception as e:
        error_Log.append("[OBJ] IN OBJ MAIN: %s " % e)
    with open("Logs\OBJ_Log.csv", 'a') as file:
        wr = writer(file, quoting=QUOTE_ALL)
        wr.writerow(Log)
    if len(error_Log):
        with open("Logs\error_Log.csv", 'a', newline="") as file:
            wr = writer(file, quoting=QUOTE_ALL)
            wr.writerow(error_Log)
    return obj_data, LIDAR_data
main()
