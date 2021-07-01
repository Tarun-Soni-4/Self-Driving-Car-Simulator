from math import isnan
import time
from typing import Any, Union, Optional
from csv import QUOTE_ALL, writer
import PID_control
import Storage
import lane_management

lane_file_path = "Data\Lane"
val = 0.0
RANGE_REAR = 0.0
RANGE_FRONT = 0.0
lane_width = 1
Storage.deleteData('change_side', lane_file_path)
Log = list()
error_Log = list()

def obj_center(dist_sensors, dist_sensor_data, auto_drive):
    global RANGE_FRONT, RANGE_REAR
    sum_d = 0
    try:
        if dist_sensor_data is not None:
            for key, value in dist_sensor_data.items():
                if(key == "front"):
                    RANGE_FRONT = abs((dist_sensors[key].getMaxValue() - value) / dist_sensors[key].getMaxValue())
                if value < (dist_sensors[key].getMaxValue() * 0.5):
                    res = round(dist_sensors[key].getMaxValue() - value) * (lane_width/dist_sensors[key].getMaxValue())* 4
                    if "right" in key:
                        sum_d = sum_d - res
                    if "left" in key:
                        sum_d = sum_d + res
                    if "rear" in key:
                        RANGE_REAR = value / dist_sensors[key].getMaxValue()
            return sum_d
    except ValueError as e:
        print("[ DATA FUSION ] error in obj_center..", e)

def check_sides(dist_sensor_data: dict) -> Optional[str]:
    try:
        is_available_left = obj_checker(dist_sensor_data, "left")
        is_available_right = obj_checker(dist_sensor_data, "right")
        if is_available_left and is_available_right:
            return "both"
        elif is_available_left:
            return "left"
        elif is_available_right:
            return "right"
        else:
            return None
    except Exception as e:
        error_Log.append("[ DATA FUSION ] error in check_sides..%s" % e)

def change_lane(display_front: object, front_cams: dict, auto_drive: vars, gps: float, status: str, side: str, respond_dict: dict) -> float:
    global val, RANGE_FRONT
    try:
        if side is None:
            if not isnan(auto_drive.getCurrentSpeed()):
                if status == "emergency":
                    val = auto_drive.getCurrentSpeed() + (auto_drive.getCurrentSpeed() / 20)
            if status == "VA_order":
                respond_dict["text1"] = "Lane changing currently not possible"
        cl = total_lane_center(display_front, front_cams, gps)
        return cl
    except Exception as e:
        error_Log.append("[ DATA FUSION ] error in change lane..%s" % e)

def obj_checker(dist_sensor_data: dict, string: str) -> bool:
    flag = True
    if string == "front":
        if string in dist_sensor_data:
            flag = False
        return flag
    for key, value in dist_sensor_data.items():
        s = key.split()
        if string in s:
            flag = False
            break
    return flag

def file_handler():
    with open("Logs\DF_Log.csv", 'a') as file:
        wr = writer(file, quoting=QUOTE_ALL)
        wr.writerow(Log)
    if len(error_Log):
        with open("Logs\error_Log.csv", 'a', newline="") as file:
            wr = writer(file, quoting=QUOTE_ALL)
            wr.writerow(error_Log)

def VA_order_respond(VA_order, auto_drive, respond_dict):
    global val
    try:
        v = auto_drive.getCurrentSpeed()
        x = isnan(v)
        if VA_order == 3:
            if not x:
                speed = str(round(v, 2))
                text = "speed is" + speed + "."
                respond_dict["text2"] = text
                return
        if VA_order == 2:
            val = v + v * 0.2

        elif VA_order == 4:
            if v > 0:
                val = v - v * 0.2
    except Exception as e:
        error_Log.append("[ DATA FUSION ] ERROR IN VA_ORDER_RESPOND %s" % e)

def total_lane_center(display_front: object, front_cams: dict, gps: float) -> Union[Optional[float], Any]:
    change_side = Storage.loadData("change_side", lane_file_path)
    cl_sum, count = 0, 0
    key: str
    for key, value in front_cams.items():
        if "center" not in key:
            cl = lane_management.main(display_front, front_cams[key], gps, change_side, flag=False)
            if cl is not None:
                cl_sum = cl + cl_sum
                count += 1
        else:
            cl = lane_management.main(display_front, front_cams[key], gps, change_side, flag=True)
            if cl is not None:
                cl_sum = 8 * cl + cl_sum
                count += 8
    if count:
        cl_sum = cl_sum / count
        return round(cl_sum, 2)
    else:
        return None

def func_lane_change(gps_total: vars, axis: int) -> bool:
    try:
        gps_for_change = Storage.loadData("gps_for_change", lane_file_path)
        if gps_for_change is not None:
            if int(abs(abs(gps_for_change) - abs(gps_total.getValues()[axis]))) >= lane_width:
                print("-----------------------------lane change OK")
                lane_changed = True
                Storage.deleteData('gps_for_change', lane_file_path)
                Storage.deleteData('change_side', lane_file_path)
            else:
                lane_changed = False
        else:
            Storage.storeData("gps_for_change", gps_total.getValues()[axis], lane_file_path)
            lane_changed = False
        return lane_changed
    except Exception as e:
        error_Log.append("[ DATA FUSION ] error in func_lane_change %s" % e)

def call_PID(cl, dist_sensors, dist_sensor_data, auto_drive, gps):
    global val
    try:
        if cl is not None:
            cl = cl + round(obj_center(dist_sensors, dist_sensor_data, auto_drive), 2)
        else:
            cl = gps + round(obj_center(dist_sensors, dist_sensor_data, auto_drive), 2)
        PID_control.main(auto_drive=auto_drive, gps=gps, DF_res=cl, val=val)
    except Exception as e:
        error_Log.append("[ DATA FUSION ] ERROR IN PID CALL: %s" % e)

def main(auto_drive: vars, gps: float, dist_sensor_data: dict, lidar_data: object, emergency: bool, display_front: object,
         front_cams: dict, dist_sensors: object, VA_order: int, respond_dict: dict, gps_total: object, axis: int) -> None:
    start_time = time.time()
    gps = abs(gps_total.getValues()[axis])
    VA_lane_change: bool = Storage.loadData("VA_lane_change", lane_file_path)
    emergency_flag: bool = Storage.loadData("emergency_flag", lane_file_path)
    front_lane_ch_flag: bool = Storage.loadData("front_lane_ch_flag", lane_file_path)
    change_side: str = Storage.loadData("change_side", lane_file_path)
    Log.clear()
    error_Log.clear()
    global RANGE_FRONT, RANGE_REAR
    try:
        try:
            if VA_order is not None:
                if VA_order == 1:
                    VA_lane_change = True
                    Storage.storeData("VA_lane_change", VA_lane_change, lane_file_path)
                elif VA_order in range(2, 5):
                    VA_order_respond(VA_order, auto_drive, respond_dict)
        except Exception as e:
            error_Log.append("[ DATA FUSION ] ERROR IN VA ORDER: %s" % e)

        try:
            if VA_lane_change is True:
                lane_changed = func_lane_change(gps_total, axis)
                if change_side is None:
                    print("[DATA FUSION] this is VA lane change..")
                    change_side = check_sides(dist_sensor_data)
                    if change_side == "both":
                        change_side = "left"
                    Storage.storeData("change_side", change_side, lane_file_path)
                if not lane_changed:
                    cl = change_lane(display_front, front_cams, auto_drive, gps, "VA_order_change", change_side,
                                     respond_dict)
                    call_PID(cl, dist_sensors, dist_sensor_data, auto_drive, gps)
                    file_handler()
                    return
                else:
                    Storage.deleteData("VA_lane_change", lane_file_path)
        except Exception as e:
            error_Log.append("[ DATA FUSION ]IN VA CHANGE LANE: %s " % e)

        if "rear" in dist_sensor_data and not isnan(auto_drive.getCurrentSpeed()):
            auto_drive.setCruisingSpeed(int(auto_drive.getCurrentSpeed()) + 5)
        try:
            front_available = obj_checker(dist_sensor_data, "front")
            if not front_available and RANGE_FRONT >= 0.2:
                front_lane_ch_flag = Storage.loadData("front_lane_ch_flag", lane_file_path)
                if front_lane_ch_flag is None:
                    front_lane_ch_flag = True
                    Storage.storeData("front_lane_ch_flag", front_lane_ch_flag, lane_file_path)

            if front_lane_ch_flag:
                lane_changed = func_lane_change(gps_total, axis)
                if change_side is None:
                    print("[DATA FUSION] this is front lane change..")
                    change_side = check_sides(dist_sensor_data)
                    if change_side == "both":
                        change_side = "left"
                    Storage.storeData("change_side", change_side, lane_file_path)
                if not lane_changed:
                    cl = change_lane(display_front, front_cams, auto_drive, gps, "front", change_side,
                                     respond_dict)
                    call_PID(cl, dist_sensors, dist_sensor_data, auto_drive, gps)
                    Log.append(str(time.time() - start_time))
                    file_handler()
                    return
                else:
                    Storage.deleteData("front_lane_ch_flag", lane_file_path)
        except Exception as e:
            error_Log.append("[ DATA FUSION ] ERROR IN FRONT IS BLOCKED: %s" % e)
            file_handler()
            
        try:
            if emergency:
                emergency_flag = Storage.loadData("emergency_flag", lane_file_path)
                if emergency_flag is None:
                    emergency_flag = True
                    Storage.storeData("emergency_flag", emergency_flag, lane_file_path)

            if emergency_flag is True:
                lane_changed = func_lane_change(gps_total, axis)
                if change_side is None:
                    print("[DATA FUSION] this is EV lane change..")
                    respond_dict["text"] = "EMERGENCY ALERT!"
                    change_side = check_sides(dist_sensor_data)
                    if change_side == "both":
                        change_side = "right"
                    Storage.storeData("change_side", change_side, lane_file_path)
                if not lane_changed:
                    cl = change_lane(display_front, front_cams, auto_drive, gps, "emergency", change_side, respond_dict)
                    call_PID(cl, dist_sensors, dist_sensor_data, auto_drive, gps)
                    Log.append(str(time.time() - start_time))
                    file_handler()
                    return
                else:
                    Storage.deleteData("emergency_flag", lane_file_path)
        except Exception as e:
            error_Log.append("[ DATA FUSION ] error in emergency..%s" % e)
            file_handler()
            
        if emergency_flag is None and front_lane_ch_flag is None and VA_lane_change is None:
            cl = total_lane_center(display_front, front_cams, gps)
            call_PID(cl, dist_sensors, dist_sensor_data, auto_drive, gps)
            # print("[DATA FUSION] this is normal lane main call..")
            Log.append(str(time.time() - start_time))
            file_handler()
            return
        Log.append(str(time.time() - start_time))
    except Exception as e:
        error_Log.append("ERROR IN DATA FUSION: .%s" % e)
        file_handler()
main()
