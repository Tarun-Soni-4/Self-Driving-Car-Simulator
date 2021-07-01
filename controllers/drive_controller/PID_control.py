from math import isnan
import time
from scipy.integrate import odeint
import Storage
from csv import QUOTE_ALL, writer

file_path = "Data\PID"
error_Log = list()
Log = list()
MAX_CRUISING_SPEED = 30
MIN_BRAKE_INTENSITY = 0.0001
MAX_BRAKE_INTENSITY = 0.9
Cd = 0.24  # drag coefficient
rho = 1.225  # air density (kg/m^3)
A = 5.0  # cross-sectional area (m^2)
Fp = 50  # thrust parameter (N/%pedal)
m = 1950  # vehicle mass (kg)
first_controller_output = 0.01
Ku = 0.04
Tu = 12
Kp = 0.6 * Ku
Ti = Tu / 2
Td = Tu / 8
control_out_max = 0.5
control_out_min = -0.5

def dv_dt(v, t, u, load):
    """
      For calculating the momentum balance formula:
        m*(dv(t)/dt)=Fp*u(t)−(1/2)ρ*A*Cd*v(t)^2
     """
    Fp = u / 100
    res = (1.0 / (m + load)) * ((Fp * u) - (0.5 * rho * Cd * A * v ** 2))
    return res

def speed_control(val, auto_drive):
    start_time = time.time()
    time_step = int(auto_drive.getBasicTimeStep())
    velocity = auto_drive.getCurrentSpeed()
    u = auto_drive.getBrakeIntensity()
    try:
        if val:
            auto_drive.setCruisingSpeed(val)
            Log.append(str(val))
            Log.append(str(time.time() - start_time))
            return
        if velocity is not None and not isnan(velocity):
            t = time_step
            load = 0
            res = odeint(dv_dt, velocity * 100, [0, t], args=(u * 100, load))
            res = float(res[-1])
            res = max(min(res, MAX_CRUISING_SPEED), -MAX_CRUISING_SPEED)
            u = max(min(u, MAX_BRAKE_INTENSITY), -MAX_BRAKE_INTENSITY)
            if res is not None:
                auto_drive.setCruisingSpeed(res)
                speedDiff = auto_drive.getCurrentSpeed() - res
                if speedDiff > 0:
                    auto_drive.setBrakeIntensity(min(speedDiff / res, 1))
                else:
                    auto_drive.setBrakeIntensity(0)
                Log.append(str(res))
                Log.append(str(time.time() - start_time))
        else:
            Log.append(str(None))
    except RuntimeError as e:
        error_Log.append("[PID] SPEED CONTROL %s" % e)

def derivative_kick(position, t, u, Kp, Tu):
    # Kp = process gain
    # Tu = process time constant
    position_dt = -position / Tu + Kp / Tu * u
    return position_dt

def follow_lane_PID(position, targetPosition, auto_drive):
    t = auto_drive.getBasicTimeStep()
    delta_t = Storage.loadData("delta_t", file_path)
    start_time = time.time()
    if delta_t is None:
        delta_t = 0.01
    e = round((targetPosition - position), 2)
    prev_position = Storage.loadData("prev_position", file_path)
    prev_integral_of_error = Storage.loadData("prev_integral_of_error", file_path)
    if prev_position is not None:  # calculate starting on second cycle
        derivative_of_position = (position - prev_position) / t
        integral_of_error = prev_integral_of_error + e * delta_t
        Storage.storeData("prev_integral_of_error", round(integral_of_error, 2), file_path)
    elif prev_integral_of_error is None:
        Storage.storeData("prev_integral_of_error", 0, file_path)
        Storage.storeData("prev_position", position, file_path)
        derivative_of_position, integral_of_error = 0, 0
    P = Kp * e  # Proportional result
    I = (Kp / Ti) * integral_of_error  # Integral result
    D = - (Kp * Td) * derivative_of_position  # Derivative result
    steering_angle = first_controller_output + P + I + D
    Storage.storeData("delta_t", round((time.time() - start_time), 4), file_path)
    if steering_angle > control_out_max:
        steering_angle = control_out_max
        integral_of_error = integral_of_error - e * delta_t
        Storage.storeData("prev_integral_of_error", round(integral_of_error, 2), file_path)
    if steering_angle < control_out_min:
        steering_angle = control_out_min
        integral_of_error = integral_of_error - e * delta_t
        Storage.storeData("prev_integral_of_error", round(integral_of_error, 2), file_path)
    y = odeint(derivative_kick, position, [0, delta_t], args=(steering_angle, Ku, Tu))
    estimated_position = int(y[-1])
    Storage.storeData("estimated_position", estimated_position, file_path)
    Log.extend([str(position), str(targetPosition), str(round(steering_angle, 2))])
    auto_drive.setSteeringAngle(round(steering_angle, 2))
    Log.append(str(delta_t))

def main(auto_drive, gps, DF_res, val):
    start_time = time.time()
    global Log, error_Log
    Log.clear()
    error_Log.clear()
    try:
        speed_control(val, auto_drive)
        if gps is not None and DF_res is not None:
            follow_lane_PID(gps, round(DF_res, 2), auto_drive)
        elif gps is not None and DF_res is None:
            error_Log.append("[PID] Warning: data fusion res is None......")
            follow_lane_PID(gps, gps, auto_drive)
        Log.append(str((time.time() - start_time)))
    except Exception as e:
        error_Log.append("[PID] IN PID CONTROL: %s\n" % e)
    with open("Logs\PID_Log.csv", 'a') as file:
        wr = writer(file, quoting=QUOTE_ALL)
        wr.writerow(Log)
    if len(error_Log):
        with open("Logs\error_Log.csv", 'a', newline="") as file:
            wr = writer(file, quoting=QUOTE_ALL)
            wr.writerow(error_Log)
main()
