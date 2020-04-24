"""

    This script responsible of velocity and position PID controls.

    Büşra Nur Bahadır 201511006

                                                                        """

import math
from scipy.integrate import odeint

import Storage


def dv_dt(v, u, t, load):
    """
      For calculating the momentum balance formula:
        m*(dv(t)/dt)=Fp*u(t)−(1/2)ρ*A*Cd*v(t)^2
     """
    Cd = 0.24  # drag coefficient
    rho = 1.225  # air density (kg/m^3)
    A = 5.0  # cross-sectional area (m^2)
    Fp = 30  # thrust parameter (N/%pedal)
    m = 500  # vehicle mass (kg)
    # derivative of the velocity over time
    res = (1.0 / (m + load)) * ((Fp * u) - (0.5 * rho * Cd * A * v ** 2))
    return res


def speed_control():
    v = auto_drive.getCurrentSpeed()  # velocity (m/s)
    if math.isnan(v):
        v = 0
    u = auto_drive.getBrakeIntensity()  # Gas Pedal Position
    t = auto_drive.getBasicTimeStep()
    load = 0  # passenger load
    v = odeint(dv_dt, v, [0, t], args=(u, load))
    v[0] = v[1]
    auto_drive.setCruisingSpeed(int(v[0]))


"""-----------------------------------------------------------------------------------------------------------"""


def follow_lane_PID(position, targetPosition):
    Kp = 0.02  # Proportional constant
    KD = 0.01  # Integral constant
    KI = 0.005  # Derivative constant
    e = round((targetPosition-position), 2)  # cross track error
    if follow_lane_PID.previousDiff is None:
        Storage.storeData("follow_lane_PID.previousDiff", e, "PID")
        follow_lane_PID.previousDiff = e
    else:
        follow_lane_PID.previousDiff = Storage.loadData("follow_lane_PID.previousDiff", "PID")

    # anti-windup mechanism
    if e > 0 and follow_lane_PID.previousDiff < 0:
        follow_lane_PID.integral = 0
    if e < 0 and follow_lane_PID.previousDiff > 0:
        follow_lane_PID.integral = 0
    follow_lane_PID.integral += e

    # compute angle by formula
    steering_angle = (Kp * e) + (KI * follow_lane_PID.integral) + (KD * (e - follow_lane_PID.previousDiff))
    return steering_angle


"""------------------------------------------------------------------------------------------------------------"""


def main(m_auto_drive, m_gps, DF_res, m_first_call):
    global auto_drive, first_call
    auto_drive = m_auto_drive
    first_call = m_first_call
    position = m_gps - 4
    if first_call:
        follow_lane_PID.integral = 0
        follow_lane_PID.previousDiff = None
    speed_control()
    if not math.isnan(position) and not math.isnan(DF_res):
        angle = round(max(min(follow_lane_PID(position, DF_res), 0.2), -0.2), 2)
        if not math.isnan(angle):
            # print("position=>", position, "target=>", DF_res, "angle=>", angle)
            auto_drive.setSteeringAngle(angle)


if __name__ == '__main__':
    main()
