"""Generated controller: Mavic 2 Pro waypoint flight (EXPERIMENTAL).

Stabilisation constants and motor mixing follow the structure of Webots' own `mavic2pro` example controller;
horizontal waypoint tracking is a simple P/D law on top of it. Pose comes from the drone's GPS, InertialUnit and Gyro.
"""
import math

from controller import Robot

from agent_common import clamp, load_config, wrap, write_result

CFG = load_config()
K_THRUST, K_OFFSET, K_VERT_P, K_ROLL_P, K_PITCH_P = 68.5, 0.6, 3.0, 50.0, 30.0


def main():
    robot = Robot()
    step_ms = int(robot.getBasicTimeStep())
    name = robot.getName()
    dev = robot.getDevice
    imu, gps, gyro = dev("inertial unit"), dev("gps"), dev("gyro")
    for d in (imu, gps, gyro):
        d.enable(step_ms)
    motors = {n: dev(n + " propeller") for n in ("front left", "front right", "rear left", "rear right")}
    for m in motors.values():
        m.setPosition(float("inf"))
        m.setVelocity(1.0)
    alt = CFG["altitude"]
    wps = [tuple(w) for w in CFG["waypoints"]]
    tol_xy, k = 0.35, 0.9
    idx, hold_until, prev, vel = 0, None, None, (0.0, 0.0)
    ok_alt_since, airborne, result = None, False, None
    metrics = {}
    print("[%s] drone controller started: altitude=%.2f m, %d waypoint(s)" % (name, alt, len(wps)))
    while robot.step(step_ms) != -1:
        t = robot.getTime()
        if t < 1.0:
            continue                                         # let motors spin up (as in Webots' example)
        if result is not None:
            continue
        roll, pitch, yaw = imu.getRollPitchYaw()
        x, y, z = gps.getValues()
        droll, dpitch, _ = gyro.getValues()
        dt = step_ms / 1000.0
        if prev is not None:
            vel = ((x - prev[0]) / dt, (y - prev[1]) / dt)
        prev = (x, y)
        if t > CFG["timeout_s"]:
            result = write_result(name, False, "simulation time limit exceeded", {"altitude_m": round(z, 2)})
            continue
        if abs(z - alt) < 0.15:
            ok_alt_since = ok_alt_since or t
        airborne = airborne or (ok_alt_since is not None and t - ok_alt_since > 1.0)
        pitch_d = roll_d = 0.0
        if airborne and idx < len(wps):
            wx, wy = wps[idx]
            ex, ey = wx - x, wy - y
            if math.hypot(ex, ey) < tol_xy:
                print("[%s] reached waypoint %d/%d at t=%.1fs" % (name, idx + 1, len(wps), t))
                idx += 1
            else:
                exb = math.cos(yaw) * ex + math.sin(yaw) * ey          # world -> body frame
                eyb = -math.sin(yaw) * ex + math.cos(yaw) * ey
                vxb = math.cos(yaw) * vel[0] + math.sin(yaw) * vel[1]
                vyb = -math.sin(yaw) * vel[0] + math.cos(yaw) * vel[1]
                pitch_d = -clamp(k * exb - 0.6 * vxb, -1.2, 1.2)        # negative pitch disturbance = forward
                roll_d = clamp(k * eyb - 0.6 * vyb, -0.8, 0.8)          # positive roll disturbance = left
        elif airborne and idx >= len(wps):
            hold_until = hold_until or t + 3.0
            if t > hold_until:
                result = write_result(name, True, "visited %d waypoint(s) and hovered" % len(wps),
                                      {"time_s": round(t, 1), "altitude_m": round(z, 2), "pose_source": "gps"})
                continue
        roll_in = K_ROLL_P * clamp(roll, -1.0, 1.0) + droll + roll_d
        pitch_in = K_PITCH_P * clamp(pitch, -1.0, 1.0) + dpitch + pitch_d
        vert_in = K_VERT_P * clamp(alt - z + K_OFFSET, -1.0, 1.0) ** 3.0
        motors["front left"].setVelocity(K_THRUST + vert_in + pitch_in - roll_in)
        motors["front right"].setVelocity(-(K_THRUST + vert_in + pitch_in + roll_in))
        motors["rear left"].setVelocity(-(K_THRUST + vert_in - pitch_in - roll_in))
        motors["rear right"].setVelocity(K_THRUST + vert_in - pitch_in + roll_in)


if __name__ == "__main__":
    main()
