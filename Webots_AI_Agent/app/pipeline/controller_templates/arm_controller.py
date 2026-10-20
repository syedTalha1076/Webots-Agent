"""Generated controller: 3-joint arm (yaw, shoulder, elbow) doing pick-and-place of one cube.

 * Analytical inverse kinematics (2-link planar arm in the vertical plane + base yaw).
 * "Grasp" is a MAGNETIC/attach grasp implemented with the Supervisor API: while attached the cube is moved with
   the arm tip each step. (Real finger-based grasping is not modelled.)
 * Cube pose is read through the Supervisor API (ground truth) for pick targeting and for the final check.
"""
import math

from controller import Supervisor

from agent_common import clamp, load_config, write_result

CFG = load_config()
G = CFG["arm"]


def ik(x, y, z):
    """Joint angles (yaw, shoulder, elbow) that put the tip at (x, y, z) in the ARM-BASE frame (z up from the base plate).
    Returns None if unreachable. Convention matches the generated model: shoulder/elbow axes are +y, so a positive
    angle swings the link DOWN; link directions are phi1 = -shoulder, phi2 = phi1 - elbow."""
    yaw = math.atan2(y, x)
    r = math.hypot(x, y)
    zz = z - G["h0"]
    l1, l2 = G["l1"], G["l2"]
    d2 = r * r + zz * zz
    c = (d2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    if abs(c) > 1.0:
        return None
    beta = -math.acos(c)                                  # elbow-up: link 2 bends downward relative to link 1
    phi1 = math.atan2(zz, r) - math.atan2(l2 * math.sin(beta), l1 + l2 * math.cos(beta))
    shoulder, elbow = -phi1, -beta
    lim = G["limits"]
    if not (lim["shoulder"][0] <= shoulder <= lim["shoulder"][1] and lim["elbow"][0] <= elbow <= lim["elbow"][1]):
        return None
    return yaw, shoulder, elbow


def fk(yaw, shoulder, elbow):
    phi1 = -shoulder
    phi2 = phi1 - elbow
    r = G["l1"] * math.cos(phi1) + G["l2"] * math.cos(phi2)
    z = G["h0"] + G["l1"] * math.sin(phi1) + G["l2"] * math.sin(phi2)
    return r * math.cos(yaw), r * math.sin(yaw), z


def main():
    robot = Supervisor()
    step_ms = int(robot.getBasicTimeStep())
    name = robot.getName()
    me = robot.getSelf()
    cube = robot.getFromDef("OBJECT")
    tip = robot.getFromDef("ARM_TIP")
    if cube is None or tip is None:
        write_result(name, False, "world is missing DEF OBJECT / DEF ARM_TIP", {})
        return
    motors = [robot.getDevice(n) for n in ("yaw motor", "shoulder motor", "elbow motor")]
    sensors = [robot.getDevice(n) for n in ("yaw sensor", "shoulder sensor", "elbow sensor")]
    for m in motors:
        m.setVelocity(1.2)
    for s in sensors:
        s.enable(step_ms)
    cube_t = cube.getField("translation")
    base = G["base"]                                      # world position of the base plate centre (x, y, z)
    pick, place = G["pick"], G["place"]                   # world xy
    cz = G["cube_z"]                                      # cube centre height when resting on the table
    hover, lift = 0.14, 0.12
    off = 0.03                                            # tip is held this far above the cube centre
    plan = [
        ("move above the cube", (pick[0], pick[1], cz + hover), False),
        ("descend to the cube", (pick[0], pick[1], cz + off), False),
        ("grasp (attach)", None, True),
        ("lift the cube", (pick[0], pick[1], cz + lift), True),
        ("move above the drop zone", (place[0], place[1], cz + lift), True),
        ("lower the cube", (place[0], place[1], cz + off + 0.004), True),
        ("release", None, False),
        ("retreat upward", (place[0], place[1], cz + hover), False),
    ]
    k, phase_t, attached, settle_until, result = 0, 0.0, False, None, None
    targets = None
    print("[%s] pick-and-place started: pick=%s place=%s" % (name, pick, place))

    def start_phase(i):
        nonlocal targets, phase_t, attached
        label, pos, att = plan[i]
        phase_t = robot.getTime()
        print("[%s] step %d/%d: %s" % (name, i + 1, len(plan), label))
        if pos is None:
            targets = None
            attached = att
            return True
        sol = ik(pos[0] - base[0], pos[1] - base[1], pos[2] - base[2])
        if sol is None:
            write_result(name, False, "target %s is not reachable by the arm" % (pos,), {})
            return False
        targets = sol
        for m, a in zip(motors, sol):
            m.setPosition(a)
        return True

    if not start_phase(0):
        return
    while robot.step(step_ms) != -1:
        t = robot.getTime()
        if attached:
            p = tip.getPosition()
            cube_t.setSFVec3f([p[0], p[1], p[2] - off])
            cube.resetPhysics()
        if result is not None:
            continue
        if t > CFG["timeout_s"]:
            result = write_result(name, False, "simulation time limit exceeded", {})
            continue
        if settle_until is not None:
            if t >= settle_until:
                p = cube.getPosition()
                d = math.hypot(p[0] - place[0], p[1] - place[1])
                ok = d < 0.05 and abs(p[2] - cz) < 0.03
                result = write_result(name, ok, "cube %.3f m from the drop target" % d + ("" if ok else " (not placed)"),
                                      {"time_s": round(t, 2), "final_error_m": round(d, 4), "grasp": "magnetic attach",
                                       "cube_position_source": "supervisor ground truth"})
            continue
        done = False
        if targets is None:
            done = t - phase_t > 0.3
        else:
            err = max(abs(s.getValue() - a) for s, a in zip(sensors, targets))
            done = err < 0.012 or t - phase_t > 8.0
        if done:
            k += 1
            if k >= len(plan):
                settle_until = t + 1.5
                continue
            if not start_phase(k):
                result = True


if __name__ == "__main__":
    main()
