"""Foot trajectories and physics-based walking for the short Microduck legs."""
import math
from dataclasses import dataclass

import mujoco as mj
import numpy as np

from design import OFFSETS, REACH, SHANK, SHANK_BIND, THIGH_BIND, build, TERRAINS

DT = .01
STAND_X = -.01453
STAND_Z = .06246
FAST_LIMIT = .82
SIDES = ("L", "R")
LEG_CONTROLS = ("xoay_hong_", "hong_nghieng_", "hong_", "goi_", "co_chan_")


@dataclass(frozen=True)
class Gait:
    freq: float = 2.701
    reach: float = .01235
    lift: float = .007
    sway: float = .0005
    stance: float = .48
    blend: float = .904


def ik(x, y, height, side):
    """Solve the offset hip roll and two pitch links in the GLB bind pose."""
    bind_y = OFFSETS["dui_" + side][1] + OFFSETS["chan_" + side][1]
    vertical = math.sqrt(max(.0004, height**2 + y**2 - bind_y**2))
    roll = math.atan2(y, height) - math.atan2(bind_y, vertical)
    cosine = (x*x + vertical*vertical - REACH**2 - SHANK**2) / (2*REACH*SHANK)
    bend = -math.acos(float(np.clip(cosine, -.84, .98)))
    thigh = -math.atan2(x, vertical) - math.atan2(SHANK * math.sin(bend), REACH + SHANK * math.cos(bend))
    hip = thigh - THIGH_BIND
    knee = bend + THIGH_BIND - SHANK_BIND
    ankle = -hip - knee
    return roll, hip, knee, ankle


class Walker:
    def __init__(self, terrain, seed, *, with_visual=True):
        self.m = mj.MjModel.from_xml_string(build(terrain, seed, with_visual=with_visual))
        self.d = mj.MjData(self.m)
        self.base = self.m.body("than").id
        self.soles = [self.m.geom("de_" + side).id for side in SIDES]
        self.floor = self.m.geom("floor").id
        self.contact_faults = {self.m.geom(name).id for name in ("vo_than", "kinh_dau")}
        self.grade = 2 * math.atan2(-self.m.geom_quat[self.floor][2], self.m.geom_quat[self.floor][0])
        self.leg_ids = [[self.m.actuator(name + side).id for name in LEG_CONTROLS] for side in SIDES]
        self.head_ids = [self.m.actuator(name).id for name in ("nghieng_co", "gat_dau", "xoay_co", "nghieng_dau")]
        self.leg_widths = [OFFSETS["dui_" + side][1] + OFFSETS["chan_" + side][1] for side in SIDES]
        Walker.reset(self)

    def reset(self):
        mj.mj_resetData(self.m, self.d)
        self.phase = 0.
        self.speed = 0.
        self.gaze = 0.
        self.nod_until = 0.
        self.fallen = False
        self.max_tilt = 0.
        self.swing = [0, 0]
        self.air_time = [0., 0.]
        self.last_step = [-math.inf, -math.inf]
        self.d.qpos[2] += STAND_Z + OFFSETS["ong_L"][2] + OFFSETS["chan_L"][2] + .0008
        for side, ids, width in zip(SIDES, self.leg_ids, self.leg_widths):
            values = (0., *ik(STAND_X, width, STAND_Z, side))
            for name, value in zip(LEG_CONTROLS, values):
                self.d.joint(name + side).qpos[0] = value
            self.d.ctrl[ids] = values
        mj.mj_forward(self.m, self.d)

    def step(self, gait, request):
        d = self.d
        rot = d.xmat[self.base].reshape(3, 3)
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = math.asin(float(np.clip(-rot[2, 0], -1, 1)))
        yaw = math.atan2(rot[1, 0], rot[0, 0])
        request = min(request, FAST_LIMIT)
        self.speed += float(np.clip(request - self.speed, -DT * 1.7, DT * 1.7))
        scale = min(abs(self.speed), 1.)
        self.phase = (self.phase + DT * gait.freq * (.78 + .22 * scale)) % 1
        shift = -gait.sway * scale * math.cos(2 * math.pi * (self.phase - .388))
        ankle_trim = .76 * pitch + .01134 * d.qvel[4] - self.grade
        heading = float(np.clip(-.23 * yaw - .0012 * d.qvel[5], -.17, .17))
        capture = .02284 * pitch + .03304 * (d.qvel[0] - .045 * self.speed)
        for i, (side, ids, width) in enumerate(zip(SIDES, self.leg_ids, self.leg_widths)):
            p = (self.phase + .5 * i) % 1
            if p < gait.stance:
                u = p / gait.stance
                stride, lift = 1 - 2*u, 0.
            else:
                u = (p - gait.stance) / (1 - gait.stance)
                stride = -1 + 2 * u*u*(3 - 2*u)
                stride -= gait.blend * 2 * (1 - gait.stance) / gait.stance * u*(1 - u)*(1 - 2*u)
                lift = gait.lift * math.sin(math.pi * u)**2 * scale**.953
            x = STAND_X + gait.reach * self.speed * stride + capture
            height = STAND_Z - lift
            bank, hip, knee, ankle = ik(x, width + shift, height, side)
            d.ctrl[ids] = (
                heading, bank - .04 * roll + .00069 * d.qvel[3],
                hip, knee, ankle + ankle_trim + .146 * lift / gait.lift,
            )
        nod = 0.
        if d.time < self.nod_until:
            progress = 1. - (self.nod_until - d.time)
            nod = .21 * math.sin(math.pi * progress)**2
        d.ctrl[self.head_ids] = (.35 * nod - .25 * pitch, nod - .30 * pitch, self.gaze, -.48 * roll)
        mj.mj_step(self.m, d, nstep=5)
        tilt = math.degrees(math.acos(float(np.clip(d.xmat[self.base, 8], -1, 1))))
        self.max_tilt = max(self.max_tilt, tilt)
        touching = {int(c.geom[0] if c.geom[1] == self.floor else c.geom[1])
                    for c in d.contact if self.floor in c.geom}
        if tilt > 48 or touching.intersection(self.contact_faults):
            self.fallen = True
        for i, sole in enumerate(self.soles):
            if sole not in touching:
                self.air_time[i] += DT
                continue
            if (scale > .2 and self.air_time[i] >= .025
                    and d.time - self.last_step[i] > .55 / gait.freq):
                self.swing[i] += 1
                self.last_step[i] = d.time
            self.air_time[i] = 0.

    def command(self, cmd):
        if cmd == "trai":
            self.gaze = .47
        elif cmd == "phai":
            self.gaze = -.47
        elif cmd == "chao":
            self.gaze = 0.
            self.nod_until = self.d.time + 1.


def run_trial(w, gait, duration=7, request=1.):
    w.reset()
    for _ in range(65):
        w.step(gait, 0.)
    origin = w.d.qpos[:3].copy()
    speeds = []
    previous = origin[0]
    for _ in range(round(duration / DT)):
        w.step(gait, request)
        speeds.append((w.d.qpos[0] - previous) / DT)
        previous = w.d.qpos[0]
        if w.fallen:
            break
    end = w.d.qpos[:2].copy()
    for _ in range(80):
        w.step(gait, 0.)
        if w.fallen:
            break
    speed = float((end[0] - origin[0]) * np.sign(request) / duration)
    lateral = abs(float(end[1] - origin[1]))
    ripple = float(np.std(speeds[70:])) if len(speeds) > 80 else 1.
    return dict(fall=w.fallen, v=round(speed, 4), ripple=round(ripple, 4), tilt=round(w.max_tilt, 2),
                drift=round(lateral, 4), steps=w.swing.copy(), stop=round(float(np.linalg.norm(w.d.qvel[:2])), 4))


