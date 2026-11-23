"""Kinematic playback with a planted-foot trajectory based on the Microduck GLB walk."""
from dataclasses import dataclass
import math

import mujoco as mj
import numpy as np

from design import GEOMETRY, OFFSETS, REACH, SHANK, SHANK_BIND, THIGH_BIND
from walker import DT, LEG_CONTROLS, SIDES, Walker, ik

NORMAL_FORWARD = .63
FAST_CADENCE = .70
SWING_START = .105
SWING_END = .985
LANDING_X = -.001
GROUND_GAP = .0003


def smoothstep(u):
    u = min(1., max(0., u))
    return u * u * u * (10. + u * (-15. + 6. * u))


@dataclass
class Step:
    side: int
    start_x: float
    distance: float
    foot_start: float
    foot_end: float
    duration: float
    lift: float
    elapsed: float = 0.


class ReferenceWalker(Walker):
    """Track the planted foot in world coordinates while posing the short leg chain."""

    def __init__(self, terrain, seed, *, with_visual=True):
        super().__init__(terrain, seed, with_visual=with_visual)
        self.ankle_ids = [self.m.body("chan_" + side).id for side in SIDES]
        self.hips = [
            np.asarray(OFFSETS["hong_xoay_" + side]) + OFFSETS["hong_doc_" + side]
            for side in SIDES
        ]
        self.sole_bottom = [GEOMETRY["sole_bounds"]["chan_" + side][0][2] for side in SIDES]
        self.leg_joints = [
            [int(self.m.joint(name + side).qposadr[0]) for name in LEG_CONTROLS]
            for side in SIDES
        ]
        self.leg_limits = np.array([
            [self.m.joint(name + side).range for name in LEG_CONTROLS]
            for side in SIDES
        ])
        # The lowest allowed knee angle sets the closest reachable ankle target.
        knee_bend = self.leg_limits[:, 3, 0] - THIGH_BIND + SHANK_BIND
        self.min_leg_reach_sq = REACH**2 + SHANK**2 + 2 * REACH * SHANK * np.cos(knee_bend)
        self.head_joints = [
            int(self.m.joint(name).qposadr[0])
            for name in ("nghieng_co", "gat_dau", "xoay_co", "nghieng_dau")
        ]
        self.floor_group = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.ray_down = np.array([0., 0., -1.])
        self.ray_hit = np.empty(1, dtype=np.int32)
        self.rough = self.m.geom_type[self.floor] == mj.mjtGeom.mjGEOM_HFIELD
        self.reset()

    def reset(self):
        mj.mj_resetData(self.m, self.d)
        mj.mj_forward(self.m, self.d)
        self.anchors = self.d.xpos[self.ankle_ids, :2].copy()
        self.active_step = None
        self.root_x = float(self.d.qpos[0])
        self.next_side = 0
        self.last_direction = 0.
        self.crouch = 0.
        self.gaze = 0.
        self.look = 0.
        self.nod_until = 0.
        self.fallen = False
        self.swing = [0, 0]
        self.max_tilt = 0.
        self._pose(self.anchors, (0., 0.), 0., 0.)

    def ground_z(self, x, y):
        if not self.rough:
            return math.tan(self.grade) * x
        origin = np.array([x, y, 1.])
        distance = mj.mj_ray(
            self.m, self.d, origin, self.ray_down, self.floor_group, True, -1, self.ray_hit,
        )
        return 1. - distance if distance >= 0. else 0.

    def _start_step(self, request):
        amount = abs(request)
        if request > 0.:
            fast = min(1., max(0., (amount - NORMAL_FORWARD) / (1. - NORMAL_FORWARD)))
            distance = .0345 + .0055 * fast
            duration = 1. - (1. - FAST_CADENCE) * fast
            lift = .0165 + .001 * fast
        else:
            distance, duration, lift = -.026, 1.05, .012
        direction = math.copysign(1., distance)
        if direction != self.last_direction:
            distance *= .5
        self.last_direction = direction
        landing_x = LANDING_X if direction > 0. else -.022
        side = self.next_side
        self.active_step = Step(
            side, self.root_x, distance, float(self.anchors[side, 0]),
            self.root_x + distance + landing_x, duration, lift,
        )

    def _pose(self, feet_xy, lifts, pitch, bob):
        targets = []
        for (x, y), bottom, lift in zip(feet_xy, self.sole_bottom, lifts):
            z = self.ground_z(x, y)
            slope = self.grade
            if self.rough:
                slope = math.atan2(self.ground_z(x + .015, y) - self.ground_z(x - .015, y), .030)
            # Tilt the sole at the ankle while retaining clearance at the lower edge.
            toe = -.055 * (lift / .017)
            foot_pitch = -slope + toe
            targets.append((x, y, z - bottom + GROUND_GAP + lift, foot_pitch))

        d = self.d
        ground = self.ground_z(self.root_x, 0.)
        root_z = self.m.qpos0[2] + ground - self.crouch + bob
        for side, hip_origin, (x, _, z, _) in zip(SIDES, self.hips, targets):
            dx = x - self.root_x - hip_origin[0] - OFFSETS["dui_" + side][0]
            reach_z = math.sqrt(max(.001, (REACH + SHANK - .002)**2 - dx**2))
            root_z = min(root_z, z - hip_origin[2] + reach_z)
        d.qpos[:7] = (self.root_x, 0., root_z, math.cos(pitch / 2), 0., math.sin(pitch / 2), 0.)
        cp, sp = math.cos(pitch), math.sin(pitch)
        for index, (side, ids, hip_origin, target, lift) in enumerate(
            zip(SIDES, self.leg_joints, self.hips, targets, lifts)
        ):
            x, y, z, foot_pitch = target
            dx, dz = x - self.root_x, z - root_z
            foot_x = cp * dx - sp * dz - hip_origin[0] - OFFSETS["dui_" + side][0]
            foot_y = y - hip_origin[1]
            foot_z = sp * dx + cp * dz - hip_origin[2] - OFFSETS["dui_" + side][2]
            if lift > 0.:
                min_height_sq = self.min_leg_reach_sq[index] - foot_x**2 - foot_y**2 + self.leg_widths[index]**2
                foot_z = min(foot_z, -math.sqrt(max(0., min_height_sq)))
            bank, hip, knee, ankle = ik(foot_x, foot_y, -foot_z, side)
            limits = self.leg_limits[index]
            d.qpos[ids] = np.clip((0., bank, hip, knee, ankle + foot_pitch - pitch), limits[:, 0], limits[:, 1])

        nod = 0.
        if d.time < self.nod_until:
            progress = 1. - (self.nod_until - d.time)
            nod = .21 * math.sin(math.pi * progress)**2
        d.qpos[self.head_joints] = (.25 * nod - .25 * pitch, nod - .35 * pitch, self.look, 0.)
        self.max_tilt = max(self.max_tilt, abs(math.degrees(pitch)))

    def step(self, _gait, request):
        d = self.d
        if self.active_step is None and abs(request) > .04:
            self._start_step(request)
        feet = self.anchors.copy()
        lifts = [0., 0.]
        pitch = bob = 0.
        step = self.active_step
        if step is not None:
            step.elapsed = min(step.elapsed + DT, step.duration)
            progress = step.elapsed / step.duration
            u = min(1., max(0., (progress - SWING_START) / (SWING_END - SWING_START)))
            travel = smoothstep(u)
            arch = math.sin(math.pi * u)**2
            feet[step.side, 0] = step.foot_start + (step.foot_end - step.foot_start) * travel
            lifts[step.side] = step.lift * arch
            body_travel = u * u * (3. - 2. * u)
            self.root_x = step.start_x + step.distance * body_travel
            self.crouch += min(.00012, .0048 - self.crouch)
            pitch = math.copysign(.019 * arch, step.distance)
            bob = .0012 * arch
            if progress >= 1.:
                self.anchors[step.side, 0] = step.foot_end
                self.swing[step.side] += 1
                self.next_side = 1 - step.side
                self.active_step = None

        previous = d.qpos.copy()
        self.look += (self.gaze - self.look) * (1. - math.exp(-DT / .13))
        d.time += DT
        self._pose(feet, lifts, pitch, bob)
        mj.mj_differentiatePos(self.m, d.qvel, DT, previous, d.qpos)
        mj.mj_forward(self.m, d)
