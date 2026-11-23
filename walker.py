"""Điều khiển chân qua khớp vật lý và đo từng lượt đi."""
import math
from dataclasses import dataclass, asdict
import mujoco as mj
import numpy as np
from design import build,TERRAINS

DT=.01

@dataclass(frozen=True)
class Gait:
    freq: float=1.55
    reach: float=.029
    lift: float=.020
    sway: float=.025
    stance: float=.60
    blend: float=0.

def ik(x,z):
    knee=2*math.acos(np.clip(math.hypot(x,z)/.284,.12,.997))
    return -math.atan2(x,z)-knee/2,knee

class Walker:
    def __init__(self,terrain,seed, *, with_visual=True):
        self.m=mj.MjModel.from_xml_string(build(terrain,seed,with_visual=with_visual))
        self.d=mj.MjData(self.m)
        self.base=self.m.body("than").id
        self.soles=[self.m.geom("de_"+s).id for s in ("L","R")]
        self.floor=self.m.geom("floor").id
        self.contact_faults={self.m.geom("vo_than").id,self.m.geom("kinh_dau").id}
        self.grade=math.atan2(-self.m.geom_quat[self.floor][2],self.m.geom_quat[self.floor][0])*2
        self.reset()
    def reset(self):
        mj.mj_resetData(self.m,self.d)
        self.phase=0.
        self.speed=0.
        self.gaze=0.
        self.nod_until=0.
        self.fallen=False
        self.max_tilt=0.
        self.swing=[0,0]
        self.contacts=[True,True]
        self.d.qpos[2]=.269
        hip,knee=ik(-.002,.246)
        for side in ("L","R"):
            for name,value in (("hong_",hip),("goi_",knee),("co_chan_",-hip-knee)):
                self.d.joint(name+side).qpos[0]=value
        mj.mj_forward(self.m,self.d)
    def step(self,gait,request):
        d=self.d
        rot=d.xmat[self.base].reshape(3,3)
        roll=math.atan2(rot[2,1],rot[2,2])
        pitch=math.asin(np.clip(-rot[2,0],-1,1))
        self.speed+=float(np.clip(request-self.speed,-DT*1.35,DT*1.35))
        scale=min(abs(self.speed),1.)
        self.phase=(self.phase+DT*gait.freq*(.74+.26*scale))%1
        for i,side in enumerate(("L","R")):
            p=(self.phase+.5*i)%1
            stance=gait.stance
            if p<stance:
                u=p/stance
                stride=1-2*u
                lift=0
            else:
                u=(p-stance)/(1-stance)
                stride=-1+2*(u*u*(3-2*u))
                stride-=gait.blend*2*(1-stance)/stance*u*(1-u)*(1-2*u)
                lift=gait.lift*math.sin(math.pi*u)**2*scale
            x=-.002+gait.reach*self.speed*stride
            y=-gait.sway*scale*math.cos(2*math.pi*(self.phase-.22))
            z=.246-lift
            bank=math.atan2(y,z)
            hip,knee=ik(x,math.hypot(z,y))
            d.ctrl[5*i:5*i+5]=(bank,hip,knee,-hip-knee+.46*pitch+.025*d.qvel[4]-self.grade,
                                -bank+.45*roll+.025*d.qvel[3])
        d.ctrl[10]=self.gaze
        d.ctrl[11]=.22*math.sin(2*math.pi*2*(self.nod_until-d.time)) if d.time<self.nod_until else 0.
        mj.mj_step(self.m,d,nstep=5)
        tilt=math.degrees(math.acos(np.clip(d.xmat[self.base,8],-1,1)))
        self.max_tilt=max(self.max_tilt,tilt)
        touching=set()
        for c in d.contact:
            a,b=c.geom
            if self.floor in (a,b):
                other=b if a==self.floor else a
                touching.add(other)
        if tilt>48 or touching.intersection(self.contact_faults):
            self.fallen=True
        for i,g in enumerate(self.soles):
            now=g in touching
            if now and not self.contacts[i] and scale>.2:
                self.swing[i]+=1
            self.contacts[i]=now
    def command(self,cmd):
        if cmd=="trai": self.gaze=.57
        elif cmd=="phai": self.gaze=-.57
        elif cmd=="chao":
            self.gaze=0.
            self.nod_until=self.d.time+1.

def run_trial(w,gait,duration=7,request=1.):
    w.reset()
    for _ in range(65): w.step(gait,0.)
    origin=w.d.qpos[:3].copy()
    speeds=[]
    previous=origin[0]
    for _ in range(round(duration/DT)):
        w.step(gait,request)
        speeds.append((w.d.qpos[0]-previous)/DT)
        previous=w.d.qpos[0]
        if w.fallen: break
    end=w.d.qpos[:2].copy()
    for _ in range(80):
        w.step(gait,0.)
        if w.fallen: break
    speed=float((end[0]-origin[0])*np.sign(request)/duration)
    lateral=abs(float(end[1]-origin[1]))
    ripple=float(np.std(speeds[70:])) if len(speeds)>80 else 1.
    return dict(fall=w.fallen,v=round(speed,4),ripple=round(ripple,4),tilt=round(w.max_tilt,2),
                drift=round(lateral,4),steps=w.swing.copy(),stop=round(float(np.linalg.norm(w.d.qvel[:2])),4))

