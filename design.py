"""Hình học MJCF của robot hai chân. Trục x hướng tới trước, z hướng lên."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from random import Random

HERE=Path(__file__).resolve().parent
TERRAINS=("phang","tron","doc","nhap_nho")
BLUE=".025 .26 .61 1"
SKY=".12 .56 .81 1"
WHITE=".84 .89 .87 1"
DARK=".035 .063 .083 1"
METAL=".27 .36 .4 1"
GOLD=".95 .55 .09 1"

def vec(values):
    return " ".join(f"{v:.6g}" for v in values)

def geom(parent,kind,pos,size,color,**extra):
    return ET.SubElement(parent,"geom",type=kind,pos=vec(pos),size=vec(size),rgba=color,**extra)

def detail(parent,kind,pos,size,color,**extra):
    return geom(parent,kind,pos,size,color,mass="0",contype="0",conaffinity="0",**extra)

def hull(asset,parent,name,length,width,height,pos,col,bevel):
    x,y,z=length/2,width/2,height/2
    outline=[(-x+bevel,-y),(x-bevel,-y),(x,-y+bevel),(x,y-bevel),
             (x-bevel,y),(-x+bevel,y),(-x,y-bevel),(-x,-y+bevel)]
    verts=[(a*s,b*s,c) for c,s in [(-z,.84),(-z*.65,1),(z*.60,1),(z,.91)] for a,b in outline]
    ET.SubElement(asset,"mesh",name=name,vertex=vec(k for xyz in verts for k in xyz))
    ET.SubElement(parent,"geom",type="mesh",mesh=name,pos=vec(pos),rgba=col,mass="0",contype="0",conaffinity="0")

def body(parent,name,pos):
    return ET.SubElement(parent,"body",name=name,pos=vec(pos))

def joint(parent,name,axis,limits):
    ET.SubElement(parent,"joint",name=name,axis=vec(axis),range=vec(limits))

def rod(parent,start,end,r,color,mass=0):
    return ET.SubElement(parent,"geom",type="capsule",fromto=vec((*start,*end)),size=str(r),
                         rgba=color,mass=str(mass),contype="0",conaffinity="0")

def add_microduck(root, asset_dir):
    visual = ET.parse(HERE / "microduck" / "visual.xml").getroot()
    asset = root.find("asset")
    for item in visual.find("asset"):
        if "file" in item.attrib:
            item.set("file", (asset_dir / item.get("file")).as_posix())
        asset.append(item)
    chassis = root.find(".//body[@name='than']")
    for geom in chassis.iter("geom"):
        geom.set("rgba", "0 0 0 0")
    for part in visual.find("attachments"):
        target = root.find(".//body[@name='%s']" % part.get("body"))
        ET.SubElement(target, "geom", type="mesh", mesh=part.get("mesh"),
                      material=part.get("material"), mass="0", contype="0",
                      conaffinity="0", group="2")


def build(terrain="phang",seed=0, *, with_visual=True, portable=False):
    rng=Random(seed)
    root=ET.Element("mujoco",model="Canard D1")
    ET.SubElement(root,"compiler",angle="radian",autolimits="true")
    ET.SubElement(root,"option",timestep=".002",integrator="implicitfast",iterations="40")
    defaults=ET.SubElement(root,"default")
    ET.SubElement(defaults,"joint",damping=".10",armature=".0025")
    ET.SubElement(defaults,"geom",condim="3",friction=".85 .006 .0002",solref=".009 1")
    ET.SubElement(root,"statistic",center="0 0 .23",extent="1")
    visual=ET.SubElement(root,"visual")
    ET.SubElement(visual,"global",offwidth="1200",offheight="900")
    ET.SubElement(visual,"headlight",ambient=".47 .47 .47",diffuse=".64 .64 .64")
    asset=ET.SubElement(root,"asset")
    ET.SubElement(asset,"texture",name="sky",type="skybox",builtin="gradient",rgb1=".17 .27 .38",rgb2=".67 .78 .80",width="64",height="384")
    a,b,shine={
        "phang":(".40 .47 .49",".44 .50 .51",".08"),
        "tron":(".32 .49 .56",".38 .56 .61",".42"),
        "doc":(".43 .42 .36",".48 .48 .42",".11"),
        "nhap_nho":(".38 .43 .34",".45 .48 .39",".06")}[terrain]
    ET.SubElement(asset,"texture",name="tile",type="2d",builtin="checker",rgb1=a,rgb2=b,width="64",height="64")
    ET.SubElement(asset,"material",name="floor",texture="tile",texrepeat="2 2",texuniform="true",specular=shine)
    world=ET.SubElement(root,"worldbody")
    ET.SubElement(world,"light",pos="-1 -2 4",dir=".2 .4 -1",diffuse=".8 .8 .8")
    grade=rng.uniform(.025,.052) if terrain=="doc" else 0.
    mu=rng.uniform(.24,.36) if terrain=="tron" else rng.uniform(.68,.94)
    # Priority makes the ground coefficient effective even on the slippery course.
    ground=dict(name="floor",material="floor",priority="2",friction=vec((mu,.005,.0002)))
    if terrain=="nhap_nho":
        nx,ny=241,65
        amp=rng.uniform(.004,.007)
        phase=rng.uniform(0,6.28)
        values=[.5+.26*math.sin(7*x+phase)*math.cos(3*y)+.20*math.sin(11*x+2*y)
                for j in range(ny) for i in range(nx) for x,y in [((i/(nx-1)-.5)*12,(j/(ny-1)-.5)*4)]]
        ET.SubElement(asset,"hfield",name="track",nrow=str(ny),ncol=str(nx),size=vec((6,2,2*amp,.1)),elevation=vec(values))
        ET.SubElement(world,"geom",type="hfield",hfield="track",pos=vec((0,0,-amp)),**ground)
    else:
        ET.SubElement(world,"geom",type="plane",pos="0 0 0",size="0 0 .1",euler=vec((0,-grade,0)),**ground)
    for i in range(-3,12):
        for s in (-1,1):
            detail(world,"box",(i*.5,s*.72,grade*i*.5+.011),(.11,.009,.002),WHITE,euler=vec((0,-grade,0)))
    base=body(world,"than",(0,0,.278))
    ET.SubElement(base,"freejoint",name="goc")
    geom(base,"box",(-.009,0,.015),(.081,.074,.043),DARK,mass="1.45",name="vo_than")
    hull(asset,base,"giap_than",.19,.17,.108,(-.009,0,.025),BLUE,.027)
    hull(asset,base,"nap",.156,.148,.028,(-.013,0,.077),WHITE,.021)
    for s in (-1,1):
        detail(base,"box",(-.012,s*.085,.040),(.046,.003,.011),WHITE)
        for x in (-.04,-.02,0,.02):
            detail(base,"box",(x,s*.089,.039),(.005,.002,.007),DARK)
    neck=body(base,"co",(.003,0,.078))
    joint(neck,"xoay_co",(0,0,1),(-.85,.85))
    rod(neck,(-.018,0,0),(.013,0,.07),.016,METAL,.085)
    for s in (-1,1):
        rod(neck,(-.024,s*.021,.004),(.003,s*.028,.069),.006,DARK)
    head=body(neck,"dau",(.013,0,.068))
    joint(head,"gat_dau",(0,1,0),(-.40,.40))
    geom(head,"box",(0,0,.001),(.048,.093,.033),DARK,mass=".27",name="kinh_dau")
    hull(asset,head,"vo_dau",.119,.226,.091,(-.003,0,.004),WHITE,.019)
    hull(asset,head,"mu_xanh",.114,.217,.026,(-.006,0,.046),BLUE,.019)
    detail(head,"box",(.059,0,.003),(.006,.09,.028),DARK)
    for s in (-1,1):
        for x,r,color in ((.064,.035,METAL),(.069,.030,DARK),(.073,.022,SKY),(.077,.014,DARK),(.08,.006,".65 .94 1 1")):
            detail(head,"cylinder",(x,s*.056,.005),(r,.0028),color,zaxis="1 0 0")
        detail(head,"cylinder",(-.022,s*.111,.005),(.020,.005),BLUE,zaxis="0 1 0")
        rod(head,(-.04,s*.082,.048),(-.053,s*.098,.145 if s==1 else .118),.0038,DARK)
        detail(head,"sphere",(-.053,s*.098,.145 if s==1 else .118),(.0055,),GOLD)
    motors=[]
    for side,s in (("L",1),("R",-1)):
        leg=body(base,"dui_"+side,(0,s*.062,0))
        joint(leg,"hong_nghieng_"+side,(1,0,0),(-.62,.62))
        joint(leg,"hong_"+side,(0,1,0),(-1.25,.9))
        rod(leg,(0,0,0),(0,0,-.142),.022,WHITE,.21)
        rod(leg,(0,s*.023,-.025),(0,s*.023,-.115),.011,BLUE)
        detail(leg,"cylinder",(0,s*.012,0),(.037,.023),BLUE,zaxis="0 1 0")
        shin=body(leg,"ong_"+side,(0,0,-.142))
        joint(shin,"goi_"+side,(0,1,0),(.005,1.95))
        rod(shin,(0,0,0),(0,0,-.142),.017,METAL,.17)
        rod(shin,(.013,s*.019,-.017),(.013,s*.019,-.115),.008,WHITE)
        for y,r,c in ((0,.028,DARK),(s*.024,.021,WHITE),(s*.028,.009,GOLD)):
            detail(shin,"cylinder",(0,y,0),(r,.006),c,zaxis="0 1 0")
        foot=body(shin,"chan_"+side,(0,0,-.142))
        joint(foot,"co_chan_"+side,(0,1,0),(-1.3,1.0))
        joint(foot,"nghieng_chan_"+side,(1,0,0),(-.68,.68))
        geom(foot,"box",(.008,0,-.012),(.089,.046,.012),DARK,mass=".19",name="de_"+side)
        hull(asset,foot,"giay_"+side,.171,.092,.039,(.007,0,.0005),GOLD,.020)
        detail(foot,"box",(.043,0,.021),(.034,.032,.004),WHITE)
        for x in (-.044,-.024):
            detail(foot,"box",(x,0,.020),(.004,.031,.005),DARK)
        motors.extend([("hong_nghieng_",70),("hong_",75),("goi_",70),("co_chan_",55),("nghieng_chan_",50)])
    actuators=ET.SubElement(root,"actuator")
    for side,group in zip(("L","R"),(motors[:5],motors[5:])):
        for name,k in group:
            ET.SubElement(actuators,"position",name=name+side,joint=name+side,kp=str(k),kv="1.9",forcerange="-13 13")
    for name in ("xoay_co","gat_dau"):
        ET.SubElement(actuators,"position",name=name,joint=name,kp="6",kv=".34",forcerange="-1.4 1.4")
    if with_visual:
        add_microduck(root, Path("microduck") if portable else HERE / "microduck")
    return ET.tostring(root,encoding="unicode")

