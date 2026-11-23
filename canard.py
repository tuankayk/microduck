"""Điều khiển mô phỏng, ghi mẫu giọng và huấn luyện dáng đi."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from queue import SimpleQueue
import sys
import time
import mujoco.viewer
import numpy as np
from design import HERE,TERRAINS
from walker import Walker,DT
from learn import current,train,exam
from voice import PROFILE,LABELS,load_profile,hear,enroll

KEYS={32:"dung",265:"di",264:"lui",263:"trai",262:"phai",70:"nhanh",71:"chao"}
SPEED={"di":.63,"nhanh":1.,"lui":-.55,"dung":0.}

def action(w,old,k):
    if k in SPEED: return SPEED[k]
    if k in LABELS: w.command(k)
    return old

def play(terrain,seed):
    w=Walker(terrain,seed)
    gait=current()
    q=SimpleQueue()
    request=0.
    pending=None
    listen_id=0
    profile=load_profile() if PROFILE.exists() else None
    print("Mũi tên lên/xuống: đi/lùi · F: nhanh · Space: dừng · trái/phải: nhìn · G: gật · R: đặt lại.")
    print("Nhấn V, nói ngay sau dòng ĐANG NGHE." if profile else "Để dùng micro, thu mẫu một lần bằng Vit.cmd giong.",flush=True)
    with ThreadPoolExecutor(max_workers=1) as pool, mujoco.viewer.launch_passive(w.m,w.d,key_callback=q.put) as viewer:
        viewer.cam.azimuth=142
        viewer.cam.elevation=-11
        viewer.cam.distance=.53
        tick=0
        while viewer.is_running():
            start=time.perf_counter()
            while not q.empty():
                key=q.get()
                if key==86:
                    if profile is None:
                        print("Chưa có mẫu giọng. Chạy Vit.cmd giong.",flush=True)
                    elif pending is None:
                        print("ĐANG NGHE · nói một lệnh",flush=True)
                        pending=(listen_id,pool.submit(hear,profile))
                elif key==82:
                    listen_id+=1
                    request=0.
                    w.reset()
                elif key in KEYS:
                    listen_id+=1
                    request=action(w,request,KEYS[key])
            if pending is not None:
                uid,future=pending
                if future.done():
                    pending=None
                    try:
                        word=future.result()
                        if uid==listen_id:
                            if word:
                                request=action(w,request,word)
                                print("Đã nghe:",LABELS[word],flush=True)
                            else:
                                print("Chưa rõ lệnh. Nhấn V để nói lại.",flush=True)
                    except (RuntimeError,OSError,ValueError) as error:
                        print(error,flush=True)
            if w.fallen:
                request=0.
            if abs(w.d.qpos[0])>5.25 and request*w.d.qpos[0]>0:
                print("Đã tới cuối đường thử. Nhấn R để về đầu đường.",flush=True)
                request=0.
            was_fallen=w.fallen
            w.step(gait,request)
            if w.fallen and not was_fallen: print("Robot đã ngã. Nhấn R để đặt lại.",flush=True)
            if tick%2==0:
                viewer.cam.lookat[:]=(w.d.qpos[0]+.016,w.d.qpos[1],.126)
                viewer.sync()
            tick+=1
            time.sleep(max(0.,DT-(time.perf_counter()-start)))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("mode",choices=("chay","giong","nghe","hoc","thi"),nargs="?",default="chay")
    parser.add_argument("--san",choices=TERRAINS,default="phang")
    parser.add_argument("--seed",type=int)
    parser.add_argument("--thu",type=int,default=32)
    parser.add_argument("--wav",type=str)
    a=parser.parse_args()
    seed=a.seed if a.seed is not None else int(np.random.SeedSequence().generate_state(1)[0])
    if seed<0 or a.thu<1: parser.error("Seed phải ≥ 0 và số lần thử phải ≥ 1.")
    if a.mode=="chay": play(a.san,seed)
    elif a.mode=="hoc": train(seed,a.thu)
    elif a.mode=="thi": exam()
    elif a.mode=="giong": enroll()
    elif a.mode=="nghe":
        if not PROFILE.exists(): raise SystemExit("Chạy Vit.cmd giong trước để robot học giọng bạn.")
        print("ĐANG NGHE",flush=True)
        word=hear(load_profile(),a.wav)
        print(LABELS.get(word,"Chưa rõ lệnh."))
        if word is None: raise SystemExit(1)

if __name__=="__main__":
    main()


