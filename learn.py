"""Tìm tham số dáng đi bằng kết quả vật lý. Học, duyệt và thi dùng hạt giống riêng."""
from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path
import uuid
import numpy as np
from walker import Gait, Walker, TERRAINS, run_trial
from design import HERE

STATE=HERE/"dang_dung.json"

def save(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+"."+uuid.uuid4().hex+".tmp")
    try:
        temp.write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding="utf-8")
        os.replace(temp,path)
    finally:
        temp.unlink(missing_ok=True)

def current():
    return Gait(**json.loads(STATE.read_text(encoding="utf-8"))["gait"]) if STATE.exists() else Gait()

def suite(seeds):
    return [(terrain,seed,Walker(terrain,seed,with_visual=False)) for seed in seeds for terrain in TERRAINS]

def assess(gait,worlds,duration=7):
    rows=[]
    for terrain,seed,w in worlds:
        row=run_trial(w,gait,duration)
        rows.append(dict(terrain=terrain,seed=seed,**row))
    return rows

def stats(rows):
    return dict(count=len(rows),falls=sum(r["fall"] for r in rows),
                v=round(float(np.mean([r["v"] for r in rows])),4),
                ripple=round(float(np.mean([r["ripple"] for r in rows])),4),
                tilt=max(r["tilt"] for r in rows),drift=max(r["drift"] for r in rows),
                min_v=min(r["v"] for r in rows),stop=max(r["stop"] for r in rows))

def stable(s):
    return s["falls"]==0 and s["min_v"]>.065 and s["tilt"]<18 and s["drift"]<.35 and s["stop"]<.055

def better(new,old):
    return stable(new) and new["v"]>=old["v"]*1.02 and new["ripple"]<=old["ripple"]

def train(seed,count):
    import msvcrt
    with (HERE/"hoc.lock").open("a+b") as lock:
        lock.seek(0)
        try:
            msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
        except OSError:
            raise SystemExit("Dang co mot vong hoc.")
        return _train(seed,count)

def _train(seed,count):
    rng=np.random.default_rng(seed)
    parent=current()
    train_worlds=suite([int(rng.integers(100000))])
    valid_worlds=suite([int(rng.integers(200000,300000)),int(rng.integers(300000,400000))])
    base=stats(assess(parent,train_worlds))
    pool=[]
    for i in range(count):
        g=Gait(round(float(rng.uniform(1.25,2.55)),3),round(float(rng.uniform(.027,.059)),4),
               round(float(rng.uniform(.016,.039)),4),round(float(rng.uniform(.013,.037)),4),
               round(float(rng.uniform(.49,.67)),3),round(float(rng.uniform(.5,1.3)),3))
        if STATE.exists() and i%2==0:
            # Explore near the current winner as well as trying fresh gaits.
            search=((1.,2.7,.20),(.020,.075,.004),(.010,.050,.005),
                    (.006,.046,.004),(.48,.73,.027),(0.,1.5,.15))
            values=[round(float(np.clip(rng.normal(v,sigma),lo,hi)),4)
                    for v,(lo,hi,sigma) in zip(asdict(parent).values(),search)]
            g=Gait(*values)
        rows=assess(g,train_worlds)
        s=stats(rows)
        pool.append(dict(gait=asdict(g),stats=s,rows=rows))
        if (i+1)%5==0: print(f"Thu {i+1}/{count}: {s['v']:.3f} m/s, {s['falls']} nga",flush=True)
    # Retest the most promising settings on separate terrain variants.
    ranked=sorted((p for p in pool if better(p["stats"],base)),
                  key=lambda p:p["stats"]["v"]-p["stats"]["ripple"],reverse=True)
    old=assess(parent,valid_worlds)
    comparisons=[]
    accepted=None
    for rec in ranked[:5]:
        rows=assess(Gait(**rec["gait"]),valid_worlds)
        comparisons.append(dict(gait=rec["gait"],rows=rows))
        if better(stats(rows),stats(old)):
            accepted=rec["gait"]
            break
    token=datetime.now().strftime("%Y%m%d_%H%M%S")+"_"+uuid.uuid4().hex[:6]
    save(HERE/"lich_su"/(token+".json"),dict(seed=seed,parent=asdict(parent),training=pool,
         baseline_validation=old,validation=comparisons,accepted=accepted))
    if accepted:
        save(STATE,dict(gait=accepted,report=token))
    print("DA DOI" if accepted else "GIU BAN CU",asdict(current()),flush=True)

def exam():
    worlds=suite([810001,810193,810377,810451])
    baseline=assess(Gait(),worlds,10)
    best=current()
    improved=assess(best,worlds,10)
    report=dict(gait=asdict(best),seconds=10,baseline=baseline,active=improved,
                before=stats(baseline),after=stats(improved))
    save(HERE/"ket_qua_thi.json",report)
    print(json.dumps({k:report[k] for k in ("before","after")},indent=2))
    return report

