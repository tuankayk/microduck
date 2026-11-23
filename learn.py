"""Search and validate gait settings for the Microduck skeleton."""
from dataclasses import asdict
from datetime import datetime
import json
import os
import uuid

import numpy as np

from design import HERE, TERRAINS
from walker import Gait, Walker, run_trial

STATE = HERE / "dang_dung.json"
MODEL = "Microduck-GLB-v2"
SEARCH = (
    (2.05, 3.20, .11), (.008, .023, .0012), (.005, .014, .0008),
    (0., .006, .0006), (.47, .60, .012), (.65, 1.25, .065),
)


def save(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        temp.write_text(json.dumps(obj, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(temp, path)
    finally:
        temp.unlink(missing_ok=True)


def current():
    if STATE.exists():
        state = json.loads(STATE.read_text(encoding="utf-8"))
        if state.get("model") == MODEL:
            return Gait(**state["gait"])
    return Gait()


def suite(seeds):
    return [(terrain, seed, Walker(terrain, seed, with_visual=False)) for seed in seeds for terrain in TERRAINS]


def assess(gait, worlds, duration=7, request=1.):
    return [
        dict(terrain=terrain, seed=seed, **run_trial(walker, gait, duration, request))
        for terrain, seed, walker in worlds
    ]


def stats(rows):
    return dict(
        count=len(rows), falls=sum(r["fall"] for r in rows),
        v=round(float(np.mean([r["v"] for r in rows])), 4),
        ripple=round(float(np.mean([r["ripple"] for r in rows])), 4),
        tilt=max(r["tilt"] for r in rows), drift=max(r["drift"] for r in rows),
        min_v=min(r["v"] for r in rows), stop=max(r["stop"] for r in rows),
    )


def stable(summary, minimum=.010):
    return (summary["falls"] == 0 and summary["min_v"] > minimum and summary["tilt"] < 23
            and summary["drift"] < .24 and summary["stop"] < .045)


def better(new, old):
    return stable(new) and new["v"] >= old["v"] * 1.02 and new["ripple"] <= old["ripple"]


def train(seed, count):
    import msvcrt
    with (HERE / "hoc.lock").open("a+b") as lock:
        lock.seek(0)
        try:
            msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
        except OSError:
            raise SystemExit("Đang có một vòng học.")
        return _train(seed, count)


def _train(seed, count):
    rng = np.random.default_rng(seed)
    parent = current()
    train_worlds = suite([int(rng.integers(100000))])
    valid_worlds = suite([int(rng.integers(200000, 300000)), int(rng.integers(300000, 400000))])
    base = stats(assess(parent, train_worlds))
    pool = []
    for i in range(count):
        if i % 4 == 3:
            values = [round(float(rng.uniform(low, high)), 5) for low, high, _ in SEARCH]
        else:
            values = [
                round(float(np.clip(rng.normal(old, sigma), low, high)), 5)
                for old, (low, high, sigma) in zip(asdict(parent).values(), SEARCH)
            ]
        gait = Gait(*values)
        rows = assess(gait, train_worlds)
        summary = stats(rows)
        pool.append(dict(gait=asdict(gait), stats=summary, rows=rows))
        if (i + 1) % 5 == 0:
            print(f"Thử {i + 1}/{count}, {summary['v']:.3f} m/s, {summary['falls']} ngã", flush=True)

    ranked = sorted((item for item in pool if better(item["stats"], base)),
                    key=lambda item: item["stats"]["v"] - item["stats"]["ripple"], reverse=True)
    old = assess(parent, valid_worlds)
    comparisons = []
    accepted = None
    for candidate in ranked[:5]:
        gait = Gait(**candidate["gait"])
        rows = assess(gait, valid_worlds)
        if not better(stats(rows), stats(old)):
            comparisons.append(dict(gait=candidate["gait"], forward=rows))
            continue
        slow = assess(gait, valid_worlds, request=.63)
        reverse = assess(gait, valid_worlds, request=-.55)
        comparisons.append(dict(gait=candidate["gait"], forward=rows, slow=slow, reverse=reverse))
        if stable(stats(slow), .006) and stable(stats(reverse), .006):
            accepted = candidate["gait"]
            break

    token = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6]
    save(HERE / "lich_su" / (token + ".json"),
         dict(model=MODEL, seed=seed, parent=asdict(parent), training=pool,
              baseline_validation=old, validation=comparisons, accepted=accepted))
    if accepted:
        save(STATE, dict(model=MODEL, gait=accepted, report=token))
    print("Đã đổi dáng đi" if accepted else "Giữ dáng đi hiện tại", asdict(current()), flush=True)


def exam():
    worlds = suite([810001, 810193, 810377, 810451])
    baseline = assess(Gait(), worlds, 10)
    best = current()
    forward = baseline if best == Gait() else assess(best, worlds, 10)
    slow = assess(best, worlds, 10, request=.63)
    reverse = assess(best, worlds, 10, request=-.55)
    report = dict(
        model=MODEL, gait=asdict(best), seconds=10, baseline=baseline,
        active=forward, slow=slow, reverse=reverse,
        before=stats(baseline), after=stats(forward), slow_stats=stats(slow), reverse_stats=stats(reverse),
    )
    save(HERE / "ket_qua_thi.json", report)
    print(json.dumps({key: report[key] for key in ("after", "slow_stats", "reverse_stats")}, indent=2))
    return report
