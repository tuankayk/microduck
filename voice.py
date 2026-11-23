"""Nhận lệnh ngắn ngoại tuyến qua MFCC + DTW, dùng mẫu của chính người nói."""
import ctypes
import json
from pathlib import Path
import tempfile
import time
import uuid
import wave
import numpy as np
from design import HERE

LABELS={"di":"đi tới","nhanh":"đi nhanh","lui":"đi lùi","dung":"dừng lại",
        "trai":"nhìn trái","phai":"nhìn phải","chao":"gật đầu"}
PROFILE=HERE/"giong_cua_toi.npz"
SR=16000

def capture(file,seconds=2.0):
    win=ctypes.windll.winmm
    def send(cmd):
        error=win.mciSendStringW(cmd,None,0,None)
        if error:
            buf=ctypes.create_unicode_buffer(256)
            win.mciGetErrorStringW(error,buf,len(buf))
            raise RuntimeError("Micro Windows: "+buf.value)
    alias="canard"+uuid.uuid4().hex
    send(f"open new type waveaudio alias {alias}")
    try:
        send(f"set {alias} time format milliseconds")
        send(f"set {alias} channels 1 samplespersec 16000 bitspersample 16 alignment 2 bytespersec 32000")
        send(f"record {alias}")
        time.sleep(seconds)
        send(f"stop {alias}")
        send(f'save {alias} "{Path(file).resolve()}"')
    finally:
        win.mciSendStringW(f"close {alias}",None,0,None)

def read_wav(path):
    with wave.open(str(path),"rb") as f:
        channels,bytes_,rate,n,*_=f.getparams()
        if bytes_!=2 or n/rate>8:
            raise ValueError("Dùng WAV PCM 16-bit, tối đa 8 giây.")
        samples=np.frombuffer(f.readframes(n),dtype="<i2").astype(float)
    x=samples.reshape(-1,channels).mean(axis=1)/32768
    if rate!=SR and len(x):
        x=np.interp(np.arange(0,len(x),rate/SR),np.arange(len(x)),x)
    return x

def make_bank():
    mel=lambda h:2595*np.log10(1+h/700)
    points=700*(10**(np.linspace(mel(70),mel(7200),28)/2595)-1)
    hz=np.fft.rfftfreq(512,1/SR)
    return np.stack([np.maximum(0,np.minimum((hz-a)/(b-a),(c-hz)/(c-b)))
                     for a,b,c in zip(points,points[1:],points[2:])])
BANK=make_bank()
DCT=np.cos(np.pi*np.arange(1,14)[:,None]*(np.arange(26)+.5)/26)

def features(x):
    if len(x)<600: raise ValueError("Âm thanh quá ngắn.")
    x=x-x.mean()
    frames=np.lib.stride_tricks.sliding_window_view(x,400)[::160]
    rms=np.sqrt((frames**2).mean(axis=1))
    voiced=np.flatnonzero(rms>max(.0025,.10*np.max(rms)))
    if len(voiced)<12:
        raise ValueError("Chưa nghe rõ tiếng nói.")
    a,b=max(0,voiced[0]-4),min(len(frames),voiced[-1]+6)
    frames=frames[a:b].copy()
    if len(frames)>220: raise ValueError("Hãy nói một lệnh ngắn.")
    frames[:,1:]-=.96*frames[:,:-1].copy()
    power=abs(np.fft.rfft(frames*np.hamming(400),512))**2
    c=np.log(np.maximum(power@BANK.T,1e-10)) @ DCT.T
    c-=.7*c.mean(axis=0)
    c/=np.sqrt(np.mean(c**2,axis=0)+4)
    delta=np.gradient(c,axis=0)
    return np.hstack((c,.7*delta)).astype("f")

def distance(a,b):
    n,m=len(a),len(b)
    if max(n,m)>2.35*min(n,m): return 9.
    cost=np.maximum(0,(a*a).sum(1)[:,None]+(b*b).sum(1)[None,:]-2*(a@b.T))/a.shape[1]
    prev=np.full(m+1,np.inf)
    prev[0]=0.
    for i,line in enumerate(cost,1):
        cur=np.full(m+1,np.inf)
        for j in range(max(1,int(i*m/n)-20),min(m,int(i*m/n)+20)+1):
            cur[j]=line[j-1]+min(prev[j-1],prev[j]+.04,cur[j-1]+.04)
        prev=cur
    return float(prev[-1]/max(n,m))

def load_profile(path=PROFILE):
    with np.load(path,allow_pickle=False) as data:
        return {k:[data[f"{k}_{i}"] for i in range(3)] for k in LABELS}

def predict(sample,profile):
    scored=[]
    for key,refs in profile.items():
        d=sorted(distance(sample,ref) for ref in refs)
        scored.append((.70*d[0]+.30*d[1],key))
    scored.sort()
    score,k=scored[0]
    if score>.90 or score+.035>=scored[1][0] or score>.90*scored[1][0]:
        return None
    return k

def hear(profile,wav=None):
    if wav:
        return predict(features(read_wav(wav)),profile)
    with tempfile.TemporaryDirectory() as tmp:
        f=Path(tmp)/"lenh.wav"
        capture(f)
        return predict(features(read_wav(f)),profile)

def enroll():
    print("Mỗi lần nhấn Enter, nói ngay cụm từ được yêu cầu. Thu 2 giây, trong phòng yên.")
    samples={}
    with tempfile.TemporaryDirectory() as tmp:
        f=Path(tmp)/"mau.wav"
        for k,label in LABELS.items():
            samples[k]=[]
            while len(samples[k])<3:
                input(f"{label.upper()} · mẫu {len(samples[k])+1}/3 · Enter để thu ")
                capture(f)
                try: seq=features(read_wav(f))
                except ValueError as err:
                    print(err)
                    continue
                samples[k].append(seq)
    temp=PROFILE.with_name("giong_"+uuid.uuid4().hex+".npz")
    try:
        np.savez_compressed(temp,**{f"{k}_{i}":s for k,v in samples.items() for i,s in enumerate(v)})
        temp.replace(PROFILE)
    finally:
        temp.unlink(missing_ok=True)
    print("Đã lưu mẫu giọng. Mở mô phỏng, nhấn V rồi nói một lệnh.")

