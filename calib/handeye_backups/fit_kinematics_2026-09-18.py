import math, numpy as np, json, glob, os
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
from harness_vision.geom import inv, xyzrpy_from_T
from pyAgxArm.api.constants import ROBOT_MDH_PRESET
MDH=[list(l) for l in ROBOT_MDH_PRESET["piper"]]
def link(d,a,al,th):
    ca,sa,ct,st=math.cos(al),math.sin(al),math.cos(th),math.sin(th)
    return np.array([[ct,-st,0,a],[ca*st,ca*ct,-sa,-sa*d],[sa*st,sa*ct,ca,ca*d],[0,0,0,1.0]])
def fk(q,dl=np.zeros(6),mdh=MDH):
    T=np.eye(4)
    for (d,a,al,off),qi,di in zip(mdh,q,dl): T=T@link(d,a,al,qi+off+di)
    return T
def T_of(v): T=np.eye(4); T[:3,:3]=Rot.from_rotvec(v[3:6]).as_matrix(); T[:3,3]=v[:3]; return T
S=json.load(open("/tmp/ident_samples.json"))
M=json.load(open("/tmp/minihe_pairs.json"))
for p in M: S.append({"joints":p["joints"],"base_flange":p["base_flange"],"cam_tag":p["cam_tag"],"src":"mini"})
f=sorted(glob.glob("calib/handeye_pairs_2026-09-18*.json"),key=os.path.getmtime)[-1]
C=json.load(open(f))["pairs"]; q_seed=np.array(S[0]["joints"])
def ik(T,seed):
    def r(q):
        Tq=fk(q); return np.concatenate([(Tq[:3,3]-T[:3,3])*1000, Rot.from_matrix(Tq[:3,:3].T@T[:3,:3]).as_rotvec()*100])
    s=least_squares(r,seed); return s.x, np.abs(s.fun).max()
nC=0
for p in C:
    q,e=ik(np.array(p["base_flange"]),q_seed)
    if e<0.05: S.append({"joints":q.tolist(),"base_flange":p["base_flange"],"cam_tag":p["cam_tag"],"src":"cart"}); nC+=1
print("samples: ident",len(S)-len(M)-nC,"mini",len(M),"cartesian(IK)",nC,"total",len(S))
Q=np.array([s["joints"] for s in S]); CT=np.array([s["cam_tag"] for s in S])
d=json.load(open("/tmp/minihe_T.json")); T_fc0=np.array(d["T_fc"]); T_bt0=np.array(d["T_bt"])
def unpack(x,mode):
    T_fc=T_of(x[0:6]); T_bt=T_of(x[6:12]); dl=np.zeros(6); k=1.0
    if "off" in mode: dl[1:5]=x[12:16]
    if "scale" in mode: k=x[-1]
    return T_fc,T_bt,dl,k
def resid(x,mode,lever=0.17):
    T_fc,T_bt,dl,k=unpack(x,mode); r=[]
    for q,Tct in zip(Q,CT):
        pred=inv(fk(q,dl)@T_fc)@T_bt
        r+=list((pred[:3,3]-k*Tct[:3,3])*1000)
        r+=list(Rot.from_matrix(pred[:3,:3].T@Tct[:3,:3]).as_rotvec()*lever*1000)
    return np.array(r)
x0=np.concatenate([T_fc0[:3,3],Rot.from_matrix(T_fc0[:3,:3]).as_rotvec(),T_bt0[:3,3],Rot.from_matrix(T_bt0[:3,:3]).as_rotvec()])
for mode in ("he","he+scale","he+off","he+off+scale"):
    x=x0.copy()
    if "off" in mode: x=np.concatenate([x,np.zeros(4)])
    if "scale" in mode: x=np.concatenate([x,[1.0]])
    s=least_squares(resid,x,args=(mode,))
    T_fc,T_bt,dl,k=unpack(s.x,mode)
    R=s.fun.reshape(len(S),6); pos=np.linalg.norm(R[:,:3],axis=1); ang=np.linalg.norm(R[:,3:],axis=1)/170
    print(f"\n== {mode}: pos rms {np.sqrt((pos**2).mean()):.2f} mm max {pos.max():.2f} | orient rms {math.degrees(np.sqrt((ang**2).mean())):.2f} deg max {math.degrees(ang.max()):.2f}")
    x6=xyzrpy_from_T(T_fc); print("   flange->cam xyz mm",np.round(np.array(x6[:3])*1000,1),"rpy deg",np.round(np.degrees(x6[3:]),2))
    if "off" in mode: print("   joint offsets j2..j5 deg", np.round(np.degrees(dl[1:5]),3))
    if "scale" in mode: print(f"   scale k {k:.4f} -> implied tag size {34.7*k:.1f} mm")
    if mode=="he+off":
        print("   per-sample pos residual mm:", np.round(pos,1))
        json.dump({"T_fc":T_fc.tolist(),"T_bt":T_bt.tolist(),"joint_offsets_rad":dl.tolist()},open("/tmp/fit_heoff.json","w"))

print("\n#### validation")
src=np.array([s.get("src","ident") for s in S])
def fit_on(mask,mode,x0=x0):
    global Q,CT
    Qa,CTa=Q,CT; Q,CT=Qa[mask],CTa[mask]
    x=np.concatenate([x0,np.zeros(4)]) if "off" in mode else x0.copy()
    s=least_squares(resid,x,args=(mode,)); Q,CT=Qa,CTa; return s.x
def evaluate(x,mask,mode):
    global Q,CT
    Qa,CTa=Q,CT; Q,CT=Qa[mask],CTa[mask]; r=resid(x,mode).reshape(-1,6); Q,CT=Qa,CTa
    pos=np.linalg.norm(r[:,:3],axis=1); return np.sqrt((pos**2).mean()), pos.max()
train=src!="cart"; test=src=="cart"
x=fit_on(train,"he+off"); print("train ident+mini -> test cartesian: rms %.2f max %.2f mm"%evaluate(x,test,"he+off"), " offsets", np.round(np.degrees(x[12:16]),2))
x=fit_on(test,"he+off"); print("train cartesian -> test ident+mini: rms %.2f max %.2f mm"%evaluate(x,train,"he+off"), " offsets", np.round(np.degrees(x[12:16]),2))
# single-offset model: only j5
def resid5(x): 
    xx=np.concatenate([x[:12],[0,0,0,x[12]]]); return resid(xx,"he+off")
s=least_squares(resid5,np.concatenate([x0,[0.0]])); r=s.fun.reshape(-1,6); pos=np.linalg.norm(r[:,:3],axis=1)
print(f"j5-only offset: rms {np.sqrt((pos**2).mean()):.2f} max {pos.max():.2f} mm, j5 offset {math.degrees(s.x[12]):.2f} deg")
