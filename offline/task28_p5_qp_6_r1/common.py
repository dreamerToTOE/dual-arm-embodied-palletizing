import json
import hashlib
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
START='cbfea96775696defb864d48d4cef30ce689ff959'
ISAAC=Path('/home/ubuntu2004/isaacsim-4.5.0')
ASSET=Path('/home/ubuntu2004/Documents/Codex/2026-10-25/files-pasted-by-the-user-task/work/g1_0b_asset_audit/fr3.usd')
ASSET_SHA='3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f'
NAMES=[f'fr3_joint{i}' for i in range(1,8)]+['fr3_finger_joint1','fr3_finger_joint2']
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def serial(v):
    if isinstance(v,np.ndarray): return v.tolist()
    if isinstance(v,np.generic): return v.item()
    raise TypeError(type(v).__name__)
def dump(p,v):
    p=Path(p); p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f: json.dump(v,f,indent=2,sort_keys=True,allow_nan=False,default=serial); f.write('\n')
def config(): return json.loads((HERE/'preregistration.json').read_text())
