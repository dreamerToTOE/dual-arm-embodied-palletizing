"""Offline fail-closed controls for R7 controller/model gates."""
from __future__ import annotations
import numpy as np

def validate_mass(M):
 M=np.asarray(M,float); return M.shape==(9,9) and np.isfinite(M).all() and np.allclose(M,M.T,atol=1e-5) and np.linalg.eigvalsh((M+M.T)/2).min()>0

def validate_command(u,prev,limits,rates,dt=1/60):
 u=np.asarray(u,float); prev=np.asarray(prev,float); limits=np.asarray(limits,float); rates=np.asarray(rates,float)
 if u.shape!=(7,) or not np.isfinite(u).all(): return False
 return bool(np.all(np.abs(u)<=limits) and np.all(np.abs(u-prev)<=rates*dt))
