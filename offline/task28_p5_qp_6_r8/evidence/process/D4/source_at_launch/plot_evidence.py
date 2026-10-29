"""Scientific trace plots; labels separate observations from model predictions."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_p5_qp_6_r8.controller import gains,reference
HERE=Path(__file__).resolve().parent

def frames(path):
 rows=[json.loads(x) for x in path.read_text().splitlines()]
 i=next((i for i,f in enumerate(rows) if f['arms']['left']['position_drives_disabled']),len(rows))
 return rows[i:]

def main():
 model=RobotModel();fig,axs=plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
 for ax,side in zip(axs,('left','right')):
  for name in ('S1','S3'):
   rows=frames(HERE.parent/'task28_p5_qp_6_r7/evidence/diagnostic'/name/'frames.jsonl');t=np.array([r['time_s'] for r in rows]);tcp=np.array([r['arms'][side]['tcp_world'] for r in rows]);ax.plot(t-t[0],1000*np.linalg.norm(tcp[:,:3,3]-tcp[0,:3,3],axis=1),label='R7 '+name+' measured')
  a=rows[0]['arms'][side];M=np.array(a['M'])[:7,:7];q0=np.array(a['q'][:7]);v0=np.array(a['dq'][:7])
  for name in ('frozen_mass_hold','frozen_mass_smooth'):
   K,D=gains(a['M'],name);q=q0.copy();v=v0.copy();traj=[model.fk(side,q)[:3,3]]
   for k in range(30):
    qr,vr=reference(q0,v0,k/60,name);v+=np.linalg.solve(M,K@(qr-q)+D@(vr-v))/60;q+=v/60;traj.append(model.fk(side,q)[:3,3])
   traj=np.array(traj);ax.plot(np.arange(31)/60,np.linalg.norm(traj-traj[0],axis=1)*1000,'--',label=('D2' if name=='frozen_mass_hold' else 'D3')+' model prediction')
  for name in ('D2','D3','D4','FORMAL'):
   path=HERE/'evidence/diagnostic'/name/'frames.jsonl'
   if not path.exists():continue
   fs=frames(path)
   if len(fs)<2:continue
   T=np.array([f['arms'][side]['tcp_world'] for f in fs]);tt=np.array([f['time_s'] for f in fs]);ax.plot(tt-tt[0],np.linalg.norm(T[:,:3,3]-T[0,:3,3],axis=1)*1000,label=name+' measured')
  ax.axhline(1,color='black',lw=1,label='1 mm performance limit');ax.set(title=side+' original L-tool TCP',xlabel='Physics time from ownership (s)',ylabel='Translation drift (mm)');ax.grid(alpha=.2);ax.legend(fontsize=8)
 fig.savefig(HERE/'evidence/drift_comparison.png',dpi=160);plt.close(fig)
if __name__=='__main__':main()
