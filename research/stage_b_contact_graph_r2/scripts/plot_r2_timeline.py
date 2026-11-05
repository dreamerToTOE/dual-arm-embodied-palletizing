#!/usr/bin/env python3
from __future__ import annotations
import gzip,json
from pathlib import Path
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
rows=[]
with gzip.open(ROOT/'dataset/r2_timeline.jsonl.gz','rt') as f:
 for l in f:
  if l.strip(): rows.append(json.loads(l))
pred=[]
with gzip.open(ROOT/'derived/estimator_predictions.jsonl.gz','rt') as f:
 for l in f:
  if l.strip(): pred.append(json.loads(l))
t=[r['time_s'] for r in rows]; x=[r['cube_world_m'][0] for r in rows]
fig,ax=plt.subplots(figsize=(12,5))
ax.plot(t,x,color='black',lw=.7,label='Cube x')
for key,color,label in [('LEFT_TOOL_CUBE','#4daf4a','left tool'),('RIGHT_TOOL_CUBE','#377eb8','right tool'),('CUBE_DEEP_WALL','#e41a1c','deep wall')]:
 active=[r['oracle_reference']['edge_states'][key]=='OBSERVED_ACTIVE' for r in rows]
 start=None
 for i,v in enumerate(active+[False]):
  if v and start is None:start=i
  if not v and start is not None:
   ax.axvspan(t[start],t[i-1],color=color,alpha=.13,label=label if start==next((j for j,z in enumerate(active) if z),-1) else None); start=None
ax.axvline(t[2151],ls='--',color='#4daf4a',lw=.8); ax.axvline(t[9235],ls='--',color='#e41a1c',lw=.8)
ax.set(xlabel='time (s)',ylabel='Cube x (m)',title='R2 physical trace: geometry replay with PhysX reference contact bands')
ax.grid(alpha=.25); ax.legend(loc='best',fontsize=8); fig.tight_layout(); fig.savefig(ROOT/'figures/r2_contact_timeline.png',dpi=160); print(ROOT/'figures/r2_contact_timeline.png')
