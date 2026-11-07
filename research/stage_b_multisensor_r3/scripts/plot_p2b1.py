#!/usr/bin/env python3
"""Plot already frozen predictions against isolated references; no estimator fitting."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from research.stage_b_multisensor_r3.scripts.replay_p2b0_history import rows


def plot(evaluation,evidence):
    examples=[('connected','long'),('loss','loss_hidden'),('slip','fully_visible'),('verification','long')]
    fig,axes=plt.subplots(4,1,figsize=(10,11),sharex=True)
    for ax,(condition,case) in zip(axes,examples):
        run='r3_p2b1_'+condition+'_holdout01';truth={r['physics_step']:r for r in rows(evidence/run/'oracle.jsonl')}
        predictions=rows(evaluation/run/(case+'_predictions.jsonl'))
        for method in ('V1','K1','F1'):
            p=[r for r in predictions if r['method']==method and r['status']=='ESTIMATED']
            t=[r['relative_time_s'] for r in p];error=[1000*np.linalg.norm(np.array(r['xy_m'])-truth[r['physics_step']]['cube_xyz_m'][:2]) for r in p]
            ax.plot(t,error,label=method,linewidth=1.5)
        if case in ('long','loss_hidden'):ax.axvspan(3,17.533334,color='gray',alpha=.17,label='Offline visual dropout')
        if condition!='connected':ax.axvline(9.2 if condition=='verification' else 8.2,color='black',linestyle=':',label='Fixture released (oracle)')
        ax.set_title(condition+' / '+case);ax.set_ylabel('XY error (mm)');ax.grid(alpha=.25);ax.legend(loc='upper right',ncol=2,fontsize=8)
    axes[-1].set_xlabel('Seconds since new camera epoch');fig.tight_layout()
    fig.savefig(evaluation/'error_curves.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--evaluation',type=Path,required=True);ap.add_argument('--evidence',type=Path,required=True);a=ap.parse_args();plot(a.evaluation,a.evidence)
