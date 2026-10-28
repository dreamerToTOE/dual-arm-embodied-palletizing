"""Static scientific figure from archived data; not a simulator."""
import argparse
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--input',required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
    with open(args.input) as stream:
        data=json.load(stream)
    fig,axes=plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
    for ax,key,title in zip(axes,['task28_p5_qp_5','task28_p5_qp_5_r1'],['QP-5 archive: retrospective, continued after failure','R1 archive: post-step, stopped at second frame']):
        rows=data['archives'][key]['rows'];t=[1000*r['time_s'] for r in rows]
        for metric,label,color in [('left_absolute_translation_m','Left absolute','#b63632'),('right_absolute_translation_m','Right absolute','#2764b0'),('relative_translation_m','Left/right relative','#087f73')]:
            ax.plot(t,[1000*r['closure']['metrics'][metric] for r in rows],label=label,color=color,marker='o' if 'r1' in key else None)
        ax.plot(t,[1000*r['closure']['common_translation_m'] for r in rows],label='Common translation',color='#7b3fa0',linestyle='--')
        ax.axhline(.1,color='#555555',linestyle=':',label='Frozen threshold 0.100 mm')
        ax.set(xlabel='Reference cursor (ms)',ylabel='Translation error (mm)',title=title)
        ax.grid(alpha=.2)
    axes[0].legend(fontsize=8)
    fig.suptitle('Archived virtual geometry only | R2 runs no new physics',fontsize=12)
    fig.savefig(args.output,dpi=170)
    plt.close(fig)


if __name__=='__main__':
    main()
