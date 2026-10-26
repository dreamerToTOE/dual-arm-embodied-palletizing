"""Lossless node CSV and optional static figure from completed planner evidence."""
import argparse
import csv
import gzip
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--experiment',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--plots',action='store_true');args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);summary=json.loads((args.experiment/'summary.json').read_text());paths=[];receipts=[]
    for run in summary['runs']:
        if not run['found']:continue
        with gzip.open(args.experiment/f'seed_{run["seed"]}'/'result.json.gz','rt') as stream:data=json.load(stream)
        rows=[]
        for name in ('lift','transport','place'):
            for index,(X,q) in enumerate(data[name]):
                rows.append({'seed':run['seed'],'segment':name,'node_in_segment':index,'object_x_m':X[0][3],'object_y_m':X[1][3],'object_z_m':X[2][3],
                             'object_rotation_matrix_row_major':json.dumps([r[:3] for r in X[:3]]),'left_q_rad':json.dumps(q[:7]),'right_q_rad':json.dumps(q[7:])})
        with (args.output/f'seed_{run["seed"]}_nodes.csv').open('x',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
        paths.append((run['seed'],rows));receipts.append({'seed':run['seed'],'lift_height_m':data['lift'][-1][0][2][3],
            'start_q14':data['lift'][0][1],'goal_q14':data['place'][-1][1],
            'source_result':str((args.experiment/f'seed_{run["seed"]}'/'result.json.gz').resolve()),'scope':'generated nodes from accepted tree path, not old A-stage waypoints'})
        del data
    with (args.output/'path_receipts.json').open('x') as stream:json.dump(receipts,stream,indent=2);stream.write('\n')
    if args.plots:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig=plt.figure(figsize=(10,5));left=fig.add_subplot(121,projection='3d');right=fig.add_subplot(122)
        for seed,rows in paths:
            xs=[r['object_x_m'] for r in rows];ys=[r['object_y_m'] for r in rows];zs=[r['object_z_m'] for r in rows]
            left.plot(xs,ys,zs,label=f'seed {seed}');right.plot(range(len(zs)),zs,label=f'seed {seed}')
        left.set(xlabel='World X (m)',ylabel='World Y (m)',zlabel='Cube centre Z (m)',title='NR-RRT generated object paths')
        right.axhline(.26,color='black',ls='--',label='frozen supported centre Z')
        right.set(xlabel='Generated node (segment boundaries retained)',ylabel='Cube centre Z (m)',title='Distinct sampled lift/transport routes')
        left.legend();right.legend();fig.tight_layout()
        for suffix in ('png','svg'):fig.savefig(args.output/('object_paths.'+suffix),dpi=160,bbox_inches='tight')
        svg=args.output/'object_paths.svg'
        svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
        plt.close(fig)

if __name__=='__main__':main()
