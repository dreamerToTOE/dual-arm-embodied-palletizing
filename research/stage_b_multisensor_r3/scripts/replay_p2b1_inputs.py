#!/usr/bin/env python3
"""Fast deterministic replay of committed observable inputs, followed by isolated rescoring."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT));R3=ROOT/'research/stage_b_multisensor_r3'
from research.stage_b_multisensor_r3.scripts.evaluate_p2b1 import predict_case,score_case
from research.stage_b_multisensor_r3.scripts.replay_p2b0_history import rows,dump,sha


def replay(source,out):
    out.mkdir(parents=True,exist_ok=False);manifest=json.loads((source/'prediction_manifest.json').read_text());summary=json.loads((source/'summary.json').read_text())
    assert manifest['estimator_source_sha256']==sha(R3/'offline_cube_estimator.py')
    cfg=json.loads((R3/'config/p2b1_estimator_frozen.json').read_text());plan=json.loads((R3/'config/p2b1_evaluation_frozen.json').read_text());all_predictions={};matches={}
    for run in sorted(p for p in source.iterdir() if p.is_dir()):
        input_manifest=json.loads((run/'input_manifest.json').read_text())
        for name,h in input_manifest['files_sha256'].items():assert sha(run/name)==h
        states=rows(run/'robot_inputs.jsonl');visual=rows(run/'visual_inputs.jsonl');condition=json.loads((R3/'evidence'/run.name/'manifest.json').read_text())['condition']
        folder=out/run.name;folder.mkdir()
        for case in plan['case_plan'][condition]:
            name=run.name+'/'+case;path=folder/(case+'_predictions.jsonl');p,_=predict_case(states,visual,plan['masks'][case],cfg,path)
            expected=manifest['files_sha256'][str(path.relative_to(out))];assert sha(path)==expected
            all_predictions[name]=(p,run.name,case);matches[name]=sha(path)
    # No oracle is opened until every prediction hash has matched.
    for name,(pred,run,case) in all_predictions.items():
        truth={r['physics_step']:r for r in rows(R3/'evidence'/run/'oracle.jsonl')}
        assert score_case(pred,truth,plan['masks'][case],plan)==summary['cases'][name]
    receipt={'all_prediction_hashes_equal':True,'all_case_scores_equal':True,'cases':len(matches),'prediction_sha256':matches,'oracle_inputs_to_estimator':[]}
    dump(out/'verification.json',receipt);return receipt


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();print(json.dumps(replay(a.source,a.out),indent=2))
