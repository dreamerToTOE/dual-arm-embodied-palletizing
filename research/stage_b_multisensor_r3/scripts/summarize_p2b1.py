#!/usr/bin/env python3
"""Readable tables and label-stratified diagnostics from already frozen predictions."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT));R3=ROOT/'research/stage_b_multisensor_r3'
from research.stage_b_multisensor_r3.scripts.replay_p2b0_history import rows,stats,dump


def summarize(evaluation):
    summary=json.loads((evaluation/'summary.json').read_text());lines=['# P2B-1 完整数值表','',
        'XY 为 median/P95/max（mm）。漏检为物理步样本计数并包括 UNKNOWN；不等同事件漏检率。所有方法共享图像/掩码；消融在共同初始注册后进行。',
        '旧三条轨迹在修复后属于开发复核，仅 verification 是修复后的独立留出。V1 位置仅视觉，连接列为共享辅助诊断；K1 的 CONNECTED 是条件假设。','',
        '|场景|方法|XY mm|预测覆盖|UNKNOWN|误报/严格漏检|恢复至3mm s /视觉更新数|',
        '|---|---|---|---|---|---|---|'];diagnostics={}
    for name,methods in summary['cases'].items():
        run,case=name.split('/');pred=rows(evaluation/run/(case+'_predictions.jsonl'));truth={r['physics_step']:r for r in rows(R3/'evidence'/run/'oracle.jsonl')}
        release=min((r['time_s'] for r in truth.values() if not r['fixture_joint_enabled']),default=None);diagnostics[name]={}
        for method,s in methods.items():
            e=s['xy_error_mm'];fmt='/'.join(f'{e[k]:.4f}' for k in ('median','p95','max')) if e else 'NA'
            recovery='; '.join(str(r['time_to_under_3mm_s'])+'/'+str(r['visual_updates_to_under_3mm']) for r in s['recovery']) or 'NA'
            lines.append(f'|{name}|{method}|{fmt}|{s["prediction_coverage"]:.3%}|{s["unknown_rate"]:.3%}|{s["false_disconnect_alarms"]}/{s["strict_missed_disconnects_including_unknown"]}|{recovery}|')
            p=[p for p in pred if p['method']==method and p['status']=='ESTIMATED'];d={}
            for label in (True,False):
                values=[np.linalg.norm(np.array(r['xy_m'])-truth[r['physics_step']]['cube_xyz_m'][:2])*1000 for r in p if truth[r['physics_step']]['fixture_joint_enabled']==label]
                d['connected_xy_mm' if label else 'disconnected_xy_mm']=stats(values)
            first=next((r['time_s'] for r in p if release is not None and r['time_s']>=release and r['connection']['state']=='DISCONNECTED'),None)
            d['first_disconnect_detection_delay_s']=first-release if first is not None else None
            d['disconnect_event_detected']=first is not None if release is not None else None
            d['uncertainty_radius_mm']=stats([r['position_uncertainty_radius_m']*1000 for r in p]);diagnostics[name][method]=d
    (evaluation/'NUMERICAL_RESULTS.md').write_text('\n'.join(lines)+'\n');dump(evaluation/'secondary_diagnostics.json',diagnostics)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--evaluation',type=Path,required=True);a=ap.parse_args();summarize(a.evaluation)
