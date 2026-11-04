#!/usr/bin/env python3
"""Bounded CPU-only final study with parameterized compartment scenes.
Each candidate is an offline MoveIt/FCL geometric/kinematic check; contact is never executed.
"""
from __future__ import annotations
import hashlib, json, math, multiprocessing as mp, os, re, subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "research/stage_b_two_level/evidence/reference"
BASE_FIXTURE = ROOT / "research/stage_b_final/evidence/pilot/y-0.13_armright_off-0.02/fixture.json"
CONTACTS = (-.02, 0.0, .02)
RULE_YAW_DEG = 2.94

def workers():
    cpu = os.cpu_count() or 1
    avail = next((int(x.split()[1]) for x in Path('/proc/meminfo').read_text().splitlines() if x.startswith('MemAvailable:')), 0)
    return max(2, min(8, cpu // 2, max(2, avail // (2 * 1024 * 1024))))

def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): h.update(b)
    return h.hexdigest()

def yaw_quat(deg):
    a = math.radians(deg) / 2
    return [math.cos(a), 0.0, 0.0, math.sin(a)]

def scenarios():
    rows = []
    for width in (.43, .41):
        for y in (-.13, .13):
            for yaw in (0.0, -4.0, 4.0):
                rows.append(("dev", width, .30, y, yaw, 0))
    for y in (-.13, .13):
        for yaw in (0.0, -4.0, 4.0):
            rows.append(("dev", .43, .30, y, yaw, 1))
    for width, depth in ((.39, .27), (.39, .32), (.43, .27), (.43, .32)):
        for y in (-.13, .13):
            for yaw in (0.0, 4.0):
                rows.append(("test", width, depth, y, yaw, 0))
    for width, depth in ((.39, .32), (.43, .27)):
        rows.append(("test", width, depth, 0.0, 0.0, 0))
    assert len(rows) == 36
    return rows

def make_fixture(base, width, depth, y, yaw, occupied, contact_x):
    f = json.loads(json.dumps(base))
    f['cube_xyz_m'] = [1.08, y, base['cube_xyz_m'][2]]
    f['cube_quaternion_wxyz'] = yaw_quat(yaw)
    boxes = []
    if occupied:
        oy = .13 if y <= 0 else -.13
        boxes.append({'id': 'occupied_cube_0', 'xyz_m': [.98, oy, f['cube_xyz_m'][2]], 'size_m': [.12, .12, .12]})
    f['occupied_boxes'] = boxes
    f['scene_width_m'], f['scene_depth_m'], f['contact_x_m'] = width, depth, contact_x
    return f, boxes

def run(cmd, env, log, timeout=90):
    t = time.monotonic()
    try:
        p = subprocess.run(cmd, env=env, text=True, capture_output=True, timeout=timeout)
        out, code = p.stdout + p.stderr, p.returncode
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or '') if isinstance(e.stdout, str) else (e.stdout or b'').decode(errors='replace')
        out += '\nTIMEOUT\n'; code = None
    elapsed = round(time.monotonic() - t, 4); log.write_text(out)
    if 'Error retrieving file' in out or 'has visual geometry but no collision geometry' in out:
        result = 'ERROR'
    else:
        m = re.search(r'(?:PRECONTACT|YAW_CANDIDATE)_(PASS|REJECT)', out); result = m.group(1) if m else 'ERROR'
    metrics = {}
    for k in ('samples','ik_attempt','ik_attempts','joint_length_rad','path_length_rad','seconds'):
        m = re.search(r'\b' + k + r'=([0-9.eE+-]+)', out)
        if m: metrics[k] = float(m.group(1))
    reject = re.search(r'YAW_CANDIDATE_REJECT phase=([^ ]+) reason=(.*?)(?: samples=|$)', out)
    return {'result': result, 'return_code': code, 'elapsed_s': elapsed,
            'phase': reject.group(1) if reject else None, 'detail': reject.group(2) if reject else None,
            'cup_footprint_pass': 'CUP_FOOTPRINT_PASS' in out, 'metrics': metrics}

def pre_task(x):
    build, urdf, srdf, fixture, path, arm, env = x
    exe = build / ('precontact_left' if arm == 'left' else 'precontact_right')
    r = run([str(exe), str(urdf), str(srdf), str(fixture), str(path)], env, path.with_suffix('.log'))
    r.update({'arm': arm, 'path': str(path)}); return r

def cand_task(x):
    build, urdf, srdf, fixture, pre, occ, path, arm, seq, env = x
    exe = build / ('yaw_left' if arm == 'left' else 'yaw_right')
    r = run([str(exe), str(urdf), str(srdf), str(fixture), str(pre), str(path), '0', seq, str(occ), '1'], env, path.with_suffix('.log'))
    r.update({'arm': arm, 'sequence': seq, 'trajectory': str(path)}); return r

def main():
    if len(sys.argv) != 5: raise SystemExit('usage: run_final.py BUILD URDF SRDF OUTPUT')
    build, urdf, srdf, out = map(Path, sys.argv[1:5]); out.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy(); env.update({'ROS_DOMAIN_ID':'183','ROS_LOCALHOST_ONLY':'1','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','STAGE_B_SEED':'4104'})
    base = json.loads(BASE_FIXTURE.read_text()); ws = workers(); cases = {}; pre_jobs = []
    for i, (split, width, depth, y, yaw, occupied) in enumerate(scenarios()):
        sid = f'{split}_{i:02d}_w{int(width*1000)}_d{int(depth*1000)}_y{int(round(y*1000))}_yaw{yaw:+g}_occ{occupied}'
        d = out / 'cases' / sid; d.mkdir(parents=True)
        f, boxes = make_fixture(base, width, depth, y, yaw, occupied, 0.0)
        fp, op = d/'fixture.json', d/'occupancy.json'; fp.write_text(json.dumps(f, indent=2)+'\n'); op.write_text(json.dumps(boxes, indent=2)+'\n')
        cases[sid] = {'split':split,'width_m':width,'depth_m':depth,'target_y_m':y,'yaw_deg':yaw,'occupied':occupied,'fixture_path':fp,'occupancy_path':op,'pre':{},'candidates':[]}
        for arm in ('left','right'):
            for cx in CONTACTS:
                fc, _ = make_fixture(base, width, depth, y, yaw, occupied, cx)
                cdir = d / f'contact_{cx:+.2f}_{arm}'; cdir.mkdir()
                cfp = cdir/'fixture.json'; cfp.write_text(json.dumps(fc, indent=2)+'\n')
                path = cdir/'pre.csv'; pre_jobs.append((build,urdf,srdf,cfp,path,arm,env))
    with mp.Pool(ws) as pool:
        for r in pool.imap_unordered(pre_task, pre_jobs):
            key = Path(r['path']).parents[1].name; contact = Path(r['path']).parent.name
            cases[key]['pre'][contact] = r
    cand_jobs = []
    for sid, c in cases.items():
        for contact, pr in c['pre'].items():
            if pr['result'] != 'PASS': continue
            pre = Path(pr['path']); arm = contact.rsplit('_', 1)[1]; cx = float(contact.split('_')[1])
            cdir = Path(pre).parent
            for seq in ('direct','correct'):
                path = cdir / f'{seq}.csv'; cand_jobs.append((build,urdf,srdf,cdir/'fixture.json',pre,c['occupancy_path'],path,arm,seq,env))
    with mp.Pool(ws) as pool:
        for r in pool.imap_unordered(cand_task, cand_jobs):
            sid = Path(r['trajectory']).parents[1].name
            cases[sid]['candidates'].append(r)
    def chosen(c, arm, cx, seq):
        rows = [r for r in c['candidates'] if r['arm']==arm and r['sequence']==seq and abs(float(Path(r['trajectory']).parent.name.split('_')[1])-cx)<1e-8]
        if not rows: return {'result':'UNAVAILABLE'}
        return rows[0]
    summary=[]
    for sid,c in cases.items():
        fixed = chosen(c,'left',0.0,'direct')
        rule_arm = 'right' if c['target_y_m'] < 0 else 'left'
        rule_seq = 'correct' if abs(c['yaw_deg']) > RULE_YAW_DEG else 'direct'
        rule = chosen(c,rule_arm,0.0,rule_seq)
        passing = [r for r in c['candidates'] if r['result']=='PASS' and r['cup_footprint_pass']]
        passing.sort(key=lambda r:(r['sequence']=='correct',r['arm']=='right',abs(float(Path(r['trajectory']).parent.name.split('_')[1])),r['elapsed_s']))
        m2 = (dict(passing[0]) if passing else {'result':'REJECT' if c['candidates'] else 'INCONCLUSIVE'})
        m2['candidate_count']=len(c['candidates']); m2['candidate_pass_count']=len(passing); m2['search_elapsed_s']=round(sum(r['elapsed_s'] for r in c['candidates']),4)
        summary.append({'case':sid,'split':c['split'],'width_m':c['width_m'],'depth_m':c['depth_m'],'target_y_m':c['target_y_m'],'yaw_deg':c['yaw_deg'],'occupied':c['occupied'],'m0_fixed':fixed,'m1_rule':rule,'m2_two_level':m2,'level1_decision':'SEQUENCE_ALTERNATIVE' if m2.get('sequence')=='correct' else 'NO_SEQUENCE_ALTERNATIVE','level2_decision':'ADVANTAGE_OVER_RULE' if m2['result']=='PASS' and rule['result']!='PASS' else 'NO_ADVANTAGE_OVER_RULE'})
    result={'study':'stage_b_final_parameterized_contact','scope':'CPU-only MoveIt/FCL geometry and kinematics; no Isaac Sim, RL, force, or physical contact','workers':ws,'cpu_count':os.cpu_count(),'mem_available_kib_at_start':next((int(x.split()[1]) for x in Path('/proc/meminfo').read_text().splitlines() if x.startswith('MemAvailable:')),None),'rule_frozen_before_test':{'yaw_threshold_deg':RULE_YAW_DEG,'arm':'right when target_y_m < 0, else left','contact_x_m':0.0},'scenario_count':len(summary),'contact_candidates_m':list(CONTACTS),'reference_sha256':{'urdf':digest(urdf),'srdf':digest(srdf)},'results':summary}
    (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    for split in ('dev','test'):
        rows=[r for r in summary if r['split']==split]
        print(split, len(rows), {m:sum(r[m]['result']=='PASS' for r in rows) for m in ('m0_fixed','m1_rule','m2_two_level')})
    print(json.dumps({'output':str(out),'workers':ws,'scenes':len(summary)},indent=2))
if __name__=='__main__': main()
