#!/usr/bin/env python3
"""Create auditable C2 metrics and six-panel physical time series."""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from offline.task28_stage_b_contact.c2_evidence import verify_evidence


def main():
    root = Path(sys.argv[1])
    rows = [json.loads(line) for line in (root / 'timeseries.jsonl').read_text().splitlines()]
    event_path = root / 'contact_events.jsonl'
    events = [json.loads(line) for line in event_path.read_text().splitlines()] if event_path.exists() else []
    summary = json.loads((root / 'summary.json').read_text())
    label = 'B_C2R' if 'preflight' in summary else 'B_C2'
    mass = summary.get('cube_runtime_mass_kg', .8)
    gravity_magnitude = summary.get('gravity_m_s2', 9.81)
    result = {'execution_status': summary['status'], 'reason': summary.get('reason'), 'samples': len(rows),
              'contact_event_file_available': event_path.exists(),
              'momentum_assumptions': f'mass={mass} kg, gravity=(0,0,-{gravity_magnitude}) m/s²; '
                                      + ('runtime values recorded' if 'cube_runtime_mass_kg' in summary and 'gravity_m_s2' in summary
                                         else 'runtime values not recovered; values assumed')}
    fig, axes = plt.subplots(6, 1, figsize=(11, 14), constrained_layout=True)
    if not rows:
        for ax in axes:
            ax.text(.5, .5, 'No physics samples recorded', ha='center', va='center', transform=ax.transAxes)
        result['status'] = f'{label}_BLOCKED'
    else:
        t = np.array([r['time_s'] for r in rows])
        force = np.array([r['force_y'] for r in rows])
        ref = np.array([r['reference_N'] for r in rows])
        state = [r['state'] for r in rows]
        ramp = np.array([s == 'FORCE_RAMP' for s in state])
        error = force[ramp] - ref[ramp]
        impulses = np.array([r['cube_contact_impulse_world_Ns'] for r in rows])
        dts = np.array([r['dt_s'] for r in rows])
        v = np.array([r['cube_velocity_xyz_m_s'] for r in rows])
        gravity = np.array([0., 0., -gravity_magnitude])
        # The first row is the initial measured state; subsequent impulses act between samples.
        residual = mass * (v[-1] - v[0]) - impulses[1:].sum(axis=0) - mass * gravity * dts[1:].sum()
        levels = ['PREFLIGHT', 'APPROACH', 'CONTACT', 'FORCE_RAMP', 'EXIT', 'DONE', 'FAILED']
        codes = np.array([levels.index(s) for s in state])
        axes[0].plot(t, force, label='tool on Cube +Y'); axes[0].axhline(1.0, color='r', ls='--', label='limit'); axes[0].set_ylabel('Force N'); axes[0].legend()
        axes[1].plot(t, ref, label='reference'); axes[1].plot(t, force, label='measured'); axes[1].set_ylabel('Force N'); axes[1].legend()
        axes[2].plot(t, np.array([r['tool_y'] for r in rows]) * 1e3, label='tool'); axes[2].plot(t, np.array([r['cube_y'] for r in rows]) * 1e3, label='Cube'); axes[2].set_ylabel('Y displacement mm'); axes[2].legend()
        axes[3].plot(t, [r['tool_velocity'] for r in rows], label='tool'); axes[3].plot(t, [r['cube_velocity'] for r in rows], label='Cube'); axes[3].set_ylabel('Y velocity m/s'); axes[3].legend()
        axes[4].step(t, codes, where='post'); axes[4].set_yticks(range(len(levels)), levels); axes[4].set_ylabel('State')
        axes[5].plot(t, [r['support_force_z'] for r in rows], label='Cube/table normal N'); axes[5].plot(t, [r['event_force_y'] for r in rows], label='tool/Cube event +Y N');
        for r in rows:
            if r['reason'] or r['unexpected_pairs']:
                axes[5].axvline(r['time_s'], color='r', alpha=.7)
        axes[5].set_ylabel('Pair / safety'); axes[5].legend()
        for ax in axes:
            ax.grid(alpha=.25)
        sleep = next((r for r in rows if r['diag_sleeping']), None)
        result.update(first_state=state[0], last_state=state[-1], states=list(dict.fromkeys(state)),
                      max_force_N=float(np.max(force)), min_force_N=float(np.min(force)),
                      ramp_samples=int(ramp.sum()), ramp_force_min_N=float(np.min(force[ramp])) if ramp.any() else None,
                      ramp_force_max_N=float(np.max(force[ramp])) if ramp.any() else None,
                      ramp_rmse_N=float(np.sqrt(np.mean(error ** 2))) if len(error) else None,
                      ramp_reference_range_N=[float(np.min(ref[ramp])), float(np.max(ref[ramp]))] if ramp.any() else None,
                      max_abs_tool_displacement_m=max(abs(r['tool_y']) for r in rows),
                      max_abs_cube_displacement_m=max(abs(r['cube_y']) for r in rows),
                      max_penetration_m=max(r['penetration_m'] for r in rows),
                      support_force_min_N=min(r['support_force_z'] for r in rows),
                      unexpected_pairs=sum(len(r['unexpected_pairs']) for r in rows),
                      final_force_N=float(force[-1]), momentum_residual_world_Ns=residual.tolist(),
                      sleep_diagnostic_ever_sleeping=any(r['diag_sleeping'] for r in rows),
                      diag_first_sleep_step=sleep['step'] if sleep else None,
                      diag_first_sleep_time_s=sleep['time_s'] if sleep else None,
                      pair_events=len(events) if event_path.exists() else None)
        force_tracking = bool(len(error) >= 30 and np.sqrt(np.mean(error ** 2)) < .03 and
                              np.max(force[ramp]) - np.min(force[ramp]) > .05)
        required = (['PREFLIGHT'] if label == 'B_C2R' else []) + ['APPROACH', 'CONTACT', 'FORCE_RAMP', 'EXIT', 'DONE']
        observed = list(dict.fromkeys(state))
        balance_limit = max(.02, .05 * mass * gravity_magnitude * float(dts[1:].sum()))
        checks = {
            'complete_state_sequence': observed == required,
            'preflight_pass': summary.get('preflight') == 'PASS' if label == 'B_C2R' else True,
            'contact_events_available': event_path.exists(),
            'no_safety_failure_or_unexpected_contact': not result['unexpected_pairs'] and all(not r['reason'] for r in rows),
            'fresh_world_frame_observations': all(r['valid'] and r['frame'] == 'world' and
                                                   r['sensor_step'] == r['step'] and 0 <= r['age_ns'] <= 50_000_000
                                                   for r in rows),
            'force_tracking': force_tracking,
            'separated_and_stopped': abs(force[-1]) < 1e-5 and abs(rows[-1]['tool_velocity']) < .001,
            'gravity_and_all_contact_momentum_balance': float(np.linalg.norm(residual)) < balance_limit,
            'runtime_mass_and_gravity_recorded': abs(summary.get('cube_runtime_mass_kg', 0) - .8) < .001 and
                                                 summary.get('gravity_m_s2') is not None,
        }
        exit_path = root / 'process_exit.json'
        process = json.loads(exit_path.read_text()) if exit_path.exists() else {}
        checks['complete_evidence_and_process_exit'] = (verify_evidence(root, process.get('child_exit_code', -1))[0]
                                                         and process.get('wrapper_exit_code') == 0)
        checks = {name: bool(value) for name, value in checks.items()}
        complete = bool(summary['status'] == 'RECORDED' and all(checks.values()))
        result['acceptance_checks'] = checks
        result['momentum_residual_limit_Ns'] = balance_limit
        result['force_tracking_criterion'] = 'at least 30 ramp samples; RMSE < 0.03 N and force span > 0.05 N'
        result['force_tracking_pass'] = force_tracking
        blocked = (label == 'B_C2R' and summary.get('preflight') == 'FAILED' and
                   'SUPPORT_OR_MOMENTUM_MISMATCH' in str(summary.get('reason')))
        result['status'] = f'{label}_PASS' if complete else f'{label}_BLOCKED' if blocked else f'{label}_PARTIAL'
    axes[-1].set_xlabel('Simulation time since control start (s)')
    fig.suptitle(f'TASK28 C2 isolated proxy, not FR3 — {summary["status"]}')
    fig.savefig(root / 'c2_timeseries.png', dpi=150)
    plt.close(fig)
    (root / 'analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
