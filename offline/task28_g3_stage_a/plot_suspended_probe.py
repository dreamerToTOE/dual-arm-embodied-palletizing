"""Plot matched Stage-A force and Cube-motion probes from their recorded traces."""
import argparse
import json
from pathlib import Path
from offline.task28_g3_stage_a.analyze_suspended_probe import open_trace, paired_commands
from offline.task28_g3_stage_a.suspended_probe import motion_progress


def read(run_dir):
    start = motion = None
    def records(stream):
        nonlocal start, motion
        for line in stream:
            row = json.loads(line)
            if row.get('event') == 'SUSPENDED_PROBE_START': start = row
            if row.get('event') == 'SUSPENDED_PROBE_MOTION_START': motion = row
            yield row
    result = []
    with open_trace(run_dir) as stream:
        for command, frame in paired_commands(records(stream)):
            cube = frame['cube_runtime']['pose_world']['position_m']
            elapsed = frame['time_s'] - start['time_s']
            target_x = None
            if motion and frame['time_s'] >= motion['time_s']:
                target_x = motion['cube_start_world_m'][0] + motion['amplitude_world_x_m'] * motion_progress(elapsed)[0]
            g3 = frame['qp'].get('g3_internal_force') or {}
            result.append((elapsed, command['force_world_N'][1], cube[0], target_x,
                           g3.get('estimated_internal_force_N'),
                           frame['qp'].get('internal_torque_target_residual_inf_Nm')))
    return result


def plot(off, on, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True, layout='constrained')
    for path, name in ((off, 'feedforward off'), (on, 'feedforward on')):
        rows = read(path)
        time = [r[0] for r in rows]
        for axis, index in ((axes[1], 4), (axes[3], 5)):
            axis.plot(time, [r[index] for r in rows], label=name)
        motion = [r for r in rows if r[3] is not None]
        if motion:
            origin = motion[0][3]
            axes[2].plot([r[0] for r in motion], [(r[2]-origin)*1000 for r in motion], label=name+' cube')
            axes[2].plot([r[0] for r in motion], [(r[3]-origin)*1000 for r in motion], '--', alpha=.6, label=name+' ideal')
        if name.endswith('off'):
            axes[0].plot(time, [r[1] for r in rows], label='injected cube force')
    axes[0].set_ylabel('Cube force Y (N)')
    axes[1].set_ylabel('Internal candidate (N)')
    axes[2].set_ylabel('Cube X motion (mm)')
    axes[3].set_ylabel('Torque residual (Nm)')
    axes[3].set_xlabel('Probe time (simulation seconds)')
    for ax in axes:
        ax.grid(alpha=.3); ax.legend(loc='best')
    fig.suptitle('Task27 attached probe: joint reaction/model candidate are not D6 force measurements')
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('off', type=Path); parser.add_argument('on', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    plot(args.off, args.on, args.output)


if __name__ == '__main__': main()
