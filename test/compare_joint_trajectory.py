"""Compare compiled probe output with the Python prototype.

Set PYTHONPATH to the marvin_controller checkout. Pass the compiled
joint_trajectory_probe executable as the sole argument. No ROS/hardware access.
"""
import subprocess
import sys
import math
from main.joint_test_trajectory import prepare

names = ('hip', 'thigh', 'calf')
plan = prepare(names=names, measured=dict.fromkeys(names, .2),
    commanded=dict.fromkeys(names, .21), ages=dict.fromkeys(names, .01),
    limits=dict.fromkeys(names, (-1., 1.)), max_speed=dict.fromkeys(names, .1),
    max_handover_error=dict.fromkeys(names, .05), max_age=.05, joint='thigh',
    amplitude=.02, transition_seconds=1., duration=4., cycles=1,
    max_total_seconds=5., max_acceleration=dict.fromkeys(names, .2))
rows = [list(map(float, line.split())) for line in
        subprocess.check_output([sys.argv[1]], text=True).splitlines()]
assert len(rows) == 5001
assert all(math.isfinite(value) for row in rows for value in row)
assert all(len(row) == 4 and abs(row[0]-i/1000) < 1e-12 for i,row in enumerate(rows))
error = max(abs(actual-expected) for t,*values in rows
            for actual,expected in zip(values, plan.sample(t)[1]))
velocities = [[(b-a)/.001 for a,b in zip(left[1:], right[1:])]
              for left,right in zip(rows, rows[1:])]
speed = max(abs(v) for values in velocities for v in values)
acceleration = max(abs(b-a)/.001 for left,right in zip(velocities, velocities[1:])
                   for a,b in zip(left,right))
assert error < 1e-12 and speed < .1 and acceleration < .2
print(f'{len(rows)} samples: max error={error:.3g} rad, '
      f'speed={speed:.6f} rad/s, acceleration={acceleration:.6f} rad/s^2')
