"""Black-box regression tests of physical hazards, progress, faults and repeatability."""
import csv
import io
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def run(scenario, controller, *extra):
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / 'trace.csv'
        p = subprocess.run([str(ROOT / 'crossing'), '--scenario', scenario,
                            '--controller', controller, '--csv', str(path),
                            '--quiet', *extra], capture_output=True, text=True, timeout=10)
        assert p.returncode in (0, 2), (p.stdout, p.stderr)
        text = path.read_text()
        rows = list(csv.DictReader(io.StringIO(text)))
        return p, rows, text

expected = {
    'nominal': (False, False),
    'sensor-stale': (False, True),
    'sensor-a-stuck': (False, True),
    'common-cause': (True, True),
    'barrier-stuck': (False, True),
    'controller-freeze': (False, False),
    'road-obstacle': (False, True),
}
for scenario, outcomes in expected.items():
    for controller, hazard in zip(('reference', 'baseline'), outcomes):
        p, rows, text = run(scenario, controller)
        assert (p.returncode == 2) == hazard, p.stdout
        assert any(any(r[h] == '1' for h in ('hazard_unprotected', 'hazard_obstacle', 'hazard_gate'))
                   for r in rows) == hazard
        assert all(0 <= float(r['barrier_fraction']) <= 1 and
                   0 <= float(r['train_speed_mps']) <= 20 for r in rows)
        if scenario == 'nominal' or (scenario == 'road-obstacle' and controller == 'reference'):
            assert float(rows[-1]['train_front_m']) > 45, p.stdout
        if controller == 'reference' and scenario in ('sensor-stale', 'sensor-a-stuck', 'barrier-stuck', 'controller-freeze'):
            assert float(rows[-1]['train_speed_mps']) == 0 and float(rows[-1]['train_front_m']) < -5
        if scenario == 'controller-freeze':
            assert any(r['watchdog'] == '1' for r in rows)
        if scenario == 'sensor-stale':
            assert float(rows[-1]['sampled_at_s']) < 2
        if scenario == 'common-cause':
            assert any(r['ground_occupied'] == '1' and r['approach_a'] == r['approach_b'] == '0' for r in rows)
        if scenario == 'barrier-stuck':
            assert all(float(r['barrier_fraction']) == 0 for r in rows)
        assert text == run(scenario, controller)[2], 'Simulation is not repeatable'
        print('PASS', scenario, controller)
# Wall-clock pacing must not change the virtual trace.
assert run('nominal', 'reference', '--duration', '0.2')[2] == run(
    'nominal', 'reference', '--duration', '0.2', '--realtime')[2]
for args in (['--scenario', 'unknown'], ['--duration', 'nan'], ['--duration', '-1'], ['--csv']):
    p = subprocess.run([str(ROOT/'crossing'), *args], capture_output=True, timeout=5)
    assert p.returncode == 1
print('PASS pacing equivalence and invalid arguments')
