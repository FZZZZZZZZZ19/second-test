"""Compile the actual hardware-independent modules and run regression tests."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cc', default=shutil.which('gcc') or 'gcc')
    args = parser.parse_args()
    out = ROOT / 'build/host'
    out.mkdir(parents=True, exist_ok=True)
    names = ['app_math', 'vehicle_param', 'speed_control', 'motion_plan', 'vision_line', 'vision_boundary',
             'vehicle_progress', 'track_element', 'path_control', 'run_manager']
    executable = out / 'test_app.exe'
    command = [args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-O2',
               '-I', str(ROOT / 'User'), str(ROOT / 'tests/test_app.c'),
               *[str(ROOT / 'User' / (n + '.c')) for n in names], '-lm', '-o', str(executable)]
    results = []
    for label, flags in [('default', []), ('configured_route', ['-DAPP_ROUTE_LENGTH=1', '-DAPP_FINISH_DISTANCE_M=0.25f'])]:
        subprocess.run(command[:1] + flags + command[1:], check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True, check=True)
        results.append({'profile': label, 'result': result.stdout.strip()})
        print(label + ': ' + result.stdout.strip())
    (out / 'test-report.json').write_text(json.dumps({
        'compiler': subprocess.check_output([args.cc, '--version'], text=True).splitlines()[0],
        'modules': names, 'results': results, 'status': 'PASS'
    }, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
