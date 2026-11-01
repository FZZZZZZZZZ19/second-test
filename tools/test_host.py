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
    names = ['app_math', 'vehicle_param', 'speed_control', 'motion_plan', 'vision_line', 'vision_boundary', 'vision_target', 'vision_pipeline',
             'vehicle_progress', 'track_element', 'path_control', 'run_manager']
    executable = out / 'test_app.exe'
    command = [args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-O2',
               *[arg for folder in ['User', *['User/'+d for d in ['vision', 'control', 'board', 'shared', 'integration', 'config']]] for arg in ['-I',str(ROOT/folder)]], str(ROOT / 'tests/test_app.c'),
               *[str(next((ROOT/'User').rglob(n+'.c'))) for n in names], '-lm', '-o', str(executable)]
    results = []
    for label, flags in [('default', []), ('configured_route', ['-DAPP_ROUTE_LENGTH=1', '-DAPP_FINISH_DISTANCE_M=0.25f'])]:
        subprocess.run(command[:1] + flags + command[1:], check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True, check=True)
        results.append({'profile': label, 'result': result.stdout.strip()})
        print(label + ': ' + result.stdout.strip())
    # Link each domain alone: catches hidden calls across the algorithm boundary.
    for label, sources in [('vision', ['vision_line','vision_boundary','vision_target','vision_pipeline','track_element','vehicle_progress','app_math']),
                           ('control', ['path_control','speed_control','motion_plan','app_math','vehicle_param'])]:
        executable = out / ('test_' + label + '.exe')
        cmd = [args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-O2',
               *[arg for folder in ['User', *['User/'+d for d in ['vision','control','board','shared','integration','config']]] for arg in ['-I',str(ROOT/folder)]],
               str(ROOT / ('tests/test_'+label+'_module.c')),
               *[str(next((ROOT/'User').rglob(n+'.c'))) for n in sources], '-lm', '-o', str(executable)]
        subprocess.run(cmd, check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True, check=True)
        results.append({'profile': 'standalone_'+label, 'result': result.stdout.strip()})
        print(result.stdout.strip())
    (out / 'test-report.json').write_text(json.dumps({
        'compiler': subprocess.check_output([args.cc, '--version'], text=True).splitlines()[0],
        'modules': names, 'results': results, 'status': 'PASS'
    }, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
