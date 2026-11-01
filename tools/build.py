"""Reproducible WCH GCC firmware build; does not flash hardware."""
import argparse
import concurrent.futures
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--toolchain', help='directory containing riscv-none-embed-gcc(.exe)')
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    directory = args.toolchain or os.environ.get('WCH_TOOLCHAIN')
    gcc = str(Path(directory) / 'riscv-none-embed-gcc.exe') if directory else shutil.which('riscv-none-embed-gcc')
    if not gcc or not Path(gcc).exists():
        parser.error('Set WCH_TOOLCHAIN or --toolchain to the WCH GCC 8.2 toolchain bin directory')
    bin_dir = Path(gcc).parent
    suffix = '.exe' if os.name == 'nt' else ''
    firmware_version = re.search(r'#define APP_VERSION "([^"]+)"', (ROOT / 'User/app_config.h').read_text()).group(1)
    stem = 'second-test-v' + '.'.join(firmware_version.split('.')[:2])
    out = ROOT / 'build' / 'target'
    out.mkdir(parents=True, exist_ok=True)
    for name in [stem + '.elf', stem + '.hex', stem + '.bin', 'build-report.json']:
        (out / name).unlink(missing_ok=True)
    inc = ['User', 'Core', 'Debug', 'Peripheral/inc', 'zf_common', 'zf_driver', 'zf_device', 'zf_components']
    common = ['-march=rv32imafcxw', '-mabi=ilp32f', '-msmall-data-limit=8',
              '-Os', '-g3', '-ffunction-sections', '-fdata-sections', '-fno-common']
    sources = []
    for folder in ['User', 'Core', 'Debug', 'Peripheral/src', 'zf_common', 'zf_driver', 'zf_device', 'zf_components']:
        sources += sorted((ROOT / folder).glob('*.c'))
    sources += [ROOT / 'Startup/startup_ch32v30x_D8C.S']

    def compile_one(src):
        rel = src.relative_to(ROOT)
        obj = out / rel.with_suffix('.o')
        obj.parent.mkdir(parents=True, exist_ok=True)
        flags = common + [flag for item in inc for flag in ['-I', str(ROOT / item)]]
        if src.suffix == '.c':
            flags += ['-std=gnu11', '-Wall', '-Wextra']
            if src.parent.name == 'User' and src.name.startswith(('main.c', 'app_', 'board_', 'camera_frame', 'debug_view', 'motor', 'motion_plan', 'param_store', 'path_control', 'run_manager', 'side_encoder', 'speed_control', 'track_element', 'vehicle_', 'vision_')):
                flags += ['-Werror']
        cmd = [gcc, *flags, '-c', str(src), '-o', str(obj)]
        result = subprocess.run(cmd, capture_output=True, text=True, errors='replace')
        return obj, result, cmd

    commands, logs, objects = [], [], []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.jobs)) as executor:
        for obj, result, cmd in executor.map(compile_one, sources):
            commands.append(cmd)
            objects.append(str(obj))
            logs.append(result.stdout + result.stderr)
            if result.returncode:
                (out / 'compile.log').write_text('\n'.join(logs), encoding='utf-8')
                print(result.stderr, file=sys.stderr)
                raise SystemExit(result.returncode)
    elf = out / (stem + '.elf')
    link = [gcc, *common, *objects, '-T', str(ROOT / 'Ld/Link.ld'),
            '-L', str(ROOT / 'zf_device'), '-nostartfiles', '--specs=nano.specs', '--specs=nosys.specs',
            '-Wl,--gc-sections', '-Wl,-Map=' + str(out / (stem + '.map')),
            '-Wl,--print-memory-usage', '-Wl,--start-group', '-lzf_device_config', '-lm', '-lc', '-Wl,--end-group', '-o', str(elf)]
    r = subprocess.run(link, capture_output=True, text=True, errors='replace')
    logs.append(r.stdout + r.stderr)
    (out / 'compile.log').write_text('\n'.join(logs), encoding='utf-8')
    (out / 'commands.json').write_text(json.dumps(commands + [link], indent=2), encoding='utf-8')
    print(r.stdout + r.stderr)
    if r.returncode:
        raise SystemExit(r.returncode)
    for fmt, ext in [('ihex', 'hex'), ('binary', 'bin')]:
        subprocess.run([str(bin_dir / ('riscv-none-embed-objcopy' + suffix)), '-O', fmt, str(elf), str(elf.with_suffix('.' + ext))], check=True)
    size = subprocess.check_output([str(bin_dir / ('riscv-none-embed-size' + suffix)), str(elf)], text=True)
    version = subprocess.check_output([gcc, '--version'], text=True).splitlines()[0]
    inputs = sorted(set(sources + [ROOT / 'Ld/Link.ld', ROOT / 'zf_device/libzf_device_config.a'] +
                        [p for folder in inc for p in (ROOT / folder).glob('*.h')]))
    manifest = {str(p.relative_to(ROOT)).replace('\\', '/'): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    report = {'firmware_version': firmware_version, 'compiler': version, 'sources': len(sources), 'size': size.replace(str(elf), elf.name),
              'source_files_sha256': manifest,
              'warning_lines': [line.replace(str(ROOT), '.') for log in logs for line in log.splitlines() if 'warning:' in line],
              'artifacts': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in [elf, elf.with_suffix('.hex'), elf.with_suffix('.bin')]}}
    (out / 'build-report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(size)
    print('BUILD PASS:', elf)


if __name__ == '__main__':
    main()
