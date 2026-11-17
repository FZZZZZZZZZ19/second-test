"""Check real target instructions retain both START hold-time loops.

This checks compiler elimination, not absolute SCL frequency or board operation.
"""
import argparse
from pathlib import Path
import re
import subprocess


def backwards(text):
    edges = []
    for line in text.splitlines():
        m = re.match(r'\s*([0-9a-f]+):\s+[0-9a-f]+\s+(\w+)\s*(.*)', line)
        if not m:
            continue
        pc, op, args = int(m[1], 16), m[2], m[3]
        if op not in ['j', 'beq', 'bne', 'blt', 'bge', 'bltu', 'bgeu', 'beqz', 'bnez', 'bgtz', 'blez']:
            continue
        dest = re.search(r'(?:^|,)\s*([0-9a-f]+)\s+<', args)
        if dest and int(dest[1], 16) < pc:
            edges.append((pc, int(dest[1], 16)))
    return len(edges)


def check(objdump, elf):
    def dump(name):
        return subprocess.check_output([str(objdump), '-d', '--disassemble='+name, str(elf)], text=True)
    text = dump('soft_iic_start')
    inline = backwards(text)
    calls = len(re.findall(r'\bjal\s+[^\n]*<soft_iic_delay>', text))
    # GCC -Os can share the delay helper instead of inlining it. Follow it;
    # merely seeing calls to a helper that was itself optimized away is not enough.
    helper_loops = backwards(dump('soft_iic_delay')) if calls else 0
    waits = inline + (calls if helper_loops else 0)
    if waits < 2:
        raise SystemExit('FAIL: soft_iic_start retains %d waits; both bus delays must survive optimization' % waits)
    print('PASS: soft_iic_start retains %d waits (%d inline, %d calls to a looping helper) in %s; absolute timing not measured' % (waits, inline, calls if helper_loops else 0, elf))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--elf', type=Path, required=True)
    p.add_argument('--objdump', type=Path, required=True)
    a = p.parse_args()
    check(a.objdump, a.elf)
