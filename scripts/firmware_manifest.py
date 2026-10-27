"""Record artifact hashes and actual source state, including uncommitted work."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess

parser = argparse.ArgumentParser()
for name in ('elf', 'source', 'compiler', 'size', 'can', 'actuation', 'watchdog', 'decoder', 'build-type'):
    parser.add_argument('--' + name, required=True)
parser.add_argument('--unpowered-signal-test', default='0')
parser.add_argument('--controlled-bench-actuation', default='0')
parser.add_argument('--commissioning-hex')
parser.add_argument('--direct-commands', default='0')
args = parser.parse_args()
root, elf = Path(args.source), Path(args.elf)


def git(*arguments):
    return subprocess.check_output(['git', '-C', str(root), *arguments])


# Hash actual build inputs, not only HEAD (this workspace is intentionally dirty).
paths = [Path(p.decode()) for p in git('ls-files', '-z', '--cached', '--others', '--exclude-standard').split(b'\0') if p]
paths = sorted(set(p for p in paths if p.parts and p.parts[0] in
                   ('App', 'Core', 'Drivers', 'Middlewares', 'lib', 'cmake') or
                   p.as_posix() in ('CMakeLists.txt', 'CMakePresets.json', 'STM32F103xx_FLASH.ld',
                              'startup_stm32f103xb.s', 'Bluepill-Actuator-Node.ioc')))
digest = hashlib.sha256()
for relative in paths:
    path = root / relative
    if path.is_file():
        digest.update(relative.as_posix().encode() + b'\0' + path.read_bytes() + b'\0')
artifacts = {}
for path in (elf, elf.with_suffix('.hex'), elf.with_suffix('.map'),
             *([Path(args.commissioning_hex)] if args.commissioning_hex else [])):
    if path.is_file():
        artifacts[path.name] = {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                'bytes': path.stat().st_size}
image = elf.read_bytes()
phoff = struct.unpack_from('<I', image, 28)[0]
phsize, phcount = struct.unpack_from('<HH', image, 42)
flash_end, ram_end = 0x08000000, 0x20000000
for i in range(phcount):
    kind, offset, virtual, physical, filesz, memsz, flags, align = struct.unpack_from('<8I', image, phoff + i * phsize)
    if kind != 1:
        continue
    if filesz and 0x08000000 <= physical < 0x08010000:
        flash_end = max(flash_end, physical + filesz)
    if memsz and 0x20000000 <= virtual < 0x20005000:
        ram_end = max(ram_end, virtual + memsz)
manifest = {
    'head': git('rev-parse', 'HEAD').decode().strip(),
    'branch': git('branch', '--show-current').decode().strip(),
    'dirty': bool(git('status', '--porcelain')),
    'source_sha256': digest.hexdigest(),
    'build_type': args.build_type,
    'memory': {'flash_used_bytes': flash_end - 0x08000000,
               'ram_linker_reserved_bytes': ram_end - 0x20000000,
               'flash_capacity_bytes': 65536, 'ram_capacity_bytes': 20480},
    'compiler': subprocess.check_output([args.compiler, '--version']).decode().splitlines()[0],
    'features': {name: bool(int(getattr(args, name))) for name in ('can', 'actuation', 'watchdog', 'decoder')},
    'unpowered_signal_test': bool(int(args.unpowered_signal_test)),
    'controlled_bench_actuation': bool(int(args.controlled_bench_actuation)),
    'direct_commands': bool(int(args.direct_commands)),
    'powered_operation_approved': False,
    'physically_verified': False,
    'commissioning_candidate': bool(int(args.controlled_bench_actuation)),
    'operation_scope': 'commissioning_candidate' if int(args.controlled_bench_actuation) else
                       'unpowered_signal_test' if int(args.unpowered_signal_test) else 'inhibited',
    'physical_safety_inputs': 'not_connected',
    'artifacts': artifacts,
}
size = subprocess.check_output([args.size, str(elf)], text=True)
(elf.parent / 'firmware.size.txt').write_text(size)
(elf.parent / 'firmware-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
(elf.parent / 'SHA.txt').write_text('HEAD=' + manifest['head'] + '\nSOURCE_SHA256=' + digest.hexdigest() +
                                  '\nDIRTY=' + str(manifest['dirty']) + '\n' +
                                  ''.join(name + '_SHA256=' + item['sha256'] + '\n'
                                          for name, item in artifacts.items()))
print('Firmware manifest:', elf.parent / 'firmware-manifest.json')
