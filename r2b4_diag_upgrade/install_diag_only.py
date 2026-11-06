"""Install ONLY the new offline DIAG analyzer into an existing r2b4 working tree.

Requires a matching current repo with canonical tools/diag/registry.py.
No robot runtime, capture, config, navigation or production source is modified.
"""
from pathlib import Path
import argparse
import shutil

SOURCE = Path(__file__).parent / 'tools/diag/analyzers/motion_forensics.py'

def install(root: Path):
    registry = root / 'tools/diag/registry.py'
    destination = root / 'tools/diag/analyzers/motion_forensics.py'
    if not registry.exists() or not (root / 'tools/diag/contracts.py').exists():
        raise SystemExit('Not a supported R2B4 DIAG checkout; no files modified.')
    contents = registry.read_text()
    if 'motion_forensics,' in contents and 'registry.register(motion_forensics.CONTRACT, motion_forensics.run)' in contents:
        print('Already registered')
    else:
        anchor = '        motion_performance,\n'
        registration = '    return registry\n'
        if contents.count(anchor) != 1 or contents.count(registration) != 1:
            raise SystemExit('Registry layout changed. No files modified; inspect source first.')
        contents = contents.replace(anchor, anchor + '        motion_forensics,\n', 1)
        contents = contents.replace(registration, '    registry.register(motion_forensics.CONTRACT, motion_forensics.run)\n' + registration, 1)
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Installation is idempotent; save original registry for rollback.
    if registry.read_text() != contents:
        backup = registry.with_suffix('.py.motion_forensics_backup')
        if not backup.exists():
            shutil.copy2(registry, backup)
        registry.write_text(contents)
    shutil.copy2(SOURCE, destination)
    print('Installed', destination, 'and registered only the diagnostic analyzer')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('repo_root', type=Path)
    args = parser.parse_args()
    install(args.repo_root.resolve())
