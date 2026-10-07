#!/usr/bin/env python3
from pathlib import Path
import argparse, shutil

BACKUP_REL = Path('runtime/upgrade_backups/prompt_budget_66ef376')
FILES = (
    'conf/r2b4_agent_system.md',
    'r2b4_orchestration/agent_core.py',
    'r2b4_voice/conversation_contracts.py',
    'r2b4_voice/conversation_service.py',
    'r2b4_voice/prompting.py',
    'r2b4_voice/robot_context.py',
    'tests/packs/feature/test_agent_core.py',
    'tests/packs/feature/test_public_robot_interface.py',
)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('repo', nargs='?', default='.')
    args=ap.parse_args()
    root=Path(args.repo).expanduser().resolve()
    backup=root/BACKUP_REL
    if not backup.is_dir():
        raise SystemExit(f'backup not found: {backup}')
    for rel in FILES:
        src=backup/rel
        if not src.is_file():
            raise SystemExit(f'backup file missing: {src}')
        dst=root/rel
        dst.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(src,dst)
    (root/'tests/packs/core/test_prompt_budget_upgrade.py').unlink(missing_ok=True)
    print('R2B4 prompt-budget upgrade rolled back.')

if __name__=='__main__':
    main()
