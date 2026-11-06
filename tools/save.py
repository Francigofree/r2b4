#!/usr/bin/env python3
"""Backup /home/alba/project_r2b4 to a timestamped ZIP, excluding runtime/."""

from datetime import datetime
from pathlib import Path
import os
import stat
import sys
import zipfile

SOURCE = Path('/home/alba/project_r2b4')
DESTINATION = Path('/home/alba/save')


def add_symlink(archive: zipfile.ZipFile, path: Path, name: str) -> None:
    """Store symlinks as links, without reading files outside the project."""
    info = zipfile.ZipInfo(name)
    info.create_system = 3
    info.external_attr = (stat.S_IFLNK | 0o777) << 16
    info.compress_type = zipfile.ZIP_STORED
    archive.writestr(info, os.readlink(path).encode('utf-8', 'surrogateescape'))


def main() -> int:
    if not SOURCE.is_dir():
        print(f'HIBA: Nem található a forráskönyvtár: {SOURCE}', file=sys.stderr)
        return 1

    DESTINATION.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    target = DESTINATION / f'{stamp}.zip'
    temporary = DESTINATION / f'.{stamp}.zip.partial'
    count = 0

    try:
        with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED,
                             compresslevel=6, allowZip64=True) as archive:
            for root, dirs, files in os.walk(SOURCE, topdown=True, followlinks=False):
                folder = Path(root)
                if folder == SOURCE:
                    dirs[:] = [d for d in dirs if d != 'runtime']

                # Don't follow symlinked directories (which might point outside SOURCE).
                for directory in list(dirs):
                    path = folder / directory
                    if path.is_symlink():
                        add_symlink(archive, path, path.relative_to(SOURCE.parent).as_posix())
                        dirs.remove(directory)
                        count += 1

                if not dirs and not files:
                    name = folder.relative_to(SOURCE.parent).as_posix() + '/'
                    archive.writestr(name, b'')

                for filename in files:
                    path = folder / filename
                    name = path.relative_to(SOURCE.parent).as_posix()
                    if path.is_symlink():
                        add_symlink(archive, path, name)
                    else:
                        archive.write(path, arcname=name)
                    count += 1

        # Verify that the archive is readable before publishing it.
        with zipfile.ZipFile(temporary) as archive:
            corrupted = archive.testzip()
            if corrupted:
                raise OSError(f'Hibás ZIP-bejegyzés: {corrupted}')
        temporary.replace(target)
    except Exception as exc:
        temporary.unlink(missing_ok=True)
        print(f'HIBA: A mentés sikertelen: {exc}', file=sys.stderr)
        return 1

    print(f'Mentés kész: {target}')
    print(f'Mentett fájlok/linkek: {count}')
    print('Kihagyva: project_r2b4/runtime/')
    return 0


if __name__ == '__main__':
    sys.exit(main())
