"""Scan → worker export → deterministic merge → verify → atomic publication."""
from concurrent.futures import ProcessPoolExecutor
import ctypes
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from . import ingest
from .export import encoded, manifest_digest, sha256, write_json
from .index import merge, topic_coverage
from .schemas import MANIFEST, COVERAGE
from .verify import verify_artifacts, verify_manifest
from .performance import Performance


def _owned_directory(path: Path):
    if path.is_symlink() or not path.is_dir() or (path / 'manifest.json').is_symlink():
        raise ValueError('overwrite requires a real compiler bundle directory')
    try:
        manifest = json.loads((path / 'manifest.json').read_bytes())
    except (OSError, ValueError) as exc:
        raise ValueError('existing directory is protected: no compiler manifest') from exc
    if manifest.get('schema') != MANIFEST:
        raise ValueError('existing directory is protected: not an MCAP Evidence Compiler bundle')
    return path.stat().st_dev, path.stat().st_ino


def _publish(staging: Path, final: Path, previous):
    # Linux renameat2 gives both no-clobber publication and atomic replacement of
    # a non-empty bundle, with the previous bundle intact until the exchange.
    if previous is not None and _owned_directory(final) != previous:
        raise ValueError('output directory changed while compiling')
    libc = ctypes.CDLL(None, use_errno=True)
    rename = getattr(libc, 'renameat2', None)
    if rename is None:
        raise RuntimeError('atomic bundle publication requires renameat2')
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    flags = 2 if previous is not None else 1  # RENAME_EXCHANGE / RENAME_NOREPLACE
    if rename(-100, os.fsencode(staging), -100, os.fsencode(final), flags):
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), str(final))
    if previous is not None:
        shutil.rmtree(staging)


def _seal(staging, manifest, performance):
    # Only lengths participate in this fixed point, not timing or I/O. The
    # manifest digest has fixed length; no recursively self-hashed file exists.
    artifact_bytes = sum(item['size'] for name, item in manifest['files'].items()
                         if name != 'compiler_performance.json')
    for _ in range(12):
        performance_bytes = encoded(performance)
        manifest['files']['compiler_performance.json'] = {
            'size': len(performance_bytes), 'sha256': hashlib.sha256(performance_bytes).hexdigest()}
        manifest['self_sha256'] = manifest_digest(manifest)
        manifest_bytes = encoded(manifest)
        bundle_bytes = artifact_bytes + len(performance_bytes) + len(manifest_bytes)
        if performance['bytes_written'] == bundle_bytes:
            break
        performance['bytes_written'] = bundle_bytes
    else:
        raise ValueError('bundle size accounting did not converge')
    (staging / 'compiler_performance.json').write_bytes(performance_bytes)
    # The only on-disk manifest is final and is written after every artifact.
    (staging / 'manifest.json').write_bytes(manifest_bytes)


def compile_evidence(source, output=None, *, workers='auto', shard_bytes=32 * 1024 * 1024,
                     overwrite=False, command=None, progress=None):
    performance = Performance()
    source = Path(source).expanduser().absolute()
    if not source.is_file() or source.is_symlink():
        raise ValueError('input must be a regular, non-symlink MCAP file')
    source = source.resolve()
    final = Path(output).expanduser().absolute() if output else source.with_suffix('.evidence')
    if final.is_symlink():
        raise ValueError('output must not be a symlink')
    final = final.resolve()
    if source == final or source.is_relative_to(final):
        raise ValueError('output must not contain the input MCAP')
    if shard_bytes < 1:
        raise ValueError('shard_bytes must be positive')
    if workers != 'auto' and (isinstance(workers, bool) or int(workers) < 1):
        raise ValueError('workers must be auto or a positive integer')
    previous = None
    if final.exists():
        if not overwrite:
            raise FileExistsError('output already exists: ' + str(final))
        previous = _owned_directory(final)
    final.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=final.name + '.tmp-', dir=final.parent))
    notify = progress or (lambda message: None)
    performance.path = staging / 'compiler_performance.json'
    performance.data.update(source_path=str(source), output_path=str(final),
                            input_mcap_bytes=source.stat().st_size)
    performance.write_progress()
    try:
        notify('scan')
        with performance.stage('scan'):
            performance.data['source_scan_count'] += 1
            integrity = ingest.scan(source, staging)
        units = integrity.pop('work_units')
        captured = integrity['recovered_by_topic']
        performance.data.update(
            input_mcap_bytes=integrity['source_snapshot_size'],
            tick_count=captured.get('/r2b4/tick', 0),
            raw_lidar_count=captured.get('/r2b4/raw_lidar', 0),
            checkpoint_count=captured.get('/r2b4/checkpoint', 0))
        count = min(os.cpu_count() or 1, len(units)) if workers == 'auto' else min(int(workers), len(units))
        parameters = {'workers_requested': workers, 'workers': count, 'shard_bytes': shard_bytes,
                      'overwrite': overwrite, 'output': str(final)}
        performance.data['parameters'] = parameters
        notify(f'export: {len(units)} units, {count} workers')
        args = ((unit, str(staging), shard_bytes) for unit in units)
        worker_roots = []
        with performance.stage('worker_export'):
            if count <= 1:
                for arg in args:
                    result = ingest.process_unit(arg)
                    performance.worker(result)
                    worker_roots.append(Path(result['root']))
            else:
                with ProcessPoolExecutor(max_workers=count, mp_context=multiprocessing.get_context('spawn')) as pool:
                    for result in pool.map(ingest.process_unit, args):
                        performance.worker(result)
                        worker_roots.append(Path(result['root']))
        notify('index and coverage')
        with performance.stage('index_merge'):
            counts = merge(staging, worker_roots)
        with performance.stage('artifact_merge'):
            for worker in worker_roots:
                for path in sorted(worker.rglob('*.ndjson')):
                    destination = staging / path.relative_to(worker)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    path.rename(destination)
            shutil.rmtree(staging / '.work')
        with performance.stage('coverage'):
            by_topic = counts.pop('exported_by_topic')
            coverage = {
                'schema': COVERAGE, 'source_bytes': integrity['source_snapshot_size'],
                'captured_messages_scope': 'structurally_recovered_messages_in_source_snapshot',
                'exact_data_available_in_mcap_scope': 'original payload bytes at snapshot scan time',
                'source_message_total_known': integrity['source_integrity'] == 'COMPLETE',
                'unrecoverable_message_count': 0 if integrity['source_integrity'] == 'COMPLETE' else None,
                'topics': {topic: topic_coverage(total, by_topic.get(topic, {}))
                           for topic, total in sorted(captured.items())},
                'unknown_topic': topic_coverage(integrity['unknown_topic_messages'], by_topic.get(None, {})),
                'messages': {'recovered': integrity['recovered_messages'],
                             'json_decoded': counts['json_decoded'],
                             'quarantined': counts['exported'] - counts['json_decoded'],
                             'exported': counts['exported']},
                'byte_ranges': integrity['byte_ranges'],
                'field_occurrences': counts['field_occurrences'], 'normalized_views': counts['normalized_views'],
                'invariants': {'all_recovered_messages_accounted_for': counts['exported'] == integrity['recovered_messages'],
                               'all_unrecoverable_ranges_reported': True},
            }
            write_json(staging / 'integrity.json', integrity)
            write_json(staging / 'coverage.json', coverage)
        status = 'COMPLETE' if integrity['source_integrity'] == 'COMPLETE' else 'PARTIAL'
        with performance.stage('manifest'):
            repo = Path(__file__).resolve().parents[2]
            commit = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=repo, capture_output=True, text=True, check=False)
            dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=normal'], cwd=repo,
                                   capture_output=True, text=True, check=False)
            sources = {p.name: sha256(p) for p in sorted(Path(__file__).parent.glob('*.py'))}
            manifest = {
                'schema': MANIFEST, 'complete': True,
                'compiler': {'git_commit': commit.stdout.strip() or None, 'git_dirty': bool(dirty.stdout),
                             'source_sha256': sources, 'command': command or ['r', 'evi', str(source)],
                             'parameters': parameters, 'workers': count},
                'source': {'path': str(source), 'snapshot_size': integrity['source_snapshot_size'],
                           'sha256': integrity['source_snapshot_sha256'], 'hash_scope': 'snapshot_prefix'},
                'analysis_profile': 'LOSSLESS_EXTRACTION_NO_ANALYZERS_NO_REPLAY',
                'compiler_status': status,
                'source_integrity': integrity['source_integrity'],
                'all_recoverable_messages_accounted_for': coverage['invariants']['all_recovered_messages_accounted_for'],
                'files': {str(p.relative_to(staging)): {'size': p.stat().st_size, 'sha256': sha256(p)}
                          for p in sorted(staging.rglob('*')) if p.is_file()},
            }
        notify('verify')
        with performance.stage('verification'):
            # The progress profile can change between stages; pin its current hash.
            manifest['files']['compiler_performance.json'] = {
                'size': performance.path.stat().st_size, 'sha256': sha256(performance.path)}
            # Verify the complete payload before sealing timing + final manifest.
            # No provisional manifest is written to disk.
            verify_artifacts(staging, manifest)
        after = source.stat()
        integrity['source_size_after_compile'] = after.st_size
        integrity['source_changed_during_compile'] = (
            [after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns]
            != integrity['source_stat_before'])
        write_json(staging / 'integrity.json', integrity)
        manifest['files']['integrity.json'] = {
            'size': (staging / 'integrity.json').stat().st_size,
            'sha256': sha256(staging / 'integrity.json')}
        _seal(staging, manifest, performance.snapshot(status))
        verify_manifest(staging)
        notify('publish')
        _publish(staging, final, previous)
        return {'compiler_status': status, 'output': str(final),
                'performance': str(final / 'compiler_performance.json'),
                'messages': counts['exported'], 'source_integrity': integrity['source_integrity']}
    except BaseException as exc:
        # A failed/interrupted attempt must not publish a final bundle. Keep its
        # measurements separately so even a slow failed run can be investigated.
        status = 'INTERRUPTED' if isinstance(exc, KeyboardInterrupt) else 'FAILED'
        try:
            diagnostic = Path(tempfile.mkdtemp(prefix='r2b4-evi-performance-', dir='/tmp')) / 'compiler_performance.json'
            measured = performance.snapshot(status)
            measured['total_duration_scope'] = 'compile entry through failure/interruption'
            measured['bytes_written_scope'] = 'partial staging logical bytes at failure; excludes removed work files'
            measured['bytes_written'] = sum(p.stat().st_size for p in staging.rglob('*') if p.is_file())
            measured['error'] = {'type': type(exc).__name__, 'message': str(exc)}
            write_json(diagnostic, measured)
            exc.compiler_performance_path = str(diagnostic)
        except OSError:
            pass  # Preserve the original failure if even diagnostic I/O fails.
        raise
    finally:
        if staging.exists():
            shutil.rmtree(staging)
