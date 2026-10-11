"""Explicit developer CLI; compiler statuses never describe robot behavior."""
import argparse
import json
import os
from pathlib import Path
import signal
import stat
import sys

from .compiler import compile_evidence
from .query import query
from .verify import verify


OPERATIONS = ('compile', 'verify', 'query')
VERIFY_OPERATIONS = ('verify', 'verify-evidence')


def latest_capture(project_root=None):
    """Select a published local capture without opening or modifying it."""
    root = Path(project_root or os.environ.get('R2B4_ROOT') or Path(__file__).resolve().parents[2])
    capture_dir = root.expanduser().resolve() / 'runtime' / 'captures'
    candidates = []
    for path in capture_dir.glob('*.mcap'):
        try:
            info = path.lstat()
        except FileNotFoundError:
            continue
        if stat.S_ISREG(info.st_mode):
            candidates.append((info.st_mtime_ns, path.name, path))
    if not candidates:
        raise ValueError('no finalized MCAP capture found under ' + str(capture_dir)
                         + '; provide an explicit MCAP input')
    return max(candidates)[2]


def parser(operation=None):
    if operation in VERIFY_OPERATIONS:
        result = argparse.ArgumentParser(prog='r evi verify')
        result.add_argument('bundle')
        result.add_argument('--source', help='also verify the original snapshot hash')
        return result
    if operation == 'query':
        result = argparse.ArgumentParser(prog='r evi query')
        result.add_argument('bundle')
        for option in ('topic', 'message-id', 'layer', 'sensor', 'field'):
            result.add_argument('--' + option)
        for option in ('tick-id', 'channel', 'sequence', 'source-offset', 'start-ns', 'end-ns'):
            result.add_argument('--' + option, type=int)
        result.add_argument('--limit', type=int, default=100)
        return result
    if operation not in (None, 'compile'):
        raise ValueError('unknown evidence operation: ' + str(operation))
    result = argparse.ArgumentParser(prog='r evi compile' if operation else 'r evi',
                                     description='MCAP → lossless, indexed evidence. No diagnosis or replay.')
    result.add_argument('source', nargs='?', default='latest',
                        help='MCAP input; omitted/latest selects the newest finalized runtime/captures/*.mcap')
    result.add_argument('--output', '--output-dir', dest='output')
    result.add_argument('--workers', default='auto')
    result.add_argument('--shard-bytes', type=int, default=32 * 1024 * 1024)
    result.add_argument('--overwrite', action='store_true')
    return result


def main(argv=None, *, project_root=None):
    args = list(sys.argv[1:] if argv is None else argv)
    old_handler = signal.getsignal(signal.SIGTERM)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        if args and args[0] in VERIFY_OPERATIONS:
            a = parser(args[0]).parse_args(args[1:])
            result = verify(a.bundle, source=a.source)
        elif args and args[0] == 'query':
            a = vars(parser('query').parse_args(args[1:]))
            bundle = a.pop('bundle')
            for row in query(bundle, **a):
                print(json.dumps(row, ensure_ascii=True, allow_nan=False))
            return 0
        else:
            operation = 'compile' if args and args[0] == 'compile' else None
            a = parser(operation).parse_args(args[1:] if operation else args)
            source = a.source
            if source == 'latest':
                source = str(latest_capture(project_root))
                print('evi input: ' + source, file=sys.stderr, flush=True)
            result = compile_evidence(source, a.output, workers=a.workers, shard_bytes=a.shard_bytes,
                                      overwrite=a.overwrite, command=['r', 'evi', *args],
                                      progress=lambda stage: print('evi: ' + stage, file=sys.stderr, flush=True))
        print(json.dumps(result, sort_keys=True))
        return 0
    except KeyboardInterrupt as exc:
        print(json.dumps({'compiler_status': 'INTERRUPTED',
                          'performance': getattr(exc, 'compiler_performance_path', None)}), file=sys.stderr)
        return 130
    except Exception as exc:
        error = str(exc)
        if isinstance(exc, FileExistsError):
            error += '; use --output /path/to/new.evidence for a separate bundle'
        print(json.dumps({'compiler_status': 'FAILED', 'error': error,
                          'performance': getattr(exc, 'compiler_performance_path', None)}), file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGTERM, old_handler)
