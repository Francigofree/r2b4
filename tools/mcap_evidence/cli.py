"""Explicit developer CLI; compiler statuses never describe robot behavior."""
import argparse
import json
import signal
import sys

from .compiler import compile_evidence
from .query import query
from .verify import verify


def parser():
    result = argparse.ArgumentParser(prog='r evi', description='MCAP → lossless, indexed evidence. No diagnosis or replay.')
    result.add_argument('source', help='MCAP input (or bundle for verify/query)')
    result.add_argument('--output', '--output-dir', dest='output')
    result.add_argument('--workers', default='auto')
    result.add_argument('--shard-bytes', type=int, default=32 * 1024 * 1024)
    result.add_argument('--overwrite', action='store_true')
    return result


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    old_handler = signal.getsignal(signal.SIGTERM)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        if args and args[0] in ('verify', 'verify-evidence'):
            p = argparse.ArgumentParser(prog='r evi verify')
            p.add_argument('bundle')
            p.add_argument('--source', help='also verify the original snapshot hash')
            a = p.parse_args(args[1:])
            result = verify(a.bundle, source=a.source)
        elif args and args[0] == 'query':
            p = argparse.ArgumentParser(prog='r evi query')
            p.add_argument('bundle')
            for option in ('topic', 'message-id', 'layer', 'sensor', 'field'):
                p.add_argument('--' + option)
            for option in ('tick-id', 'channel', 'sequence', 'source-offset', 'start-ns', 'end-ns'):
                p.add_argument('--' + option, type=int)
            p.add_argument('--limit', type=int, default=100)
            a = vars(p.parse_args(args[1:]))
            bundle = a.pop('bundle')
            for row in query(bundle, **a):
                print(json.dumps(row, ensure_ascii=True, allow_nan=False))
            return 0
        else:
            a = parser().parse_args(args)
            result = compile_evidence(a.source, a.output, workers=a.workers, shard_bytes=a.shard_bytes,
                                      overwrite=a.overwrite, command=['r', 'evi', *args],
                                      progress=lambda stage: print('evi: ' + stage, file=sys.stderr, flush=True))
        print(json.dumps(result, sort_keys=True))
        return 0
    except KeyboardInterrupt as exc:
        print(json.dumps({'compiler_status': 'INTERRUPTED',
                          'performance': getattr(exc, 'compiler_performance_path', None)}), file=sys.stderr)
        return 130
    except Exception as exc:
        print(json.dumps({'compiler_status': 'FAILED', 'error': str(exc),
                          'performance': getattr(exc, 'compiler_performance_path', None)}), file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGTERM, old_handler)
