# V2 installer correction

The V1 installer had one overly broad text anchor. The four-line pending-state
clear block occurs in both `_accept_pending_rollout()` and
`_abandon_pending_rollout()`, so the installer's uniqueness guard stopped
before writing any file.

V2 changes only installer robustness:
- the final anchor is scoped to `_abandon_pending_rollout()`;
- exact Git blob SHA is checked for all five target files;
- `--check` performs a dry run and writes nothing;
- anchor errors report the exact patch index.

The production patch design itself is unchanged from V1.
