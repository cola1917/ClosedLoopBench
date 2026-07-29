# Development and Remote Sync Policy

This repository uses GitHub as the source of truth for tracked source code.
The local checkout is the only development checkout. The remote host at
`${CLB_REMOTE_SSH_HOST}:${CLB_REMOTE_SSH_PORT}` is a test checkout and must not be used for ad-hoc code
edits or bundle-based source transfer.

## Normal flow

1. Develop and run focused tests locally.
2. Commit the reviewed change on local `master` (or a short-lived feature
   branch merged locally).
3. Push `master` to `origin` with a normal fast-forward push.
4. On the remote checkout, fetch GitHub and update `master` with
   `git pull --ff-only origin master`.
5. Run the remote validation command from that exact commit and preserve its
   evidence separately from the source checkout.

The remote checkout must report the same `master` commit as the local checkout
before a result is attributed to the new code. `tools/scene0061_sync.py status`
can be used for an exact SHA comparison when the remote checkout is on the
intended branch.

## Branch and history rules

- Never use `git reset --hard`, force-push, or overwrite a branch to resolve a
  divergence.
- Before reconciliation, create a dated backup ref for every affected branch
  and record the current worktree status.
- Reconcile only with an explicit merge or a verified fast-forward. If the
  histories are not related, stop and review the commit/file overlap before
  choosing a merge strategy.
- Keep M-series experiment branches and evidence refs available for audit;
  they are not the development baseline unless deliberately merged.

## Runtime and evidence boundary

Tracked source is synchronized through Git only. Do not create or consume
source bundles for normal development. Runtime state and evidence are not part
of the source synchronization contract and must remain on the host that
created them, including the remote `.runtime/`, `.sync-backups/`, `incoming/`,
output, cache, container, and checkpoint directories. Do not delete or prune
those paths during Git governance.

Remote tests must be run from a clean tracked checkout. The only expected
untracked paths are explicitly documented runtime directories; they must not
be staged or committed.

## Current governance record

On 2026-07-29, local `master` was verified to contain remote `master` and to
be a fast-forward descendant of GitHub `origin/master`. Dated backup refs were
created under `refs/backup/closedloopbench/20260729/` before synchronization.
The remote M8 WIP worktree and its runtime directories were intentionally left
untouched.
