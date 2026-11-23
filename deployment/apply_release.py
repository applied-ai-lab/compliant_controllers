#!/usr/bin/env python3
"""Apply this release's committed source bundles to an existing native ROS2 workspace.

No robot, service, process or hardware state is changed. Existing checkouts must be clean;
their previous commits/branches are recorded for rollback. No network is used by this script.
"""
import argparse
import datetime
import hashlib
import json
import pathlib
import subprocess


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=pathlib.Path, required=True)
    parser.add_argument('--machine', choices=['control', 'operator', 'all'], required=True)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    release = pathlib.Path(__file__).resolve().parent
    manifest = json.loads((release/'release.json').read_text())
    workspace = args.workspace.expanduser().resolve()
    plans = []
    # Check every selected repository before changing any of them.
    for name, entry in manifest['repositories'].items():
        if args.machine != 'all' and args.machine not in entry['machines']:
            continue
        target = workspace/entry['path']
        bundle = release/entry['bundle']
        if hashlib.sha256(bundle.read_bytes()).hexdigest() != entry['bundle_sha256']:
            raise RuntimeError(f'Bundle checksum mismatch: {bundle.name}')
        previous = None
        if (target/'.git').exists():
            if git(target, 'status', '--porcelain'):
                raise RuntimeError(f'{target} has local changes; preserve them before applying this snapshot')
            previous = {'commit': git(target, 'rev-parse', 'HEAD'),
                        'branch': git(target, 'branch', '--show-current')}
        elif target.exists() and any(target.iterdir()):
            raise RuntimeError(f'{target} is nonempty and is not a Git checkout')
        plans.append((name, entry, target, bundle, previous))
        print(f'{name}: {target} -> {entry["commit"]}')
    if args.dry_run:
        return
    workspace.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    rollback = workspace/f'frank-torque-rollback-{stamp}.json'
    rollback.write_text(json.dumps({str(t): prev for _, _, t, _, prev in plans}, indent=2)+'\n')
    for _, entry, target, bundle, previous in plans:
        target.parent.mkdir(parents=True, exist_ok=True)
        if previous is None:
            subprocess.run(['git', 'clone', '--branch', entry['branch'], str(bundle), str(target)], check=True)
            subprocess.run(['git', '-C', str(target), 'remote', 'set-url', 'origin', entry['url']], check=True)
        else:
            subprocess.run(['git', '-C', str(target), 'fetch', str(bundle), f'refs/heads/{entry["branch"]}'], check=True)
        if git(target, 'rev-parse', entry['commit']) != entry['commit']:
            raise RuntimeError(f'Unexpected source commit in {target}')
        subprocess.run(['git', '-C', str(target), 'switch', '--detach', entry['commit']], check=True)
    print(f'Applied source snapshots. Previous checkouts: {rollback}')
    print('Build and source the workspace before following the commissioning runbook.')


if __name__ == '__main__':
    main()
