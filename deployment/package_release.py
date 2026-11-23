#!/usr/bin/env python3
"""Package committed source snapshots; uncommitted working-tree changes are excluded."""
import argparse
import hashlib
import json
import pathlib
import shutil
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCES = {
    'compliant_controllers': ('ros2', ['control'], 'src/compliant_controllers'),
    'ros2_kortex': ('ros2-compliant', ['control'], 'src/ros2_kortex'),
    'frank-robot-ros-2': ('compliant-torque', ['control', 'operator'], 'src/frank-robot-ros-2'),
    'frank_gello': ('compliant-torque', ['operator'], 'src/frank_demos/frank_gello'),
}


def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repos-root', type=pathlib.Path, default=ROOT.parent)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = {'target': 'ROS2 Jazzy / Ubuntu 24.04 amd64; Kinova TORQUE; Tesollo first',
                'ros_snapshot': '2026-11-16',
                'exclaim_source': '140f7965604730b7f753612e0521a8eafac1af2f',
                'ros1_controller_reference': 'e146030dc8e5bb264ccecf8efaedeb80906c10cf',
                'robotiq_model_dependency': '77ba6fd7520d8c197368e744384c20910a1f6adb',
                'repositories': {}}
    for name, (branch, machines, target) in SOURCES.items():
        repo = args.repos_root/name
        commit = git(repo, 'rev-parse', branch)
        bundle = output/f'{name}.bundle'
        subprocess.run(['git', '-C', str(repo), 'bundle', 'create', str(bundle), branch], check=True)
        subprocess.run(['git', '-C', str(repo), 'bundle', 'verify', str(bundle)], check=True)
        manifest['repositories'][name] = {
            'commit': commit, 'branch': branch, 'machines': machines, 'path': target,
            'url': git(repo, 'remote', 'get-url', 'origin'), 'bundle': bundle.name,
            'bundle_sha256': hashlib.sha256(bundle.read_bytes()).hexdigest(),
            'excluded_worktree_changes': git(repo, 'status', '--porcelain').splitlines(),
        }
    (output/'release.json').write_text(json.dumps(manifest, indent=2)+'\n')
    shutil.copy2(ROOT/'deployment/apply_release.py', output)
    shutil.copy2(ROOT/'deployment/README.md', output/'README.md')
    shutil.copy2(ROOT/'docs/commissioning.md', output/'commissioning.md')
    shutil.copy2(ROOT/'docs/validation.md', output/'validation.md')
    print(output)


if __name__ == '__main__':
    main()
