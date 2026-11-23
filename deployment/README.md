# FRANK TORQUE trial source release

This package contains local, self-contained Git bundles of the tested sources, exact commits and
checksums in `release.json`, a source installer, the Tesollo commissioning procedure and the
software validation report. It starts no hardware and makes no remote GitHub changes.

Copy the complete release directory to the control NUC and operator PC. Inside that directory:

```bash
# Control NUC (ori's existing frank-ros-2-control-nuc workspace):
python3 apply_release.py --workspace ~/ros2_ws --machine control --dry-run
python3 apply_release.py --workspace ~/ros2_ws --machine control
# Operator PC: use the HOST path of frank-ros-2-operator-pc, which is bind-mounted into its container:
python3 apply_release.py --workspace /path/to/frank-ros-2-operator-pc --machine operator --dry-run
python3 apply_release.py --workspace /path/to/frank-ros-2-operator-pc --machine operator
```

The installer selects exact commits in detached checkouts, preserving existing branches. It
refuses dirty repositories and records previous commits/branches in a timestamped rollback JSON
in the workspace. Operator GELLO lives at `src/frank_demos/frank_gello`, matching `demos.repos`;
it is not duplicated at the workspace root. No network access is needed to import these commits.
`--machine all` imports all four sources into a software-validation workspace.

On the control NUC, install dependencies/build/source as described in `commissioning.md`.
In the operator container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select frank_gello
source install/setup.bash
```

Keep the existing model publisher, soft stop, native hand driver and machine/network setup from
the ROS2 repositories. The first trial uses the manual commands in `commissioning.md` in place
of the normal Procman arm-driver command, so there is only one driver per arm. The original
Procman command/configuration is preserved.

The existing `frank.repos` / `demos.repos` manifests are not rewritten. After acceptance, use each
workspace's `scripts/sync_repos.py` to record the selected revisions according to its normal
workflow. Source branches are local until deliberately published; these bundles make tomorrow's
trial independent of publishing them.

To roll back, stop the trial processes, use the recorded previous branch/commit in each checkout,
rebuild and source the workspace, and restart the previous launch. Do not mix a previously built
driver library with newly checked-out headers/model configuration.

To regenerate a release from committed branches in the development parent directory:

```bash
python3 compliant_controllers/deployment/package_release.py --output /path/to/new-release-directory
```

Only committed files enter a bundle. The manifest records any excluded local changes.
