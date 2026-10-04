# CONTRIBUTING — Autonomous Navigation Mission 2026  

---

## Branch model

```text
main       ← stable, tagged releases only (protected)
develop    ← integration branch (CI + at least 1 review)

sw/*       ← software feature / bugfix branches
             e.g. sw/nav2-costmap-tuning, sw/imu-bridge, sw/zed-perception

misc/*     ← docs, CI, repo-wide chores
             e.g. misc/readme-update, misc/ci-pipeline
````

**Goal:** keep `main` always deployable on the rover, and keep `develop` always buildable + sim-tested.

---

## 1 · Observe before you change 📋

Before creating a branch or editing files, see where you stand:

```bash
git status                               # staged / dirty files
git log --graph --oneline --decorate -6  # recent history
gitk --all &                             # full GUI graph (optional)
```

If your local `develop` is behind:

```bash
git checkout develop
git pull
```

---

## 2 · Example — add Nav2 config + node to `src/maya_nav2`

### 2-A  Create a software feature branch

Always branch off **develop**:

```bash
git checkout develop
git pull
git switch -c sw/nav2-costmap-tuning
```

### 2-B  Code + local test (ROS 2 workspace)

Assuming this repo is the root of the ROS 2 workspace:

```bash
# Create / edit code and configs
nano src/maya_nav2/src/dynamic_obstacle_layer.cpp
nano config/nav2/costmap_common_params.yaml

# Build
colcon build --symlink-install

# Source (if not already)
source install/setup.bash

# Quick sim check (example)
ros2 launch maya_bringup sim_nav2_gazebo.launch.py
```

If this repo is inside a larger workspace’s `src/`, run those `colcon` commands from the workspace root instead.

### 2-C  Stage & commit (Conventional Commit)

Keep commits small and focused:

```bash
git add src/maya_nav2/src/dynamic_obstacle_layer.cpp
git add config/nav2/costmap_common_params.yaml

git commit -m "feat(nav2): add custom dynamic obstacle layer"
```

For a pure config change, something like:

```bash
git commit -m "chore(nav2): tune global costmap inflation radius"
```

### 2-D  Visual sanity-check

```bash
git log --graph --oneline --decorate --all -4
```

Make sure you only see the commits you expect on your branch.

### 2-E  Push & open PR (GitHub CLI)

```bash
git push -u origin sw/nav2-costmap-tuning

# If using GitHub CLI:
gh pr create --base develop --fill
```

If you don’t use `gh`, just open the PR in the GitHub web UI with base = `develop`.

---

## 3 · Review loop

Reviewers can:

```bash
# See diff
gh pr diff <#>

# Check out branch locally
gh pr checkout <#>
colcon build --symlink-install

# Run sim tests (example)
ros2 launch maya_bringup sim_nav2_gazebo.launch.py
```

If changes are needed, the author updates and amends:

```bash
# Edit files, then:
git commit --amend
git push --force-with-lease
```

**Rule of thumb:**

* feature branches: allowed to rewrite history (`--force-with-lease`)
* `develop` / `main`: never rewritten

---

## 4 · Integrate: squash-merge → `develop`

Once the PR is approved and CI is green:

```bash
gh pr merge <#> --squash --delete-branch
```

Squash keeps `develop` history linear and readable:

* One PR → one commit on `develop`
* The commit message should summarize the feature and include any relevant context (e.g., “validated in Gazebo rock-field world”).

Field tests on the rover can be noted in the PR description and/or commit body.

---

## 5 · Baseline: fast-forward `develop → main` + tag

When `develop` is in a state you’re willing to deploy to the rover for URC-style runs:

```bash
git checkout main
git pull
git merge --ff-only develop
git push origin main
```

Tag the release:

```bash
git tag -a v0.2.0 -m "Autonomous Navigation: Nav2 tuning for rock-field scenario"
git push origin v0.2.0
```

Tag names and messages should reflect **mission milestones** (e.g., “first full 2 km sim run”, “MVP URC route demo”).

---

## 6 · Quick command reference

| Task                | Command                                            |
| ------------------- | -------------------------------------------------- |
| Stage / dirty files | `git status`                                       |
| Compact graph       | `git log --graph --oneline --decorate --all`       |
| GUI graph           | `gitk --all &`                                     |
| New branch          | `git switch -c sw/<feature>`                       |
| Diff vs develop     | `git diff develop...HEAD`                          |
| Push & track        | `git push -u origin <branch>`                      |
| Open PR (CLI)       | `gh pr create --base develop --fill`               |
| List PRs            | `gh pr list`                                       |
| Approve (CLI)       | `gh pr review <#> --approve`                       |
| Squash-merge        | `gh pr merge <#> --squash --delete-branch`         |
| Fast-forward main   | `git checkout main && git merge --ff-only develop` |

---

## 7 · Commit type cheat-sheet

Use Conventional Commit–style prefixes:

| type    | purpose               | example                                      |
| ------- | --------------------- | -------------------------------------------- |
| `feat`  | new capability        | `feat(nav2): add custom progress checker`    |
| `fix`   | bug / regression      | `fix(imu-bridge): normalize quaternion`      |
| `docs`  | docs only             | `docs(sim): document Mars field worlds`      |
| `ci`    | CI / pipelines        | `ci: add ROS 2 humble colcon action`         |
| `chore` | house-keeping / bumps | `chore: update dependencies in requirements` |
| `misc`  | repo-wide tweaks      | `misc: clean up .gitignore entries`          |

---
