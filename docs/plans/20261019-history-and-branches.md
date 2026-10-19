# 按新规则重写 main 的计划提交，并清理分支

- Status: done

## Goal

新规则：计划随实现它的代码一起提交，状态变更不单独提交。`main` 上有 15 个只改计划与 `TODO.md` 的
`docs:` 提交（起草、批准、修订、关闭）。维护者选择重写历史（方案 B），把能干净并入实现提交的并进去，
并删除已无用的本地分支。

## Done when

- `main` 重写后与重写前的树完全相同：`git diff 2b2fe90 main` 只含下文"提交引用"的替换
- 每个新提交除 `docs/plans/`、`TODO.md` 外的树与对应原提交相同（脚本逐个比对，见 Approach 第 4 步）
- 最终 `./scripts/check.sh` 通过
- 强推后 `git ls-remote github main` 与本地 `main` 一致
- 五个本地分支已删除

## Context

- `main` 与 `github/main` 相同（`2b2fe90`，85 个提交）：**已推送**，重写须强推。
- 只改 `docs/plans/` 与 `TODO.md` 的提交（`git show --name-only`）共 16 个；其中 `67a130c` 是首次加入 `TODO.md`，
  不属状态提交，余下 15 个：
  `55356c9` `ec8c32a` `14ba474` `6c48da3` `924c5a4` `8ce7478` `989ec7c` `d3afff5`
  `5bf9bc3` `e16921d` `d6dea2d` `42f487b` `012dfc4` `174cd67` `a3709ea`
- 已提交的计划里引用了 15 个会被改写的提交哈希（`git grep`）：`pre-merge-cleanup`（2 处）、
  `kiss-driver-tests`（7）、`plugin-logic-split`（6）、`servo-lag-and-cancellation`（2）。
- `pre-merge-cleanup` 的对照表引用的是上次整理前的哈希，这些提交只存在于 `agent/dev`；删除该分支后本地也解析不到。
- 演练（临时 worktree，`git rebase -i 4fe21a1`，按下表的待办列表）：
  - 无冲突；85 → 76 个提交；`git diff main HEAD` 为空；
  - 逐个比对，每个新提交除计划与 TODO 外的树与原提交相同；
  - 在 rebase 过程中逐个替换哈希会和后续提交的上下文冲突（`93a3f66` 处），所以替换放到最后单独一个提交。
- 基线：`./scripts/check.sh` 通过（218 个测试，0 失败，4 跳过）。

## Approach

### 并入（9 个，均与目标提交相邻或只隔不改同一文件的提交）

| 状态提交 | 并入 | 说明 |
|---|---|---|
| `6c48da3` 夹爪计划改为方案 A | `14ba474` | 同一计划的起草与修订，二者相邻 |
| `924c5a4` 批准夹爪与测试精简 | `509aa5b` | 计划随第一个实现提交 |
| `d3afff5`、`5bf9bc3` 伺服滞后计划起草、批准 | `d0e0b26` | 同上；该实现后被 `0899a53` 回退，计划以 abandoned 留在 `2c59c91` |
| `e16921d`、`d6dea2d`、`42f487b` MoveIt 计划批准、更新、关闭 | `eb71b38` | MoveIt 只有这一个实现提交 |
| `174cd67` 关闭合入前计划 | `071fa63` | 相邻的最后一个实现提交 |
| `a3709ea` 插件逻辑拆分计划 | `0f40d4b` | 第一个实现提交 |

并入时用目标提交的提交信息与作者日期（`pick <目标>` 后跟 `fixup <状态提交>`）。

### 保留（6 个）

| 提交 | 原因 |
|---|---|
| `55356c9` MoveIt 计划起草 | 与实现隔 30 多个提交，中间 `8ce7478`、`989ec7c` 也改它；挪动要拆提交、解冲突 |
| `ec8c32a` 合入前计划 + KISS 重构计划 | KISS 重构计划是 abandoned，按规则单独提交 |
| `14ba474` 真机复测记录与夹爪计划 | 含真机复测结论 |
| `8ce7478` 关闭夹爪计划、修订 MoveIt 计划 | 同时改两份计划，拆开收益小 |
| `989ec7c` 协调合入前与 MoveIt 计划 | 同上 |
| `012dfc4` 关闭精简计划 | 记录的是事后的真机夹爪验证，挪回实现处会把验证时间提前 |

### 步骤

1. 先按 [20261019-agents-md-project-rules.md](20261019-agents-md-project-rules.md) 决定 1 提交 `docs/architecture.md`，工作区干净
2. `git branch backup/before-history-rewrite main`
3. `git rebase -i 4fe21a1`，待办列表按上表生成（`14ba474` 及之前的 35 个提交哈希不变）
4. 校验：rebase 后的 `main` 与 `2b2fe90` 的 `git diff` 为空；按提交主题逐个比对新旧提交的
   `git diff <旧> <新> -- . ':!docs/plans' ':!TODO.md'` 为空
5. 一个提交更新计划中的哈希引用（旧 → 新，被并入的指向并入后的提交），本计划随此提交、Result 列出旧 → 新对照表
6. `./scripts/check.sh`；`git push --force-with-lease=main:2b2fe90 github main`；`git ls-remote` 核对
7. 删除分支：`agent/dev`、`agent/dev-clean`、`backup-before-reword`、`backup/before-regroup`、
   `backup/direct-flange-20260901`；备份分支 `backup/before-history-rewrite` 保留到维护者确认后再删

### 未采纳

- **全部 15 个都并入**：其余 6 个要跨越改同一文件的提交挪动或拆分，冲突需人工解决，且会把真机验证、
  废弃计划的记录混进实现提交，违背新规则本身。
- **重写全部历史**（含最早 5 个非 Conventional Commits 的提交）：改动全部 85 个哈希，收益只是格式。
- **rebase 中逐提交替换哈希引用**：演练中与后续提交冲突。

## Risks

- 不可撤回：强推后，别人已有的 clone/fork 与外部链接里的 50 个哈希（`14ba474` 起）失效；
  删除 `agent/dev` 后，`pre-merge-cleanup` 对照表里的旧哈希本地也无法解析（远程从未有过）。
- check 查不到：提交信息与作者日期是否合适、哈希引用是否替换完整（用 `git grep` 旧哈希为空核对）。
- Undo：强推前删掉新 `main` 即可；强推后 `git push --force github backup/before-history-rewrite:main`。

## Result

- `main` 从 85 个提交（`2b2fe90`）重写为 76 个，另加 `docs: add the architecture diagrams` 共 77 个；按 Approach 并入 9 个、保留 6 个，
  `git rebase -i 4fe21a1` 无冲突。`14ba474` 之前的 35 个提交哈希不变。
- 计划中的 17 处哈希引用改为新哈希（4 份计划），被并入的状态提交指向并入后的提交；`git grep` 旧哈希为空，新哈希都能解析。
- 备份分支 `backup/before-history-rewrite` 指向重写前的 `main`（`2b2fe90` 加 architecture 提交）。
- 五个旧分支的删除由维护者执行（`git branch -D`，未合并分支须强删）。

- Commits: `docs: update commit references after the history rewrite`（本计划随此提交）
- Checked by: 重写后 `git diff backup/before-history-rewrite main` 为空；77 个提交按主题逐个比对，
  除 `docs/plans/`、`TODO.md` 外的树与原提交相同；`./scripts/check.sh` 通过。
  Not checked: 中间提交未逐个跑 check（代码树与原提交相同）。
- Undo: `git push --force-with-lease github 2b2fe90:main`

旧 → 新对照（`14ba474` 起；被并入的状态提交指向并入后的提交）：

```text
2b2fe90 62dc09e refactor(test): prune redundant hardware interface tests
db1f1f1 8032cdf refactor(description): declare gripper command interfaces on mock only
8060f50 a7dcc7c refactor(driver): decide startup and activation rules in preconditions
3fd5d83 91468a6 refactor(driver): decide the gripper goals in GripperGoals
0f40d4b 12415e3 refactor(driver): decide the arm's segments in ArmGate
071fa63 c111883 test: drop timeout tests and stop waiting longer than needed
ad7479c eca1e77 docs: state measurements plainly and drop site details
0aa7874 0830ca6 docs(moveit): state the known limits and what the acceptance left out
542233d 526c4fe test(driver): drop tests that mirror firmware quirks
4dfadc1 fb90641 docs: record review findings before the merge
cdf8abf 3d29351 feat: default to fairino5_v6
0899a53 042cac2 revert(driver): stop on a trajectory cancel instead of ramping back to the feedback
012dfc4 589b05a docs: close KISS driver test plan upon real-robot gripper verification
8fd66b4 c7dd9d6 docs: record S21 for 250 Hz endurance servoing under 100% CPU load
eb71b38 c2ac435 feat(moveit): add fr_moveit_config package with full model support and tests
2c59c91 2662162 docs: record trajectory cancel jump resolution and close plan
d0e0b26 d53bb1c fix(driver): smoothly decelerate to hold position on trajectory cancel
989ec7c 6a3fb24 docs: reconcile pre-merge cleanup and moveit planning notes
93a3f66 8bbaae2 docs: describe the simplified gripper driver and verification
b26d448 6bc973e refactor(driver): remove ineffective gripper speed controls
1ac7674 1e2808c refactor(tests): stop simulating gripper firmware quirks
319b405 155b65d test(driver): replay gripper frames without wall-clock waits
509aa5b 952bdb5 refactor(driver): isolate gripper motion inference
3fe1abe ad37539 docs: gripper speed has no effect, flags change until max_time
c01922a 1f2ebd8 docs: correct the lag, misuse and status-bit notes
8ce7478 5409cac docs: close the gripper retest plan, revise the MoveIt plan
731cdc5 78b45ca fix(driver): end a gripper motion only on flags that stay
c83e93f 6265a31 refactor(driver): drop gripper_object from ~/status
74a22fd f0f425c docs: record that motion done is two latched flags
92468d8 856eca2 docs: record the gripper fixes and flaky wall-clock tests
342e4a1 06c000b test(driver): run the trajectory pause under a controller manager
8798298 e46f122 fix(driver): do not report the first frame after configure as stale
8503128 d0d880c fix(driver): size the path tolerance for the measured following lag
55f1029 359c6ae fix(driver): keep the opening during a motion to where the gripper is
e49ed41 f1a0745 feat(driver): serve the gripper action from the plugin on real hardware
dec5d06 8c00515 feat(driver): let a trajectory resume right after the gripper
dcaa328 d172b9a feat(driver): pause trajectories while the gripper is busy
fa6bd87 93b68a3 docs: settle the gripper action name and the scaling gpio, drop G16
8f547e4 0950705 feat(client): report the following lag in fr_servo_bench
1749a2c 6f15c35 docs: record the gripper and servo-lag experiments
14ba474 c2eac1c docs: record the real-robot retest and plan the gripper fixes
a3709ea 12415e3 refactor(driver): decide the arm's segments in ArmGate
174cd67 c111883 test: drop timeout tests and stop waiting longer than needed
42f487b c2ac435 feat(moveit): add fr_moveit_config package with full model support and tests
d6dea2d c2ac435 feat(moveit): add fr_moveit_config package with full model support and tests
e16921d c2ac435 feat(moveit): add fr_moveit_config package with full model support and tests
5bf9bc3 d53bb1c fix(driver): smoothly decelerate to hold position on trajectory cancel
d3afff5 d53bb1c fix(driver): smoothly decelerate to hold position on trajectory cancel
924c5a4 952bdb5 refactor(driver): isolate gripper motion inference
6c48da3 c2eac1c docs: record the real-robot retest and plan the gripper fixes
```
