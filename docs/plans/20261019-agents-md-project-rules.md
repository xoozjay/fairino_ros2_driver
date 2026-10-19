# 精简 AGENTS.md，把本项目特有的工作流写成 Project rules

- Status: done

## Goal

`AGENTS.md` 有 136 行，混着项目事实、参考资料（版本锁定表、恢复命令、各包验证步骤）和通用工作流（计划审批流程）。
每次会话都全量加载它，参考资料在这里既占上下文，又和 `docs/`、各包 README 重复。

工作流 skill 已更新：通用部分留在 skill，`AGENTS.md` 只放命令、红线、坑和本项目特有的工作流；
交接与审查放 `.agents/notes/`，不入库。本计划让本仓库跟上。

## Done when

- `AGENTS.md` 只剩：项目一段、命令、Project rules、文档地图；不复述 skill，不复述 `docs/` 与各包 README
- 下文"去向"表里每一段都有新位置或明确删除，`rg` 查不到指向已删段落的链接
- `.agents/notes/` 列入 `.git/info/exclude`
- `./scripts/check.sh` 通过

## Context

- `AGENTS.md` 结构：项目 `:7`、命令 `:18`、核心规则 `:31`（vendor `:33`、fr_description `:44`、驱动 `:53`、版本 `:72`、真机与故障 `:105`）、references `:118`、相关文档 `:124`。
- 工作区有未提交改动：`AGENTS.md` 文档地图新增一行 `docs/architecture.md`，`docs/architecture.md` 未跟踪（10-16 的会话留下）。
- `docs/sdk_quirks.md` 已有来源标记（🔍 源码确认、📋 维护者实测/经验、❓ 待真机验证），就是 skill 里"事实注明来源"的做法，不用另起格式。
- `docs/sdk_quirks.md` §一 已是版本相关陷阱，版本锁定表放过去不需要新文件。
- 真机实验的脚本与数据在工作区根目录 `exp/`，不在本仓库；脚本里含只适用于当时现场的点位，维护者已要求这类数值不入库。
- `192.168.58.2` 是官方 SimMachine 与 SDK 的默认地址（sdk_quirks L6），属公开事实；真机地址从未入库。
- `docs/plans/README.md`（7 行）复述了 skill 的计划流程。

## Approach

### 去向

| 现在的段落 | 去向 |
|---|---|
| 项目（`:7`），含"渐进式开发协议" | 留一段；协议删除（真机已验证，已成历史） |
| 常用命令（`:18`） | 留；Environment 改为 `ip:=$FR_ROBOT_IP`，未设置就问，不写真机地址 |
| 规则 1 vendor（`:33`） | 留一条红线：`references/fairino_vendor/` 只读、`src/` 不得引用；子目录清单移到文档地图 |
| 规则 2 fr_description（`:44`） | 架构与新增型号步骤并入 `fr_description/README.md`；留一条：改参数改 `config/*.yaml`，不碰 vendor |
| 规则 3 驱动（`:53`） | 验证步骤与 Jazzy 约定并入 `fr_robot_driver/README.md`；留两条：初始位形是场景属性；`on_deactivate`/`on_error` 在实时线程内，不做网络操作 |
| 规则 4 版本（`:72`） | 锁定表与恢复命令移到 `docs/sdk_quirks.md` §一；留一条：一律以 3.8.4 为准，不借用更新版本的 API；版本以 `GetSoftwareVersion` 的 `webVersion` 为准 |
| 规则 5 真机（`:105`） | 留，压短：先读 sdk_quirks 与 fault_handling；`read()`/`write()` 内不调 SDK；按 SDK 3.8.4 原样实现；只引用公开资料；真机操作规程 |
| references（`:118`） | 并入文档地图 |
| 相关文档（`:124`） | 改成表；删去"先 Status: draft，批准后才动代码"（skill 已有） |
| `docs/plans/README.md` | 压成一句：本目录放什么 |

### 新增 Project rules（本项目特有，skill 不管）

- 外部系统（控制器、SDK、夹爪）的行为写进 `docs/sdk_quirks.md`，每条带来源标记
- 实验脚本与数据不入本仓库（含现场点位）；结论写进 `docs/` 时注明来源与日期范围
- 控制器地址取 `$FR_ROBOT_IP`，未设置就问；不把真机地址写进任何文件
- 实时状态（起始位形、夹爪开度、是否激活）每次用 `fr_probe` 读，不记录

### 顺序

1. 未提交的 `docs/architecture.md` 与 `AGENTS.md` 文档地图一行先单独提交（`docs: add the architecture diagrams`），
   在 [20261019-history-and-branches.md](20261019-history-and-branches.md) 的重写之前；本计划在重写之后做
2. 移动参考内容到目标文档，各自一个提交
3. 重写 `AGENTS.md` 与 `docs/plans/README.md`，与本计划同一提交
4. `.git/info/exclude` 加 `.agents/notes/`（本地，不入库）

## Decisions

维护者 10-19 决定：

1. `docs/architecture.md` 与 `AGENTS.md` 的未提交改动先单独提交，本计划只做搬移
2. 删除"渐进式开发协议"
3. `exp/` 留在工作区根目录，不入库

## Result

`AGENTS.md` 由 136 行减到 62 行：项目一段、常用命令、Project rules、文档地图（表）。去向：

- 版本锁定表、`references/` 恢复命令、`libfairino/docs/` 说明 → `docs/sdk_quirks.md` §一"锁定版本"
- 新增型号步骤、参数源头 → `fr_description/README.md`
- mock 与真机的验证步骤 → `fr_robot_driver/README.md`"验证"；Jazzy 约定与初始位形的说明原已在该 README
- `on_deactivate`/`on_error` 不做网络操作：留作 Project rule，细节原已在 `docs/architecture.md`
- 删除：渐进式开发协议、计划流程的复述（`docs/plans/README.md` 压成一句）、`references/fairino_vendor/` 子目录清单
- `TODO.md` 开头不再把 `AGENTS.md` 当作设计与现状的出处
- `.agents/notes/` 已加入 `.git/info/exclude`（本地）

- Commits: `docs: move the version lock table into sdk_quirks`、
  `docs(description): state how to add a model and where parameters live`、
  `docs(driver): list the bring-up checks for mock and real hardware`、
  `docs: reduce AGENTS.md to commands, project rules and a document map`
- Checked by: `./scripts/check.sh` 通过（218 tests，0 failures）；`AGENTS.md` 与 `docs/plans/README.md` 中的相对链接都存在；
  `git grep` 无指向已删段落（`AGENTS.md#…`、"AGENTS.md 版本约束"）的引用。
  Not checked: 搬移前后逐句对照只做了人工比对。
- Undo: `git revert`
