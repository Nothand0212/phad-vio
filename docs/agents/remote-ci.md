# 局域网 Remote CI 与 EuRoC 并行实验

本文档定义 `scripts/remote_ci.py` 的当前运行合同。首个 profile 是
`ci-euroc11`：一次 Release 共享构建、unit tests，以及 EuRoC 11 序列
record-only benchmark。序列最多 8 路并行，单个失败不会取消其他已就绪任务。

## 固定身份与安全合同

SSH 机器身份读本机共享配置，供各仓库共用：

- 路径：环境变量 `PHAD_REMOTE_CI_CONFIG`；否则
  `$XDG_CONFIG_HOME/phad-remote-ci/config.json`；再否则
  `~/.config/phad-remote-ci/config.json`。
- 字段仅 `host`、`user`、`fingerprint`。缺文件、JSON 不合法或字段不对时
  失败，并打印路径与示例。
- `--help` 与远端 worker 子命令不读该文件。

```json
{
  "host": "192.168.110.34",
  "user": "lin",
  "fingerprint": "SHA256:gVOFWNnhMg035iardU+Z8GxRKunxHIp3ZQrjhgY/9XE"
}
```

本仓库远端路径：

- 远端项目根：`/home/lin/Projects/tigerfish`。
- 数据集：`/home/lin/data/euroc/native`。

每次连接先用 `ssh-keyscan` 取得当前 `host` 的 ED25519 key，再用
`ssh-keygen -E sha256` 与配置中的 `fingerprint` 比较。只有精确匹配时，
脚本才会通过临时 `known_hosts` 继续。
SSH/SCP 固定使用 `StrictHostKeyChecking=yes`、`BatchMode=yes`、public-key-only；
密码和 keyboard-interactive 被关闭，agent 不转发，也不依赖用户 SSH config。

脚本不修改服务器全局配置。数据集、快照源码和完成的共享构建产物都以
readonly bind mount 提供给下游容器。容器使用远端用户 UID/GID，
`--network none`、readonly root filesystem、`--cap-drop ALL` 和
`no-new-privileges`。

## 源码快照

`start` 从调用时的工作区生成两个输入：

1. 当前 `HEAD` 的完整 Git bundle；
2. tracked 文件和非 ignored untracked 文件的工作树归档。

快照包含 tracked 修改、模式位、symlink、tracked 删除和非 ignored 新文件。
`.git/`、`.codex/`、Git ignored 的 build/venv/thirdparty，以及
`artifacts/remote-ci/` 不进入快照。canonical manifest 记录路径、类型、
mode、内容 SHA-256、Git HEAD/tree/branch 和 dirty 状态；manifest 的
canonical JSON SHA-256 即 `source_id`。

上传后会先核对 archive 和 bundle SHA-256。服务器通过
`git clone --no-checkout`、`git read-tree HEAD` 和工作树归档重建快照，
然后重新计算 `source_id`、HEAD/tree 和 dirty/untracked/deleted 状态。实验不使用
服务器上的旧 checkout。

## 工具链镜像

`docker/remote-ci/Dockerfile` 以 Ubuntu 24.04 为基础，包含 GCC、CMake、
Ninja、项目 apt 依赖和隔离 Python venv。GTSAM 4.3a1 固定到
`2f3e56c0ddbd3a1aa54ed043643b553d26a069f6`，源码 archive SHA-256 固定为
`50bd99ddbb363f03f145d814995df234c83ad38f867080fba5b60f6c151b348a`。
Python venv 通过 image 内的 Ubuntu `python3-numpy` 提供运行依赖，不访问
PyPI。镜像构建期会单独编译、链接并运行 dependency smoke executable，同时生成
`/opt/phad-toolchain.json` 记录依赖版本。

镜像身份同时记录 OCI manifest/config digest 和两端 Docker native ID；这是
classic 与 containerd image store 共用的精确身份合同。toolchain hash 覆盖整个
`docker/remote-ci/` build context（Python cache 不参与 identity 或 build）。
远端复用或 load 后必须匹配 archive 内的
manifest/config digest、RootFS layer digests、platform、toolchain label 和依赖
版本，并再次运行 smoke。否则本地 `docker image save` 后用 zstd 压缩、SCP
上传、核对 SHA-256，再由远端 `docker image load`。

## 命令

在仓库根目录执行：

```bash
python3 scripts/remote_ci.py doctor
python3 scripts/remote_ci.py start --profile ci-euroc11
python3 scripts/remote_ci.py status <run-id>
python3 scripts/remote_ci.py status <run-id> --json
python3 scripts/remote_ci.py wait <run-id>
python3 scripts/remote_ci.py fetch <run-id>
```

`doctor` 只做连接、Docker 和 11 序列完整性检查。`start` 同步完成镜像/快照
准备，创建唯一 `run-id`，用 `nohup` + `setsid --fork` 启动后台 worker；核对
worker PID、独立 session ID 和命令行后返回。`wait`
默认每 30 秒轮询一次，仅输出状态变化；终态后自动执行 `fetch`。
长时实验应使用该脚本轮询，不应以高频 SSH 或对话轮询替代。

## DAG 和目录

`build` 先以 Release、`PHAD_BUILD_TESTS=ON`、16 路完成共享构建。构建成功后，
unit 和 11 个序列任务独立运行。每个任务拥有独立的 `HOME`、
`ROS_HOME`、build、cache、tmp、logs 和 artifacts。序列命令固定为：

```bash
phad_vo_bench <sequence-root> \
  --out <task-artifacts>/bench \
  --sequence-name <sequence> \
  --repo /src \
  --estimator-enable-moving-bootstrap
```

有 bench 输出时继续运行 `segment_ate_decomp.py`。单任务的 container ID、
起止时间、返回码、命令、失败原因和绝对日志路径会以原子 rename 更新到
`status.json`。

```text
/home/lin/Projects/tigerfish/
├── images/<image-config-id>/image.tar.zst
├── sources/<source-id>/{inputs,repo,ready.json}
└── runs/<run-id>/
    ├── run.json
    ├── run-status.json
    ├── orchestration.log
    └── tasks/<task>/{home,ros_home,build,cache,tmp,logs,artifacts}
```

## 结果与 record-only 语义

`fetch` 核对远端 results archive SHA-256，拉回状态、日志和 benchmark 产物，
然后生成：

- `artifacts/remote-ci/runs/<run-id>/summary.md`；
- `artifacts/remote-ci/latest/ci-euroc11.md`；
- `artifacts/remote-ci/current/ci-euroc11.md`，仅在拉取时重算的当前
  `source_id` 与 run 完全匹配时更新。

汇总包含 source/image 身份、任务状态与返回码、ATE/RPE、completion、
coverage、segments/reanchors、段内加权 RMS、失败原因和远端日志绝对路径。
benchmark 的 `completed_with_failures` / `completed_with_warnings` 在此 profile
记为 task `warning`；进程或分解脚本非零、`eval_failed` / `failed` 才记为
task `failed`。两者都会保留完整指标和日志。
`ci-euroc11` 结果是 remote CI/record-only 证据，不替代 Slice 1a 的顺序
产品门，也不自动形成正式 benchmark checkpoint。

## 恢复与诊断

- SSH 中断后先执行 `status <run-id>`；worker 已脱离 SSH，连接中断不会取消 run。
- `wait` 中断后可以用同一 `run-id` 重新执行；终态 run 也可重新 `fetch`。
- 任务失败时先看 summary 的绝对日志路径，再检查对应 `stderr.log`、
  `stdout.log`、`status.json` 和容器 ID。兄弟任务会继续。
- 镜像或快照上传中断时，重跑 `start` 会复用已通过 SHA-256 验证的输入。
  若 `sources/<source-id>` 已存在但没有 `ready.json`，脚本会停止；先检查该目录和
  上传 checksum，再将不完整目录改名保留后重试。
- `orchestration-error.log` 记录 worker 级异常；该情况也会生成可 `fetch` 的
  failed 终态。

历史 run、快照、镜像归档和任务容器默认保留。清理属于独立的服务器维护操作，
需要明确审核目标后再执行。
