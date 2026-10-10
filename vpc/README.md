# place_bread_basket VPC 仿真诊断

本阶段先测仿真真值位姿下的规划控制上限，再测位姿估计误差的容忍度。此处的“oracle”从仿真读取面包和篮子位姿、模型编号及物体局部抓取点；它还不是视觉检测器。

`scene.py` 定义 `ObjectEstimate`（世界坐标系 xyz、wxyz 四元数、置信度、模型编号）、`RobotEstimate`（双臂末端位姿和夹爪状态）与 `SceneEstimate`。`OracleSceneProvider` 将仿真对象包装成 `EstimatedActor`。包装器只偏移提供给抓取/放置算法的位姿，不移动真实物体。物体运动后，当前实验仍以仿真真值跟踪其运动，并叠加同一个固定偏差，因此扰动结果是**理想跟踪条件下的位姿误差敏感性**，不是端到端视觉性能。此阶段的置信度固定为 1，尚未校准。

`place_bread_basket.py` 复用原任务专家的左右臂选择、双臂/单臂分支和抓取/放置参数。`scripts/vpc_oracle.py` 对相同种子重复运行不同误差级别，按 `plan_success`、`task_success` 和 `joints_legal` 三项共同判定成功，并将每回合结果写为 JSONL。位姿扰动是各物体 xyz 独立高斯噪声，标准差由 `--sigmas-mm` 指定；相同种子的不同误差级别共享同一组标准化随机误差。0 mm 是真值基线。

在仓库根目录运行（使用仿真依赖所在的 RoboTwin Conda 环境）：

```bash
conda activate RoboTwin
python -m scripts.vpc_oracle --seeds 0 2 3 --sigmas-mm 0 5 10 20 --output vpc_results/oracle_seed_0_2_3.jsonl
```

该程序不会采集演示数据或训练模型。后续阶段需要让相机模块输出同一场景字段，加入物体尺寸/遮挡不确定性、闭环重定位与失败恢复，并在更多随机种子上统计置信区间。当前小样本结果只能做接口及趋势检查。

已验证的 3 个双臂种子（0、2、3）在 0/5/10/20 mm 下分别成功 3/3、2/3、1/3、1/3；额外的同侧单臂种子 5 在 0/5 mm 下均成功。这些数字只是小样本实测，不能推断总体成功率。

## 分对象扰动和阶段日志

`--noise-targets` 支持 `all`、`bread`、`basket`、`none`。相同任务种子和标准差下，四组先采样相同的面包与篮子误差向量，再将未选择的对象误差置零，因此对照保持配对。`none` 是同场景真值参照。`stages` 在每组高层动作**结束后**记录真实物体位置、篮子水平漂移、面包到篮子中心的水平距离、机械臂末端位姿、夹爪状态、规划标志和阶段序号；这不是逐仿真帧的接触日志。

```bash
conda activate RoboTwin
python -m scripts.vpc_oracle --seeds 2 --sigmas-mm 5 --noise-targets all bread basket none --output vpc_results/seed2_5mm_ablation.jsonl
python -m scripts.vpc_oracle --seeds 0 --sigmas-mm 10 --noise-targets all bread basket none --output vpc_results/seed0_10mm_ablation.jsonl
```

早期单回合检查中，种子 2、5 mm 的 `all` 条件失败，`left_place` 阶段结束时篮子已漂移约 537 mm；当时的 `bread` 和 `basket` 两个单独扰动条件都成功。种子 0、10 mm 的 `all` 和 `bread` 条件失败；双臂抬起后，面包 0 的高度仍约 0.741 m，`basket` 和 `none` 条件则将它抬至约 0.789 m。这些是具体回合的观察，重复实验发现同一条件也可能有不同结果，不能当作稳定成功率或确定因果结论。

## 阶段选择性扰动与重复性

`--noise-phases` 支持 `all`、`grasp`、`place`、`none`。`grasp` 只在选臂和抓取规划时应用位姿偏差；进入放置规划前恢复真值。`place` 在抓取时使用真值，进入放置规划前应用偏差。同侧双面包分支会在第二块面包抓取前重新切回抓取阶段。这种切换模拟理想的阶段边界重定位，用来诊断误差作用时机，不表示真实视觉系统具备瞬间恢复真值的能力。

`--repeats N` 在相同任务种子、误差向量和配置下重复运行 N 次，结果中记录 `repeat_index`。重复间轮换阶段条件的执行顺序，以减轻固定条件顺序造成的偏差。运行器不改变原专家路径或仿真器的随机性设置；即使输入相同，物理接触和运动规划结果仍可能不同。因此单次成功/失败不能当作稳定的因果结论，应同时报告重复次数与结果。`stages[].stage_index` 是高层动作阶段序号，并非物理仿真步数。

```bash
conda activate RoboTwin
python -m scripts.vpc_oracle --seeds 2 --sigmas-mm 5 --noise-targets all --noise-phases all grasp place none --repeats 3 --output vpc_results/seed2_5mm_phase_repeats.jsonl
```

种子 2、5 mm 的一次交错顺序实验中，`all`、`grasp`、`place`、`none` 分别成功 0/3、3/3、1/3、3/3。`all` 三个回合在 `left_place` 结束时篮子水平漂移分别约 201、500、88 mm；`place` 两个失败回合在同一阶段漂移约 69、97 mm。这支持放置阶段位姿误差与篮子失稳有关，但三次重复仍不足以估计稳定的失败概率。

种子 0、10 mm 且只扰动面包时，`all`、`grasp`、`place`、`none` 分别成功 0/3、0/3、3/3、3/3。`all` 和 `grasp` 的三个回合在 `dual_lift` 后，面包 0 均留在约 0.741 m；`place` 和 `none` 均抬至约 0.789 m。这较直接地定位了该场景的抓取失效。

重复性审计还发现：在同一种子 2、5 mm、`all` 条件下，另一次连续三回合运行得到 2/3 成功，和上述交错实验的 0/3 不同。初始物体位姿及注入误差相同，但执行结果并非严格确定；目前未查明波动来自运动规划、物理接触还是其他运行时因素。后续统计必须增加重复次数，避免用单回合结果定义误差阈值。

## 动作内接触与路径追踪

可选的 `--trace-stages` 只在指定动作阶段逐仿真步读取状态；默认关闭。`--trace-stride` 指定位置轨迹采样间隔，接触类别、面包首次抬高 20 mm 和篮子相对阶段起点首次偏移 5/10/50/100 mm 的事件仍逐步检查。`step_trace` 保存每阶段的目标动作、起始关节状态、规划关节路径长度及 SHA-256 摘要、首次接触事件和稀疏物体/末端轨迹。接触来自 SAPIEN 的接触流形，记录“首次检测到该类别”，并非接触力或完整的接触持续时间；两个同名面包按每场景实体 ID 区分。

```bash
conda activate RoboTwin
python -m scripts.vpc_oracle --seeds 0 --sigmas-mm 10 --noise-targets bread --noise-phases grasp none --trace-stages dual_grasp dual_lift --trace-stride 25 --output vpc_results/seed0_contact_trace.jsonl
python -m scripts.vpc_oracle --seeds 2 --sigmas-mm 5 --noise-targets all --noise-phases all place --repeats 3 --trace-stages left_place --trace-stride 10 --output vpc_results/seed2_place_contact_trace.jsonl
```

种子 0 的逐步对照中，误差抓取的 `dual_lift` 只检测到一块面包抬高 20 mm，真值抓取则检测到两块；阶段结束时前者的一块面包仍在约 0.741 m，后者两块都在约 0.789 m。种子 2 的六个放置追踪回合中，失败回合在 `left_place` 内记录到面包—篮子、篮子—夹爪/机械臂接触，随后篮子位移越过 50 mm；成功回合也发生这些接触，但篮子位移未越过 10 mm。因此不能仅凭“是否接触”判定失败，仍需分析轨迹、接触几何或力。

## 固定关节轨迹重放

`scripts/vpc_replay.py` 先正常规划一次，复制左右臂的关节路径，然后在相同任务种子下以 `need_plan=False` 重放这些路径。脚本检查左右臂的路径摘要和消费数量，并记录每个高层阶段结束时的真实物体位置。它用于区分“规划器给出了不同路径”和“相同路径执行后产生不同物理结果”。重放时仍使用原有夹爪动作序列，但不重新计算抓取/放置目标位姿。

```bash
conda activate RoboTwin
python -m scripts.vpc_replay --seed 2 --sigma-mm 5 --noise-target all --noise-phase all --replays 3 --output vpc_results/seed2_fixed_plan_replay.jsonl
python -m scripts.vpc_replay --seed 0 --sigma-mm 0 --noise-target all --noise-phase all --replays 2 --output vpc_results/seed0_success_plan_replay.jsonl
```

种子 2 的三次独立规划审计中，初始关节状态与 `dual_grasp` 目标动作完全相同，但该阶段首段关节路径长度与 SHA-256 摘要不同；三次回合一成两败。因此，路径差异已在抓取规划阶段出现，早于放置接触。固定其中一条失败路径后，基准回合与三次重放均失败，所有阶段的面包和篮子位置逐项完全相同。种子 0 的零误差成功路径也在两次重放中成功，阶段位置逐项相同。这支持“规划输出变化是当前重复性问题的重要上游来源”；尚未证明规划器内部哪一环节产生差异，也不能以有限重放证明物理仿真在所有情况下严格确定。

## cuRobo 采样器重置实验

`--reset-planner-before-plan` 是默认关闭的诊断开关。开启时，左右臂每次调用 cuRobo 单目标或批量规划前，分别调用对应 `MotionGen.reset(reset_seed=True)`。双臂规划器位于独立子进程，开关通过已有管道发送。此操作重置 cuRobo 的内部采样器，不保证 GPU 内核、优化器或仿真全链条确定性；也可能改变路径选择，不能视为性能提升。JSONL 中的 `reset_planner_before_plan` 记录是否开启。

```bash
conda activate RoboTwin
python -m scripts.vpc_oracle --seeds 2 --sigmas-mm 5 --noise-targets all --noise-phases all --repeats 3 --trace-stages dual_grasp left_place --reset-planner-before-plan --output vpc_results/seed2_reset_planner_audit.jsonl
```

该条件下三次初始物体位姿、`dual_grasp` 初始关节状态和目标动作逐项一致；三次均规划成功但任务失败。右臂首段路径摘要三次相同，左臂第一次与后两次不同，后两次相同。左臂首次路径步数都为 431，但轨迹数据不同；第一次在 `left_place` 后篮子水平漂移约 86 mm，后两次约 425 mm。说明仅重置采样器仍未消除全部路径/执行波动。样本量只有三次，不能由 0/3 推断真实成功率；后续需进一步隔离 GPU 求解差异与规划调用顺序，并在固定路径条件下评估控制。
