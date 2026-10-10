# place_bread_basket VPC 第一阶段

本阶段先测仿真真值位姿下的规划控制上限，再测位姿估计误差的容忍度。此处的“oracle”从仿真读取面包和篮子位姿、模型编号及物体局部抓取点；它还不是视觉检测器。

`scene.py` 定义 `ObjectEstimate`（世界坐标系 xyz、wxyz 四元数、置信度、模型编号）、`RobotEstimate`（双臂末端位姿和夹爪状态）与 `SceneEstimate`。`OracleSceneProvider` 将仿真对象包装成 `EstimatedActor`。包装器只偏移提供给抓取/放置算法的位姿，不移动真实物体。物体运动后，当前实验仍以仿真真值跟踪其运动，并叠加同一个固定偏差，因此扰动结果是**理想跟踪条件下的位姿误差敏感性**，不是端到端视觉性能。此阶段的置信度固定为 1，尚未校准。

`place_bread_basket.py` 复用原任务专家的左右臂选择、双臂/单臂分支和抓取/放置参数。`scripts/vpc_oracle.py` 对相同种子重复运行不同误差级别，按 `plan_success`、`task_success` 和 `joints_legal` 三项共同判定成功，并将每回合结果写为 JSONL。位姿扰动是各物体 xyz 独立高斯噪声，标准差由 `--sigmas-mm` 指定；相同种子的不同误差级别共享同一组标准化随机误差。0 mm 是真值基线。

在仓库根目录运行（使用仿真依赖所在的 RoboTwin Conda 环境）：

```bash
conda activate RoboTwin
python -m scripts.vpc_oracle --seeds 0 2 3 --sigmas-mm 0 5 10 20 --output vpc_results/oracle_seed_0_2_3.jsonl
```

该程序不会采集演示数据或训练模型。后续阶段需要让相机模块输出同一场景字段，加入物体尺寸/遮挡不确定性、闭环重定位与失败恢复，并在更多随机种子上统计置信区间。当前小样本结果只能做接口及趋势检查。

已验证的 3 个双臂种子（0、2、3）在 0/5/10/20 mm 下分别成功 3/3、2/3、1/3、1/3；额外的同侧单臂种子 5 在 0/5 mm 下均成功。种子 2 的 5 mm 回合中运动规划成功，但最后一块面包和篮子都离开桌面；需要动作阶段日志或视频才能确定最早的接触失误。这些数字只是小样本实测，不能推断总体成功率。
