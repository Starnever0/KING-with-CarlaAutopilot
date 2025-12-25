# KING 混合仿真模式

> **English Version**: [README_EN.md](README_EN.md)  
> **原始KING项目**: [README_ORIGINAL.md](README_ORIGINAL.md)

<div align="center">
  <img src="assets/混合仿真插图.png" alt="混合仿真" width="800"/>
  <p><i>实时混合仿真：CARLA 自动驾驶（主车）+ 优化对抗车辆</i></p>
</div>

---

📚 **快速开始**: 查看 [HYBRID_QUICKSTART.md](HYBRID_QUICKSTART.md) 了解一分钟快速设置

---

## 概述

本仓库扩展了原始的 [KING 项目](https://github.com/autonomousvision/king)，增加了**混合仿真模式**，结合了 CARLA 真实物理仿真和 ProxySimulator 可微分优化的优势。

### 核心特性

- **主车（Ego Vehicle）**: 在 CARLA 真实物理环境中由 autopilot 控制
- **对抗车（Adversarial Vehicles）**: 在 ProxySimulator 中通过基于梯度的方法优化轨迹
- **实时同步**: CARLA 与 ProxySimulator 之间的双向状态同步
- **易于集成**: 可替换主车仿真方式，同时保持对抗优化完整

### 为什么需要混合模式？

此实现服务于多个目的：

1. **研究基线**: 为使用 KING 作为基线且需要更换主车仿真的研究者提供方案
2. **联合仿真示例**: 为实现 CARLA 与可微分仿真器联合仿真提供实用参考
3. **灵活测试**: 使用优化的对抗场景测试不同的主车策略（autopilot、自定义控制器等）
4. **教育价值**: 学习如何将真实物理引擎与基于梯度的优化相结合

---

## 安装

安装过程与原始 KING 项目**完全相同**，请参考 [原始 KING README](README_ORIGINAL.md) 中的安装说明。

### 可选依赖

为了调试方便，混合模式实现使用了一些额外的库（如 `icecream`）。你有两个选项：

**选项1：安装调试库**

```bash
# 使用 pip
pip install icecream

# 或使用 conda
conda install -c conda-forge icecream
```

**选项2：移除调试代码**

如果不想安装额外依赖，可以移除或注释掉 `proxy_simulator/hybrid_simulator.py` 中的调试代码：

```python
# 注释掉这些行：
# from icecream import ic
# ic.configureOutput(...)
# ic.disable()

# 并移除所有 ic(...) 调用
```

这不会影响程序功能。

---

## 使用方法

### 运行混合模式

#### 步骤 1：启动 CARLA 服务器

在单独的终端中：

```bash
carla_server/CarlaUE4.sh --world-port=2000 -opengl
```

#### 步骤 2：运行混合生成

```bash
bash run_generation_hybrid.sh
```

该脚本将：
1. 生成1辆对抗车的场景
2. 自动解析并分析结果
3. 可视化生成的场景

**输出**：结果将保存到 `experiments/results_hybrid/`

#### 步骤 3（可选）：在CARLA中实时查看仿真

在另一个终端中实时观察仿真过程：

```bash
# 设置环境变量
export CARLA_ROOT=carla_server  # CARLA 根目录路径
export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:$(pwd -P)/leaderboard
export PYTHONPATH=$PYTHONPATH:$(pwd -P)/scenario_runner

# 运行跟随主车的观察相机
python -c "
import carla
import time

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
spectator = world.get_spectator()

print('正在跟随主车... 按 Ctrl+C 停止。')

try:
    while True:
        # 查找主车 (Tesla Model 3)
        vehicles = world.get_actors().filter('vehicle.tesla*')
        if len(vehicles) > 0:
            ego = vehicles[0]
            transform = ego.get_transform()
            
            # 将相机置于主车后上方
            spectator.set_transform(carla.Transform(
                transform.location + carla.Location(z=50),
                carla.Rotation(pitch=-90)
            ))
        
        time.sleep(0.1)
except KeyboardInterrupt:
    print('\n已停止。')
"
```

这将显示仿真的鸟瞰图。你也可以调整相机位置以获得不同角度。

---

## 配置

### 关键参数

可在 `run_generation_hybrid.sh` 中修改这些参数：

| 参数 | 说明 | 默认值 |
|-----|------|--------|
| `--use_hybrid_mode` | 启用混合模式 | 必需 |
| `--num_agents` | 对抗车辆数量 | 1, 2, 4 |
| `--opt_iters` | 优化迭代次数 | 100-120 |
| `--sim_horizon` | 仿真时间步数 | 80 |
| `--initial_speed` | 初始车速 (m/s) | 4.0 |
| `--tm_speed_percentage` | 交通管理器速度调整 | 40 |
| `--port` | CARLA 服务器端口 | 2000 |

---

## 主要区别

### 模式对比

| 特性 | 标准模式 | 混合模式 |
|-----|---------|---------|
| **主车控制** | 神经网络 (AIM-BEV/TransFuser) | CARLA 自动驾驶 |
| **主车仿真** | ProxySimulator（运动学） | CARLA（物理引擎） |
| **对抗车仿真** | ProxySimulator（可微分） | ProxySimulator（可微分） |
| **批处理** | 支持多批次 | 仅支持单批次 (batch_size=1) |
| **可微分性** | 完全可微分 | 仅对抗车可微分 |
| **真实性** | 运动学近似 | 主车真实物理 |
| **速度** | 快 | 较慢（CARLA tick） |

---

## 实现说明

### 核心文件

- `generate_scenarios_hybrid.py`: 带 `--use_hybrid_mode` 标志的主生成脚本
- `proxy_simulator/hybrid_simulator.py`: 混合仿真器实现
- `run_generation_hybrid.sh`: 便捷执行脚本

### 核心特性

- **状态同步**: CARLA 与 ProxySimulator 之间的实时双向同步
- **梯度流**: 保持对抗车优化的可微分性
- **模块化设计**: 易于更换主车控制器或集成其他仿真器

---

## 自定义

混合模式支持多种自定义：

### 主车控制器

替换 CARLA autopilot 为自定义控制器。编辑 `HybridSimulator.spawn_ego_vehicle()`：

```python
# 注释掉自动驾驶
# self.ego_vehicle_carla.set_autopilot(True)

# 在 step() 中添加自定义控制器
control = self.custom_controller.run_step(observations)
self.ego_vehicle_carla.apply_control(control)
```

### 交通管理器参数

在 `HybridSimulator.spawn_ego_vehicle()` 中调整驾驶行为：

```python
# 更激进的驾驶
self.traffic_manager.vehicle_percentage_speed_difference(ego_vehicle, -50)

# 忽略红绿灯
self.traffic_manager.ignore_lights_percentage(ego_vehicle, 100)

# 保持车道
self.traffic_manager.auto_lane_change(ego_vehicle, False)
```

实现细节请参考 `proxy_simulator/hybrid_simulator.py`。

---

## 致谢

本混合仿真模式示例基于：

- [KING](https://github.com/autonomousvision/king) - 原始场景生成框架
- [CARLA](https://carla.org/) - 开源自动驾驶仿真器
- 自动驾驶研究社区

---

## 其他资源

- 📖 [HYBRID_QUICKSTART.md](HYBRID_QUICKSTART.md) - 快速开始指南
- 📖 [README_EN.md](README_EN.md) - 英文版文档
- 📄 [KING 论文](https://arxiv.org/pdf/2204.13683.pdf) - 原始论文
- 🔧 [CARLA 文档](https://carla.readthedocs.io/) - CARLA 官方文档
- 🎓 [Traffic Manager 指南](https://carla.readthedocs.io/en/latest/adv_traffic_manager/) - Traffic Manager 高级用法
