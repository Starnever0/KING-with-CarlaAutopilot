# Hybrid Mode Quick Start / 混合模式快速开始

## One-Minute Setup / 一分钟上手

```bash
# 1. Start CARLA / 启动 CARLA
./carla_server/CarlaUE4.sh --world-port=2000 -RenderOffScreen

# 2. Run hybrid generation / 运行混合生成
bash run_generation_hybrid.sh
```

That's it! / 就这么简单！

---

## What's Different? / 有什么不同？

| Standard Mode | Hybrid Mode |
|--------------|-------------|
| Ego: Neural Network | Ego: CARLA Autopilot ✨ |
| Physics: Kinematic | Physics: Real CARLA Engine ✨ |
| Speed: Fast | Speed: Realistic |

---

## Key Files / 关键文件

- `run_generation_hybrid.sh` - Main script / 主脚本
- `generate_scenarios_hybrid.py` - Generator with `--use_hybrid_mode`
- `proxy_simulator/hybrid_simulator.py` - Core implementation / 核心实现

---

## Configuration / 配置

Edit `run_generation_hybrid.sh` to customize / 编辑脚本以自定义：

```bash
INITIAL_SPEED=4.0              # Initial vehicle speed / 初始车速 (m/s)
TM_SPEED_PERCENTAGE=40         # Traffic Manager speed / 速度调整 (%)
ROUTES_FILE_PATH="..."         # Route file / 路由文件
```

---

## Output / 输出

Results in / 结果位于: `experiments/results_hybrid/`

```
agents_1/RouteScenario_*/
├── results.json          # Metrics / 指标
└── scenario_records.json # Trajectories / 轨迹
```

---

## Troubleshooting / 故障排查

**Problem / 问题**: CARLA connection timeout / 连接超时  
**Solution / 解决**: Check CARLA is running on port 2000 / 检查 CARLA 运行在 2000 端口

**Problem / 问题**: No results generated / 没有生成结果  
**Solution / 解决**: Check route files exist / 检查路由文件是否存在

---

📖 **Full Documentation** / 完整文档: [README.md](README.md)
