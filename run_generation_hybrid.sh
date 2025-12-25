#!/bin/bash

# KING Scenario Generation with Hybrid Mode
# Ego vehicle: CARLA autopilot
# Adversarial vehicles: Optimized in ProxySimulator

export CARLA_ROOT=carla_server # path to your carla root
export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:$(pwd -P)/leaderboard
export PYTHONPATH=$PYTHONPATH:$(pwd -P)/scenario_runner

# CARLA server should be running in another terminal:
# ./carla_server/CarlaUE4.sh --world-port=2000 -RenderOffScreen

echo "========================================="
echo "KING Hybrid Scenario Generation"
echo "========================================="
echo ""
echo "⚙️  Configuration:"
echo "  - Ego: CARLA Autopilot"
echo "  - Adversarial: ProxySimulator (optimized)"
echo "  - Batch size: 1 (forced)"
echo "  - Port: 2000"
echo ""

# 配置参数
INITIAL_SPEED=4.0
TM_SPEED_PERCENTAGE=40
SAVE_PATH="experiments/results_hybrid"
ROUTES_FILE_PATH="leaderboard/data/routes/subset_20perTown.xml"

# 创建输出目录
mkdir -p "${SAVE_PATH}"

# echo "Running for 4 agents..."
# python generate_scenarios_hybrid.py \
#     --use_hybrid_mode \
#     --num_agents 4 \
#     --opt_iters 100 \
#     --sim_horizon 80 \
#     --routes_file leaderboard/data/routes/subset_20perTown.xml \
#     --save_path "${SAVE_PATH}/agents_4" \
#     --beta1 0.8 \
#     --beta2 0.99 \
#     --w_ego_col 1.0 \
#     --w_adv_col 3.0 \
#     --w_adv_rd 20.0 \
#     --adv_col_thresh 1.25 \
#     --initial_speed ${INITIAL_SPEED} \
#     --tm_speed_percentage ${TM_SPEED_PERCENTAGE}

# echo ""
# echo "Running for 2 agents..."
# python generate_scenarios_hybrid.py \
#     --use_hybrid_mode \
#     --num_agents 2 \
#     --opt_iters 120 \
#     --sim_horizon 80 \
#     --routes_file leaderboard/data/routes/subset_20perTown.xml \
#     --save_path "${SAVE_PATH}/agents_2" \
#     --beta1 0.8 \
#     --beta2 0.99 \
#     --w_ego_col 1.0 \
#     --w_adv_col 3.0 \
#     --w_adv_rd 20.0 \
#     --adv_col_thresh 1.25 \
#     --initial_speed ${INITIAL_SPEED} \
#     --tm_speed_percentage ${TM_SPEED_PERCENTAGE}

# echo ""
echo "Running for 1 agent..."
python generate_scenarios_hybrid.py \
    --use_hybrid_mode \
    --num_agents 1 \
    --opt_iters 100 \
    --sim_horizon 80 \
    --routes_file ${ROUTES_FILE_PATH} \
    --save_path "${SAVE_PATH}/agents_1" \
    --beta1 0.8 \
    --beta2 0.99 \
    --w_ego_col 1.0 \
    --w_adv_col 3.0 \
    --w_adv_rd 20.0 \
    --adv_col_thresh 1.25 \
    --initial_speed ${INITIAL_SPEED} \
    --tm_speed_percentage ${TM_SPEED_PERCENTAGE}
    
echo ""
echo "✅ Generation complete! Results saved to ${SAVE_PATH}"
echo ""

echo "========================================="
echo "Parsing Results"
echo "========================================="

echo ""
echo "Overall results"
echo "==============="
python3 tools/parse_generation_results.py --results_dir "${SAVE_PATH}"

# echo ""
# echo "4 agents"
# echo "==============="
# python3 tools/parse_generation_results.py --results_dir "${SAVE_PATH}/agents_4" --num_agents 4

# echo ""
# echo "2 agents"
# echo "==============="
# python3 tools/parse_generation_results.py --results_dir "${SAVE_PATH}/agents_2" --num_agents 2

echo ""
echo "========================================="
echo "Visualizing Scenarios"
echo "========================================="
python3 tools/visualize_scenarios.py --scenario_log_dir "${SAVE_PATH}"

echo ""
echo "✅ All done!"