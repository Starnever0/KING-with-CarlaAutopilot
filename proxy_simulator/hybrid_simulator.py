"""
混合仿真器：主车在CARLA中使用autopilot，对抗车在ProxySimulator中优化
"""
import torch
import carla
import numpy as np
from icecream import ic
from proxy_simulator.simulator import ProxySimulator
from proxy_simulator.carla_wrapper import CarlaWrapper

# 配置icecream
ic.configureOutput(prefix='[HYBRID_DEBUG] ')
ic.disable()  # 默认禁用调试输出，按需启用

class DummyEgoExpert:
    """
    用于混合模式的伪专家类，防止ProxySimulator报错
    """
    def set_global_plan(self, gps_route, route):
        pass
        
    def reset(self):
        pass
    
    def eval(self):
        pass
        
    def to(self, device):
        return self

class HybridSimulator:
    """
    混合仿真器：
    - Ego车辆在真实CARLA中运行（使用autopilot）
    - 对抗车辆在ProxySimulator中优化（可微分）
    - 实时双向同步两个仿真器的状态
    """
    
    def __init__(self, args, adv_policy, motion_model):
        self.args = args
        
        # ==================== CARLA 真实仿真器 ====================
        self.carla_client = carla.Client('localhost', args.port)
        self.carla_client.set_timeout(10.0)
        self.carla_world = self.carla_client.get_world()
        
        # 获取原始设置
        self.original_settings = self.carla_world.get_settings()
        
        # 同步模式设置
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / self.args.sim_tickrate  # 与ProxySimulator同步
        self.carla_world.apply_settings(settings)
        
        # Traffic Manager配置
        self.traffic_manager = self.carla_client.get_trafficmanager(8000)
        self.traffic_manager.set_synchronous_mode(True)
        self.traffic_manager.set_random_device_seed(args.seed)
        
        # Ego车辆（真实CARLA对象）
        self.ego_vehicle_carla = None
        
        # 对抗车辆（CARLA中的影子，仅用于可视化）
        self.adv_vehicles_carla = []
        
        # ==================== ProxySimulator (对抗车辆) ====================
        self.proxy_simulator = ProxySimulator(
            args,
            ego_policy=DummyEgoExpert(),  # ego不在这里管理
            ego_expert=DummyEgoExpert(), # FIX: 传入伪专家防止set_route报错
            adv_policy=adv_policy,
            motion_model=motion_model,
        )
        
        # ==================== 状态同步变量 ====================
        self.timestep = 0
        
        # 坐标转换：CARLA世界坐标 <-> ProxySimulator像素坐标
        self.carla_wrapper = self.proxy_simulator.carla_wrapper
        
        # 用于记录主车动作的缓冲区
        self.ego_action_buffer = []
        
        # ==================== 调试信息 ====================
        self.prev_ego_pos = None
        self.prev_adv_pos = []
        
        ic("HybridSimulator initialized")
        
    def spawn_ego_vehicle(self, spawn_transform):
        """在CARLA中生成ego车辆并启用autopilot"""

        # Cleanup existing ego vehicle (安全检查)
        if self.ego_vehicle_carla is not None:
            try:
                if self.ego_vehicle_carla.is_alive:
                    self.ego_vehicle_carla.destroy()
            except RuntimeError:
                pass  # 已经被销毁
            self.ego_vehicle_carla = None

        blueprint_library = self.carla_world.get_blueprint_library()
        ego_bp = blueprint_library.find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name', 'hero')
        
        # 尝试稍微抬高一点生成，避免与地面碰撞
        spawn_transform.location.z += 0.2
        
        try:
            self.ego_vehicle_carla = self.carla_world.spawn_actor(ego_bp, spawn_transform)
        except RuntimeError as e:
            print(f"⚠️  Spawn failed: {e}. Trying to cleanup area...")
            # 尝试清理该区域的车辆
            self._cleanup_area(spawn_transform.location, radius=5.0)
            try:
                self.ego_vehicle_carla = self.carla_world.spawn_actor(ego_bp, spawn_transform)
            except RuntimeError as e2:
                print(f"❌ Spawn failed again: {e2}")
                raise e2
        
        # ✅ 设置初始速度（根据车辆朝向计算正确的速度方向）
        starting_speed = self.args.initial_speed  # m/s，从命令行参数读取
        # starting_speed = 10.0  # 硬编码主车速度
        yaw_rad = np.radians(spawn_transform.rotation.yaw)
        
        # 根据朝向计算速度分量
        vx = starting_speed * np.cos(yaw_rad)
        vy = starting_speed * np.sin(yaw_rad)
        
        initial_velocity = carla.Vector3D(x=float(vx), y=float(vy), z=0.0)
        self.ego_vehicle_carla.set_target_velocity(initial_velocity)
        
        # 启用autopilot
        self.ego_vehicle_carla.set_autopilot(True, self.traffic_manager.get_port())
        
        # 设置Traffic Manager参数
        self.traffic_manager.ignore_lights_percentage(self.ego_vehicle_carla, 100)  # 忽略红绿灯
        # self.traffic_manager.distance_to_leading_vehicle(self.ego_vehicle_carla,0) # 危险驾驶行为
        tm_speed_pct = self.args.tm_speed_percentage  # 从命令行参数读取
        self.traffic_manager.vehicle_percentage_speed_difference(self.ego_vehicle_carla, tm_speed_pct) # 速度调整（20代表低于限速30km/h的20%）
        
        # ✅ 调试：记录初始状态
        ego_loc = spawn_transform.location
        ego_vel = self.ego_vehicle_carla.get_velocity()
        ego_speed = np.sqrt(ego_vel.x**2 + ego_vel.y**2)
        self.prev_ego_pos = np.array([ego_loc.x, ego_loc.y])
        
        ic("Ego spawned", 
           ego_loc.x, ego_loc.y, ego_loc.z,
           spawn_transform.rotation.yaw,
           starting_speed, tm_speed_pct, ego_speed)
        
        
    def _cleanup_area(self, location, radius=2.0):
        """清理指定位置周围的物体，防止生成碰撞"""
        # 获取周围的所有actor
        world = self.carla_world
        # 简单的清理逻辑：查找距离过近的车辆并销毁（除了主车）
        # 注意：这可能比较危险，但在生成阶段通常是可以的
        actors = world.get_actors()
        vehicles = actors.filter('vehicle.*')
        
        for vehicle in vehicles:
            # 安全检查：车辆是否存活
            try:
                if not vehicle.is_alive:
                    continue
            except RuntimeError:
                continue
                
            if self.ego_vehicle_carla and vehicle.id == self.ego_vehicle_carla.id:
                continue
            
            try:
                v_loc = vehicle.get_location()
                dist = v_loc.distance(location)
                if dist < radius:
                    print(f"  - Removing conflicting vehicle {vehicle.id} at distance {dist:.2f}")
                    vehicle.destroy()
            except Exception as e:
                pass  # 车辆可能已经被销毁

    def spawn_adversarial_vehicles(self, adv_spawn_points):
        """在CARLA中生成对抗车辆（视觉占位符，禁用物理）"""
        blueprint_library = self.carla_world.get_blueprint_library()
        adv_bp = blueprint_library.find('vehicle.audi.a2')
        
        self.adv_vehicles_carla = []
        
        # # 获取主车位置（如果存在）
        # ego_loc = None
        # if self.ego_vehicle_carla is not None:
        #     ego_loc = self.ego_vehicle_carla.get_location()

        for i, spawn_point in enumerate(adv_spawn_points):
            # # 检查与主车的距离
            # if ego_loc is not None:
            #     dist = spawn_point.location.distance(ego_loc)
            #     if dist < 5.0:
            #         print(f"ℹ️  Skipping adversarial vehicle {i}: Too close to ego ({dist:.2f}m)")
            #         self.adv_vehicles_carla.append(None)
            #         continue

            # 使用变换的副本，避免修改原始数据
            sp = carla.Transform(spawn_point.location, spawn_point.rotation)
            sp.location.z += 1.5  # 初始尝试抬高1.5米
            
            adv_vehicle = None
            try:
                adv_vehicle = self.carla_world.spawn_actor(adv_bp, sp)
            except RuntimeError as e:
                print(f"Warning: Failed to spawn adversarial vehicle {i}: {e}")
                
                # 尝试清理区域
                self._cleanup_area(sp.location, radius=2.0)
                
                # 重试策略 1: 再抬高
                try:
                    print(f"  - Retry 1: Elevating...")
                    sp.location.z += 1.0 # 总共 +2.5m
                    adv_vehicle = self.carla_world.spawn_actor(adv_bp, sp)
                except RuntimeError:
                    # 重试策略 2: 抬高 + 稍微移动
                    try:
                        print(f"  - Retry 2: Nudging...")
                        sp.location.x += 0.5
                        sp.location.y += 0.5
                        adv_vehicle = self.carla_world.spawn_actor(adv_bp, sp)
                    except RuntimeError as e2:
                        print(f"❌ Completely failed to spawn adversarial vehicle {i}: {e2}")
                        adv_vehicle = None
            
            if adv_vehicle is not None:
                adv_vehicle.set_simulate_physics(False)
                self.adv_vehicles_carla.append(adv_vehicle)
            else:
                self.adv_vehicles_carla.append(None)
        
        # ✅ 调试：记录对抗车初始位置
        self.prev_adv_pos = []
        for i, adv_vehicle in enumerate(self.adv_vehicles_carla):
            if adv_vehicle is not None:
                loc = adv_vehicle.get_transform().location
                self.prev_adv_pos.append(np.array([loc.x, loc.y]))
                ic(f"Adv{i} spawned", loc.x, loc.y, loc.z)
            else:
                self.prev_adv_pos.append(None)
                
        # print(f"✓ {len([v for v in self.adv_vehicles_carla if v is not None])} adversarial vehicles spawned")

    def sync_ego_state_to_proxy(self):
        """从CARLA读取ego状态，同步到ProxySimulator"""
        if self.ego_vehicle_carla is None:
            raise RuntimeError("Ego vehicle not spawned!")
        
        # 读取CARLA中的ego状态
        carla_transform = self.ego_vehicle_carla.get_transform()
        carla_velocity = self.ego_vehicle_carla.get_velocity()
        
        # 转换为ProxySimulator的状态格式
        ego_pos_world = torch.tensor(
            [[carla_transform.location.x, carla_transform.location.y]], 
            dtype=torch.float32, 
            device=self.args.device
        ).repeat(self.args.batch_size, 1)
        
        ego_yaw = torch.tensor(
            [[np.radians(carla_transform.rotation.yaw)]], 
            dtype=torch.float32, 
            device=self.args.device
        ).repeat(self.args.batch_size, 1)
        
        ego_vel = torch.tensor(
            [[carla_velocity.x, carla_velocity.y]], 
            dtype=torch.float32, 
            device=self.args.device
        ).repeat(self.args.batch_size, 1)

        ego_pos_world = ego_pos_world.unsqueeze(1)
        ego_yaw = ego_yaw.unsqueeze(1)
        ego_vel = ego_vel.unsqueeze(1)
        
        ego_state = {
            'pos': ego_pos_world.detach(),  # 断开梯度
            'yaw': ego_yaw.detach(),
            'vel': ego_vel.detach()
        }
        
        # 更新ProxySimulator的ego状态
        self.proxy_simulator.set_ego_state(ego_state)
        
        return ego_state
        
    def sync_adv_state_to_carla(self):
        """从ProxySimulator读取对抗车状态，同步到CARLA"""
        adv_state = self.proxy_simulator.get_adv_state()
        
        # 批量更新对抗车位置
        for i, adv_vehicle in enumerate(self.adv_vehicles_carla):
            if adv_vehicle is None:
                continue

            if i >= adv_state['pos'].shape[1]:
                break
                
            # 构造CARLA Transform
            # FIX: 添加 .detach() 断开梯度，并使用 .item() 获取标量值
            transform = carla.Transform(
                carla.Location(
                    x=float(adv_state['pos'][0, i, 0].detach().cpu().item()),
                    y=float(adv_state['pos'][0, i, 1].detach().cpu().item()),
                    z=0.3  # 固定高度
                ),
                carla.Rotation(
                    yaw=float(np.rad2deg(adv_state['yaw'][0, i, 0].detach().cpu().item()))
                )
            )
            
            # 更新CARLA中的对抗车位置
            adv_vehicle.set_transform(transform)
    
    def reset_ego_to_start(self, start_transform):
        """
        重置ego车辆到起点位置
        
        用于优化迭代时，将ego重置回起始位置而不重新生成车辆
        """
        if self.ego_vehicle_carla is None:
            raise RuntimeError("Ego vehicle not spawned!")
        
        # 重置位置和朝向
        self.ego_vehicle_carla.set_transform(start_transform)
        
        # 重置速度
        starting_speed = self.args.initial_speed
        yaw_rad = np.radians(start_transform.rotation.yaw)
        vx = starting_speed * np.cos(yaw_rad)
        vy = starting_speed * np.sin(yaw_rad)
        initial_velocity = carla.Vector3D(x=float(vx), y=float(vy), z=0.0)
        self.ego_vehicle_carla.set_target_velocity(initial_velocity)
        
        # 注意：CARLA 0.9.10 没有 set_angular_velocity 方法
        # 重置autopilot状态（停用再启用，清除内部状态）
        self.ego_vehicle_carla.set_autopilot(False, self.traffic_manager.get_port())
        self.ego_vehicle_carla.set_autopilot(True, self.traffic_manager.get_port())
        
    def reset_adv_to_start(self, adv_spawn_points):
        """
        重置对抗车辆到起点位置
        
        用于优化迭代时，将对抗车重置回起始位置
        """
        for i, (adv_vehicle, spawn_point) in enumerate(zip(self.adv_vehicles_carla, adv_spawn_points)):
            if adv_vehicle is None:
                continue
            
            # 重置位置和朝向
            transform = carla.Transform(
                carla.Location(
                    x=spawn_point.location.x,
                    y=spawn_point.location.y,
                    z=spawn_point.location.z
                ),
                carla.Rotation(yaw=spawn_point.rotation.yaw)
            )
            adv_vehicle.set_transform(transform)
    
    def get_ego_action_from_carla(self):
        """从CARLA读取ego车辆的当前控制指令"""
        if self.ego_vehicle_carla is None:
            raise RuntimeError("Ego vehicle not spawned!")
        
        control = self.ego_vehicle_carla.get_control()
        
        ego_actions = {
            "steer": torch.tensor([[control.steer]], device=self.args.device, dtype=torch.float32),
            "throttle": torch.tensor([[control.throttle]], device=self.args.device, dtype=torch.float32),
            "brake": torch.tensor([[control.brake]], device=self.args.device, dtype=torch.float32),
        }
        
        return ego_actions
            
    def step(self, adv_actions):
        """
        混合仿真的单步执行
        
        Args:
            adv_actions: 对抗车辆的动作（来自优化）
        """
        # ============ 0. 检查终止状态 ============
        # 如果已经终止，不执行任何更新，直接返回当前状态
        if self.proxy_simulator.is_terminated[0].item():
            ego_state = self.sync_ego_state_to_proxy()
            return ego_state
        
        # ============ 1. ProxySimulator中更新对抗车状态 ============
        # 只更新对抗车，不更新ego（ego由CARLA管理）
        adv_current_state = self.proxy_simulator.get_adv_state()
        adv_next_state = self.proxy_simulator.motion_model(
            adv_current_state,
            adv_actions,
            self.proxy_simulator.is_terminated
        )
        self.proxy_simulator.set_adv_state(adv_next_state)
        
        # ============ 2. 同步对抗车状态到CARLA (Proxy -> CARLA) ============
        # 先将计算出的对抗车下一时刻状态同步给CARLA，以便在Tick中生效（视觉/碰撞）
        self.sync_adv_state_to_carla()
        
        # ============ 3. CARLA世界tick（推进物理仿真） ============
        # 推进物理世界时间，Ego车辆会在此期间由Autopilot控制移动
        self.carla_world.tick()
        
        # ============ 4. 同步主车状态到Proxy (CARLA -> Proxy) ============
        # Tick之后，读取Ego车辆的最新状态（修正：确保读取的是Tick后的状态）
        ego_state = self.sync_ego_state_to_proxy()
        
        # ============ 5. 记录ego动作 ============
        # 获取Tick期间Autopilot施加的控制
        ego_actions = self.get_ego_action_from_carla()
        self.ego_action_buffer.append({
            k: v.clone().detach() for k, v in ego_actions.items()
        })
        
        # ============ 6. 记录对抗车动作 ============
        self.proxy_simulator.adv_action_buffer.append({
            k: v.clone().detach() for k, v in adv_actions.items()
        })
        
        # ============ 7. 记录状态 ============
        state_detached = {
            "pos": torch.cat([ego_state["pos"], adv_next_state["pos"]], dim=1).clone().detach(),
            "yaw": torch.cat([ego_state["yaw"], adv_next_state["yaw"]], dim=1).clone().detach(),
            "vel": torch.cat([ego_state["vel"], adv_next_state["vel"]], dim=1).clone().detach()
        }
        self.proxy_simulator.state_buffer.append(state_detached)
        
        # ✅ 调试：打印运动信息（每10步或前5步）- 直接从CARLA读取
        if self.timestep < 5 or self.timestep % 10 == 0:
            # === Ego车辆：直接从CARLA读取原始数据 ===
            carla_transform = self.ego_vehicle_carla.get_transform()
            carla_velocity = self.ego_vehicle_carla.get_velocity()
            carla_control = self.ego_vehicle_carla.get_control()
            
            ego_pos_carla_raw = np.array([carla_transform.location.x, carla_transform.location.y])
            ego_vel_carla_raw = np.array([carla_velocity.x, carla_velocity.y])
            ego_speed_carla_raw = np.sqrt(carla_velocity.x**2 + carla_velocity.y**2)
            ego_yaw_carla_raw = carla_transform.rotation.yaw
            
            # 计算ego移动距离
            ego_distance = 0.0
            if self.prev_ego_pos is not None:
                ego_distance = np.linalg.norm(ego_pos_carla_raw - self.prev_ego_pos)
            self.prev_ego_pos = ego_pos_carla_raw.copy()
            
            # 同步后的Proxy中的ego状态（用于对比）
            ego_pos_proxy = ego_state['pos'][0, 0].cpu().numpy()
            ego_vel_proxy = ego_state['vel'][0, 0].cpu().numpy()
            ego_speed_proxy = np.linalg.norm(ego_vel_proxy)
            
            # 检查同步误差
            sync_pos_error = np.linalg.norm(ego_pos_carla_raw - ego_pos_proxy)
            sync_speed_error = abs(ego_speed_carla_raw - ego_speed_proxy)
            
            ic(f"t={self.timestep}",
                "Ego[CARLA_RAW]",
                ego_pos_carla_raw, 
                ego_speed_carla_raw,
                ego_distance,
                f"steer={carla_control.steer:.3f}",
                f"throttle={carla_control.throttle:.3f}",
                f"brake={carla_control.brake:.3f}")
            
            ic(f"t={self.timestep}",
               "Ego[Proxy_Synced]",
               ego_pos_proxy,
               ego_speed_proxy,
               f"pos_err={sync_pos_error:.4f}",
               f"spd_err={sync_speed_error:.4f}")
            
            # === 对抗车：对比CARLA和Proxy中的状态 ===
            for i in range(min(len(self.adv_vehicles_carla), adv_next_state['pos'].shape[1])):
                if self.adv_vehicles_carla[i] is None:
                    continue
                
                # 从CARLA直接读取对抗车状态
                adv_carla_transform = self.adv_vehicles_carla[i].get_transform()
                adv_pos_carla_raw = np.array([adv_carla_transform.location.x, adv_carla_transform.location.y])
                adv_yaw_carla_raw = adv_carla_transform.rotation.yaw
                
                # 从Proxy读取对抗车状态
                adv_pos_proxy = adv_next_state['pos'][0, i].detach().cpu().numpy()
                adv_vel_proxy = adv_next_state['vel'][0, i].detach().cpu().numpy()
                adv_speed_proxy = np.linalg.norm(adv_vel_proxy)
                adv_yaw_proxy = np.rad2deg(adv_next_state['yaw'][0, i, 0].detach().cpu().item())
                
                # 计算adv移动距离（基于CARLA位置）
                adv_distance = 0.0
                if i < len(self.prev_adv_pos) and self.prev_adv_pos[i] is not None:
                    adv_distance = np.linalg.norm(adv_pos_carla_raw - self.prev_adv_pos[i])
                    self.prev_adv_pos[i] = adv_pos_carla_raw.copy()
                elif i < len(self.prev_adv_pos):
                    self.prev_adv_pos[i] = adv_pos_carla_raw.copy()
                else:
                    self.prev_adv_pos.append(adv_pos_carla_raw.copy())
                
                # 检查同步误差
                adv_sync_pos_error = np.linalg.norm(adv_pos_carla_raw - adv_pos_proxy)
                adv_sync_yaw_error = abs(adv_yaw_carla_raw - adv_yaw_proxy)
                
                # 计算ego与adv的距离（基于CARLA位置）
                ego_adv_distance = np.linalg.norm(ego_pos_carla_raw - adv_pos_carla_raw)
                
                ic(f"t={self.timestep}",
                   f"Adv{i}[CARLA_RAW]",
                   adv_pos_carla_raw,
                   adv_distance,
                   f"dist_to_ego={ego_adv_distance:.2f}")
                
                ic(f"t={self.timestep}",
                   f"Adv{i}[Proxy]",
                   adv_pos_proxy,
                   adv_speed_proxy,
                   f"pos_err={adv_sync_pos_error:.4f}",
                   f"yaw_err={adv_sync_yaw_error:.2f}°")
        
        self.timestep += 1
        self.proxy_simulator.timestep += 1
        
        return ego_state
        
    def set_route(self, gps_route, route, route_config):
        """设置路线并初始化状态"""
        # 重置时间步
        self.timestep = 0
        self.proxy_simulator.timestep = 0
        
        # 清空缓冲区
        self.proxy_simulator.state_buffer = []
        self.ego_action_buffer = []
        self.proxy_simulator.adv_action_buffer = []
        
        # 重置调试信息
        self.prev_ego_pos = None
        self.prev_adv_pos = []
        
        # 设置ProxySimulator的路线
        self.proxy_simulator.set_route(gps_route, route, route_config)
        
        ic("Route set", route_config[0].name if route_config else "unknown")
        
    def reset(self):
        """重置仿真状态"""
        # 销毁现有车辆（添加安全检查）
        if self.ego_vehicle_carla is not None:
            try:
                if self.ego_vehicle_carla.is_alive:
                    self.ego_vehicle_carla.destroy()
            except RuntimeError:
                pass  # 已经被销毁
            self.ego_vehicle_carla = None
            
        for adv_vehicle in self.adv_vehicles_carla:
            if adv_vehicle is not None:
                try:
                    if adv_vehicle.is_alive:
                        adv_vehicle.destroy()
                except RuntimeError:
                    pass  # 已经被销毁
        self.adv_vehicles_carla = []
            
        # 重置时间步
        self.timestep = 0
        
    def cleanup(self):
        """清理资源并恢复CARLA设置"""
        # 先清理车辆（带安全检查）
        self.reset()
        
        # 恢复CARLA原始设置
        try:
            self.carla_world.apply_settings(self.original_settings)
            print("✓ HybridSimulator cleaned up")
        except Exception as e:
            print(f"⚠️  Warning during cleanup: {e}")
        
    # ==================== 代理方法（兼容GenerationEngine） ====================
    def get_ego_state(self):
        """代理方法"""
        return self.proxy_simulator.get_ego_state()
    
    def get_adv_state(self):
        """代理方法"""
        return self.proxy_simulator.get_adv_state()
    
    def get_ego_sensor(self):
        """代理方法"""
        return self.proxy_simulator.get_ego_sensor()
    
    def run_termination_checks(self):
        """代理方法"""
        return self.proxy_simulator.run_termination_checks()
    
    def set_new_town(self, args, town):
        """代理方法"""
        self.proxy_simulator.set_new_town(args, town)
    
    @property
    def map(self):
        """代理属性"""
        return self.proxy_simulator.map
    
    @property
    def renderer(self):
        """代理属性"""
        return self.proxy_simulator.renderer
    
    @property
    def adv_policy(self):
        """代理属性"""
        return self.proxy_simulator.adv_policy
    
    @property
    def ego_extent(self):
        """代理属性"""
        return self.proxy_simulator.ego_extent
    
    @property
    def adv_extent(self):
        """代理属性"""
        return self.proxy_simulator.adv_extent
    
    @property
    def is_terminated(self):
        """代理属性"""
        return self.proxy_simulator.is_terminated
    
    @property
    def tot(self):
        """代理属性"""
        return self.proxy_simulator.tot
    
    @property
    def ego_collision(self):
        """代理属性"""
        return self.proxy_simulator.ego_collision
    
    @property
    def adv_collision(self):
        """代理属性"""
        return self.proxy_simulator.adv_collision
    
    @property
    def adv_rel_pos_at_collision(self):
        """代理属性"""
        return self.proxy_simulator.adv_rel_pos_at_collision
    
    @property
    def adv_rel_yaw_at_collision(self):
        """代理属性"""
        return self.proxy_simulator.adv_rel_yaw_at_collision
    
    @property
    def state_buffer(self):
        """代理属性"""
        return self.proxy_simulator.state_buffer
    
    @property
    def adv_action_buffer(self):
        """代理属性"""
        return self.proxy_simulator.adv_action_buffer
