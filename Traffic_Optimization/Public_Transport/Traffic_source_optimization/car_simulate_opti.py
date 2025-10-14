from cmath import phase
import traci
import traci
import sumolib
import os
import argparse
from tool.Carttils import  *
import geopandas as gpd
import xml.etree.ElementTree as ET
from collections import defaultdict
import numpy as np
import shutil
import geopandas as gpd
import numpy as np
import folium
from matplotlib import colors
import random
from shapely.geometry import Polygon
import numpy as np
import random
import time
import shutil

import time
from datetime import datetime
import argparse
import traci
import sumolib
import os
import xml.etree.ElementTree as ET
import random
import collections
import numpy as np
import pandas as pd
from collections import defaultdict

import xml.etree.ElementTree as ET
from collections import deque
import copy
import pickle

import shutil

# 强化学习相关导入
from typing import Dict, List, Optional
try:
    from .rl_controllers import TrafficLightControllerManager
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    try:
        from tool.rl_controllers import TrafficLightControllerManager
    except ImportError:
        print("警告: 无法导入RL控制器，RL功能将不可用")
        TrafficLightControllerManager = None







def evalute_mae(sumo_result,real_traffic,scale):
    dict_re={}

    # 初始化变量以存储匹配的值
    real_values = []
    sumo_values = []
    
    # 遍历real_traffic字典中的每个路段ID
    for road_id in real_traffic.keys():
        # 检查该路段ID是否也存在于sumo_result中
        if road_id in sumo_result:
            # if real_traffic[road_id]==0 and sumo_result[road_id]==0:
            #     continue
            # if sumo_result[road_id]==0 and real_traffic[road_id]==3:
            #     print(road_id)
            # 如果存在，将对应的值添加到列表中
            real_values.append(real_traffic[road_id])
            sumo_values.append(sumo_result[road_id])
    
    # 将列表转换为numpy数组以便计算
    real_values = np.array(real_values)
    sumo_values = np.array(sumo_values)
    
    # 找出real_values中非零的索引
    non_zero_indices = np.where(real_values != 0)[0]
    
    # 使用非零值计算召回率
    if len(non_zero_indices) > 0:
        # 获取对应的sumo_values
        relevant_sumo_values = sumo_values[non_zero_indices]
        relevant_real_values = real_values[non_zero_indices]
        
        # 计算召回率 (真正例 / (真正例 + 假负例))
        true_positives = np.sum((relevant_sumo_values != 0) & (relevant_real_values != 0))
        false_negatives = np.sum((relevant_sumo_values == 0) & (relevant_real_values != 0))
        
        recall = true_positives / (true_positives + false_negatives) if (true_positives + false_negatives) > 0 else 0
        print(f"召回率: {recall:.4f}")
    else:
        print("没有非零的真实值，无法计算召回率")
    
    
    # 计算准确率
    # 计算准确率 (真正例 / (真正例 + 假正例))
    true_positives = np.sum((sumo_values != 0) & (real_values != 0))
    false_positives = np.sum((sumo_values != 0) & (real_values == 0))
    
    precision = true_positives / (true_positives + false_positives) if (true_positives + false_positives) > 0 else 0
    print(f"准确率: {precision:.4f}")
    
    differences = []
    for i in range(len(real_values)):
        if real_values[i] != sumo_values[i]:
            differences.append((i, real_values[i], sumo_values[i]))
    
    # 打印不同的值
    print(f"共发现 {len(differences)} 处不同:")
    real_greater_count = 0
    sumo_greater_count = 0

    for diff in differences:
        index, real_val, sumo_val = diff
        if real_val > sumo_val:
            real_greater_count += 1
        elif sumo_val > real_val:
            sumo_greater_count += 1

    print(f"真实值大于模拟值的个数: {real_greater_count}")
    print(f"模拟值大于真实值的个数: {sumo_greater_count}")
    # 计算real_greater_count/sumo_greater_count
    ratio=real_greater_count/(sumo_greater_count+0.0000001)
    print(f"真实值大于模拟值的个数与模拟值大于真实值的个数的比值: {ratio}")

    
    # 计算平均绝对误差(MAE)
    mae = np.mean(np.abs(real_values - sumo_values))
    print(f"平均绝对误差(MAE): {mae}")
    dict_re[scale] = {
        'mae': mae,
        'ratio': ratio,
        'real_greater_count': real_greater_count,
        'sumo_greater_count': sumo_greater_count,
        'differences': differences,
        'recall': recall,
        'precision': precision
    }
    
    return scale,dict_re[scale]
    
def calculate_congest_R(result_data,average_waiting_vehicles):
    # 初始化一个新字典来保存拥堵评分
    congestion_scores = {}
    
    # 遍历result_data中的每个路段ID
    for road_id in result_data:
        # if road_id=='200022041':
        #     a=1
        ratio=result_data[road_id]
        
        # 检查该路段ID是否也存在于road_limit_file中
        
        # 计算自由流速度与实际速度的比值
     
        # 根据比值分配拥堵评分
        if ratio > 2 and average_waiting_vehicles[road_id] > 20:
            result_data[road_id] = 5  # 严重拥堵
        elif 1.5 < ratio <= 2 and average_waiting_vehicles[road_id] > 20:
            result_data[road_id] = 3 # 中度拥堵
        else:  # ratio <= 1.5
            result_data[road_id] = 0  # 畅通
    
    return result_data





import os
import sys
import argparse
import numpy as np
import time

# 确保SUMO环境变量设置正确
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit('请设置SUMO_HOME环境变量')

import traci
import sumolib



class LTF_Traci:
    """
    LTF_Traci (LibSignal Traffic Flow with Traci) 类用于加载SUMO网络和路由文件，
    使用traci接口与SUMO通信，并使用MaxPressure算法优化所有路口
    """
    def __init__(self, net_file, route_file, use_gui=True, 
                 end_time=3600, yellow_time=3, t_min=10, seed=42,need_optimize_tlds=[],given_tlds_type='road',part_optimize=False,sumobinary='sumo',
                 # 新增RL参数（向后兼容）
                 rl_mode: str = 'none',  # 'none', 'dqn', 'ppo'
                 rl_tls_ids: List[str] = None,  # 参与RL的路口ID列表
                 non_rl_policy: str = 'greedy',  # 'greedy', 'static'
                 training: bool = False,  # 训练模式还是推理模式
                 checkpoint_dir: str = None,  # 模型保存/加载路径
                 rl_config: Dict = None,  # RL超参数配置
                 num_episodes: int = 1,
                 action_interval: int = 10,
                 learning_start_time: int = 200,
                 update_model_rate: int = 10,
                 cal_reward_circle: int = 1,
                 reward_static_pkl: str = "pkl/reward_static_all.pkl",
                 update_target_rate: int = 100):  # 训练回合数
        """
        初始化LTF_Traci对象

        参数:
            net_file (str): SUMO网络文件路径(.net.xml)
            route_file (str): SUMO路由文件路径(.rou.xml)
            use_gui (bool): 是否使用SUMO GUI
            end_time (int): 仿真结束时间(秒)
            yellow_time (int): 黄灯持续时间(秒)
            t_min (int): 一个相位的最小持续时间(秒)
            seed (int): 随机种子
            need_optimize_tlds (list): 需要优化的交通灯列表
            given_tlds_type (str): 交通灯类型
            part_optimize (bool): 是否部分优化
            sumobinary (str): SUMO二进制文件路径
            rl_mode (str): RL模式 ('none', 'dqn', 'ppo')
            rl_tls_ids (List[str]): 参与RL的路口ID列表
            non_rl_policy (str): 非RL路口的控制策略 ('greedy', 'static')
            training (bool): 训练模式还是推理模式
            checkpoint_dir (str): 模型保存/加载路径
            rl_config (Dict): RL超参数配置
            num_episodes (int): 训练回合数
        """
        self.net_file = net_file
        self.route_file = route_file
        self.use_gui = False
        self.end_time = end_time
        self.yellow_time = yellow_time
        self.t_min = t_min
        self.seed = seed
        self.part_optimize=part_optimize
        self.sumo_binary=sumobinary
        self.cal_reward_circle=cal_reward_circle
        # RL相关参数
        self.rl_mode = rl_mode
        self.rl_tls_ids = rl_tls_ids or []
        self.non_rl_policy = non_rl_policy
        self.training = training
        self.checkpoint_dir = checkpoint_dir
        self.rl_config = rl_config or {}
        self.num_episodes = num_episodes
        self.action_interval_ = action_interval
        self.learning_start_time = learning_start_time
        self.update_model_rate_ = update_model_rate
        self.update_target_rate_ = update_target_rate
        self.reward_static_pkl = reward_static_pkl
        # RL控制器管理器（将在_init_connection中初始化）
        self.controller_manager = None
        
        # 初始化SUMO网络
        self.net = sumolib.net.readNet(net_file)
        
        # 存储交叉路口信息
        self.intersections = {}
        self.phase_data = {}
        self.raw_phase_data = {}
        
        # 当前相位和相位时间
        self.current_phases = {}
        self.current_phase_times = {}
        
        # 黄灯状态跟踪
        self.yellow_phase_states = {}  # 交通灯ID -> 是否处于黄灯状态
        self.yellow_phase_countdown = {}  # 交通灯ID -> 黄灯倒计时
        self.target_green_phases = {}  # 交通灯ID -> 目标绿灯相位
        self.net_file_staticTlc='xml/static_tlc_mapall.add.xml'
        self.road_need_add=[]
        self.press=None
        self.pressure={}
        self.n_episodes=0
        
        # 车辆旅行时间跟踪
        self.inside_vehicles = {}  # 记录当前在仿真中的车辆及其进入时间
        self.completed_trips = {}  # 记录已完成旅行的车辆及其旅行时间
        
        self.arrival_carnum=0
    
    
        
        # 连接信息
        self.lanelinks = {}
        
        self.need_optimize_tlds=[]
        if given_tlds_type=='road':
            self.need_optimize_tlds=need_optimize_tlds
        
        
        
        
        
        self.tls_phase_history=None
        self.need_add=[]
        
        # 初始化连接
        self._init_connection()
        
        
    def _init_connection_train(self):
        """初始化SUMO连接，使用traci接口"""
        # 选择合适的SUMO可执行文件
        sumo_binary = self.sumo_binary 
        
        # 构建SUMO命令
        sumocmd = [
            sumo_binary,
            "-n", self.net_file,
            "-r", self.route_file,
            "--no-step-log", "true",
            "--seed", str(self.seed),
            "--delay", "0",
            "--time-to-teleport", "-1",
            "--no-warnings", "true"
        ]
        
        # 如果使用GUI，添加GUI相关参数
        if self.use_gui:
            sumocmd.extend([
                "--start", "true",  # 自动开始仿真
                "--quit-on-end", "true"  # 仿真结束时自动关闭
            ])

        # 记录命令
        print(f"启动SUMO命令: {' '.join(sumocmd)}")
        
        # 启动traci连接
        self.conn.start(sumocmd)
        self.arrival_carnum=0
        self.inside_vehicles = {}  # 记录当前在仿真中的车辆及其进入时间
        self.completed_trips = {}  # 记录已完成旅行的车辆及其旅行时间

        
        # 获取交通灯列表
        self.tls_ids = self.conn.trafficlight.getIDList()
        print(f"找到 {len(self.tls_ids)} 个交通灯")
    
        
        # 初始化交通灯信息
        self._init_traffic_lights()
        
        

    def calculate_pressure(self, tl_id,step):
        """
       
        参数:
            tl_id (str): 交通灯ID
            
        返回:
            int: 最佳相位ID
        """

        # 获取所有车道的车辆数
        lane_counts = {}
        
        # 只获取相关车道的车辆数
        for phase_links in self.intersections[tl_id]['phase_all_lanelinks']:
            for start_lane, end_lane in phase_links[1]:
                if start_lane not in lane_counts:
                    lane_counts[start_lane] = self.conn.lane.getLastStepVehicleNumber(start_lane)
                if end_lane not in lane_counts:
                    lane_counts[end_lane] = self.conn.lane.getLastStepVehicleNumber(end_lane)
        # 计算每个相位的压力
      
        
        current_phase=self.current_phases[tl_id]
        
        if self.n_episodes not in self.pressure:
            self.pressure[self.n_episodes] = {}
        if step not in self.pressure[self.n_episodes]:
            self.pressure[self.n_episodes][step] = {}
        self.pressure[self.n_episodes][step][tl_id] = (lane_counts,current_phase)
        
        return 
    
    
    def compute_pressure(self):
    
        from collections import defaultdict, Counter
        self.press=self.pressure
        # 统计每个信号灯各相位出现次数
        phase_count = defaultdict(Counter)
        # 存储所有车道的车辆数据用于计算均值
        all_lane_counts = defaultdict(list)
        # 存储每个信号灯最常见相位及其非红灯链接压力数据
        tl_phase_pressure_data = defaultdict(list)
        
        # 遍历所有episode和step收集数据
        for episode in self.press:
            for step in self.press[episode]:
                for tl_id_key, (lane_counts, current_phase) in self.press[episode][step].items():
                    # 统计相位出现次数
                    phase_count[tl_id_key][current_phase] += 1
                    
                    # 收集车道车辆数据
                    for lane_id, vehicle_count in lane_counts.items():
                        all_lane_counts[lane_id].append(vehicle_count)
                    
                    # 收集相位压力数据
                    tl_phase_pressure_data[tl_id_key].append((current_phase, lane_counts))
        
        # 计算每个信号灯最常出现的相位
        most_common_phases = {tl_id: phases.most_common(1)[0][0] for tl_id, phases in phase_count.items()}
        
        # 计算所有车道的车辆均值
        lane_averages = {}
        for lane_id, counts in all_lane_counts.items():
            lane_averages[lane_id] = sum(counts) / len(counts)
        
        # 计算每个信号灯最常见相位的非红灯链接压力均值
        tl_phase_result = {}
        
        for tl_id_key, phase_data_list in tl_phase_pressure_data.items():
            if tl_id_key not in most_common_phases:
                continue
                
            most_common_phase = most_common_phases[tl_id_key]
            
            # 获取该信号灯的相位状态字符串
            phase_state = None
            if tl_id_key in self.phase_data:
                phase_state = self.phase_data[tl_id_key][most_common_phase].state
            
            # 找到该信号灯对应的相位链接信息
            phase_links_info = None
            if tl_id_key in self.intersections and 'phase_all_lanelinks' in self.intersections[tl_id_key]:
                for links_ in self.intersections[tl_id_key]['phase_all_lanelinks']:
                    if links_[0] == most_common_phase:
                        phase_links_info = links_[1]
                        break
            
            if phase_state is None or phase_links_info is None:
                continue
                
            # 收集该最常见相位的所有压力数据
            phase_pressures = defaultdict(list)
            
            for phase_id, lane_counts in phase_data_list:
                if phase_id == most_common_phase:
                    # 计算每个链接的压力
                    for link_idx, (start_lane, end_lane) in enumerate(phase_links_info):
                        # 检查该链接是否为非红灯状态（假设'r'或'R'表示红灯）
                        if link_idx < len(phase_state) and phase_state[link_idx].lower() != 'r':
                            pressure = lane_counts.get(start_lane, 0) - lane_counts.get(end_lane, 0)
                            phase_pressures[link_idx].append(pressure)
            
            # 计算非红灯链接的平均压力
            non_red_pressure_dict = {}
            for link_idx, pressure_list in phase_pressures.items():
                if pressure_list:  # 确保有数据
                    non_red_pressure_dict[link_idx] = sum(pressure_list) / len(pressure_list)
            
            # 存储结果：[相位状态字符串, 非红灯链接压力字典]
            if phase_state and non_red_pressure_dict:
                tl_phase_result[tl_id_key] = [phase_state, non_red_pressure_dict]
        
        # with open("pkl/tl_phase_result.pkl", "wb") as f:
        #     pickle.dump(tl_phase_result, f)
        # with open("pkl/lane_averages.pkl", "wb") as f:
        #     pickle.dump(lane_averages, f)
        return tl_phase_result, lane_averages
   
        
    def _init_connection(self):
        """初始化SUMO连接，使用traci接口"""
        # 选择合适的SUMO可执行文件
        sumo_binary = self.sumo_binary 
        
        # 构建SUMO命令
        sumocmd = [
            sumo_binary,
            "-n", self.net_file,
            "-r", self.route_file,
            "--no-step-log", "true",
            "--seed", str(self.seed),
            "--delay", "0",
            "--time-to-teleport", "-1",
            "--no-warnings", "true"
        ]
        
        # 如果使用GUI，添加GUI相关参数
        if self.use_gui:
            sumocmd.extend([
                "--start", "true",  # 自动开始仿真
                "--quit-on-end", "true"  # 仿真结束时自动关闭
            ])
  
        
        # 记录命令
        print(f"启动SUMO命令: {' '.join(sumocmd)}")
        
        # 启动traci连接
        traci.start(sumocmd)
        self.conn = traci
            
        # 获取交通灯列表
        self.tls_ids = self.conn.trafficlight.getIDList()
        print(f"找到 {len(self.tls_ids)} 个交通灯")
        
        
        
        
        
        # 初始化交通灯信息
        self._init_traffic_lights()
        
        
        
        
        
        # 初始化RL控制器管理器
        if self.rl_mode != 'none' and TrafficLightControllerManager is not None:
            self._init_rl_controllers()

    
    def _init_rl_controllers(self):
        
        """初始化RL控制器管理器"""
        try:
            self.controller_manager = TrafficLightControllerManager(
                tls_ids=self.tls_ids,
                intersections=self.intersections,
                rl_mode=self.rl_mode,
                rl_tls_ids=self.rl_tls_ids,
                non_rl_policy=self.non_rl_policy,
                training=self.training,
                checkpoint_dir=self.checkpoint_dir,
                rl_config=self.rl_config,
                ltf_instance=self
            )
            if self.rl_tls_ids:
                print(f"RL控制路口: {self.rl_tls_ids}")
            
            # 如果不是训练模式，尝试加载模型
            if not self.training:
                self.controller_manager.load_checkpoint()
                
        except Exception as e:
            print(f"初始化RL控制器失败: {e}")
            self.controller_manager = None
            self.rl_mode = 'none'
        
        
        
    def get_tls_id_opitmizer(self):
        tls_id_opitmizer=[]
        for k in self.need_optimize_tlds:
            if k in self.road_to_tls.keys():
                tls_id_opitmizer.append(self.road_to_tls[k])
        
        self.tls_id_opitmizer=tls_id_opitmizer
        
                
        
    # def get_road_to_tls(self):
   
    #     # 读取网络文件,建立lane id到edge id的映射
    #     self.lane_to_edge = {}
        
    #     self.road_to_tls={}
        
    #     # 使用ElementTree解析XML文件
    #     tree = ET.parse(self.net_file)
    #     root = tree.getroot()
        
    #     # 遍历所有edge标签
    #     for edge in root.findall('edge'):
    #         edge_id = edge.get('id')
            
    #         # 遍历edge下的所有lane标签
    #         for lane in edge.findall('lane'):
    #             lane_id = lane.get('id')
    #             # 记录lane id到edge id的映射关系
    #             self.lane_to_edge[lane_id] = edge_id
                
    #     print(f"成功建立 {len(self.lane_to_edge)} 条车道与路段的映射关系")
        
        
    #     for tl_id in self.tls_ids:
    #         if tl_id in self.intersections:
    #             for k in self.intersections[tl_id]['phase_available_lanelinks']:
    #                 for start, end in k:
    #                     edgeiop=self.lane_to_edge[start]
    #                     if edgeiop not in self.road_to_tls:
    #                         self.road_to_tls[edgeiop]=tl_id

    #     print(f"成功建立 {len(self.road_to_tls)} 条道路与交通灯的映射关系")
    
    def _init_traffic_lights(self):
        """初始化交通灯信息"""
        for tl_id in self.tls_ids:
            # 获取所有控制的车道连接

            links = self.conn.trafficlight.getControlledLinks(tl_id)
            if links:
                self.lanelinks[tl_id] = []
                links_dict = {}
                
                # 处理每个link
                for i, link_set in enumerate(links):
                    if link_set:  # 跳过空链接
                        link = link_set[0]  # 获取第一个连接
                        self.lanelinks[tl_id].append((link[0], link[1]))
                        
                        # 记录起始车道到结束车道的映射
                        if link[0] not in links_dict:
                            links_dict[link[0]] = []
                        links_dict[link[0]].append(link[1])
            else: # 如果没有links，则跳过此交通灯的初始化
                print(f"警告: 交通灯 {tl_id} 没有可控制的连接，跳过初始化")
                continue
            
            # 获取交通灯的程序逻辑
            programs = self.conn.trafficlight.getAllProgramLogics(tl_id)
            if programs:
                program = programs[0]  # 取第一个程序
                phases = []
                raw_phase=[]
                # 获取所有绿灯相位
                for i, phase in enumerate(program.phases):
                    phase_state = phase.state
                    phase_time=phase.duration
                    phases.append(phase)
                    raw_phase.append((phase,phase_time))
                self.raw_phase_data[tl_id] = raw_phase

                if phases:
                    self.phase_data[tl_id] = phases
                    self.current_phases[tl_id] = 0
                    self.current_phase_times[tl_id] = 0
                    self.yellow_phase_states[tl_id] = False
                    self.yellow_phase_countdown[tl_id] = 0
                    self.target_green_phases[tl_id] = None
                    
                    # 为每个相位创建可用的车道连接
                    self.intersections[tl_id] = {
                        'phase_available_lanelinks': [],
                        'phase_all_lanelinks': []
                    }
                    phaseid=0
                    for phase in phases:
                        available_links_for_current_phase = []
                        available_links_for_current_phase_all = []
                        for j, state_char in enumerate(phase.state):
                            if state_char.lower() == 'g' or state_char.lower() == 's' or state_char.lower() == 'G':  # 绿灯或直行信号
                                # 确保索引在links范围内并且links[j]非空
                                if j < len(links) and links[j]: 
                                    link_info = links[j][0] # (from_lane, to_lane, via_lane)
                                    available_links_for_current_phase.append((link_info[0], link_info[1]))
                            if j < len(links) and links[j]: 
                                    link_info_ = links[j][0] # (from_lane, to_lane, via_lane)
                                    available_links_for_current_phase_all.append((link_info_[0], link_info_[1]))
                        self.intersections[tl_id]['phase_all_lanelinks'].append((phaseid,available_links_for_current_phase_all))
                        self.intersections[tl_id]['phase_available_lanelinks'].append((phaseid,available_links_for_current_phase))
                        phaseid+=1
   # 移到 if phases 内部
                    # needed_lanes = set()
                    # if tl_id in self.intersections: # 再次确认，虽然理论上此时应该存在
                    #     for phase_links in self.intersections[tl_id]['phase_available_lanelinks']:
                    #         for start, end in phase_links[1]:
                    #             needed_lanes.add(start)
                    #             needed_lanes.add(end)
                    
                else:
                    print(f"警告: 交通灯 {tl_id} 没有有效的绿灯相位，跳过MaxPressure控制")
            else:
                print(f"警告: 交通灯 {tl_id} 没有程序逻辑，跳过初始化")
        
        print(f"成功初始化了 {len(self.intersections)} 个交通灯用于MaxPressure控制")
    


       


    def get_max_pressure_action(self, tl_id):
            """
            使用MaxPressure算法为指定交通灯获取最佳相位
            
            参数:
                tl_id (str): 交通灯ID
                
            返回:
                int: 最佳相位ID
            """
            # 检查最小持续时间
            if self.current_phase_times[tl_id] < self.t_min:
                return self.current_phases[tl_id]
            
            # 获取所有车道的车辆数
            lane_counts = {}
            
            # 只获取相关车道的车辆数
            for phase_links in self.intersections[tl_id]['phase_available_lanelinks']:
                for start_lane, end_lane in phase_links[1]:
                    if start_lane not in lane_counts:
                        lane_counts[start_lane] = self.conn.lane.getLastStepHaltingNumber(start_lane)
                    if end_lane not in lane_counts:
                        lane_counts[end_lane] = self.conn.lane.getLastStepHaltingNumber(end_lane)
            # 获取self.intersections[tl_id]['phase_available_lanelinks']车道的车辆数即可
            
            # for lane_id in self.conn.lane.getIDList():
            #     lane_counts[lane_id] = self.conn.lane.getLastStepVehicleNumber(lane_id)
            
            # 计算每个相位的压力
            max_pressure = None
            best_phase = -1
            
            current_phase=self.current_phases[tl_id]
            current_phase_time=self.current_phase_times[tl_id]
            
            # flag_skip=False
            phaselist=[]
            for links in self.intersections[tl_id]['phase_available_lanelinks']:
                phaselist.append(links[0])
            
            # if current_phase_time>100:
            #     _num=phaselist.index(current_phase)
            #     return phaselist[(_num+1)%len(phaselist)]
            

            # if tl_id=='cluster_588546891_7685535239' and step>75:
            #     a=1
        
            for  links_ in self.intersections[tl_id]['phase_available_lanelinks']:
                # 计算该相位下的总压力
                links=links_[1]
                phase_id=links_[0]
                

                    
                
                phase_pressure = 0
                for start_lane, end_lane in links:
                    # 压力 = 起始车道车辆数 - 结束车道车辆数
                    phase_pressure += lane_counts.get(start_lane, 0) - lane_counts.get(end_lane, 0)
                
                # 更新最大压力相位
                if max_pressure is None or phase_pressure > max_pressure:
                    max_pressure = phase_pressure
                    best_phase = phase_id
            return best_phase
        
    
    
    
    def apply_actions(self, step):
        """为所有交通灯应用控制策略（支持RL和传统方法）"""

        
        # 强化学习模式：按action_interval进行决策
        if self.training and self.controller_manager is not None and hasattr(self, 'rl_tls_ids'):
            return self._apply_rl_actions(step)
        else:
            # 测试模式：每步都决策
            return self._apply_traditional_actions(step)
    
    def _apply_traditional_actions(self, step):
        """传统控制模式（每步决策）"""
        for tl_id in self.tls_ids:
            if self.part_optimize:
                if tl_id not in self.tls_id_opitmizer:
                    continue
           
            if step == 0:
                self.conn.trafficlight.setPhase(tl_id, self.current_phases[tl_id] * 2)
                self.conn.trafficlight.setPhaseDuration(tl_id, 1000)

            if tl_id in self.intersections:
                # 提取当前观测
                observation = self._extract_observation(tl_id)
                
                # 通过控制器管理器获取动作
                if self.controller_manager is not None:
                    target_phase = self.controller_manager.decide_action(tl_id, observation, step)
                else:
                    # 回退到原有贪心逻辑
                    target_phase = self.get_max_pressure_action(tl_id)
                
                # 应用动作约束和执行
                self._apply_action_with_transition(tl_id, target_phase, step)
        
        return step + 1  # 返回下一个步数
    
    
    
    def _apply_rl_actions(self, step):
        """强化学习控制模式（按action_interval决策）"""
        # 检查是否到了决策时刻
        is_decision_step = (step % self.action_interval == 0)
        
        if is_decision_step:
            # === 决策步骤 ===
            
            
            if len(self.rl_tls_ids)!=0:
            # 1. 获取当前状态观测       
                current_observations = self._get_traffic_observations()
                
                # 2. 处理上一轮的经验存储（如果存在）
                if self.total_decision_num > 0 and self.last_observations:
                    self._store_experience_batch(current_observations, step)
            
            
            # 3. 决策新动作
            current_actions = {}
            for tl_id in self.rl_tls_ids:
                if self.total_decision_num > self.learning_start:
                    # 使用学习到的策略
                    action = self.controller_manager.decide_action(tl_id, current_observations.get(tl_id, {}), step)
                else:
                    # 初期随机探索
                    action = self._get_random_action(tl_id)
                current_actions[tl_id] = action
            
            if len(self.rl_tls_ids)!=0:
            # 4. 存储当前状态和动作，准备下次使用
                self.last_observations = current_observations.copy()
                self.last_actions = current_actions.copy()
                self.last_decision_step = step
            
            # 5. 清空奖励累积器，准备收集新的奖励
            self.reward_accumulator.clear()
            
            # 6. 应用动作到所有交通灯
            self._apply_actions_to_traffic_lights(step, current_actions)
            
            # 7. 学习更新检查
            self._check_and_update_models()
            
            self.total_decision_num += 1
        
        else:
            # === 非决策步骤 ===
            # 继续使用上次的动作，只是正常仿真和收集奖励
            self._apply_actions_to_traffic_lights(step, self.last_actions if hasattr(self, 'last_actions') else {})
        
        self.conn.simulation.step()
   
        # 每步跟踪车辆进出情况
        self.track_vehicle_movements()
        self.arrival_carnum+=len(self.conn.simulation.getArrivedIDList())
        
        # 收集即时奖励（每步都收集）
        if self.training:
            if step % self.cal_reward_circle == 0:
                step_rewards = self._calculate_rewards()
                for tl_id, reward in step_rewards.items():
                    self.reward_accumulator[tl_id].append(reward)
        
        return step + 1  # 返回下一个步数
    
    def _apply_actions_to_traffic_lights(self, step, actions_dict):
        """将动作应用到交通灯"""
        for tl_id in self.tls_ids:
            if tl_id in self.intersections:
               
                # 获取目标相位
                if tl_id in actions_dict:
                    # 使用RL决策的动作
                    target_phase = actions_dict[tl_id]
                elif self.non_rl_policy == 'greedy':
                    # 回退到贪心逻辑
                    target_phase = self.get_max_pressure_action(tl_id)
                elif self.non_rl_policy == 'static':
                    target_phase = self.controller_manager.controllers[tl_id].decide_action(tl_id, {}, step)
                
                if step % 5==0:
                    self.calculate_pressure(tl_id,step)
                try:
                    self._apply_action_with_transition(tl_id, target_phase, step)
                    # 应用动作约束和执行
                except Exception as e:
                    print(f"警告: 交通灯 {tl_id} 应用动作约束和执行时出现错误: {e}")
                    pass
                
                
    
    def _store_experience_batch(self, current_observations, current_step):
        """批量存储经验到经验回放缓冲区"""
        if not self.reward_accumulator:
            return
        
        # 计算平均延迟奖励
        avg_rewards = {}
        for tl_id in self.rl_tls_ids:
            if tl_id in self.reward_accumulator:
                rewards_list = self.reward_accumulator[tl_id]
                avg_rewards[tl_id] = np.mean(rewards_list) if rewards_list else 0.0
            else:
                avg_rewards[tl_id] = 0.0
        
        # print(f"平均延迟奖励: {avg_rewards}")
        
        # 存储经验
        for tl_id in self.rl_tls_ids:
            if tl_id in self.last_observations and tl_id in self.last_actions:
                transition = {
                    'state': self.last_observations[tl_id],
                    'action': self.last_actions[tl_id],
                    'reward': avg_rewards[tl_id],
                    'next_state': current_observations.get(tl_id, {}),
                    'done': current_step >= self.end_time
                }
                
                # 存储到经验回放缓冲区 - 通过控制器管理器的controllers字典访问
                if tl_id in self.controller_manager.controllers:
                    controller = self.controller_manager.controllers[tl_id]
                    controller.on_step_end(tl_id, transition)
    
    def _check_and_update_models(self):
        """检查并执行模型更新"""
        if self.total_decision_num < 100:
            return
        
        # 模型学习更新
        if self.total_decision_num % self.update_model_rate == 0:
            print(f"执行模型学习更新 (决策次数: {self.total_decision_num})")
            for tl_id in self.rl_tls_ids:
                if tl_id in self.controller_manager.controllers:
                    controller = self.controller_manager.controllers[tl_id]
                    if hasattr(controller, 'agent'):
                        metrics = controller.agent.learn()
                        if metrics:
                            print(f"TL {tl_id} 学习指标: {metrics}")
        
        # 目标网络更新
        if self.total_decision_num % self.update_target_rate == 0:
            print(f"执行目标网络更新 (决策次数: {self.total_decision_num})")
            for tl_id in self.rl_tls_ids:
                if tl_id in self.controller_manager.controllers:
                    controller = self.controller_manager.controllers[tl_id]
                    if hasattr(controller, 'agent') and hasattr(controller.agent, 'update_target_network'):
                        controller.agent.update_target_network()
                        print(f"TL {tl_id} 目标网络已更新")
    
    def _extract_observation(self, tl_id: str) -> Dict:
        """提取路口观测信息"""
        observation = {
            'current_phase': self.current_phases.get(tl_id, 0),
            'current_phase_time': self.current_phase_times.get(tl_id, 0),
            'timestep': 0  # 将在调用时设置
        }
        return observation
    
    def _apply_action_with_transition(self, tl_id: str, target_phase: int, step: int):
        """应用动作并处理相位转换"""
        if tl_id not in self.rl_tls_ids and  self.current_phase_times[tl_id] < self.t_min:
            self.current_phase_times[tl_id] += 1
            return

            
        if tl_id in self.rl_tls_ids or self.current_phase_times[tl_id] >= self.t_min:
            current_phase = self.current_phases[tl_id]
            
            if target_phase != current_phase:
                # 相位变化，执行切换
                phase_index = target_phase
                
                if phase_index != -1:
                    self.conn.trafficlight.setPhase(tl_id, phase_index)
                    self.conn.trafficlight.setPhaseDuration(tl_id, 3000)
                    self.current_phases[tl_id] = phase_index
                    self.current_phase_times[tl_id] = 0
                else:
                    print(f"警告: 交通灯 {tl_id} 的相位索引 {target_phase} 超出范围")
            else:
                self.current_phase_times[tl_id] += 1
        return 
               
    def collect_metrics(self):
        """收集和返回当前时间步的指标"""
        current_time = self.conn.simulation.getTime()
        
        # 计算平均旅行时间（基于已完成的旅行）
        mean_travel_time = 0
        if self.completed_trips:
            mean_travel_time = sum(self.completed_trips.values()) / len(self.completed_trips)
        
        metrics = {
            'time': current_time,
            'vehicles': self.conn.vehicle.getIDCount(),
            'waiting_time': 0,
            'queue_length': 0,
            'mean_speed': 0,
            'mean_travel_time': mean_travel_time,
            'arrival_carnum': self.arrival_carnum
        }
        
        # 计算总等待时间
        total_waiting = 0
        vehicle_ids = self.conn.vehicle.getIDList()
        for veh_id in vehicle_ids:
            total_waiting += self.conn.vehicle.getWaitingTime(veh_id)
        metrics['waiting_time'] = total_waiting
        
        # 计算总排队长度
        total_halting = 0
        for lane_id in self.conn.lane.getIDList():
            total_halting += self.conn.lane.getLastStepHaltingNumber(lane_id)
        metrics['queue_length'] = total_halting
        
        # 计算平均速度
        if vehicle_ids:
            total_speed = 0
            for veh_id in vehicle_ids:
                total_speed += self.conn.vehicle.getSpeed(veh_id)
            metrics['mean_speed'] = total_speed / len(vehicle_ids) if vehicle_ids else 0
        
        return metrics
    
    def track_vehicle_movements(self):
        """每个仿真步骤调用，跟踪车辆进出"""
        current_time = self.conn.simulation.getTime()
        
        # 跟踪新进入的车辆
        entering_vehicles = self.conn.simulation.getDepartedIDList()
        for vehicle_id in entering_vehicles:
            if vehicle_id =='sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssst4387.0':
                a=1
            self.inside_vehicles[vehicle_id] = current_time
        
        # 跟踪离开的车辆并计算旅行时间 
        exiting_vehicles = self.conn.simulation.getArrivedIDList()
        for vehicle_id in exiting_vehicles:
            if vehicle_id in self.inside_vehicles:
                travel_time = current_time - self.inside_vehicles[vehicle_id]
                self.completed_trips[vehicle_id] = travel_time
                # 从当前车辆列表中移除
                del self.inside_vehicles[vehicle_id]
    
    def analyze_phase_sequence(self,phase_sequence):
        
        """
        分析相位序列,统计非黄灯相位的出现频次和平均显示时间
        
        Args:
            phase_sequence: 字典,key为信号灯id,value为相位序列列表
            
        Returns:
            phase_stats: 字典,key为信号灯id,value为该信号灯各相位的统计信息
        """
        phase_stats = {}
        
        for tls_id, phases in phase_sequence.items():
            phase_stats[tls_id] = {}
            
            # 记录当前正在统计的相位信息
            current_phase = None
            current_duration = 0
            
            # 用于存储每个相位的所有持续时间
            phase_durations = {}
            phase_counts = {}
            
            for phase, is_yellow in phases:
                if is_yellow == 'noty':  # 只统计非黄灯相位
                    if current_phase is None:  # 第一个相位
                        current_phase = phase
                        current_duration = 1
                    elif phase == current_phase:  # 相同相位继续计数
                        current_duration += 1
                    else:  # 相位改变,保存前一个相位的统计信息
                        if current_phase not in phase_durations:
                            phase_durations[current_phase] = []
                            phase_counts[current_phase] = 0
                        phase_durations[current_phase].append(current_duration)
                        phase_counts[current_phase] += 1
                        
                        # 重置为新相位
                        current_phase = phase
                        current_duration = 1
                        
            # 处理最后一个相位
            if current_phase is not None:
                if current_phase not in phase_durations:
                    phase_durations[current_phase] = []
                    phase_counts[current_phase] = 0
                phase_durations[current_phase].append(current_duration)
                phase_counts[current_phase] += 1
                
            # 计算每个相位的平均持续时间
            for phase in phase_durations:
                avg_duration = sum(phase_durations[phase]) / len(phase_durations[phase])
                phase_stats[tls_id][phase] = {
                    'frequency': phase_counts[phase],
                    'avg_duration': avg_duration
                }
                
        return phase_stats
   
    def generate_static_program(self):
        phase_stats=self.analyze_phase_sequence(self.tls_phase_history)

        # 存下来phase_stats为pkl文件
        with open('pkl/tls_phase_history.pkl', 'wb') as f:
            pickle.dump(self.tls_phase_history, f)
        
        programs={}
        for tls_id, phases in phase_stats.items():
            

          

            phase_setting=[]
            for i in phase_stats[tls_id].keys():
                iop={}
                iop['duration']=phase_stats[tls_id][i]['avg_duration']
                iop['state']=i
                phase_setting.append(iop.copy())
                phase_setting.append({'duration':5,'state':i+1})
                
                
                
            programs[tls_id]=phase_setting.copy()
        
        # 存programs为pkl文件
        with open('pkl/programs.pkl', 'wb') as f:
            pickle.dump(programs, f)
        return programs
    # def generate_signal_program(self, programs):
    #     """生成SUMO格式的信号灯配置文件"""
    #     tls_file = os.path.join(self.output_dir, "static_program.add.xml")
        
    #     with open(tls_file, 'w') as f:
    #         f.write('<additional>\n')
            
    #         # 为每个路口生成配置
    #         for tls_id, tls_programs in programs.items():
    #             f.write(f'  <tlLogic id="{tls_id}" type="static" programID="static_program" offset="0">\n')
                
    #             # 为每个时段创建不同的program
    #             for period, program in tls_programs.items():
    #                 for phase_name in program["phase_order"]:
    #                     duration = program["phase_timings"][phase_name]["duration"]
    #                     # 根据相位名称生成对应的信号状态
    #                     if phase_name == "NS_straight":
    #                         state = "GGrrrrGGrrrr"
    #                     elif phase_name == "NS_left":
    #                         state = "rrGGrrrrGGrr"
    #                     elif phase_name == "EW_straight":
    #                         state = "rrrrGGrrrrGG"
    #                     else:  # EW_left
    #                         state = "rrrrrrGGrrGG"
                        
    #                     f.write(f'    <phase duration="{duration}" state="{state}" minDur="5" maxDur="60"/>\n')
                
    #             f.write('  </tlLogic>\n')
    #         f.write('</additional>\n')
        
    #     return tls_file
    

    
    
    def run(self, need_add=[]):
        rewar_all=[]
        """多回合训练循环"""
        print(f"开始多回合RL训练，共 {self.num_episodes} 回合")
        print(f"RL模式: {self.rl_mode}")
        
        all_metrics =[]
        all_metrics_new=[]
        training_stats = {
            'episode_rewards': [],
            'episode_metrics': [],
            'learning_curves': defaultdict(list)
        }
        for episode in range(self.num_episodes):
            self.n_episodes=episode
            print(f"\n=== 第 {episode + 1}/{self.num_episodes} 回合 ===")
            if episode!=0:
            # 重新初始化SUMO连接
                self._init_connection_train()
            
            # 运行单回合
            episode_rewards,metrics_history,metrics_history_all = self._run_single_episode_training(episode, need_add)
            all_metrics.append(metrics_history)
      
            # 记录统计信息
            training_stats['episode_rewards'].append(episode_rewards)
            training_stats['episode_metrics'].append(metrics_history)
            # 保存检查点
            if (episode + 1) % 10 == 0 and self.controller_manager is not None:
                self.controller_manager.save_checkpoint(episode)
                print(f"已保存第 {episode} 回合的检查点")
              
            
            if metrics_history_all:
                all_metrics_new.append(metrics_history_all)
            # 打印回合统计
            if episode_rewards:
                avg_rewards = {tl_id: np.mean(rewards) for tl_id, rewards in episode_rewards.items()}
                rewar_all.append(episode_rewards)
        
        
        tl_phase_result, lane_averages=self.compute_pressure()
        print(all_metrics_new)
        with open(self.reward_static_pkl, 'wb') as f:
            pickle.dump(all_metrics_new, f)
       
        # with open('pkl/all_metrics_3.pkl', 'wb') as f:
        #     pickle.dump(all_metrics, f)
        return tl_phase_result, lane_averages
    
    def _run_single_episode_training(self, episode_id: int, need_add=[]):
        """单回合训练仿真"""
        # 重置训练相关变量
    
    
        # 初始化训练相关变量（如果不存在）
        if not hasattr(self, 'total_decision_num'):
            self.total_decision_num = 0
            self.action_interval = getattr(self, 'action_interval', self.action_interval_)
            self.learning_start = getattr(self, 'learning_start', self.learning_start_time)
            self.update_model_rate = getattr(self, 'update_model_rate', self.update_model_rate_)
            self.update_target_rate = getattr(self, 'update_target_rate', self.update_target_rate_)
            self.last_observations = {}
            self.last_actions = {}
            self.reward_accumulator = defaultdict(list)  # 累积奖励
            self.last_decision_step = -1  # 上次决策的步数
            
        step = 0
        episode_rewards = defaultdict(list)
        metrics_history = []
        

        
        while step < self.end_time:
            if step % 100 == 0:
                print(f"第 {step} 步")
  
            # 应用控制策略（新版本会返回更新后的step）
            step = self.apply_actions(step)
            if step % 100 == 0:
                metrics = self.collect_metrics()
                metrics_history.append(metrics)
          
        
        self.conn.close()
        
        # 收集最终的episode奖励
        if hasattr(self, 'reward_accumulator') and self.reward_accumulator:
            for tl_id, rewards in self.reward_accumulator.items():
                if rewards:
                    episode_rewards[tl_id] = rewards.copy()
        
        # 计算metrics_history中所有时间点的各项指标的平均值
        if metrics_history:
            avg_metrics = {}
            keys = metrics_history[0].keys()
            for key in keys:
                avg_metrics[key] = np.mean([m[key] for m in metrics_history])
        else:
            avg_metrics = {}

        return episode_rewards, avg_metrics,metrics_history
    
    def _get_traffic_observations(self) -> Dict[str, Dict]:
        """获取交通状态观测"""
        observations = {}
        
    
        for tl_id in self.rl_tls_ids:
            obs=self.controller_manager.state_extractor.extract_observation(tl_id,self.current_phases,self.current_phase_times)
            
            observations[tl_id] = obs
                
            
        return observations
    
    def _get_random_action(self, tl_id: str) -> int:
        """获取随机动作（用于初期探索）"""
        try:
            # 获取该路口的可用相位数量
            logic = self.phase_data[tl_id]
            num_phases = len(logic)
            return np.random.randint(0, num_phases)
        except:
            # 默认返回4个相位中的随机一个
            return np.random.randint(0, 4)
    
    def _calculate_rewards(self) -> Dict[str, float]:
        """计算即时奖励"""
        rewards = {}
        
        try:
            for tl_id in self.tls_ids:
              
            
                # 获取路口连接的车道
                controlled_lanes = self.conn.trafficlight.getControlledLanes(tl_id)
                
                if controlled_lanes:
                    # 计算奖励：基于等待时间、车辆数量等
                    total_waiting = 0
                    total_vehicles = 0
                    total_speed = 0
                    
                    for lane_id in controlled_lanes:
                        # 等待车辆数量（负奖励）
                        waiting_count = self.conn.lane.getLastStepHaltingNumber(lane_id)
                        total_waiting += waiting_count
                        
                        # # 车辆数量
                        # vehicle_count = self.conn.lane.getLastStepVehicleNumber(lane_id)
                        # total_vehicles += vehicle_count
                        
                        # # 平均速度（正奖励）
                        # if vehicle_count > 0:
                        #     avg_speed = self.conn.lane.getLastStepMeanSpeed(lane_id)
                        #     total_speed += avg_speed
                    
                    # 计算综合奖励
                    # 奖励 = 速度奖励 - 等待惩罚 - 拥堵惩罚
                    # speed_reward = total_speed / max(len(controlled_lanes), 1)  # 平均速度
                    waiting_penalty = total_waiting   # 等待惩罚
                    # congestion_penalty = max(0, total_vehicles - len(controlled_lanes) * 3) # 拥堵惩罚
                    
                    reward =  -waiting_penalty
                    rewards[tl_id] = reward
                else:
                    rewards[tl_id] = 0.0
                    
        except Exception as e:
            
            print(f"计算奖励时出错: {str(e)}")
            # 返回零奖励
            for tl_id in self.rl_tls_ids:
                rewards[tl_id] = 0.0
        
        return rewards
    
    def _reset_sumo_connection(self):
        """重置SUMO连接"""
        try:
            # 关闭现有连接
            if hasattr(self, 'conn'):
                self.conn.close()
        except:
            pass
        # 重新初始化连接
        self._init_connection()

    def modify_tls_durations(self, net_file, tlc_dict, output_file):
        """修改net.xml文件中的信号灯配时方案
        
        参数:
            net_file (str): 输入的net.xml文件路径
            tlc_dict (dict): 信号灯配时字典,格式为 {tls_id: [{duration, state}, ...]}
            output_file (str): 输出的net.xml文件路径
        """
        # 解析XML文件
        tree = ET.parse(net_file)
        root = tree.getroot()
        
        # 遍历所有tlLogic节点
        for tlLogic in root.findall('.//tlLogic'):
            tls_id = tlLogic.get('id')
            
            # 如果该信号灯在配时字典中
            if tls_id in tlc_dict:
                # 删除原有的phase节点
                for phase in tlLogic.findall('phase'):
                    tlLogic.remove(phase)
                    
                # 按state排序phase_config列表
                sorted_phase_configs = sorted(tlc_dict[tls_id], key=lambda x: x['state'])
                
                # 添加新的phase节点
                for phase_config in sorted_phase_configs:
                    # 获取原始state字符串
                    print(self.phase_data[tls_id])
                    if phase_config['state']%2==1:
                        # 黄灯
                        original_state = self.phase_data[tls_id][int((phase_config['state']-1)/2)].state
                                # 创建新的phase节点
                        original_state=original_state.replace('g','y')
                        phase = ET.SubElement(tlLogic, 'phase')
                        phase.set('duration', str(phase_config['duration']))
                        phase.set('state', original_state)
                    else:
                        # 绿灯
                        original_state = self.phase_data[tls_id][int(phase_config['state']/2)].state
                        phase = ET.SubElement(tlLogic, 'phase')
                        duration = max(10, min(100, phase_config['duration']))  # 限制在10-100秒之间
                        phase.set('duration', str(duration))
                        phase.set('state', original_state)
    
        
        # 保存修改后的文件
        tree.write(output_file, encoding='utf-8', xml_declaration=True)
        print(f"已将修改后的信号灯配时方案保存至: {output_file}")



    def run_static(self):
            """运行仿真直到结束时间"""
            print(f"开始运行LTF仿真，使用MaxPressure算法优化交通信号...")
            print(f"网络文件: {self.net_file}")
            print(f"路由文件: {self.route_file}")
            print(f"结束时间: {self.end_time}秒")
            print(f"GUI模式: {'启用' if self.use_gui else '禁用'}")
            
            sumo_binary = self.sumo_binary 
            
            self.modify_tls_durations(self.net_file, self.tls_static_programs, self.net_file_staticTlc)
                    
            # 构建SUMO命令
            sumocmd = [
                sumo_binary,
                "-n", self.net_file_staticTlc,
                "-r", self.route_file,
                "--no-step-log", "true",
                "--seed", str(self.seed),
                "--delay", "0",
                "--time-to-teleport", "-1",
                "--no-warnings", "true"
            ]
            
            # 如果使用GUI，添加GUI相关参数
            if self.use_gui:
                sumocmd.extend([
                    "--start", "true",  # 自动开始仿真
                    "--quit-on-end", "true"  # 仿真结束时自动关闭
                ])
    
            
            # 记录命令
            print(f"启动SUMO命令: {' '.join(sumocmd)}")
            
            # 启动traci连接
            traci.start(sumocmd)
            self.conn = traci
                
            # 获取交通灯列表
            self.tls_ids = self.conn.trafficlight.getIDList()
            print(f"找到 {len(self.tls_ids)} 个交通灯")
            
            # 初始化交通灯信息
            self._init_traffic_lights()
            
            # self.get_road_to_tls()
            self.get_tls_id_opitmizer()
            
            step = 0
            metrics_history = []
            start_time = time.time()
            road_ids = traci.edge.getIDList()
            # 过滤掉首字符为':'的road_id
            road_ids = [road_id for road_id in road_ids if not road_id.startswith(':')]
            road_speeds = {road_id:[] for road_id in road_ids}
            road_waiting_vehicles = {road_id:[] for road_id in road_ids}
            road_vehicles = {}
            # 记录每个红绿灯在每个时间步的相位选择

            try:
                while step < self.end_time:
            
        
                    if step >= self.end_time-10:
                        for road_id in road_ids:
                            road_speeds[road_id].append(traci.edge.getLastStepMeanSpeed(road_id))

                            road_waiting_vehicles[road_id].append(traci.edge.getLastStepHaltingNumber(road_id))
                

        
                    # 应用MaxPressure控制策略（每步都执行）
            

                    # 收集指标（可以改为每10步收集一次）
                    if step % 100 == 0:
                        metrics = self.collect_metrics()
                        metrics_history.append(metrics)
                    
                    # 仿真前进一步
                    self.conn.simulation.step()
                    step += 1
                average_speeds = {}
                average_waiting_vehicles = {}
                        # 
                for road_id in road_ids:

                    # 获取当前道路上的所有车辆ID
                    vehicle_ids = traci.edge.getLastStepVehicleIDs(road_id)
                    road_vehicles[road_id] = vehicle_ids
                    
                for road_id in road_ids:
                    average_speeds[road_id] = np.max(road_speeds[road_id])
                    average_waiting_vehicles[road_id] = np.min(road_waiting_vehicles[road_id])
            except Exception as e:
                print(f"仿真过程中出现错误: {str(e)}")
            finally:
                # 关闭连接
                try:
                    self.conn.close()
                    print("已关闭traci连接")
                except:
                    pass
            
            total_time = time.time() - start_time
            print(f"仿真完成！总用时: {total_time:.2f}秒")

            return metrics_history,average_speeds,road_vehicles,average_waiting_vehicles    

    @staticmethod
    def compute_summary(metrics_history):
        """计算并打印指标摘要"""
        if not metrics_history:
            print("没有收集到指标数据")
            return
        
        # 计算平均值
        avg_vehicles = np.mean([m['vehicles'] for m in metrics_history])
        avg_waiting_time = np.mean([m['waiting_time'] for m in metrics_history])
        avg_queue_length = np.mean([m['queue_length'] for m in metrics_history])
        avg_speed = np.mean([m.get('mean_speed', 0) for m in metrics_history])
        
        # 计算最大值
        max_vehicles = max([m['vehicles'] for m in metrics_history])
        max_waiting = max([m['waiting_time'] for m in metrics_history])
        max_queue = max([m['queue_length'] for m in metrics_history])
        
        print("\n===== 性能指标摘要 =====")
        print(f"平均车辆数: {avg_vehicles:.2f} (最大: {max_vehicles})")
        print(f"平均等待时间: {avg_waiting_time:.2f} (最大: {max_waiting:.2f})")
        print(f"平均排队长度: {avg_queue_length:.2f} (最大: {max_queue})")
        print(f"平均车速: {avg_speed:.2f} m/s")
        print("========================")
        
        return {
            "avg_vehicles": avg_vehicles,
            "avg_waiting_time": avg_waiting_time,
            "avg_queue_length": avg_queue_length,
            "avg_speed": avg_speed,
            "max_vehicles": max_vehicles,
            "max_waiting": max_waiting,
            "max_queue": max_queue
        }

def run_ltf_traci(net_file, route_file, use_gui=True, 
                  end_time=3600, yellow_time=5, t_min=20, seed=42):
    """
    运行LTF_Traci仿真的便捷函数
    
    参数:
        net_file (str): SUMO网络文件路径(.net.xml)
        route_file (str): SUMO路由文件路径(.rou.xml)
        use_gui (bool): 是否使用SUMO GUI
        end_time (int): 仿真结束时间(秒)
        yellow_time (int): 黄灯持续时间(秒)
        t_min (int): 一个相位的最小持续时间(秒)
        seed (int): 随机种子
    
    返回:
        dict: 性能指标摘要
    """
    ltf = LTF_Traci(net_file, route_file, use_gui, end_time, yellow_time, t_min, seed)
    metrics_history,average_speeds,_ = ltf.run()
    stats = ltf.compute_summary(metrics_history)
    return stats,average_speeds







