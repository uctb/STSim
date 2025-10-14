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

import shutil






class RegionMerger:
    def __init__(self, shp_file_path, signal,third_adjust_firstSTEP):
        self.shp_file_path = shp_file_path
        self.signal = signal
        self.rows_per_region = 2
        self.cols_per_region = 2
        self.third_adjust_firstSTEP=third_adjust_firstSTEP
        
    def get_polygon_bounds(self):
        data = gpd.read_file(self.shp_file_path)
        result = []
        for index, row in data.iterrows():
            geometry = row.geometry
            exterior = geometry.exterior
            bounds_points = [(coord[0], coord[1]) for coord in exterior.coords]
            result.append(bounds_points)
        return result

    def get_polygon_centers(self, polygon_bounds):
        polygon_centers = []
        for poly in polygon_bounds:
            x_coords = [p[0] for p in poly]
            y_coords = [p[1] for p in poly]
            center_x = sum(x_coords) / len(x_coords)
            center_y = sum(y_coords) / len(y_coords)
            polygon_centers.append((center_x, center_y))
        return polygon_centers

    def get_grid_dimensions(self, polygon_centers):
        # 找出所有具有相似x坐标的点来确定列数
        x_coords = [center[0] for center in polygon_centers]
        x_coords.sort()
        x_unique = []
        for x in x_coords:
            if not x_unique or abs(x - x_unique[-1]) > 0.0001:  # 容差
                x_unique.append(x)
        num_cols = len(x_unique)

        # 找出所有具有相似y坐标的点来确定行数
        y_coords = [center[1] for center in polygon_centers]
        y_coords.sort(reverse=True)  # 从北到南排序
        y_unique = []
        for y in y_coords:
            if not y_unique or abs(y - y_unique[-1]) > 0.0001:  # 容差
                y_unique.append(y)
        num_rows = len(y_unique)

        return num_rows, num_cols, x_unique, y_unique

    def create_grid(self, polygon_centers, num_rows, num_cols, x_unique, y_unique):
        grid = [[None for _ in range(num_cols)] for _ in range(num_rows)]
        for i, (x, y) in enumerate(polygon_centers):
            col_idx = min(range(len(x_unique)), key=lambda j: abs(x - x_unique[j]))
            row_idx = min(range(len(y_unique)), key=lambda j: abs(y - y_unique[j]))
            grid[row_idx][col_idx] = i
        return grid

    def merge_regions(self):
        if os.path.exists('pkl/region_mapping_{}.pkl'.format(self.signal)) and self.third_adjust_firstSTEP==False:
            region_mapping = loadfile('pkl/region_mapping_{}.pkl'.format(self.signal))
            return region_mapping
        # 获取多边形边界点
        polygon_bounds = self.get_polygon_bounds()
        
        # 获取多边形中心点
        polygon_centers = self.get_polygon_centers(polygon_bounds)
        
        # 获取网格维度
        num_rows, num_cols, x_unique, y_unique = self.get_grid_dimensions(polygon_centers)
        print(f"检测到栅格排列为 {num_rows} 行 x {num_cols} 列")
     
        # 创建网格
        grid = self.create_grid(polygon_centers, num_rows, num_cols, x_unique, y_unique)

        # 计算可以完整合并的区域数量
        complete_regions_rows = num_rows // self.rows_per_region
        complete_regions_cols = num_cols // self.cols_per_region
        remaining_rows = num_rows % self.rows_per_region
        remaining_cols = num_cols % self.cols_per_region

        # 创建映射字典
        region_mapping = {}
        old_to_new_mapping = {}
        region_idx = 0

        # 处理完整的区域
        for r in range(complete_regions_rows):
            for c in range(complete_regions_cols):
                region_mapping[region_idx] = []
                for i in range(self.rows_per_region):
                    for j in range(self.cols_per_region):
                        grid_idx = grid[r*self.rows_per_region + i][c*self.cols_per_region + j]
                        if grid_idx is not None:
                            region_mapping[region_idx].append(grid_idx)
                            old_to_new_mapping[grid_idx] = region_idx
                region_idx += 1

        # 处理剩余的行
        if remaining_rows > 0:
            for c in range(complete_regions_cols):
                region_mapping[region_idx] = []
                for i in range(remaining_rows):
                    for j in range(self.cols_per_region):
                        row_idx = complete_regions_rows * self.rows_per_region + i
                        if row_idx < num_rows and c*self.cols_per_region + j < num_cols:
                            grid_idx = grid[row_idx][c*self.cols_per_region + j]
                            if grid_idx is not None:
                                region_mapping[region_idx].append(grid_idx)
                                old_to_new_mapping[grid_idx] = region_idx
                region_idx += 1

        # 处理剩余的列
        if remaining_cols > 0:
            for r in range(complete_regions_rows):
                region_mapping[region_idx] = []
                for i in range(self.rows_per_region):
                    for j in range(remaining_cols):
                        col_idx = complete_regions_cols * self.cols_per_region + j
                        if r*self.rows_per_region + i < num_rows and col_idx < num_cols:
                            grid_idx = grid[r*self.rows_per_region + i][col_idx]
                            if grid_idx is not None:
                                region_mapping[region_idx].append(grid_idx)
                                old_to_new_mapping[grid_idx] = region_idx
                region_idx += 1

        # 处理右下角剩余区域
        if remaining_rows > 0 and remaining_cols > 0:
            region_mapping[region_idx] = []
            for i in range(remaining_rows):
                for j in range(remaining_cols):
                    row_idx = complete_regions_rows * self.rows_per_region + i
                    col_idx = complete_regions_cols * self.cols_per_region + j
                    if row_idx < num_rows and col_idx < num_cols:
                        grid_idx = grid[row_idx][col_idx]
                        if grid_idx is not None:
                            region_mapping[region_idx].append(grid_idx)
                            old_to_new_mapping[grid_idx] = region_idx
            region_idx += 1

        print(f"合并完成，从{len(polygon_bounds)}个方块合并为{len(region_mapping)}个区域")
        print(f"区域数量: {len(region_mapping)}")
        print(len(polygon_bounds))
        print(region_mapping)
        dumpfile(region_mapping,'pkl/region_mapping_{}.pkl'.format(self.signal))
        
        
        
        
        
                
                
                # 为每个合并区域计算外接矩形
        region_rectangles = {}
        for new_idx, old_indices in region_mapping.items():
            # 收集该区域内所有多边形的所有点
            all_points = []
            for old_idx in old_indices:
                all_points.extend(polygon_bounds[old_idx])
            
            # 计算最小和最大坐标以形成外接矩形
            x_coords = [p[0] for p in all_points]
            y_coords = [p[1] for p in all_points]
            
            min_x = min(x_coords)
            max_x = max(x_coords)
            min_y = min(y_coords)
            max_y = max(y_coords)
            
            # 创建矩形的四个角点
            rectangle = [
                (min_x, min_y),
                (max_x, min_y),
                (max_x, max_y),
                (min_x, max_y),
                (min_x, min_y)  # 闭合多边形
            ]
            
            region_rectangles[new_idx] = rectangle

        # 将region_rectangles存为pkl
        with open('pkl/merge_rectangles_location.pkl', 'wb') as f:
            pickle.dump(region_rectangles, f)
        
        return region_mapping


### process_rou_file


class Generate_NewOD:
    def __init__(self, rou_file, grid_shp,third_adjust_firstSTEP,od_pkl):
        """
        初始化Generate_NewOD类
        
        参数:
        rou_file: 路由文件路径
        grid_shp: 网格shapefile路径
        raw_od_matrix_file: 原始OD矩阵文件路径
        """
        self.rou_file = rou_file
        self.grid_shp = grid_shp
        self.signal = 'big'
        self.third_adjust_firstSTEP=third_adjust_firstSTEP
        self.od_pkl=loadfile(od_pkl)
       

    def create_new_od_dict(self, old_od_dict, region_mapping):
        """
        根据区域映射关系创建新的OD字典
        """
        new_od_dict = {}
        
        # 创建反向映射，从旧区域ID到新区域ID
        old_to_new_mapping = {}
        for new_region, old_regions in region_mapping.items():
            for old_region in old_regions:
                old_to_new_mapping[old_region] = new_region
        
        # 遍历原始OD字典
        for (old_from, old_to), vehicles in old_od_dict.items():
            # 获取对应的新区域ID
            if old_from in old_to_new_mapping and old_to in old_to_new_mapping:
                new_from = old_to_new_mapping[old_from]
                new_to = old_to_new_mapping[old_to]
                
                # 将车辆添加到新的OD对中
                if (new_from, new_to) not in new_od_dict:
                    new_od_dict[(new_from, new_to)] = []
                
                new_od_dict[(new_from, new_to)].extend(vehicles)
        
        return new_od_dict

    def create_new_od_matrix(self, dataOD, region_mapping):
        """
        根据区域映射创建新的OD矩阵
        """
        # 获取新区域的数量
        num_new_regions = len(region_mapping)

        # 创建一个新的OD矩阵，初始化为0
        new_od_matrix = np.zeros((num_new_regions, num_new_regions))

        # 遍历每个新区域作为起点
        for new_origin_idx, old_origin_indices in region_mapping.items():
            # 遍历每个新区域作为终点
            for new_dest_idx, old_dest_indices in region_mapping.items():
                # 对于每对新区域，累加原始OD矩阵中对应的所有值
                for old_origin in old_origin_indices:
                    for old_dest in old_dest_indices:
                        # 确保索引在原始OD矩阵范围内
                        if old_origin < len(dataOD) and old_dest < len(dataOD[0]):
                            new_od_matrix[new_origin_idx][new_dest_idx] += dataOD[old_origin][old_dest]
        return new_od_matrix

    def process_rou_file(self):
        """
        处理路由文件，生成OD字典和矩阵
        """
        # 解析XML文件
        tree = ET.parse(self.rou_file)
        root = tree.getroot()

        # 初始化OD字典和TAZ集合
        od_dict = defaultdict(list)
        od_matrix = np.zeros((len(self.od_pkl),len(self.od_pkl)))
        count = 0

        # 遍历每个vehicle元素
        for vehicle in root.findall('vehicle'):
            count += 1
            vid = vehicle.get('id')
            from_taz = int(vehicle.get('fromTaz'))
            to_taz = int(vehicle.get('toTaz'))
            
            # print(from_taz,to_taz)
            
            # 更新OD字典
            od_pair = (from_taz, to_taz)
            od_dict[od_pair].append(vid)
            od_matrix[int(from_taz)][int(to_taz)] += 1
        print(f"处理的车辆总数: {count}")

        return od_dict, od_matrix

    def generate(self,needmerge=True):
        """
        生成新的OD矩阵和OD字典
        """
        # 处理路由文件
        od_dict, dataOD = self.process_rou_file()
        
        if needmerge==False:
            return od_dict,dataOD,len(dataOD)
        print(f"原始OD字典中的OD对数量: {len(od_dict.keys())}")

        # 合并区域
        merger = RegionMerger(self.grid_shp, self.signal,self.third_adjust_firstSTEP)
        region_mapping = merger.merge_regions()
        import pdb
        pdb.set_trace()
        print(region_mapping)
  
        # 创建新的OD矩阵
        new_od_matrix = self.create_new_od_matrix(dataOD, region_mapping)
        
        # 将对角线元素置为0
        # np.fill_diagonal(new_od_matrix, 0)
        print(f"新OD矩阵总流量: {np.sum(new_od_matrix)}")


        # 保存新的OD矩阵
        np.save(f'npy/merged_od_matrix_{self.signal}.npy', new_od_matrix)

        # 创建新的OD字典
        new_od_dict = self.create_new_od_dict(od_dict, region_mapping)
        print(f"新OD字典中的OD对数量: {len(new_od_dict)}")

        # 保存新的OD字典
        with open(f'pkl/mergeod2trip_dict_{self.signal}.pkl', 'wb') as f:
            pickle.dump(new_od_dict, f)
        return new_od_dict,new_od_matrix,len(region_mapping)





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
                 end_time=3600, yellow_time=3, t_min=10, seed=42,need_optimize_tlds=[],given_tlds_type='road',part_optimize=False,sumobinary='sumo'):
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
        
        # 初始化SUMO网络
        self.net = sumolib.net.readNet(net_file)
        
        # 存储交叉路口信息
        self.intersections = {}
        self.phase_data = {}
        
        # 当前相位和相位时间
        self.current_phases = {}
        self.current_phase_times = {}
        
        # 黄灯状态跟踪
        self.yellow_phase_states = {}  # 交通灯ID -> 是否处于黄灯状态
        self.yellow_phase_countdown = {}  # 交通灯ID -> 黄灯倒计时
        self.target_green_phases = {}  # 交通灯ID -> 目标绿灯相位
        self.net_file_staticTlc='xml/static_tlc_mapall.add.xml'
        self.road_need_add=[]
        
        # 连接信息
        self.lanelinks = {}
        
        self.need_optimize_tlds=[]
        if given_tlds_type=='road':
            self.need_optimize_tlds=need_optimize_tlds
        
        
        self.tls_phase_history=None
        self.need_add=[]
        
        # 车辆旅行时间跟踪
        self.inside_vehicles = {}  # 记录当前在仿真中的车辆及其进入时间
        self.completed_trips = {}  # 记录已完成旅行的车辆及其旅行时间
        self.arrival_num=0
        # 初始化连接
        self._init_connection()
        
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
            "--no-warnings", "true",
            "--error-log", "NUL"  # 抑制错误输出（Windows使用NUL，Linux使用/dev/null）
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
        # self.get_tls_id_opitmizer()
        
        

        
        
        
        
    def get_tls_id_opitmizer(self):
        tls_id_opitmizer=[]
        for k in self.need_optimize_tlds:
            if k in self.road_to_tls.keys():
                tls_id_opitmizer.append(self.road_to_tls[k])
        
        self.tls_id_opitmizer=tls_id_opitmizer
        
                
        
    def get_road_to_tls(self):
   
        # 读取网络文件,建立lane id到edge id的映射
        self.lane_to_edge = {}
        
        self.road_to_tls={}
        
        # 使用ElementTree解析XML文件
        tree = ET.parse(self.net_file)
        root = tree.getroot()
        
        # 遍历所有edge标签
        for edge in root.findall('edge'):
            edge_id = edge.get('id')
            
            # 遍历edge下的所有lane标签
            for lane in edge.findall('lane'):
                lane_id = lane.get('id')
                # 记录lane id到edge id的映射关系
                self.lane_to_edge[lane_id] = edge_id
                
        print(f"成功建立 {len(self.lane_to_edge)} 条车道与路段的映射关系")
        
        
        for tl_id in self.tls_ids:
            if tl_id in self.intersections:
                for k in self.intersections[tl_id]['phase_available_lanelinks']:
                    for start, end in k:
                        edgeiop=self.lane_to_edge[start]
                        if edgeiop not in self.road_to_tls:
                            self.road_to_tls[edgeiop]=tl_id

        print(f"成功建立 {len(self.road_to_tls)} 条道路与交通灯的映射关系")
    
    def _init_traffic_lights(self):
        """初始化交通灯信息"""
        for tl_id in self.tls_ids:
            if tl_id=='cluster_4915153326_65452784_8603357960_8649166716_#3more':
                a=1
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
                
                # 获取所有绿灯相位
                for i, phase in enumerate(program.phases):
                    phase_state = phase.state
                    phases.append(phase)
                
                if phases:
                    self.phase_data[tl_id] = phases
                    self.current_phases[tl_id] = 0
                    self.current_phase_times[tl_id] = 0
                    self.yellow_phase_states[tl_id] = False
                    self.yellow_phase_countdown[tl_id] = 0
                    self.target_green_phases[tl_id] = None
                    
                    # 为每个相位创建可用的车道连接
                    self.intersections[tl_id] = {
                        'phase_available_lanelinks': []
                    }
                    phaseid=0
                    for phase in phases:
                        available_links_for_current_phase = []
                        for j, state_char in enumerate(phase.state):
                            if state_char.lower() == 'g' or state_char.lower() == 's':  # 绿灯或直行信号
                                # 确保索引在links范围内并且links[j]非空
                                if j < len(links) and links[j]: 
                                    link_info = links[j][0] # (from_lane, to_lane, via_lane)
                                    available_links_for_current_phase.append((link_info[0], link_info[1]))
                        
                        self.intersections[tl_id]['phase_available_lanelinks'].append((phaseid,available_links_for_current_phase))
                        phaseid+=1
                    # 移到 if phases 内部
              
                    
                else:
                    print(f"警告: 交通灯 {tl_id} 没有有效的绿灯相位，跳过MaxPressure控制")
            else:
                print(f"警告: 交通灯 {tl_id} 没有程序逻辑，跳过初始化")
        
        print(f"成功初始化了 {len(self.intersections)} 个交通灯用于MaxPressure控制")
    
    # def get_max_pressure_action(self, tl_id,step):
    #     """
    #     使用MaxPressure算法为指定交通灯获取最佳相位
        
    #     参数:
    #         tl_id (str): 交通灯ID
            
    #     返回:
    #         int: 最佳相位ID
    #     """
    #     # 检查最小持续时间
    #     if self.current_phase_times[tl_id] < self.t_min:
    #         return self.current_phases[tl_id]
        
    #     # 获取所有车道的车辆数
    #     lane_counts = {}
        
    #     # 只获取相关车道的车辆数
    #     for phase_links in self.intersections[tl_id]['phase_available_lanelinks']:
    #         for start_lane, end_lane in phase_links[1]:
    #             if start_lane not in lane_counts:
    #                 lane_counts[start_lane] = self.conn.lane.getLastStepHaltingNumber(start_lane)
    #             if end_lane not in lane_counts:
    #                 lane_counts[end_lane] = self.conn.lane.getLastStepHaltingNumber(end_lane)
    #     # 获取self.intersections[tl_id]['phase_available_lanelinks']车道的车辆数即可
        
    #     # for lane_id in self.conn.lane.getIDList():
    #     #     lane_counts[lane_id] = self.conn.lane.getLastStepVehicleNumber(lane_id)
        
    #     # 计算每个相位的压力
    #     max_pressure = None
    #     best_phase = -1
        
    #     current_phase=self.current_phases[tl_id]
    #     current_phase_time=self.current_phase_times[tl_id]
        
    #     # flag_skip=False
    #     phaselist=[]
    #     for links in self.intersections[tl_id]['phase_available_lanelinks']:
    #         phaselist.append(links[0])
        
    #     # if current_phase_time>100:
    #     #     _num=phaselist.index(current_phase)
    #     #     return phaselist[(_num+1)%len(phaselist)]
        

    #     # if tl_id=='cluster_588546891_7685535239' and step>75:
    #     #     a=1
      
    #     for  links_ in self.intersections[tl_id]['phase_available_lanelinks']:
    #         # 计算该相位下的总压力
    #         links=links_[1]
    #         phase_id=links_[0]
            

                
            
    #         phase_pressure = 0
    #         for start_lane, end_lane in links:
    #             # 压力 = 起始车道车辆数 - 结束车道车辆数
    #             phase_pressure += lane_counts.get(start_lane, 0) - lane_counts.get(end_lane, 0)
            
    #         # 更新最大压力相位
    #         if max_pressure is None or phase_pressure > max_pressure:
    #             max_pressure = phase_pressure
    #             best_phase = phase_id
    #     return best_phase
    #     # return best_phase
    
    def get_max_pressure_action(self, tl_id,step):
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
        

            
      
        for  links_ in self.intersections[tl_id]['phase_available_lanelinks']:
            # 计算该相位下的总压力
            links=links_[1]
            phase_id=links_[0]
            
            phase_pressure = 0
            for start_lane, end_lane in links:
                # 压力 = 起始车道车辆数 - 结束车道车辆数
                phase_pressure += lane_counts.get(start_lane, 0)
            # 更新最大压力相位
            if max_pressure is None or phase_pressure > max_pressure:
                max_pressure = phase_pressure
                best_phase = phase_id
       
       
       
        if self.current_phase_times[tl_id]<60:
            action= best_phase
        else:
            action =(self.current_phases[tl_id]+1)%len(self.intersections[tl_id]['phase_available_lanelinks'])
        # 如果相位变化，设置黄灯过渡
        if action != current_phase:
            action=(current_phase+1)%len(self.intersections[tl_id]['phase_available_lanelinks'])
        return action
    
    
    
    def apply_actions(self,step):
        
        """为所有交通灯应用MaxPressure算法计算的相位"""
        for tl_id in self.tls_ids:
            yellow_iop=None
            if self.part_optimize:
                if tl_id not in self.tls_id_opitmizer:
                    continue
           
            # if tl_id != 'cluster7618209715_8992828602':
            #     continue
            if tl_id=='cluster_65460845_7360554343_8603322556_8603322558' and step>75:
                a=1
            if tl_id=='300000283':
                a=1
            if step==0:
                self.conn.trafficlight.setPhase(tl_id, self.current_phases[tl_id]*2)
                self.conn.trafficlight.setPhaseDuration(tl_id, 1000)
            if tl_id in self.intersections:
                flag_skip=False
                if self.intersections[tl_id]['phase_available_lanelinks'][self.current_phases[tl_id]][1]==[] and self.current_phase_times[tl_id] >= 3:
                    flag_skip=True
                if self.current_phase_times[tl_id] >= self.t_min or flag_skip:
                    
                    action=self.get_max_pressure_action(tl_id,step)
                    # if self.current_phase_times[tl_id]<60:
                    #     action = self.get_max_pressure_action(tl_id,step)
                    # else:
                    #     action =(self.current_phases[tl_id]+1)%len(self.intersections[tl_id]['phase_available_lanelinks'])
                    
                    # 如果相位变化，设置黄灯过渡
                    current_phase = self.current_phases[tl_id]  # 使用我们自己跟踪的当前相位
                    if action != current_phase:
                        # action=(current_phase+1)%len(self.intersections[tl_id]['phase_available_lanelinks'])
                        # 确定当前相位的索引并获取对应的黄灯相位
                        # 假设SUMO的相位是按 [绿灯0, 黄灯0, 绿灯1, 黄灯1, ...] 排列的
                        program = self.conn.trafficlight.getAllProgramLogics(tl_id)[0]
                        
                        # 找到当前绿灯相位在完整相位列表中的索引
                        phase_index = action
                        
                
                        
                        if phase_index != -1:

                            self.conn.trafficlight.setPhase(tl_id, action)
                            self.conn.trafficlight.setPhaseDuration(tl_id, 3000)
                            self.current_phases[tl_id]=action
                            self.current_phase_times[tl_id]=0
                           
                        else:
                            print(f"警告: 交通灯 {tl_id} 的绿灯相位索引 超出范围")
                      
                    else:
                        self.current_phase_times[tl_id] += 1            
                else:
                    # 更新相位时间
                    self.current_phase_times[tl_id] += 1
               
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
            'arrival_carnum': self.arrival_num
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
    def run(self,need_add=[]):
        """运行仿真直到结束时间"""
        print(f"开始运行LTF仿真，使用MaxPressure算法优化交通信号...")
        print(f"网络文件: {self.net_file}")
        print(f"路由文件: {self.route_file}")
        print(f"结束时间: {self.end_time}秒")
        print(f"GUI模式: {'启用' if self.use_gui else '禁用'}")
        
        step = 0
        metrics_history = []
        start_time = time.time()
        road_ids = traci.edge.getIDList()
        # 过滤掉首字符为':'的road_id
        road_ids = [road_id for road_id in road_ids if not road_id.startswith(':')]
        road_speeds = {road_id:[] for road_id in road_ids}
        road_waiting_vehicles = {road_id:[] for road_id in road_ids}
        road_vehicles_need_add = {}
        # 记录每个红绿灯在每个时间步的相位选择
        road_vehicles = {}

        self.tls_phase_history = {tls_id: [] for tls_id in self.tls_ids}
        need_find_road=need_add
        try:
            while step < self.end_time:
                if len(need_find_road)>0:
                    for road_id in need_find_road:
                        if road_id not in road_vehicles_need_add.keys():      
                            road_vehicles_need_add[road_id] = [traci.edge.getLastStepVehicleIDs(road_id)]
                        else:
                            road_vehicles_need_add[road_id].append(traci.edge.getLastStepVehicleIDs(road_id))
                
                
 
    
                if step >= self.end_time-10:
                    for road_id in road_ids:
                        # if road_id=='-1156810185#1':
                        #     a=traci.edge.getLastStepMeanSpeed(road_id)
         
                        road_speeds[road_id].append(traci.edge.getLastStepMeanSpeed(road_id))
                      
                        road_waiting_vehicles[road_id].append(traci.edge.getLastStepHaltingNumber(road_id))
             

    
                # 应用MaxPressure控制策略（每步都执行）
           
                self.apply_actions(step)
                
                # 收集指标（可以改为每10步收集一次）
                if step % 100 == 0:
                    metrics = self.collect_metrics()
                    metrics_history.append(metrics)
                
                # 仿真前进一步
                self.conn.simulation.step()
                
                # 每步跟踪车辆进出情况
                self.track_vehicle_movements()
                self.arrival_num+=len(self.conn.simulation.getArrivedIDList())
                
                step += 1
            average_speeds = {}
            average_waiting_vehicles = {}
            
            for road_id in road_ids:

                # 获取当前道路上的所有车辆ID
                vehicle_ids = traci.edge.getLastStepVehicleIDs(road_id)
                road_vehicles[road_id] = vehicle_ids
                
            for road_id in road_ids:
                average_speeds[road_id] = np.max(road_speeds[road_id])
                average_waiting_vehicles[road_id] = np.min(road_waiting_vehicles[road_id])
        except Exception as e:
            print(f"仿真过程中出现错误: {str(e)}")
        finally :
            # 关闭连接
            try:
                self.conn.close()
                print("已关闭traci连接")
            except:
                pass
        
        total_time = time.time() - start_time
        # print(f"仿真完成！总用时: {total_time:.2f}秒")
        # print(self.tls_phase_history)
        # self.tls_static_programs=self.generate_static_program()
        print(metrics_history)
        
        return metrics_history,average_speeds,road_vehicles,average_waiting_vehicles,road_vehicles_need_add

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







def copy_and_rename_files(source_rou_file, source_net_file,target_dir):
    # 目标目录
  
    # 确保目标目录存在
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
        
    # 目标文件路径
    target_rou_file = os.path.join(target_dir, 'best_junctions.rou.xml')
    target_net_file = os.path.join(target_dir, 'Corenet.net.xml')
    
    try:
        # 复制并重命名文件
        shutil.copy2(source_rou_file, target_rou_file)
        shutil.copy2(source_net_file, target_net_file)
        print("文件复制和重命名成功!")
    except Exception as e:
        print(f"发生错误: {str(e)}")
        
        
        







