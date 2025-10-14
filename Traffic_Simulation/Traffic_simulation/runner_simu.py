"""
SUMO仿真运行器
基于traci控制SUMO，使用LTF_Traci进行仿真
"""
import os
import sys
import pickle
import pandas as pd
from typing import Dict, Any, Optional, Tuple, List
import traci
import sumolib
from collections import defaultdict
import tempfile
import shutil
import time
import numpy as np
import xml.etree.ElementTree as ET
from tool.Carttils import *

# 添加父目录到路径以导入现有模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.car_simulate_simu import LTF_Traci
from tool.Carttils import sort_vehicles_by_depart_time


class SimulationRunner:
    """基于traci和LTF_Traci的SUMO仿真运行器"""
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化仿真运行器
        
        Args:
            config: 仿真配置参数
        """
        
        self.config = config
        self.temp_dir = config.get('temp_dir', 'temp/')
        self.output_dir = config.get('output_dir', 'output/')
        self.end_time = config.get('end_time', 3600)
        self.yellow_time = config.get('yellow_time', 3)
        self.t_min = config.get('t_min', 10)
        self.seed = config.get('seed', 42)
        self.sumo_binary = config.get('sumo_binary', 'sumo')
        self.part_optimize = config.get('part_optimize', False)
        self.current_step = 0
        
        # OD调整相关属性
        self.max_steps = config.get('max_steps', 100)
        self.num_regions = config.get('num_regions', 25)
        self.target_score = config.get('target_score', 0.85)
        
        # 道路相关配置
        self.test_road_ids = config.get('test_road_ids', [])
        self.real_data = config.get('real_data', {})
        
        # OD相关数据结构
        self.od_dict = None
        self.mergeOD2tripID = None
        self.trip2OD = None
        self.sumo_roudata = None
        self.hot_add_dicct_pos2vehicles = {}
        self.hot_reduce_dicct_pos2vehicles = {}
        self.add_record_dict = {}
        self.reduce_record_dict = {}
        self.summary_adjust = {'need_add': [], 'need_delete': []}
        self.important_junctions = set()
        self.junctions_dict = {}
        self.road_vehicles_all = {}
        self.stop_reduce = False
        self.stop_add = False
        self.bigger_change = None
        self.dir_OD = None
        self.need_add_road = []
        
        # 文件路径
        self.net_file = None
        self.rou_file = None
        self.iop_file = None
        self.real_data_file = None
        self.real_data_type = 'pkl'
        self.edge_limit_file = None
        self.need_add = []
        
        self.f1scorelist=[]
        self.prelist=[]
        self.recallist=[]
        
        self.ensure_directories()
        
    def ensure_directories(self):
        """确保必要的目录存在"""
        for dir_path in [self.temp_dir, self.output_dir]:
            os.makedirs(dir_path, exist_ok=True)
    
    def run_simulation_with_traci(self, 
                                 net_file: str, 
                                 rou_file: str,
                                 real_data: Dict[str, Any] = None,
                                 need_add: List[str] = []) -> Dict[str, Any]:
        """
        使用traci控制运行SUMO仿真
        
        Args:
            net_file: 路网文件路径
            rou_file: 路由文件路径  
            real_data: 真实数据（用于确定测试道路ID）
            
        Returns:
            包含仿真结果的字典
        """
        print(f"开始运行仿真...")
        print(f"网络文件: {net_file}")
        print(f"路由文件: {rou_file}")
        
        # 1. 对车辆按出发时间排序
        sorted_rou_file = self._sort_vehicles_by_depart_time(rou_file)
        self.iop_file = sorted_rou_file
        
        filter_vehicles_with_single_edge(self.iop_file, self.iop_file)
        
        # 2. 获取有效的道路ID
        valid_road_ids = self._get_valid_road_ids(net_file, sorted_rou_file, real_data)
        
        # 3. 使用LTF_Traci运行仿真
        simulation_results = self._run_ltf_traci_simulation(net_file, sorted_rou_file, valid_road_ids,need_add)
        
        # 4. 保存结果
        output_file = self._save_simulation_results(simulation_results)
        
        return {
            'success': True,
            'output_file': output_file,
            'average_speeds': simulation_results['average_speeds'],
            'road_vehicles': simulation_results['road_vehicles'],
            'average_waiting_vehicles': simulation_results['average_waiting_vehicles'],
            'metrics_history': simulation_results['metrics_history'],
            'valid_road_ids': valid_road_ids,
            'road_vehicles_need_add': simulation_results['road_vehicles_need_add']
        }
    
    def _sort_vehicles_by_depart_time(self, rou_file: str) -> str:
        """对车辆按出发时间排序"""
        # 创建临时文件用于存储排序后的路由

        sort_vehicles_by_depart_time(rou_file, rou_file)
        
        # 检查文件大小
        file_size = os.path.getsize(rou_file)
        print(f"排序后文件大小: {file_size} 字节")
        
        return rou_file
    
    def _get_valid_road_ids(self, net_file: str, rou_file: str, real_data: Dict[str, Any] = None) -> List[str]:
        """获取SUMO网络中有效的道路ID"""
        print("获取有效道路ID...")
        
        # 设置SUMO命令
        cmd = [
            self.sumo_binary,
            "--net-file", net_file,
            "--route-files", rou_file,
            "--begin", "0",
            "--end", str(self.end_time),
            "--no-warnings", "true",
        ]
        
        # 启动traci验证道路ID
        traci.start(cmd)
        
        try:
            # 确定要测试的道路ID
            if real_data:
                road_ids = list(real_data.keys())
            else:
                # 如果没有真实数据，获取所有道路ID
                road_ids = traci.edge.getIDList()
                # 过滤掉内部道路（以':'开头的）
                road_ids = [road_id for road_id in road_ids if not road_id.startswith(':')]
            # else:
            #     road_ids = self.test_road_ids
            
            # 获取SUMO网络中所有的边（道路）
            all_edges = traci.edge.getIDList()
            
            # 检查每个road_id是否存在于SUMO网络中
            valid_road_ids = []
            for road_id in road_ids:
                if road_id in all_edges:
                    valid_road_ids.append(road_id)
            
            # print(f"有效道路ID数量: {len(valid_road_ids)}/{len(road_ids)}")
            
        finally:
            # 关闭traci连接
            traci.close()
        
        return valid_road_ids
    


    def reset(self, rou_file: str, od_dict: Dict, real_data_file: str, edge_limit_file: str, net_file: str, iop_file: str = None, taz_file: str = None):
        """
        重置仿真环境，对应ODEnv_agent.reset()
        
        Args:
            rou_file: 路由文件路径
            od_dict: OD到trip的映射字典
            real_data_file: 真实数据文件路径
            edge_limit_file: 道路限速文件路径
            net_file: 网络文件路径
            iop_file: 输出路由文件路径，如果为None则使用临时文件
        """
        self.current_step = 0
        self.net_file = net_file
        self.rou_file = rou_file
        self.real_data_file = real_data_file
        self.edge_limit_file = edge_limit_file
        
        if iop_file is None:
            self.iop_file = os.path.join(self.temp_dir, f"temp_{os.path.basename(rou_file)}")
        else:
            self.iop_file = iop_file

        # 初始化OD映射
        self.od_dict = od_dict
        self.mergeOD2tripID = od_dict.copy()
        
        self.trip2OD = {}
        for od_pair, trip_list in self.mergeOD2tripID.items():
            for trip_id in trip_list:
                self.trip2OD[trip_id] = od_pair
        
        # 加载路由XML数据
        self.sumo_roudata = loadxmlfile(rou_file)
        
        # 读取taz文件，建立edge到taz id的映射字典（多对一）
      
        
        # 加载真实数据
        self.real_data = loadfile(real_data_file)
        self.edge2taz = self.parse_taz_file(taz_file)
        
        # 处理真实数据的值映射
        for key in self.real_data:
            if self.real_data[key] == 0:
                self.real_data[key] = 0
            elif self.real_data[key] == 1:
                self.real_data[key] = 3
            elif self.real_data[key] == 2:
                self.real_data[key] = 5
        
        # 初始化其他状态变量
        self.add_record_dict = {}
        self.reduce_record_dict = {}
        self.summary_adjust = {'need_add': [], 'need_delete': []}
        self.road_vehicles_all = {}
        self.stop_reduce = False
        
        # 将初始路由数据写入输出文件
        tree = ET.ElementTree(self.sumo_roudata)
        tree.write(self.iop_file, encoding='utf-8', xml_declaration=True)
        
        
        sim_result = self.run_simulation_with_traci(self.net_file, self.iop_file, self.real_data,[])
        sumo_average_speeds = sim_result['average_speeds']
        road2vehicleid = sim_result['road_vehicles']
        average_waiting_vehicles = sim_result['average_waiting_vehicles']
        self.road_vehicles_need_add = sim_result['road_vehicles_need_add']

    
        # 如果没有提供comparator，使用简单的评价方法
        sumo_data = self.calculate_congest_R(sumo_average_speeds, average_waiting_vehicles)
        score = self._simple_evaluate(sumo_data)
        print(f"简单评分: {score:.4f}")

        if score>0.42:
            with open('sumo_data.pkl', 'wb') as f:
                pickle.dump(sumo_data, f)
        
        
           # 5. 检查是否需要停止减少操作
        # 6. 分析下一轮需要调整的方向
        self.need_add = self.direction_reward(sumo_data, road2vehicleid)

        
        
        print(f"环境重置完成，输出文件: {self.iop_file}")
        return self._get_state()

    def parse_taz_file(self,taz_file):
                """
                解析taz文件，返回edge到taz id的映射字典（多对一）
                :param taz_file: taz文件路径
                :return: dict, key为edge id, value为taz id（int）
                """
                import xml.etree.ElementTree as ET
                edge2taz = {}
                tree = ET.parse(taz_file)
                root = tree.getroot()
                for taz in root.findall('taz'):
                    taz_id = taz.get('id')
                    edges_str = taz.get('edges', '')
                    for edge in edges_str.strip().split():
                        edge2taz[edge] = int(taz_id)
                return edge2taz
    def calculate_congest_R(self,result_data,average_waiting_vehicles):
        road_limit_file = loadfile(self.edge_limit_file)
        # 初始化一个新字典来保存拥堵评分
        congestion_scores = {}
        # 遍历result_data中的每个路段ID
        for road_id in result_data:
            if road_id=='-1335332353#0':
                a=1
            # 检查该路段ID是否也存在于road_limit_file中
            if road_id in road_limit_file:
                ratio = road_limit_file[road_id] / result_data[road_id]     
                # 根据比值分配拥堵评分
                if ratio > 2 or average_waiting_vehicles[road_id] > 20:
                    congestion_scores[road_id] = 5  # 严重拥堵
                elif 1.5 < ratio <= 2 or average_waiting_vehicles[road_id] > 20:
                    congestion_scores[road_id] = 3  # 中度拥堵
                else:  # ratio <= 1.5
                    congestion_scores[road_id] = 0  # 畅通
        
        return congestion_scores

    # def calculate_congest_R(self, result_data, average_waiting_vehicles):
    #     """计算拥堵评分"""
    #     congestion_scores = {}
        
    #     # 加载道路限速数据
    #     road_limit_file = loadfile(self.edge_limit_file)
        
    #     for road_id in result_data:
    #         if result_data[road_id] == 0:
    #             continue
            
    #         if road_id in road_limit_file:
    #             ratio = road_limit_file[road_id] / result_data[road_id]
                
    #             if ratio > 2 :
    #                 congestion_scores[road_id] = 5  # 严重拥堵
    #             elif 1.5 < ratio <= 2 :
    #                 congestion_scores[road_id] = 3  # 中度拥堵
    #             else:
    #                 congestion_scores[road_id] = 0  # 畅通
        
    #     return congestion_scores

    def _simple_evaluate(self, sumo_data):
        """简单的评价方法，当没有comparator时使用"""
        real_traffic = self.real_data
        
        # 计算匹配的值
        real_values = []
        sumo_values = []
        
        for road_id in real_traffic.keys():
            if road_id in sumo_data:
                real_values.append(real_traffic[road_id])
                sumo_values.append(sumo_data[road_id])

        if not real_values:
            return 1.0  # 最差评分
            
        real_values = np.array(real_values)
        sumo_values = np.array(sumo_values)
         
        # 计算准确率、召回率和F1分数
        true_positives = np.sum((sumo_values != 0) & (real_values != 0))
        false_positives = np.sum((sumo_values != 0) & (real_values == 0))
        false_negatives = np.sum((sumo_values == 0) & (real_values != 0))
        
        precision = true_positives / (true_positives + false_positives) if (true_positives + false_positives) > 0 else 0
        recall = true_positives / (true_positives + false_negatives) if (true_positives + false_negatives) > 0 else 0
        f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
        
        if recall>0.81:
            self.stop_add=True
        print(f"f1_score: {f1_score:.4f}")
        print(f"precision: {precision:.4f}")
        print(f"recall: {recall:.4f}")
        self.f1scorelist.append(f1_score)
        self.prelist.append(precision)
        self.recallist.append(recall)
        
        
        
        # 返回1-f1_score作为损失（越低越好）
        return f1_score

    def _get_state(self):
        """获取当前状态"""
        return {
            'current_step': self.current_step,
            'od_dict': self.mergeOD2tripID,
            'trip2OD': self.trip2OD,
            'road_vehicles_all': self.road_vehicles_all
        }

    def add_vehicle(self,road_id):
        current_car=self.road_vehicles_all[road_id]
        current_car=list(set(current_car))
        return current_car
    
    def merge_need_add(self,road_id):
        """合并并去重嵌套列表中的所有元素"""
        merged_list = []
        if road_id not in self.road_vehicles_need_add.keys():
            return []
        for sublist in self.road_vehicles_need_add[road_id]:
            merged_list.extend(sublist)
        # 去重
        merged_list = list(set(merged_list))
        return merged_list
        
    
    
    def direction_reward(self, sumo_result,road2vehicleid):
        
        dict_OD_dir=np.zeros((self.num_regions,self.num_regions))
        
        real_traffic = self.real_data
        self.need_add_road=[]
        
        # 初始化变量以存储匹配的值
        real_values = []
        sumo_values = []

        road_need_add=[]
        # 遍历real_traffic字典中的每个路段ID

        

        self.hot_add_dicct_pos2vehicles={}
        self.hot_reduce_dicct_pos2vehicles={}
        self.important_junctions=set()
        for road_id in real_traffic.keys():
            # 检查该路段ID是否也存在于sumo_result中
            if road_id in sumo_result:
                if sumo_result[road_id] > real_traffic[road_id]:
                    if real_traffic[road_id]!=0:
                        continue
                    # if road_id in self.junctions_dict:
                    #     self.important_junctions.add(self.junctions_dict[road_id])
                
                    # 获取所有与该road_id相关的车辆
                    vehicles = road2vehicleid[road_id]
                    
                    for vehicle in vehicles:
                        # 找到车辆对应的OD位置
                        od_position = self.trip2OD.get(vehicle)
                        if od_position not in self.reduce_record_dict:
                            self.reduce_record_dict[od_position]=1
                        else:
                            self.reduce_record_dict[od_position]+=1
                            self.add_record_dict[od_position]=0
                        self.summary_adjust['need_delete'].append((od_position,self.current_step))
                        if od_position not in self.hot_reduce_dicct_pos2vehicles:
                            self.hot_reduce_dicct_pos2vehicles[od_position]=[vehicle]
                        else:
                            self.hot_reduce_dicct_pos2vehicles[od_position].append(vehicle)
                elif sumo_result[road_id] < real_traffic[road_id]:
                    self.need_add_road.append(road_id)
                    
                    if sumo_result[road_id]!=0:
                        continue
                    # 获取所有与该road_id相关的车辆
                    vehicles = road2vehicleid[road_id]
                    # if len(vehicles)==0:
                    #     vehicles=self.add_vehicle(road_id)
           
                    if len(vehicles)>0:
                        for vehicle in vehicles:
                            # 找到车辆对应的OD位置
                            od_position = self.trip2OD.get(vehicle)
                            # if od_position ==(45, 16):
                            #     a=1
                            #     print(road_id)
                            
                            self.summary_adjust['need_add'].append((od_position,self.current_step))
                            if od_position not in self.add_record_dict:
                                self.add_record_dict[od_position]=1
                            else:
                                self.add_record_dict[od_position]+=1
                                self.reduce_record_dict[od_position]=0
                            if od_position not in self.hot_add_dicct_pos2vehicles:
                                self.hot_add_dicct_pos2vehicles[od_position]=[vehicle]
                            else:
                                self.hot_add_dicct_pos2vehicles[od_position].append(vehicle)
                    elif len(vehicles)==0:
                        road_need_add.append(road_id)

                        vehicles_=self.merge_need_add(road_id)
                        if len(vehicles_)>0:
                            for vehicle in vehicles_:
                                # 找到车辆对应的OD位置
                                od_position = self.trip2OD.get(vehicle)
                                # if od_position ==(45, 16):
                                #     a=1
                                #     print(road_id)
                                
                                self.summary_adjust['need_add'].append((od_position,self.current_step))
                                if od_position not in self.add_record_dict:
                                    self.add_record_dict[od_position]=1
                                else:
                                    self.add_record_dict[od_position]+=1
                                    self.reduce_record_dict[od_position]=0
                                if od_position not in self.hot_add_dicct_pos2vehicles:
                                    self.hot_add_dicct_pos2vehicles[od_position]=[vehicle]
                                else:
                                    self.hot_add_dicct_pos2vehicles[od_position].append(vehicle)

                        

                
 
        # print(len(self.important_junctions),"需要优化的路口数量")
        return road_need_add
        
    def find_bigger_change(self):
        # 获取两个字典的所有key的并集
        all_keys = set(self.hot_reduce_dicct_pos2vehicles.keys()) | set(self.hot_add_dicct_pos2vehicles.keys())
        
        # 移除None值
        all_keys = {key for key in all_keys if key is not None}
        
        # 建立新字典存储结果
        result_dict = {}
        
        # 遍历所有key
        for key in all_keys:
            # 只在hot_add_dicct_pos2vehicle中存在
            if key in self.hot_add_dicct_pos2vehicles and key not in self.hot_reduce_dicct_pos2vehicles:
                result_dict[key] = len(self.hot_add_dicct_pos2vehicles[key])
            # 只在hot_reduce_dicct_pos2vehicles中存在  
            elif key in self.hot_reduce_dicct_pos2vehicles and key not in self.hot_add_dicct_pos2vehicles:
                result_dict[key] = -len(self.hot_reduce_dicct_pos2vehicles[key])
            # 两个字典都存在
            else:
                # 比较value列表长度
                if len(self.hot_reduce_dicct_pos2vehicles[key]) > len(self.hot_add_dicct_pos2vehicles[key]):
                    result_dict[key] = -len(self.hot_reduce_dicct_pos2vehicles[key])
                else:
                    result_dict[key] = len(self.hot_add_dicct_pos2vehicles[key])
                    
        return result_dict
        
        
        
      
    def find_od_changes(self, original_od, updated_od):
        """
        查找OD矩阵中发生变化的位置及变化量
        
        参数:
        original_od: 原始OD矩阵
        updated_od: 更新后的OD矩阵
        
        返回:
        changes_dict: 包含变化位置和变化量的字典，格式为 {(i,j): 变化量}
        """
        changes_dict = {}
        
        # 确保两个矩阵形状相同
        if original_od.shape != updated_od.shape:
            raise ValueError("原始矩阵和更新矩阵的形状不一致")
        
        # 获取矩阵的行数和列数
        rows, cols = original_od.shape
        
        # 遍历矩阵的每个元素
        for i in range(rows):
            for j in range(cols):
                # 计算变化量
                change = updated_od[i, j] - original_od[i, j]
                
                # 如果有变化，则记录到字典中
                if change != 0:
                    changes_dict[(i, j)] = change
        
        return changes_dict

    def generate_trips_and_extract_vehicles(self,edge_list, net_file, n, iop_xml_path, duarouter_path="duarouter"):
        """
        根据给定的edge列表、路网文件和trip个数n，生成iop.xml文件，并用duarouter生成rou文件，提取vehicle标签对象返回。

        参数:
        edge_list: 边的列表
        net_file: 路网文件路径
        n: 每对起止边生成的trip数量
        iop_xml_path: 生成的iop.xml文件路径
        duarouter_path: duarouter命令路径，默认"duarouter"

        返回:
        vehicle_elements: vehicle标签对象的列表
        """
        import xml.etree.ElementTree as ET
        import random
        import os

        # 1. 生成iop.xml文件
        import uuid
        routes = ET.Element("routes")
        trip_id_set = set()
        trip_to_Taz={}
        for from_edge in edge_list:
            possible_to_edges = [e for e in edge_list if e != from_edge]
            if not possible_to_edges:
                continue
            for _ in range(n):
                to_edge = random.choice(possible_to_edges)
                if to_edge not in self.edge2taz or from_edge not in self.edge2taz: 
                    continue
                # 使用uuid生成唯一trip id
                trip_id = f"trip_{uuid.uuid4().hex}-{from_edge}-{to_edge}-{n}"
                trip_to_Taz[trip_id]=(self.edge2taz[from_edge],self.edge2taz[to_edge])
                trip_elem = ET.SubElement(routes, "trip")
                trip_elem.set("id", trip_id)
                trip_elem.set("depart", "0")
                trip_elem.set("from", from_edge)
                trip_elem.set("to", to_edge)
                trip_id_set.add(trip_id)
        tree = ET.ElementTree(routes)
        tree.write(iop_xml_path, encoding="utf-8", xml_declaration=True)

        # 2. 使用duarouter生成rou文件
        output_rou = os.path.splitext(iop_xml_path)[0] + "_routes.rou.xml"
        cmd = f'{duarouter_path} --route-files={iop_xml_path} --net-file={net_file} --output-file={output_rou}'
        os.system(cmd)

        # 3. 解析rou文件，提取vehicle标签
        rou_tree = ET.parse(output_rou)
        rou_root = rou_tree.getroot()
        # 简化：合并遍历，直接收集所有vehicle标签并设置属性
        vehicle_elements = []
        for vehicle in rou_root.iter("vehicle"):
            trip_id = vehicle.get("id")
            if trip_id in trip_to_Taz:
                from_taz, to_taz = trip_to_Taz[trip_id]
                vehicle.set("fromTaz", str(from_taz))
                vehicle.set("toTaz", str(to_taz))
            vehicle_elements.append(vehicle)
        return vehicle_elements

    # 状态空间的更迭进行存疑存疑 疑惑
    def update_rouxml(self,changedict):
        if self.stop_add==False:
            vehicle_elements = self.generate_trips_and_extract_vehicles(self.need_add_road, self.net_file, 12, 'io.xml')
        # 首先，将vehicle_elements中的每个vehicle标签添加到self.sumo_roudata中
            for vehicle in vehicle_elements:
                self.sumo_roudata.append(vehicle)

            # 然后，更新self.mergeOD2tripID
            # vehicle_elements中的每个vehicle标签都包含fromTaz和toTaz属性
            # 我们将这些新trip的id加入到对应(fromTaz, toTaz)的列表中
            for vehicle in vehicle_elements: 
                trip_id = vehicle.get("id")
                from_taz = int(vehicle.get("fromTaz"))
                to_taz = int(vehicle.get("toTaz"))
                od_pair = (from_taz, to_taz)
                if od_pair not in self.mergeOD2tripID:
                    self.mergeOD2tripID[od_pair] = []
                self.mergeOD2tripID[od_pair].append(trip_id)


        tree_to_vehicle={}
        for vehicle in self.sumo_roudata.findall('vehicle'):
            tree_to_vehicle[vehicle.get('id')]=vehicle
        next_time = time.time()
        
        # # 查找id为ssst124.0的vehicle
        # target_vehicle = self.sumo_roudata.find(".//vehicle[@id='sssst124.0']")
        # if target_vehicle is not None:
        #     print("找到id为sssst124.0的vehicle")
  
        for i in self.bigger_change.keys():
            
      

            triplist=self.mergeOD2tripID[i]

            if self.bigger_change[i]>0:

                orginlist=self.mergeOD2tripID[i]
                if i not in self.hot_add_dicct_pos2vehicles.keys():
                    continue
                triplist=self.hot_add_dicct_pos2vehicles[i]
                
                if self.add_record_dict[i]>6:
                    self.mergeOD2tripID[i],iop,self.sumo_roudata=random_expend_list(orginlist,triplist,int(self.bigger_change[i]),self.sumo_roudata,tree_to_vehicle,Faster=True)
                else: 
                    self.mergeOD2tripID[i],iop,self.sumo_roudata=random_expend_list(orginlist,triplist,int(self.bigger_change[i]),self.sumo_roudata,tree_to_vehicle)

                
                # print(self.mergeOD2tripID[i])
                
                
           
                
            else:
                # if self.stop_reduce==True:
                #     continue
                orginlist=self.mergeOD2tripID[i]
                if i not in self.hot_reduce_dicct_pos2vehicles.keys():
                    continue
                triplist=self.hot_reduce_dicct_pos2vehicles[i]
                if self.reduce_record_dict[i]>6:
              
                    
                    self.mergeOD2tripID[i],iop,self.sumo_roudata=random_reduce_list(orginlist,triplist,int(self.bigger_change[i]),self.sumo_roudata,tree_to_vehicle,Faster=True)
                else:
                    self.mergeOD2tripID[i],iop,self.sumo_roudata=random_reduce_list(orginlist,triplist,int(self.bigger_change[i]),self.sumo_roudata,tree_to_vehicle)
        
        self.trip2OD = {}
        for od_pair, trip_list in self.mergeOD2tripID.items():
            for trip_id in trip_list:
                self.trip2OD[trip_id] = od_pair
        # 将更新后的self.sumo_roudata写入到roufile中
        tree = ET.ElementTree(self.sumo_roudata)

          
        
            
        vehicle_count = len(self.sumo_roudata.findall('vehicle'))
        print(f"变动后车辆标签的数量: {vehicle_count}")
        tree.write(self.iop_file, encoding='utf-8', xml_declaration=True)

        return       
            
   
        
    def step(self, comparator=None):
        """
        执行一步仿真-分析-调整循环，对应ODEnv_agent.step()
        
        Args:
            comparator: 评价器实例，用于计算仿真质量评分
            
        Returns:
            (state, reward, done, info, sumo_average_speeds)
        """
        # 1. 分析需要调整的OD对
        self.bigger_change = self.find_bigger_change()
        self.arrival_num=0
 
        # 2. 根据分析结果更新路由文件
        if self.bigger_change:
            self.update_rouxml(None)  # changedict参数暂时传None
        
        # 3. 运行仿真
        sim_result = self.run_simulation_with_traci(self.net_file, self.iop_file, self.real_data,self.need_add)
        sumo_average_speeds = sim_result['average_speeds']
        road2vehicleid = sim_result['road_vehicles']
        average_waiting_vehicles = sim_result['average_waiting_vehicles']
        self.road_vehicles_need_add = sim_result['road_vehicles_need_add']
        history_average_speeds = sim_result['metrics_history']

   
        # 如果没有提供comparator，使用简单的评价方法
        sumo_data = self.calculate_congest_R(sumo_average_speeds, average_waiting_vehicles)
        score = self._simple_evaluate(sumo_data)
        print(f"简单评分: {score:.4f}")
        # if score>0.42: 
        #     with open('sumo_data.pkl', 'wb') as f:
        #         pickle.dump(sumo_data, f)
        
        # # 5. 检查是否需要停止减少操作
        # if score <= 0.15:  # 当评分很好时停止减少操作
        #     self.stop_reduce = True

        # 6. 分析下一轮需要调整的方向
        # sumo_data = self.calculate_congest_R(sumo_average_speeds, average_waiting_vehicles)
        self.need_add = self.direction_reward(sumo_data, road2vehicleid)

        # 7. 计算奖励和完成状态
        
        self.current_step += 1
        done = (score >= self.target_score) or (self.current_step >= self.max_steps)
        
        info = {
            'score': score,
            'current_step': self.current_step,
            'converged': score <= self.target_score
        }
        
        return self._get_state(), score, done, info, sumo_average_speeds,sumo_data,history_average_speeds





    
    def _run_ltf_traci_simulation(self, net_file: str, rou_file: str, valid_road_ids: List[str],need_add: List[str] = []) -> Dict[str, Any]:
        """使用LTF_Traci运行仿真"""
        print("使用LTF_Traci运行仿真...")
        
        # 创建LTF_Traci实例
        ltf = LTF_Traci(
            net_file=net_file,
            route_file=rou_file, 
            use_gui=False,
            
            end_time=self.end_time,
            yellow_time=self.yellow_time,
            t_min=self.t_min,
            seed=self.seed,
            need_optimize_tlds=[],
            given_tlds_type='road',
            part_optimize=self.part_optimize,
            sumobinary='sumo'
        )
        
        # 运行仿真  
        start_time = time.time()
        metrics_history, average_speeds, road_vehicles, average_waiting_vehicles,road_vehicles_need_add = ltf.run(need_add)
        simulation_time = time.time() - start_time
        
        print(f"仿真完成! 耗时: {simulation_time:.2f}秒")
        print(f"获得 {len(average_speeds)} 条道路的速度数据")
        print(f"获得 {len(road_vehicles)} 条道路的车辆数据")
        
        return {
            'metrics_history': metrics_history,
            'average_speeds': average_speeds,
            'road_vehicles': road_vehicles,
            'average_waiting_vehicles': average_waiting_vehicles,
            'simulation_time': simulation_time,
            'valid_road_ids': valid_road_ids,
            'road_vehicles_need_add': road_vehicles_need_add
        }
    
    def _save_simulation_results(self, results: Dict[str, Any]) -> str:
        """保存仿真结果"""
        timestamp = int(time.time())
        output_file = os.path.join(self.output_dir, f"sim_results_{timestamp}.pkl")
        
        # with open(output_file, 'wb') as f:
        #     pickle.dump(results, f)
        
        return output_file
    
    def run_with_existing_args(self, args) -> Dict[str, Any]:
        """
        使用现有的args对象运行仿真，兼容现有代码
        
        Args:
            args: 包含所有仿真参数的对象
            
        Returns:
            仿真结果字典
        """
        # 从args对象提取参数
        net_file = getattr(args, 'sumo_net', None) or getattr(args, 'net_file', None)
        rou_file = getattr(args, 'iop_file', None) or getattr(args, 'rou_file', None)
        real_data = getattr(args, 'real_data', {})
        
        if not net_file or not rou_file:
            raise ValueError("缺少必要的文件路径参数")
        
        # 更新配置
        if hasattr(args, 'end_time'):
            self.end_time = args.end_time
        if hasattr(args, 'test_road_ids'):
            self.test_road_ids = args.test_road_ids
        
        return self.run_simulation_with_traci(net_file, rou_file, real_data)
    
    def get_simulation_metrics(self, output_file: str) -> Dict[str, Any]:
        """
        从保存的结果文件中提取仿真指标
        
        Args:
            output_file: 仿真结果文件路径
            
        Returns:
            仿真指标字典
        """
        with open(output_file, 'rb') as f:
            results = pickle.load(f)
        
        average_speeds = results['average_speeds']
        average_waiting_vehicles = results['average_waiting_vehicles']
        
        # 计算汇总指标
        if average_speeds:
            overall_avg_speed = sum(average_speeds.values()) / len(average_speeds)
            max_speed = max(average_speeds.values())
            min_speed = min(average_speeds.values())
        else:
            overall_avg_speed = max_speed = min_speed = 0
            
        if average_waiting_vehicles:
            overall_avg_waiting = sum(average_waiting_vehicles.values()) / len(average_waiting_vehicles)
            max_waiting = max(average_waiting_vehicles.values())
            min_waiting = min(average_waiting_vehicles.values())
        else:
            overall_avg_waiting = max_waiting = min_waiting = 0
        
        return {
            'overall_avg_speed': overall_avg_speed,
            'max_speed': max_speed,
            'min_speed': min_speed,
            'overall_avg_waiting': overall_avg_waiting,
            'max_waiting': max_waiting,
            'min_waiting': min_waiting,
            'road_count': len(average_speeds),
            'simulation_time': results.get('simulation_time', 0)
        }



def filter_vehicles_with_single_edge(input_file, output_file):
    """
    Parses a SUMO route file and removes vehicles that have only a single edge in their route.

    A vehicle is removed if its <route> element's "edges" attribute does not contain a space.

    Args:
        input_file (str): The path to the input .rou.xml file.
        output_file (str): The path where the filtered .rou.xml file will be saved.
    """
    print(f"正在解析文件: {input_file}...")
    try:
        tree = ET.parse(input_file)
        root = tree.getroot()

        vehicles_to_remove = []
        # 遍历根目录下的所有 vehicle 元素
        for vehicle in root.findall('vehicle'):
            route = vehicle.find('route')
            # 检查 route 元素和 edges 属性是否存在
            if route is not None and 'edges' in route.attrib:
                edges = route.get('edges', '')
                # 单个 edge 的情况下，edges 属性值中不会包含空格
                if ' ' not in edges.strip():
                    vehicles_to_remove.append(vehicle)
        
        if vehicles_to_remove:
            print(f"找到 {len(vehicles_to_remove)} 个需要移除的单边车辆。")
            for vehicle in vehicles_to_remove:
                root.remove(vehicle)
            
            print(f"正在将过滤后的数据写入: {output_file}...")
            tree.write(output_file, encoding='UTF-8', xml_declaration=True)
            print("完成。")
        else:
            print("未找到需要移除的单边车辆，文件未改动。")

    except ET.ParseError as e:
        print(f"解析 XML 文件时出错: {e}")
    except FileNotFoundError:
        print(f"错误：找不到输入文件: {input_file}")
    except Exception as e:
        print(f"发生未知错误: {e}")


def create_simulation_runner(config: Dict[str, Any] = None) -> SimulationRunner:
    """
    创建仿真运行器的工厂函数
    
    Args:
        config: 仿真配置参数
        
    Returns:
        配置好的SimulationRunner实例
    """
    if config is None:
        config = {
            'end_time': 3600,
            'yellow_time': 3,
            't_min': 10,
            'seed': 42,
            'sumo_binary': 'sumo',
            'part_optimize': False,
            'temp_dir': 'temp/',
            'output_dir': 'output/',
            'max_steps': 100,
            'num_regions': 25,
            'target_score': 0.70
        }
    
    return SimulationRunner(config) 