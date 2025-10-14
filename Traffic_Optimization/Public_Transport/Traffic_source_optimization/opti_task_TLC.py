"""
交通灯控优化任务类
基于only_ppo_Manha.py重构，实现单独交通优化任务工作流
"""

import logging
import os
import json
from datetime import datetime
import shutil
import pickle
import uuid
import xml.etree.ElementTree as ET
import yaml
from typing import Dict, Any

from simulation.runner_opti import OptimizationRunner, get_all_tls_ids_from_netxml
from utils.lane_change_utils import get_average_reward

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


class OptiTaskTLC:
    """交通灯控优化任务类 - 完成城市区域的交通优化工作流"""
    
    def __init__(self, config_file: str = None, config_dict: Dict = None):
        """
        初始化交通灯控优化任务
        
        Args:
            config_file (str): 配置文件路径
            config_dict (Dict): 配置字典，如果提供则优先使用
        """
        if config_dict:
            self.config = config_dict
        elif config_file:
            self.config = self._load_config(config_file)
        else:
            raise ValueError("必须提供config_file或config_dict之一")
            
        # 获取配置
        self.city = self.config.get('city', 'Manha')
        self.data_paths = self.config.get('data_paths', {})
        self.rl_config = self.config.get('rl_config', {})
        
        # 初始化优化运行器
        self.optimization_runner = OptimizationRunner(self.config)
        run_prefix = self.config.get('results_dir_prefix', 'optimization_run')
        # 动态生成的路径
        self.run_id = run_prefix+f"_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.results_dir = os.path.join(os.getcwd(), self.run_id)
        self.results_file = os.path.join(self.results_dir, "optimization_summary.json")
        
        self.logger = logging.getLogger("OptiTaskTLC")
        
    def _load_config(self, config_file: str) -> Dict[str, Any]:
        """加载配置文件"""
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"加载配置文件失败: {e}")
            return {}

    def generate_final_report(self, all_results):
        """根据所有成功的结果生成最终报告"""
        if not all_results:
            return {}

        report = {
            "best_by_modification_count": {},
            "overall_best": None
        }
        
        # 找到每种修改次数下的最优解
        best_by_mods = {}
        for result in all_results.values():
            mod_count = result['mod_count']
            if mod_count not in best_by_mods or result['reward'] > best_by_mods[mod_count]['reward']:
                best_by_mods[mod_count] = result
        
        report['best_by_modification_count'] = best_by_mods

        # 找到全局最优解
        overall_best = max(all_results.values(), key=lambda x: x['reward'])
        report['overall_best'] = overall_best
        
        return report

    def get_tl_ids_from_net(self, net_file_path: str) -> list:
        """从net.xml文件中轻量化地读取所有交通灯控交叉口的ID"""
        ids = []
        try:
            tree = ET.parse(net_file_path)
            root = tree.getroot()
            for junction in root.findall('junction'):
                if 'traffic_light' in junction.get('type', ''):
                    ids.append(junction.get('id'))
        except ET.ParseError as e:
            self.logger.error(f"无法解析XML文件 {net_file_path}: {e}")
        return ids

    def load_or_generate_initial_data(self):
        """加载或生成初始决策数据"""
        decisions_data_pkl = self.data_paths.get('decisions_data_pkl')
        lane_averages_pkl = self.data_paths.get('lane_averages_pkl')
        
        if os.path.exists(decisions_data_pkl) and os.path.exists(lane_averages_pkl):
            # 加载现有数据
            with open(decisions_data_pkl, 'rb') as f: 
                initial_tl_phase = pickle.load(f)
            with open(lane_averages_pkl, 'rb') as f: 
                initial_lane_avg = pickle.load(f)
            self.logger.info("已加载现有的初始决策数据")
        else:
            # 生成新数据
            self.logger.info("未发现初始数据文件，将在原始路网上运行一次仿真以生成数据。")
            
            # 获取交通灯IDs
            if self.config.get('rl_tls_ids_all'):
                rl_tls_ids = get_all_tls_ids_from_netxml(self.data_paths.get('original_net_file'))
            else:
                rl_tls_ids = self.config.get('rl_tls_ids', [])
            
            if not rl_tls_ids:
                # 可以添加默认的交通灯IDs或者自动检测
                rl_tls_ids = []
            
            # 运行仿真生成初始数据
            initial_tl_phase, initial_lane_avg = self.optimization_runner.run_optimization_simulation(
                net_file_path=self.data_paths.get('original_net_file'),
                route_file=self.data_paths.get('route_file'),
                rl_tls_ids=rl_tls_ids
            )
            self.logger.info("初始数据生成完毕。")
        
        return initial_tl_phase, initial_lane_avg

    def run(self):
        """运行交通灯控优化任务主流程"""
        os.makedirs(self.results_dir, exist_ok=True)
        self.logger.info(f"本次优化运行ID: {self.run_id}")
        
        # 1. 初始数据准备
        self.logger.info("--- 检查初始决策数据 ---")
        initial_tl_phase, initial_lane_avg = self.load_or_generate_initial_data()
        
        # 2. 初始化迭代参数
        simulation_count = 0
        
        # a. 计算原始路网的基准奖励
        reward_static_pkl = self.data_paths.get('reward_static_pkl')
        initial_reward = get_average_reward(reward_static_pkl)
        initial_reward = initial_reward[0]
        print(initial_reward)
        simulation_count += 1
        self.logger.info(f"原始路网基准奖励为: {initial_reward:.4f}")
        
        # 返回结果
        result = {
            'run_id': self.run_id,
            'initial_reward': initial_reward,
            'simulation_count': simulation_count,
            'initial_tl_phase': initial_tl_phase,
            'initial_lane_avg': initial_lane_avg,
            'results_dir': self.results_dir,
            'config': self.config
        }
        
        self.logger.info("交通灯控优化任务初始化完成")
        return result

    def run_extended_optimization(self):
        """运行扩展的优化流程（如果需要实现完整的优化逻辑）"""
        # 这里可以实现更复杂的优化逻辑
        # 例如多轮迭代、参数调优等
        pass


def main():
    """主执行函数 - 用于独立测试"""
    task = OptiTaskTLC("configs/opti_task_TLC.yml")
    result = task.run()
    print(f"交通灯控优化任务完成，结果: {result}")


if __name__ == "__main__":
    main()
