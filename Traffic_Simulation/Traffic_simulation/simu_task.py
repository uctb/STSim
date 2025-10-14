"""
仿真任务类
基于simu_Manha.py重构，实现城市区域的仿真工作流
"""

import yaml
import logging
import os
import shutil
from typing import Dict, Any, List

from simulation.runner_simu import SimulationRunner
from evaluation.comparator import Comparator
from adjustment.scale import ScaleAdjuster
from utils.car_simulate_simu import Generate_NewOD, dumpfile, loadfile

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class SimuTask:
    """仿真任务类 - 完成城市区域的仿真工作流"""
    
    def __init__(self, config_file: str = None, config_dict: Dict = None):
        """
        初始化仿真任务
        
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
            
        # 初始化组件
        self.simulation_runner = SimulationRunner(self.config.get('simulation', {}))
        self.comparator = Comparator(self.config.get('optimizer', {}))
        self.scale_adjuster = ScaleAdjuster(self.config.get('optimizer', {}))
        
        # 初始化状态
        self.state = self._init_state()
        
    def _load_config(self, config_file: str) -> Dict[str, Any]:
        """加载配置文件"""
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except Exception as e:
            logger.error(f"加载配置文件失败: {e}")
            return {}
    
    def _init_state(self) -> Dict[str, Any]:
        """初始化任务状态"""
        data_paths = self.config.get('data_paths', {})
        simulation_config = self.config.get('simulation', {})
        
        state = {
            "iteration": 0,
            "current_score": float('inf'),
            "best_score": float('inf'),
            "best_params": None,
            "history": [],
            "last_action": None,
            "convergence_count": 0,
            "exploration_counter": 0,
            "scale_test_results": [],
        }
        
        # 添加数据路径
        state.update(data_paths)
        
        # 添加仿真参数
        state.update({
            'need_mergeRegion': simulation_config.get('need_mergeRegion', False),
            'max_od_iterations': simulation_config.get('max_od_iterations', 50),
        })
        
        return state
    
    def iterative_optimization(self, max_od_iterations: int = None):
        """
        迭代优化框架
        
        参数:
        - max_od_iterations: OD优化最大迭代次数
        """
        if max_od_iterations is None:
            max_od_iterations = self.state.get('max_od_iterations', 50)
            
        logger.info("开始OD矩阵优化阶段...")
        
        generator = Generate_NewOD(
            rou_file=self.state['rou_file'],
            grid_shp=self.state["grid_shp"],
            third_adjust_firstSTEP=False,
            od_pkl=self.state["od_file"]
        )
        
        new_od_dict, new_od_matrix, num_regions = generator.generate(
            needmerge=self.state["need_mergeRegion"]
        )
    
        # 复制路由文件到输出文件（如果需要）
        if 'rou_file' in self.state and 'iop_file' in self.state:
            shutil.copy(self.state['rou_file'], self.state['iop_file'])

        self.simulation_runner.reset(
            rou_file=self.state['rou_file'],
            od_dict=new_od_dict,
            real_data_file=self.state['real_data_file'],
            edge_limit_file=self.state['edge_limit_file'],
            net_file=self.state['net_file'],
            iop_file=self.state.get('iop_file', None),
            taz_file=self.state.get('taz_file', None)
        ) 
        
        # 执行OD微调训练循环
        reward_max = -9999
        best_sumodata = None
        history_average_speeds_best = None
        
        for step in range(max_od_iterations):
            logger.info(f"OD微调步骤 {step+1}/{max_od_iterations}")
            
            # 执行一步仿真-分析-调整
            state_info, reward, done, info, av_speed, sumodata, history_average_speeds = \
                self.simulation_runner.step(self.comparator)
            logger.info(f'best: {history_average_speeds}')
            if reward>0.81:
                break
            if reward > reward_max:
                reward_max = reward
                best_sumodata = sumodata
                history_average_speeds_best = history_average_speeds
                shutil.copy(self.simulation_runner.iop_file, self.state['best_rou_file'])
                logger.info(f"发现更好结果，评分: {info['score']:.4f}, 奖励: {reward:.4f}")
                  
            if done:
                logger.info(f"在第{step+1}步达到收敛条件")
                break
                
            logger.info(f"步骤 {step+1} 完成，评分: {info['score']:.4f}, 奖励: {reward:.4f}")
        
        if history_average_speeds_best:
            logger.info(f'{reward_max} 最佳分数')
            logger.info(f'{history_average_speeds_best}')
        if best_sumodata:
            dumpfile(history_average_speeds_best, 'pkl/history_average_speeds_best_Manha.pkl')
            dumpfile(best_sumodata, 'pkl/best_sumodata.pkl')

    def run_single_simulation(self):
        """运行单次仿真测试"""
        logger.info("执行常规单次仿真测试")
        
        # 运行仿真
        real_data = loadfile(self.state.get("real_data_file", ""))
        sim_metrics_file = self.simulation_runner.run_simulation_with_traci(
            self.state.get("net_file", ""), 
            self.state.get("rou_file", ""),
            real_data
        )
        
        # 比较仿真结果与真实数据
        score = self.comparator.compare(
            self.state.get("real_data_file", ""),
            self.state.get("real_data_type", "pkl"),
            sim_metrics_file['average_speeds'],
            sim_metrics_file['average_waiting_vehicles'],
            self.state.get("edge_limit_file", "")
        )
        
        self.state["current_score"] = score
        return score

    def run_multi_scale_test(self):
        """运行多scale测试"""
        optimizer_config = self.config.get('optimizer', {})
        test_scales = optimizer_config.get('test_scales', [1.0])
        
        logger.info(f"执行多scale测试 (exploration_counter: {self.state['exploration_counter']})")
        
        scale_results = []
        
        for scale in test_scales:
            logger.info(f"测试scale: {scale}")
             
            # 使用当前scale调整路由文件
            temp_rou_file = self.scale_adjuster.apply_adjustment(
                self.state.get("rou_file", ""), 
                {'scale': scale},
                'testscale'
            )
            
            # 运行仿真 
            real_data = loadfile(self.state.get("real_data_file", ""))
            sim_metrics_file = self.simulation_runner.run_simulation_with_traci(
                self.state.get("net_file", ""), 
                temp_rou_file,
                real_data
            )
            
            # 比较仿真结果与真实数据
            score = self.comparator.compare(
                self.state.get("real_data_file", ""),
                self.state.get("real_data_type", "pkl"),
                sim_metrics_file['average_speeds'],
                sim_metrics_file['average_waiting_vehicles'],
                self.state.get("edge_limit_file", "")
            )
            
            # 保存测试结果
            scale_result = {
                'scale': scale,
                'score': score,
                'rou_file': temp_rou_file,
                'sim_metrics': sim_metrics_file
            }
            scale_results.append(scale_result)
            logger.info(f"Scale {scale} 得分: {score:.4f}")
            self.state["iteration"] += 1
        
        # 选择最佳结果
        best_result = min(scale_results, key=lambda x: x['score'])
        self.state["scale_test_results"] = scale_results
        
        # 更新状态为最佳结果
        self.state["current_score"] = best_result['score']
        self.state["rou_file"] = best_result['rou_file']
        
        logger.info(f"最佳scale: {best_result['scale']}, 得分: {best_result['score']:.4f}")
        
        return best_result

    def run(self):
        """运行仿真任务主流程"""
        try:
            # 每次执行后exploration_counter加一
            self.state["exploration_counter"] = self.state.get("exploration_counter", 0) + 1
            
            optimizer_config = self.config.get('optimizer', {})
            exploration_limit = optimizer_config.get('exploration_counter_limit', 10)
            
            # 如果exploration_counter < 10，执行多次scale测试
            if self.state["exploration_counter"] < exploration_limit:
                best_result = self.run_multi_scale_test()
                
                # 调用迭代优化
                self.iterative_optimization()
                
            else:
                # exploration_counter >= 10时，使用常规单次测试
                score = self.run_single_simulation()
            
            # 更新状态
            self.state["iteration"] = self.state.get("iteration", 0) + 1
            
            # 更新最佳结果
            current_score = self.state.get("current_score", float('inf'))
            best_score = self.state.get("best_score", float('inf'))
            
            if current_score > best_score:
                self.state["best_score"] = current_score
                self.state["best_params"] = {
                    'scale': getattr(self.scale_adjuster, 'current_scale', 1.0),
                    'net_file': self.state.get("net_file", ""),
                    'rou_file': self.state.get("rou_file", "")
                }
                self.state["convergence_count"] = 0
            else:
                self.state["convergence_count"] = self.state.get("convergence_count", 0) + 1
            
            logger.info(f"仿真评分: {current_score:.4f}, 最佳评分: {self.state['best_score']:.4f}")
            
            return {
                'current_score': current_score,
                'best_score': self.state['best_score'],
                'iteration': self.state['iteration'],
                'state': self.state
            }
            
        except Exception as e:
            logger.error(f"执行仿真任务出错: {e}", exc_info=True)
            raise


def main():
    """主执行函数 - 用于独立测试"""
    task = SimuTask("configs/simu_task.yml")
    result = task.run()
    logger.info(f"仿真任务完成，结果: {result}")


if __name__ == "__main__":
    main()
