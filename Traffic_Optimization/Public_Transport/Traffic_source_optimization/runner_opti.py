"""
优化任务的仿真运行器
整合了test_RL.py的功能，用于优化任务（交通灯控优化和变道优化）
"""

from utils.car_simulate_opti import LTF_Traci
import os
import pickle


def get_all_tls_ids_from_netxml(netxml_path):
    """
    读取SUMO网络文件（.net.xml），获取所有信号灯（traffic light）的id，返回id列表。

    参数:
        netxml_path (str): .net.xml文件路径

    返回:
        list: 所有信号灯id的列表
    """
    import xml.etree.ElementTree as ET
    tls_ids = []
    tree = ET.parse(netxml_path)
    root = tree.getroot()
    for tlLogic in root.findall('tlLogic'):
        tl_id = tlLogic.get('id')
        if tl_id is not None:
            tls_ids.append(tl_id)
    return tls_ids


def run_simulation(net_file_path: str, end_time: int = 500, use_gui: bool = False,
                   route_file: str = "tool/test_RL/hangzhou.rou.xml", cal_reward_circle: int = 1,
                   rl_tls_ids=[], rl_mode='ppo', num_episodes: int = 1,
                   reward_static_pkl: str = "pkl/reward_static_all.pkl",
                   non_rl_policy: str = 'greedy',
                   training: bool = True,
                   checkpoint_dir: str = 'test_pth/rl_models',
                   rl_config: dict = None,
                   action_interval: int = 5,
                   learning_start_time: int = 256,
                   update_model_rate: int = 10,
                   update_target_rate: int = 100):
    """
    运行一次SUMO仿真，并返回新生成的决策数据。

    Args:
        net_file_path (str): 用于仿真的 .net.xml 文件路径。
        end_time (int): 仿真结束时间。
        use_gui (bool): 是否使用SUMO GUI。
        route_file (str): 路由文件路径
        cal_reward_circle (int): 奖励计算周期
        rl_tls_ids (list): 强化学习控制的交通灯ID列表
        rl_mode (str): 强化学习模式
        num_episodes (int): 训练轮数
        reward_static_pkl (str): 静态奖励文件路径
        non_rl_policy (str): 非强化学习策略
        training (bool): 是否为训练模式
        checkpoint_dir (str): 模型检查点目录
        rl_config (dict): 强化学习相关配置
        action_interval (int): 动作间隔
        learning_start_time (int): 学习开始时间
        update_model_rate (int): 模型更新频率
        update_target_rate (int): 目标网络更新频率
        
    Returns:
        Tuple[Dict, Dict]: (tl_phase_result, lane_averages)
    """
    print(f"=== 开始在路网 {os.path.basename(net_file_path)} 上运行仿真 ===")
    
    if rl_config is None:
        rl_config = {}

    ltf = LTF_Traci(
        net_file=net_file_path,
        route_file=route_file,
        use_gui=use_gui,
        end_time=end_time,
        rl_mode=rl_mode,
        rl_tls_ids=rl_tls_ids,
        non_rl_policy=non_rl_policy,
        training=training,
        checkpoint_dir=checkpoint_dir,
        rl_config=rl_config,
        num_episodes=num_episodes,
        action_interval=action_interval,
        learning_start_time=learning_start_time,
        update_model_rate=update_model_rate,
        update_target_rate=update_target_rate,
        cal_reward_circle=cal_reward_circle,
        reward_static_pkl=reward_static_pkl
    )

    try:
        tl_phase_result, lane_averages = ltf.run()
        print(f"仿真完成")
        return tl_phase_result, lane_averages
        
    except Exception as e:
        print(f"仿真过程中出现错误: {e}")
        raise  # 将异常向上抛出，以便主流程捕获


class OptimizationRunner:
    """优化仿真运行器类，封装优化任务的通用仿真功能"""
    
    def __init__(self, config):
        """
        初始化优化运行器
        
        Args:
            config (dict): 配置字典
        """
        self.config = config
        self.rl_config = config.get('rl_config', {})
        self.data_paths = config.get('data_paths', {})
        
    def run_optimization_simulation(self, net_file_path: str, route_file: str = None,
                                  rl_tls_ids: list = None, **kwargs):
        """
        运行优化仿真
        
        Args:
            net_file_path (str): 网络文件路径
            route_file (str): 路由文件路径
            rl_tls_ids (list): 强化学习交通灯ID列表
            **kwargs: 其他参数
            
        Returns:
            tuple: (tl_phase_result, lane_averages)
        """
        # 使用配置中的默认值，如果参数未提供
        route_file = route_file or self.data_paths.get('route_file')
        rl_tls_ids = rl_tls_ids or self.config.get('rl_tls_ids', [])
        
        # 合并参数
        sim_params = {
            'end_time': self.rl_config.get('end_time', 500),
            'use_gui': self.rl_config.get('use_gui', False),
            'cal_reward_circle': self.rl_config.get('cal_reward_circle', 5),
            'rl_mode': self.rl_config.get('algorithm', 'ppo'),
            'num_episodes': self.rl_config.get('episodes', 1),
            'reward_static_pkl': self.data_paths.get('reward_static_pkl'),
            'non_rl_policy': self.config.get('non_rl_policy', 'greedy'),
            'training': self.config.get('training', True),
            'checkpoint_dir': self.rl_config.get('checkpoint_dir', 'test_pth/rl_models'),
            'rl_config': self.rl_config,
            'action_interval': self.rl_config.get('action_interval', 5),
            'learning_start_time': self.rl_config.get('learning_start_time', 256),
            'update_model_rate': self.rl_config.get('update_model_rate', 10),
            'update_target_rate': self.rl_config.get('update_target_rate', 100),
        }
        sim_params.update(kwargs)
        
        return run_simulation(
            net_file_path=net_file_path,
            route_file=route_file,
            rl_tls_ids=rl_tls_ids,
            **sim_params
        )


if __name__ == "__main__":
    # 这个 main 模块现在主要用于独立测试此脚本
    default_net_file = "hangzhou_test9.net.xml"
    print(f"--- 独立测试 run_simulation (路网: {default_net_file}) ---")
    # 独立测试时，我们只关心它是否能跑完
    run_simulation(net_file_path=default_net_file, use_gui=True, rl_config={
        'algorithm': 'ppo',
        'reward_config': {
            'local_weight': 0.75,
            'rl_group_weight': 0.15,
            'global_weight': 0.10,
            'use_local_queue': True,
            'use_local_waiting': True,
            'use_local_throughput': True
        },
        'ppo_params': {
            'lr': 3e-4, 'n_steps': 256, 'batch_size': 16, 'n_epochs': 10,
            'gamma': 0.99, 'gae_lambda': 0.95, 'clip_range': 0.2
        }
    })
