# 1. Assign a block to a truck; 2. Greedy strategy for each truck to choose the direction with the highest traffic flow at each intersection.

import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
import random
from collections import defaultdict
import networkx as nx
import heapq

def point_in_rectangle(point, rect):
    x, y = point
    return (rect["min_x"] <= x <= rect["max_x"] and 
            rect["min_y"] <= y <= rect["max_y"])
    
def euclid_dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

regions = {
    "region1": {"min_x": 7500, "min_y": -36.63, "max_x": 12469.37, "max_y": 2000},
    "region2": {"min_x": 7500, "min_y": 2000, "max_x": 12611.07, "max_y": 4702.42},
    "region3": {"min_x": -355.68, "min_y": 2500, "max_x": 4785.48, "max_y": 7506.39},
    "region4": {"min_x": -153.18, "min_y": -244.72, "max_x": 5500, "max_y": 2500}
}
region_colors = {
    "region1": "red",
    "region2": "blue", 
    "region3": "green",
    "region4": "orange",
    "region5": "purple"
}
start_edges = {
    "region1": "200000627",
    "region2": "200061365",
    "region3": "200062111",
    "region4": "200014459",
    "region5": "200002088"
}
net_file_path = "core_withshape_with_light_changing.net.xml"
tree = ET.parse(net_file_path)
root = tree.getroot()
region_edges = {
    "region1": [],
    "region2": [],
    "region3": [],
    "region4": [],
    "region5": []
}
region_paths = {
    "region1": [],
    "region2": [],
    "region3": [],
    "region4": [],
    "region5": []
}
route_len_lst = []

# # Assign each edge to a region
edges_info = {}
for edge in root.findall('edge'):
    edge_id = edge.get('id')
    if edge_id.startswith(':'):
        continue
    shape_attr = edge.get('shape')
    if shape_attr is None:
        lanes = edge.findall('lane')
        if lanes:
            shape_attr = lanes[0].get('shape')
    if shape_attr:
        coords = [tuple(map(float, point.split(','))) for point in shape_attr.split()]
        center_x = sum(coord[0] for coord in coords) / len(coords)
        center_y = sum(coord[1] for coord in coords) / len(coords)
        center_point = (center_x, center_y)
        edges_info[edge_id] = {
            'coords': coords,
            'center': center_point,
            'region': None
        }
        assigned = False
        for region_name, rect in regions.items():
            if point_in_rectangle(center_point, rect):
                region_edges[region_name].append(edge_id)
                edges_info[edge_id]['region'] = region_name
                assigned = True
                break
        if not assigned:
            region_edges["region5"].append(edge_id)
            edges_info[edge_id]['region'] = "region5"

# # Visualize the region split;
# fig, ax = plt.subplots(figsize=(15, 12))
# for region_name, rect in regions.items():
#     width = rect["max_x"] - rect["min_x"]
#     height = rect["max_y"] - rect["min_y"]
#     rect_patch = patches.Rectangle(
#         (rect["min_x"], rect["min_y"]), width, height,
#         linewidth=3, edgecolor=region_colors[region_name], 
#         facecolor=region_colors[region_name],  # 半透明填充
#         label=f'{region_name} ({len(region_edges[region_name])}条)',
#         alpha=0.3
#     )
#     ax.add_patch(rect_patch)
#     ax.text(rect["min_x"] + width/2, rect["min_y"] + height/2, 
#             region_name, ha='center', va='center', 
#             fontsize=12, fontweight='bold',
#             bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
# for edge_id, info in edges_info.items():
#     coords = info['coords']
#     region = info['region']
#     color = region_colors.get(region, 'gray')
#     coords_array = np.array(coords)
#     ax.plot(coords_array[:, 0], coords_array[:, 1], 
#            color=color, linewidth=1.2, alpha=0.8)
# ax.set_xlabel('X Coordinate', fontsize=12)
# ax.set_ylabel('Y Coordinate', fontsize=12)
# ax.set_title('SUMO道路网络区域划分可视化\n(按道路中心点坐标分配)', fontsize=14, fontweight='bold')
# ax.grid(True, alpha=0.2)
# ax.legend(loc='upper right', fontsize=10)
# all_x = []
# all_y = []
# for rect in regions.values():
#     all_x.extend([rect["min_x"], rect["max_x"]])
#     all_y.extend([rect["min_y"], rect["max_y"]])
# margin_x = (max(all_x) - min(all_x)) * 0.1
# margin_y = (max(all_y) - min(all_y)) * 0.1
# ax.set_xlim(min(all_x) - margin_x, max(all_x) + margin_x)
# ax.set_ylim(min(all_y) - margin_y, max(all_y) + margin_y)
# ax.set_aspect('equal')
# plt.tight_layout()
# plt.show()
# plt.savefig('sumo_network_regions.png', dpi=300, bbox_inches='tight')
# print("可视化图像已保存为 'sumo_network_regions.png'")

route_file_path = "mapcore_500m_core_withshape_with_light_test.rou.xml"
target_percentages = [5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]

# Initialize Traffic Flow
traffic_flow = defaultdict(int)
tree = ET.parse(route_file_path)
root_route = tree.getroot()
for vehicle in root_route.findall('vehicle'):
    route = vehicle.find('route')
    if route is not None and route.get('edges'):
        edges = route.get('edges').split()
        for edge in edges:
            if not edge.startswith(':'):
                traffic_flow[edge] += 1
for flow in root_route.findall('flow'):
    route = flow.find('route')
    if route is not None and route.get('edges'):
        edges = route.get('edges').split()
        for edge in edges:
            if not edge.startswith(':'):
                traffic_flow[edge] += int(flow.get('number', 1))

# initialize the graph
all_nodes = set()
G = nx.DiGraph()
for edge in root.findall('edge'):
    edge_id = edge.get('id')
    if edge_id.startswith(':'):
        continue
    from_node = edge.get('from')
    to_node = edge.get('to')
    if from_node:
        all_nodes.add(from_node)
    if to_node:
        all_nodes.add(to_node)
for node in all_nodes:
    G.add_node(node)
for edge in root.findall('edge'):
    edge_id = edge.get('id')
    if edge_id.startswith(':'):
        continue
    from_node = edge.get('from')
    to_node = edge.get('to')
    if from_node and to_node:
        lanes = edge.findall('lane')
        edge_length = float(lanes[0].get('length'))
        lane_count = len(lanes) if lanes else 1
        G.add_edge(from_node, to_node, 
                         id=edge_id,
                         length = edge_length,
                         lane_count=lane_count,
                         total_length=edge_length*lane_count)

# Initialize node position;    
node_positions = {} 
for edge in root.findall('edge'):
    edge_id = edge.get('id')
    if edge_id.startswith(':'):
        continue
    lanes = edge.findall('lane')
    if lanes:
        shape_attr = lanes[0].get('shape')
    if shape_attr:
        coords = [tuple(map(float, p.split(','))) for p in shape_attr.split()]
        if len(coords) >= 2:
            from_coord = coords[0]
            to_coord = coords[-1]
            from_node = edge.get("from")
            to_node = edge.get("to")
            if from_node in G.nodes and from_node not in node_positions:
                node_positions[from_node] = from_coord
            if to_node in G.nodes and to_node not in node_positions:
                node_positions[to_node] = to_coord
                
 
network_total_length = 0
for edge in G.edges(data=True):
    network_total_length += edge[2]['total_length']
    
edge_data_dict = {}
node_edges_dict = {} 
for u, v, d in G.edges(data=True):
    eid = d["id"]
    edge_data_dict[eid] = {
        "from": u,
        "to": v,
        "single_length": float(d["length"]),
        "lane_count": d["lane_count"],
        "length": float(d["length"]) * d["lane_count"],
        "flow": traffic_flow.get(eid, 0),
        "region": None
    }
    node_edges_dict.setdefault(u, []).append(eid)
for region, edges in region_edges.items():
    for eid in edges:
        if eid in edge_data_dict:
            edge_data_dict[eid]["region"] = region

car_states = {}
for region, start_node in start_edges.items():
    car_states[region] = {
        "current_node": edge_data_dict[start_node]["from"],
        "cleaned_edges": set(),
        "cleaning_paths": [],
        "cleaned_length": 0,
        "penalty": 0
    }
path_num = 0
for region in region_edges.keys():
    cleaned_edges = set()
    state = car_states[region]
    sub_G = nx.DiGraph()
    for u, v, d in G.edges(data=True):
        if d["id"] in region_edges[region]:
            sub_G.add_edge(u, v, **d)
            if u in G.nodes:
                sub_G.add_node(u, **G.nodes[u])
            if v in G.nodes:
                sub_G.add_node(v, **G.nodes[v])
    for i in range(1000):
        
        current_node = state["current_node"]
        successors = []
        uncleaned_successors = []
        
        # Greedy Strategy
        for kk, vv, d in sub_G.out_edges(current_node, data=True):
            if d["id"] not in cleaned_edges:
                uncleaned_successors.append((kk, vv, d))
            successors.append((kk, vv, d))
        if len(uncleaned_successors) > 0:
            print("uncleaned")
            uncleaned_successors.sort(key=lambda x: traffic_flow[x[2]["id"]], reverse=True)
            this_step_start_node, next_node, chosen_edge = uncleaned_successors[0]
            if chosen_edge['lane_count'] % 2 == 0:
                this_step_next_node = this_step_start_node
            else:
                this_step_next_node = next_node
            cleaned_edges.add(chosen_edge["id"])
            state["cleaned_edges"].add(chosen_edge["id"])
            edge_len = edge_data_dict[chosen_edge["id"]]["length"]
            state["cleaned_length"] += edge_len * edge_data_dict[chosen_edge["id"]]["lane_count"]
            state["cleaning_paths"].extend([chosen_edge["id"]]*edge_data_dict[chosen_edge["id"]]["lane_count"])
            state["current_node"] = this_step_next_node
        else:
            print("cleaned")
            candidates = [eid for eid in region_edges[region] if eid in edge_data_dict and eid not in cleaned_edges]
            if len(candidates) == 0:
                break
            else:
                candidate_nodes = []
                for eid in candidates:
                    ed = edge_data_dict[eid]
                    if ed["from"] != current_node:
                        candidate_nodes.append(ed["from"])
                candidate_nodes = list(set(candidate_nodes))
                
                def node_min_dist(nid):
                    if nid in node_positions:
                        return euclid_dist(node_positions[current_node], node_positions[nid])
                    else:
                        return float("inf") 
                
                candidate_node_list = heapq.nsmallest(min(10, len(candidates)), candidate_nodes, key=node_min_dist)
                path_found = False
                for nid in candidate_node_list:
                    try:
                        path_nodes = nx.shortest_path(sub_G, source=current_node, target=nid, weight='length')
                        path_found = True
                        temp_route_length = 0
                        for j in range(len(path_nodes) - 1):
                            u, v = path_nodes[j], path_nodes[j + 1]
                            elen = sub_G[u][v]["length"]
                            temp_route_length += elen
                            eid = sub_G[u][v]["id"]
                            if eid not in cleaned_edges:
                                route_len_lst.append(temp_route_length)
                                cleaned_edges.add(eid)
                                state["cleaned_edges"].add(eid)
                                edge_len = edge_data_dict[eid]["length"]
                                state["cleaned_length"] += edge_len * edge_data_dict[eid]["lane_count"]
                                state["cleaning_paths"].extend([eid] * edge_data_dict[eid]["lane_count"])
                                if edge_data_dict[eid]["lane_count"] % 2 == 0:
                                    this_step_next_node = u
                                else:
                                    this_step_next_node = v
                                state["current_node"] = this_step_next_node
                                break
                        state["current_node"] = nid
                        path_num += 1
                        route_len_lst.append(temp_route_length)
                        break
                    except:
                        print("no paths")
                        continue
                if not path_found:
                    state["current_node"] = candidate_node_list[0]
                    state['cleaning_paths'].append("penalty")
                    print("Warning: No path found, using the closest node")
    region_paths[region] = state["cleaning_paths"]


# # Visualize the snow plowing process
# import matplotlib.pyplot as plt
# import networkx as nx
# from matplotlib.animation import FuncAnimation
# path_edge_ids = region_paths["region5"][:100]
# cleaned_set = set() 
# pos = node_positions
# fig, ax = plt.subplots(figsize=(12, 12))
# nx.draw_networkx_nodes(G, pos, node_size=40, node_color="gray", alpha=0.8, ax=ax)
# edges_artist = nx.draw_networkx_edges(G, pos, edge_color="lightgray", width=1, alpha=0.4, ax=ax)
# def update(frame):
#     ax.clear()
#     cleaned_set = set(path_edge_ids[:frame+1]) 
#     path_edges = [(u, v) for u, v, d in G.edges(data=True) if d["id"] in cleaned_set]
#     other_edges = [(u, v) for u, v, d in G.edges(data=True) if d["id"] not in cleaned_set]
#     nx.draw_networkx_nodes(G, pos, node_size=40, node_color="gray", alpha=0.8, ax=ax)
#     nx.draw_networkx_edges(G, pos, edgelist=other_edges, edge_color="lightgray", width=1, alpha=0.4, ax=ax)
#     nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="red", width=2.5, alpha=0.8, ax=ax)
#     if frame < len(path_edge_ids):
#         last_edge_id = path_edge_ids[frame]
#         for u, v, d in G.edges(data=True):
#             if d["id"] == last_edge_id:
#                 nx.draw_networkx_nodes(G, pos, nodelist=[v], node_size=80, node_color="blue", ax=ax)
#                 break
#     ax.set_title(f"Step {frame+1}", fontsize=14)
#     ax.axis("off")
# ani = FuncAnimation(fig, update, frames=len(path_edge_ids), interval=5)  # 0.5秒/步
# plt.show()


# Snow plowing strategy execution
speed_clean = 1  # 清扫速度
speed_pass = 2   # 路过速度
percent_steps = [i/20 for i in range(1, 21)]  # 5%, 10%, ..., 100%
car_times = {}       # region -> list of (eid, cumulative_time)
car_total_times = {} # region -> 总时间
total_time = 0       # 所有车完成最长时间
for region, path in region_paths.items():
    t = 0
    cleaned_set = set()
    seq = []
    for eid in path:
        # Penalty when no available paths
        if eid == "penalty":
            length = 6000
        else:
            length = edge_data_dict[eid]["length"]
        if eid not in cleaned_set:
            t += length / speed_clean
            cleaned_set.add(eid)
        else:
            t += length / speed_pass
        seq.append((eid, t))
    car_times[region] = seq
    car_total_times[region] = t
    total_time = max(total_time, t)  # 更新总时间
print("每辆车完成时间：")
for region, t in car_total_times.items():
    print(f"{region}: {t:.2f}")
print(f"\n总时间（五辆车最长完成时间）: {total_time:.2f}")

progress_cumulative = set() 
time_cleaned_edges = {}
for p in range(1, 21):
    current_time = total_time * (p * 0.05)
    edges_done = set()
    for seq in car_times.values():
        for eid, t in seq:
            if t <= current_time:
                edges_done.add(eid)
            else:
                break
    progress_cumulative.update(edges_done)
    time_cleaned_edges[p*5] = edges_done
    print(f"{int(p*5)}% 时间，累积扫过的边数量: {len(progress_cumulative)}")


#Simulation environment
import os
import json
import traci
import random
import time
import sumolib
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from collections import defaultdict

NET_FILE = "core_withshape_with_light_changing.net.xml"

def get_cleared_edges_greedy(ratio):
    if ratio == 0:
        return []
    return list(time_cleaned_edges[ratio])

# Evaluate the snow plowing strategy in the sumo simulation environment
def run_simulation(ratio):
    res = {}
    traci.start(["sumo", "-c", "Core_500m_test.sumocfg"])
    cleared_edges = get_cleared_edges_greedy(ratio)
    lane_ids = traci.lane.getIDList()
    last_saved_count = 3000
    while len(traci.vehicle.getIDList()) <=12000:
        traci.simulationStep()
        current_vehicles = traci.vehicle.getIDList()
        current_count = len(current_vehicles)
        for veh_id in traci.vehicle.getIDList():
            current_edge = traci.vehicle.getRoadID(veh_id)
            if current_edge in cleared_edges:
                traci.vehicle.setAcceleration(veh_id, 2.6, 1)
                traci.vehicle.setDecel(veh_id, 4.5)
                traci.vehicle.setMaxSpeed(veh_id, 33)
                traci.vehicle.setMinGap(veh_id, 2.5)
            else:
                traci.vehicle.setAcceleration(veh_id, 1.5, 1)
                traci.vehicle.setDecel(veh_id, 2.5)
                traci.vehicle.setMaxSpeed(veh_id, 20)
                traci.vehicle.setMinGap(veh_id, 4)
        if (current_count >= 3000) and (current_count - last_saved_count >= 1000):
            congested_lanes = 0
            lane_total_queue = 0
            last_saved_count = current_count
            for lane_id in lane_ids:
                lane_queue = traci.lane.getLastStepHaltingNumber(lane_id)
                lane_total_queue += lane_queue
                if lane_queue > 10:
                    congested_lanes += 1
            lane_cong_ratio = congested_lanes / len(lane_ids)
            lane_avg_queue_len = lane_total_queue / len(lane_ids)
            global_avg_speed = sum(traci.vehicle.getSpeed(veh) for veh in traci.vehicle.getIDList()) / max(1, len(traci.vehicle.getIDList()))
    
            junction_metrics = {}
            junction_ids = traci.junction.getIDList()
            global_total_queue = 0
            global_total_speed = 0
            global_vehicle_count = 0

            for junction_id in junction_ids:
                inc_edges = traci.junction.getIncomingEdges(junction_id)
                total_queue = 0
                total_speed = 0
                lane_count = 0
                vehicle_count = 0
                for edge_id in inc_edges:
                    lane_num = traci.edge.getLaneNumber(edge_id)
                    temp_lane_lst = []
                    for t in range(lane_num):
                        temp_lane_lst.append(f"{edge_id}_{t}")
                    for lane_id in temp_lane_lst:
                        queue = traci.lane.getLastStepHaltingNumber(lane_id)
                        total_queue += queue
                        lane_count += 1
                        vehicles = traci.lane.getLastStepVehicleIDs(lane_id)
                        for veh_id in vehicles:
                            speed = traci.vehicle.getSpeed(veh_id)
                            total_speed += speed
                            vehicle_count += 1
                avg_queue = total_queue / lane_count if lane_count > 0 else 0
                avg_speed = total_speed / vehicle_count if vehicle_count > 0 else 35
                junction_metrics[junction_id] = {
                            "avg_queue": avg_queue,
                            "avg_speed": avg_speed,
                            "total_queue": total_queue,
                            "is_congested": avg_queue > 10 or avg_speed < 5  # 自定义拥堵标准
                        }
            
            total_junctions = len(junction_ids)
            congested_junctions = sum(1 for metrics in junction_metrics.values() if metrics["is_congested"])
            junction_cong_ratio = congested_junctions / total_junctions if total_junctions > 0 else 0
            sum_junction_len = 0
            sum_junction_speed = 0
            for k, v in junction_metrics.items():
                sum_junction_len += v['total_queue']
                sum_junction_speed += v['avg_speed']
            junction_avg_queue_len = sum_junction_len / total_junctions
            junction_avg_speed = sum_junction_speed / total_junctions
            print(junction_avg_queue_len)
            res[int(current_count / 1000)*1000] = [lane_cong_ratio, lane_avg_queue_len, global_avg_speed, junction_cong_ratio, junction_avg_queue_len, junction_avg_speed]
            print(res)
    return res


if __name__ == "__main__":
    results = {}
    for ratio in [0, 20, 40, 60, 80, 100]:
        results[ratio] = run_simulation(ratio)
        traci.close()
        with open('snowplow_res.json', 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=4, ensure_ascii=False)