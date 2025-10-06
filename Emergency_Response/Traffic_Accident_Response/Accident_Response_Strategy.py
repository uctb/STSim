from networkx.algorithms.simple_paths import shortest_simple_paths
import xml.etree.ElementTree as ET
import networkx as nx
import pandas as pd
import math
import traci
import random
import time

def sumo_net_to_networkx(net_file_path):
    tree = ET.parse(net_file_path)
    root = tree.getroot()
    G = nx.DiGraph()
    junction_positions = {}
    for junction in root.findall("junction"):
        node_id = junction.get("id")
        x = float(junction.get("x"))
        y = float(junction.get("y"))
        junction_positions[node_id] = (x, y)
    edge_info = {}
    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        if edge.get("from") is None or edge.get("to") is None:
            continue
        from_node = f"{edge_id}_out"
        to_node = f"{edge_id}_in"
        lane = edge.find("lane")
        length = float(lane.get("length")) if lane is not None else 0.0
        edge_info[edge_id] = (from_node, to_node, length)
        G.add_edge(from_node, to_node, edge_id=edge_id, length=length)
    for conn in root.findall("connection"):
        from_edge = conn.get("from")
        to_edge = conn.get("to")
        if from_edge in edge_info and to_edge in edge_info:
            # _, from_edge_end, _ = edge_info[from_edge]
            # to_edge_start, _, _ = edge_info[to_edge]
            G.add_edge(
                f"{from_edge}_in",  # e.g., "J1_out" (from_edge 的出口)
                f"{to_edge}_out",  # e.g., "J2_in"  (to_edge 的入口)
                edge_id = f"{from_edge}_in_{to_edge}_out",
                turn_type=conn.get("dir", "unknown"),  # 可选：转向类型
            )
    for node in G.nodes():
        if node.endswith("_out"):
            junc_id = node.split("_out")[0]
            if junc_id in junction_positions:
                G.nodes[node]["pos"] = junction_positions[junc_id]
        elif node.endswith("_in"):
            junc_id = node.split("_in")[0]
            if junc_id in junction_positions:
                G.nodes[node]["pos"] = junction_positions[junc_id]
    return G

def get_nodes_from_edge_id(G, edge_id):
    for (u, v, data) in G.edges(data=True):
        if data.get("edge_id") == edge_id:
            return u, v
    raise ValueError(f"Edge ID {edge_id} not found in the graph!")

def heuristic(u, v, G):
    pos_u = G.nodes[u].get("pos") 
    pos_v = G.nodes[v].get("pos")
    return math.sqrt((pos_u[0] - pos_v[0])**2 + (pos_u[1] - pos_v[1])**2)

def find_shortest_path(G, start_edge_id, end_edge_id, weight = "length"): # A_Star
    start_from, start_to = None, None
    for u, v, data in G.edges(data=True):
        print(data)
        if data.get("edge_id") == start_edge_id:
            start_from, start_to = u, v
            break
    if not start_from:
        raise ValueError(f"Edge ID {start_edge_id} not found!")
    if end_edge_id is None:
        return start_edge_id
    end_from, end_to = None, None
    for u, v, data in G.edges(data=True):
        if data.get("edge_id") == end_edge_id:
            end_from, end_to = u, v
            break
    if not end_from:
        raise ValueError(f"Edge ID {end_edge_id} not found!")

    path_nodes = nx.astar_path(
        G, 
        start_from, 
        end_to, 
        heuristic=lambda u, v: heuristic(u, v, G),
        weight = weight
    )
    path_edges = []
    for i in range(len(path_nodes) - 1):
        u, v = path_nodes[i], path_nodes[i + 1]
        edge_data = G.get_edge_data(u, v)
        if edge_data and "edge_id" in edge_data:
            path_edges.append(edge_data["edge_id"])
    return path_edges if path_edges else None

def find_k_shortest_paths(G, start_edge_id, end_edge_id=None, k=5):
    start_from, start_to = None, None
    for u, v, data in G.edges(data=True):
        if data.get("edge_id") == start_edge_id:
            start_from, start_to = u, v
            break
    if not start_from:
        raise ValueError(f"Edge ID {start_edge_id} not found!")
    if end_edge_id is None:
        return [[start_edge_id]]
    end_from, end_to = None, None
    for u, v, data in G.edges(data=True):
        if data.get("edge_id") == end_edge_id:
            end_from, end_to = u, v
            break
    if not end_from:
        raise ValueError(f"Edge ID {end_edge_id} not found!")
    # try:
    paths = []
    for path_nodes in shortest_simple_paths(G, start_from, end_to, weight="length"):
        path_edges = []
        for i in range(len(path_nodes) - 1):
            u, v = path_nodes[i], path_nodes[i + 1]
            edge_data = G.get_edge_data(u, v)
            if edge_data and "edge_id" in edge_data:
                path_edges.append(edge_data["edge_id"])
        if path_edges:
            paths.append(path_edges)
            if len(paths) >= k: 
                break
    return paths
    
def filter_lst(path):
    filtered_lst = [
    item for item in path 
    if not ("_in_" in item and "_out" in item)
]
    return filtered_lst

def is_vehicle_finished(vehicle_id, edge_id):
    current_road = traci.vehicle.getRoadID(vehicle_id)
    return current_road == edge_id

sumo_net_file = "E:\\Traffic_Simulation\\Traffic_accident\\full_net\\new_add_light.net.xml" 
input_rou = "E:\\Traffic_Simulation\\Traffic_accident\\full_rou\\mapall_addline.rou.xml"
output_rou = "E:\\Traffic_Simulation\\Traffic_accident\\full_rou\\mapall_addline_response.rou.xml"
G = sumo_net_to_networkx(sumo_net_file)
# accident case;
accident_spots = ["200042649", "200040849", "200063134", "200002421", "200040901"]
hospital_df = pd.read_csv("Hospital_Location.csv")
hospital_dict = {}
ambulance_number = {}
accident_spots_simu_flag = {}
for acs in accident_spots:
    accident_spots_simu_flag[acs] = False
for i in range(hospital_df.shape[0]):
    hospital_dict[f"{hospital_df['name'][i]}index{i+1}"] = str(hospital_df['road_id'][i])
    ambulance_number[f"{hospital_df['name'][i]}index{i+1}"] = 2
ambulance_lst = []
ambulance_id = 1
for k, v in hospital_dict.items():
    temp_index = k.split("index")[-1]
    for acc_s in accident_spots:
        ambulance_paths = find_k_shortest_paths(G, v, acc_s)
        for amb_p in ambulance_paths:
            ambulance_lst.append({'ambulance_id': ambulance_id, 'path': " ".join(filter_lst(amb_p)), 'hospital_index': temp_index, 'accident_spot': acc_s, 'pair': f"{temp_index}_{acc_s}"})
            ambulance_id += 1
ambulance_df = pd.DataFrame(ambulance_lst)
print(ambulance_df)
tree = ET.parse(input_rou)
root = tree.getroot()
for i in range(ambulance_df.shape[0]):
    new_vehicle_data = {
        'id': f"ambulance_{ambulance_df['ambulance_id'][i]}",
        'depart': '100.00',
        'color': '1,0,0',
        'type': 'ambulance'
    }
    new_vehicle = ET.Element('vehicle', new_vehicle_data)
    ET.SubElement(new_vehicle, 'route', {'edges': ambulance_df['path'][i]})
    inserted = False
    for i, vehicle in enumerate(root.findall('vehicle')):
        if float(vehicle.get('depart')) > 100.00:
            root.insert(i, new_vehicle)
            inserted = True
            break
    if not inserted:
        root.append(new_vehicle)
tree.write(output_rou, encoding='utf-8', xml_declaration=True)
traci.start(["sumo", "-c", "E:\\Traffic_Simulation\\Traffic_accident\\response.sumocfg", "--no-warnings"])
current_step = 0
arrival_time = {}
unfinished_ambulance_lst = list(ambulance_df['ambulance_id'])
finished_ambulance_lst = []
start_time = time.time()
while current_step < 1200:
    if current_step % 100 == 0:
        print(current_step)
    traci.simulationStep()
    current_step += 1
    if current_step == 101:
        for amb in unfinished_ambulance_lst:
            traci.vehicle.setSpeed(f"ambulance_{amb}", 30)
            traci.vehicle.setSpeedMode(f"ambulance_{amb}", 0)
    if current_step >= 101:
        if not all(accident_spots_simu_flag.values()):
            print(accident_spots_simu_flag)
            for acs in accident_spots:
                if (accident_spots_simu_flag[acs] == False) and (len(list(traci.edge.getLastStepVehicleIDs(acs))) > 0):
                    veh_id = random.choice(list(traci.edge.getLastStepVehicleIDs(acs)))
                    try:
                        traci.vehicle.setSpeed(veh_id, 0)
                        traci.vehicle.setStop(veh_id, acs, 
                                            traci.vehicle.getLanePosition(veh_id), 
                                            laneIndex=traci.vehicle.getLaneIndex(veh_id), 
                                            duration=9999)
                        traci.vehicle.setColor(veh_id, (255, 0, 0))
                        accident_spots_simu_flag[acs] = True
                    except: 
                        continue
        if current_step % 10 == 0:
            for amb in unfinished_ambulance_lst:
                try:
                    if is_vehicle_finished(f"ambulance_{amb}", ambulance_df['path'][amb - 1].split(" ")[-1]):
                        print(f"车辆 ambulance_{amb} 已完成模拟！")
                        arrival_time[f"ambulance_{amb}"] = current_step
                        unfinished_ambulance_lst.remove(amb)
                        finished_ambulance_lst.append(amb)
                        print(finished_ambulance_lst)
                except:
                    continue
end_time = time.time()
print(f"模拟时间: {end_time - start_time} 秒")
traci.close()
print(arrival_time)