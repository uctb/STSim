import os
import re
import json
import traci
import pandas as pd
from sumolib.net import readNet
from shapely.geometry import LineString
import xml.etree.ElementTree as ET
import pyproj
import csv

simulation_config = {
    "sumo_cfg": "Core_500m_test_normal.sumocfg",
    "output_dir": "simulation_results",
    "net_path": "core_withshape_with_light_changing.net.xml",
    "road_data_csv": "actual_speed_variation.csv",
}

if not os.path.exists(simulation_config['output_dir']):
    os.makedirs(simulation_config['output_dir'])
    print(f"创建输出目录: {simulation_config['output_dir']}")
    

tree = ET.parse(simulation_config['net_path'])
root = tree.getroot()
location_tag = root.find("location")
netOffset_str = location_tag.attrib["netOffset"]
projParameter = location_tag.attrib["projParameter"]
netOffsetX, netOffsetY = map(float, netOffset_str.split(","))
transformer = pyproj.Transformer.from_proj(
    pyproj.Proj(projParameter),
    pyproj.Proj(proj="latlong", datum="WGS84"),
    always_xy=True
)


def init_sumo():
    sumo_cmd = [
        "sumo", 
        "-c", 
        simulation_config['sumo_cfg']
    ]
    traci.start(sumo_cmd)
    print("SUMO连接已建立")
    return traci


def coordinate_transform(transformer, netOffsetX, netOffsetY, x_sumo, y_sumo):
    x_proj = x_sumo - netOffsetX
    y_proj = y_sumo - netOffsetY
    lon, lat = transformer.transform(x_proj, y_proj)
    return (lon, lat)


def load_road_data(csv_path):
    df = pd.read_csv(csv_path)
    return df


def string_to_linestring(linestring_str):
    coordinates_str = re.findall(r'\(([^)]+)', linestring_str)
    if not coordinates_str:
        raise ValueError("Invalid LINESTRING format")
    coordinates = []
    for coord in coordinates_str[0].split(','):
        x, y = map(float, coord.strip().split())
        coordinates.append((x, y))
    linestring = LineString(coordinates)
    return linestring


def is_road_matched(short_line_coords, long_line_coords, tolerance=1e-6):
    short_line = LineString(short_line_coords)
    long_line = string_to_linestring(long_line_coords)
    long_line_buffer = long_line.buffer(tolerance)
    if long_line_buffer.contains(short_line):
        return True
    intersection = short_line.intersection(long_line_buffer)
    if intersection.length >= short_line.length * (1 - tolerance):
        return True
    return False


def get_edge_coordinates(net, edge_id):
    edge = net.getEdge(edge_id)
    shape = edge.getShape()
    coords_lst = []
    coord1, coord2 = (float(shape[0][0]), float(shape[0][1])), (float(shape[1][0]), float(shape[1][1]))
    return [coord1, coord2]


def match_edges_to_roads(net, road_df):
    matched_data = [] 
    for edge in net.getEdges():
        edge_id = edge.getID()
        edge_coords = get_edge_coordinates(net, edge_id)
        for _, road in road_df.iterrows():
            if is_road_matched((coordinate_transform(transformer, netOffsetX, netOffsetY, edge_coords[0][0], edge_coords[0][1]), coordinate_transform(transformer, netOffsetX, netOffsetY, edge_coords[1][0], edge_coords[1][1])), road['geometry'], 0.0002):
                matched_data.append({
                    'edge_id': edge_id,
                    'road_name': road['name'],
                    'direction': road['direction'],
                    'normal_speed': road['normal_speed'],
                    'geometry': road['geometry']
                })
    return pd.DataFrame(matched_data)
    
    
def run_simulation_scale(matched_df, output_json_path="simulation_results\\normal_average_speeds_scale.json"):
    time_road_speeds = {}
    last_saved_count = 8000
    while traci.vehicle.getIDCount() <= 26000:
        traci.simulationStep()
        current_count = traci.vehicle.getIDCount()
        if (current_count >= 10000) and (current_count - last_saved_count >= 2000):
            current_speeds = {}
            for edge_id in matched_df['edge_id']:
                vehicles = traci.edge.getLastStepVehicleIDs(edge_id)
                if vehicles:
                    avg_speed = sum(traci.vehicle.getSpeed(veh) for veh in vehicles) / len(vehicles)
                    current_speeds[edge_id] = avg_speed
                else:
                    current_speeds[edge_id] = None
            time_road_speeds[int(current_count/2000)*2000] = current_speeds
            last_saved_count = current_count
    with open(output_json_path, "w") as f:
        json.dump(time_road_speeds, f, indent=4)
    traci.close()


if __name__ == '__main__':
    net = readNet(simulation_config['net_path'])
    road_df = load_road_data(simulation_config['road_data_csv'])
    df = match_edges_to_roads(net, road_df)
    init_sumo()
    run_simulation_scale(df)