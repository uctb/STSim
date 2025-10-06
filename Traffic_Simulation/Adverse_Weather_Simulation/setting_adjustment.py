import os
import re
import math
import json
import copy
import traci
import pandas as pd
from sumolib.net import readNet
from shapely.geometry import LineString
import xml.etree.ElementTree as ET
import pyproj
import itertools
import numpy as np
from collections import defaultdict


simulation_config = {
    "net_path": "core_withshape_with_light_changing.net.xml",
    "road_data_csv": "actual_speed_variation.csv",
}


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


def coordinate_transform(transformer, netOffsetX, netOffsetY, x_sumo, y_sumo):
    x_proj = x_sumo - netOffsetX
    y_proj = y_sumo - netOffsetY
    lon, lat = transformer.transform(x_proj, y_proj)
    return (lon, lat)


def load_road_data(csv_path):
    return pd.read_csv(csv_path)


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
    matched_edges = []
    matched_regions = []
    mapping_dict = {}
    for edge in net.getEdges():
        edge_id = edge.getID()
        edge_coords = get_edge_coordinates(net, edge_id)
        for _, road in road_df.iterrows():
            if is_road_matched((coordinate_transform(transformer, netOffsetX, netOffsetY, edge_coords[0][0], edge_coords[0][1]), coordinate_transform(transformer, netOffsetX, netOffsetY, edge_coords[1][0], edge_coords[1][1])), road['geometry'], 0.0002):
                matched_data.append({
                    'edge_id': edge_id,
                    'name': road['name'],
                    'direction': road['direction'],
                    'normal_speed': road['normal_speed'],
                    'geometry': road['geometry']
                })
                matched_edges.append(edge_id)
                matched_regions.append((road['name'], road['direction']))
                if (road['name'], road['direction']) in mapping_dict:
                    mapping_dict[(road['name'], road['direction'])].append(edge_id)
                else:
                    mapping_dict[(road['name'], road['direction'])] = [edge_id]
    print(len(set(matched_edges)))
    print(len(set(matched_regions)))
    return pd.DataFrame(matched_data), mapping_dict


def three_categories(percentage):
    if percentage > -10:
        return 1
    if percentage > -40 and percentage <= -10:
        return 2
    else:
        return 3
    
    
def calculate_acc(mapping_dict, road_df, test_dict_normal, test_dict_rain):
    average_speed_dict = {}
    acc_dataframe = []
    for k, v in mapping_dict.items():
        average_speed_dict[k] = []
        for edge in v:
            if test_dict_normal[edge] and test_dict_rain[edge] and test_dict_normal[edge]!=0:
                speed_variation = (test_dict_rain[edge] - test_dict_normal[edge]) * 100 / test_dict_normal[edge]
                average_speed_dict[k].append(speed_variation)
            if test_dict_normal[edge] and (test_dict_rain[edge] == None):
                speed_variation = (35 - test_dict_normal[edge]) * 100 / test_dict_normal[edge]
                average_speed_dict[k].append(speed_variation)
            if (test_dict_normal[edge] == None) and test_dict_rain[edge]:
                speed_variation = (test_dict_rain[edge] - 35) * 100 / 35
                average_speed_dict[k].append(speed_variation)
            if (test_dict_normal[edge] == None) and (test_dict_rain[edge] == None):
                speed_variation = 0
                average_speed_dict[k].append(speed_variation)
    for i in range(road_df.shape[0]):
        if ((road_df['name'][i], road_df['direction'][i]) in mapping_dict) and (len(average_speed_dict[(road_df['name'][i], road_df['direction'][i])]) > 0):
            temp_speed_variation = sum(average_speed_dict[(road_df['name'][i], road_df['direction'][i])]) / len(average_speed_dict[(road_df['name'][i], road_df['direction'][i])])
            acc_dataframe.append({"name": road_df['name'][i], "direction": road_df['direction'][i], "normal_speed": road_df['normal_speed'][i], "rainy_speed": road_df['rainy_speed'][i], "geometry": road_df['geometry'][i], "speed_diff_percent": road_df['speed_diff_percent'][i], "speed_variation": temp_speed_variation, "GT_category": three_categories(road_df['speed_diff_percent'][i]), "predict_category": three_categories(temp_speed_variation)})
    acc_dataframe = pd.DataFrame(acc_dataframe)
    correct_num_1 = 0
    total_num_1 = 0
    correct_num_2 = 0
    total_num_2 = 0
    for i in range(acc_dataframe.shape[0]):
        if acc_dataframe['GT_category'][i] != 1:
            total_num_1 += 1
            if acc_dataframe['predict_category'][i] != 1:
                correct_num_1 += 1
    for i in range(acc_dataframe.shape[0]):
            total_num_2 += 1
            if acc_dataframe['predict_category'][i] == acc_dataframe['GT_category'][i]:
                correct_num_2 += 1
    acc_1 = correct_num_1 / total_num_1
    acc_2 = correct_num_2 / total_num_2
    return acc_1, acc_2, acc_dataframe


net = readNet(simulation_config['net_path'])
road_df = load_road_data(simulation_config['road_data_csv'])
df, mapping_dict = match_edges_to_roads(net, road_df)
print(mapping_dict)
recall_factor = (2/3)
with open("simulation_results\\rain_average_speeds_scale.json", 'r') as fcc_file:
    rain_dict = json.load(fcc_file)
with open("simulation_results\\normal_average_speeds_scale.json", 'r') as fcc_file:
    normal_dict = json.load(fcc_file)
time_intervals = list(normal_dict.keys())
param_combination = list(rain_dict.keys())
param_time_accuracy = {}
for param in param_combination:
    for time_ in time_intervals:
        accuracy_1, accuracy_2, df_ = calculate_acc(mapping_dict, road_df, normal_dict[time_], rain_dict[param][time_])
        param_time_accuracy[f"{param}_{time_}_{str(accuracy_1)}_{str(accuracy_2)}"] = accuracy_1*recall_factor + accuracy_2*(1-recall_factor)
with open("simulation_results/param_scale_accuracy.json", "w") as f:
    json.dump(param_time_accuracy, f, indent=4)
    
    
sorted_data = sorted(param_time_accuracy.items(), key=lambda x: x[1], reverse=True)
print("准确率排名前10的参数组合：")
for i, (params, accuracy) in enumerate(sorted_data[:10], 1):
    print(f"{i}. {params}: {accuracy}")
all_accuracies = [accuracy for _, accuracy in param_time_accuracy.items()]
average_accuracy = np.mean(all_accuracies)
median_accuracy = np.median(all_accuracies)
min_accuracy = min(all_accuracies)
max_accuracy = max(all_accuracies)
print("\n准确率统计摘要：")
print(f"参数组合总数: {len(param_time_accuracy)}")
print(f"平均准确率: {average_accuracy:.4f}")
print(f"中位数准确率: {median_accuracy:.4f}")
print(f"最高准确率: {max_accuracy:.4f}")
print(f"最低准确率: {min_accuracy:.4f}")