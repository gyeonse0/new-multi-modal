import copy
import random
from types import SimpleNamespace
import vrplib 
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd
from typing import List
from itertools import groupby

from RouteGenerator import *
from FileReader import *

vrp_file_path = r'C:\Users\82102\Desktop\ALNS-master\examples\data\multi_modal_data.vrp'
sol_file_path = r'C:\Users\82102\Desktop\ALNS-master\examples\data\multi_modal_data.sol'

file_reader = FileReader()
data = file_reader.read_vrp_file(vrp_file_path)
bks = file_reader.read_sol_file(sol_file_path)


class MultiModalState:
    """
    routes 딕셔너리 집합을 input으로 받아서 copy를 수행한 뒤, 해당 routes 에서의 정보를 추출하는 함수
    output: objective cost value / 특정 customer node를 포함한 route  
    """
    def __init__(self, routes, unassigned=None):
        self.routes = routes
        self.unassigned = unassigned if unassigned is not None else []
        
        unassigned_check=[]
        for node_id in range(1, data["dimension"]):
            is_in_routes = any(node_id == node[0] for route in routes for node in route)
            if not is_in_routes and (node_id, None) not in unassigned_check:
                unassigned_check.append((node_id, None))
        
        self.unassigned = unassigned_check

    def copy(self):
        return MultiModalState(
            copy.deepcopy(self.routes, self.unassigned.copy())
        )
        
    def __str__(self):
        return f"Routes: {self.routes}, Unassigned: {self.unassigned}" 
    
    def __iter__(self):
        return iter(self.routes)       
        
        
    def objective(self):
        """
        data와 routes 딕셔너리 집합을 이용하여 objective value 계산해주는 함수
        our objective cost value = energy_consunmption(kwh)
        energy_consunmption(kwh)={Truck edge cost(km), Truck energy consumption(kwh/km), Drone edge cost(km), Drone energy consumption(kwh/km)}
        TO DO: 이후에 logistic_load 등의 데이터 등을 추가로 활용하여 energy_consumption 모델링 확장 필요
        """
        divided_routes = apply_dividing_route_to_routes(self.routes)
        
        energy_consumption = 0.0

        for route_info in divided_routes:
            vtype = route_info['vtype']
            path = route_info['path']

            if vtype == 'truck':
                for i in range(len(path) - 1):
                    loc_from = path[i][0] if isinstance(path[i], tuple) else path[i]
                    loc_to = path[i+1][0] if isinstance(path[i+1], tuple) else path[i+1]

                    edge_weight = data["edge_km_t"][loc_from][loc_to]
                    energy_consumption += edge_weight * data["energy_kwh/km_t"]


            elif vtype == 'drone': #드론은 1(fly)부터 3(catch)까지만의 edge를 반복적으로 고려해준다는 알고리즘
                flag = 0
                for j in range(len(path)):
                    if flag == 0 and path[j][1] == 1:
                        start_index = j
                        flag = 1
                    elif path[j][1] == 3 and flag == 1:
                        for k in range(start_index, j):
                            edge_weight = data["edge_km_d"][path[k][0]][path[k+1][0]]
                            energy_consumption += edge_weight * data["energy_kwh/km_d"]
                        flag=0

        return energy_consumption
    
    def objective_time_penalty(self):
        """
        energy_consumption + waiting time penalty
        """
        divided_routes = apply_dividing_route_to_routes(self.routes)
        energy_consumption = 0.0
        truck_time_cost = 0.0
        drone_time_cost = 0.0

        for route_info in divided_routes:
            vtype = route_info['vtype']
            path = route_info['path']
            route_info['time'] = []

            if vtype == 'truck':
                #에너지
                for i in range(len(path) - 1):
                    loc_from = path[i][0] if isinstance(path[i], tuple) else path[i]
                    loc_to = path[i+1][0] if isinstance(path[i+1], tuple) else path[i+1]

                    edge_weight = data["edge_km_t"][loc_from][loc_to]
                    energy_consumption += edge_weight * data["energy_kwh/km_t"]
                #시간
                for j in range(len(path)):
                    if path[j][1] == 1:
                        start_index = j
                    elif path[j][1] == 3 and start_index is not None:
                        for k in range(start_index, j):
                            time_cost = data["edge_km_t"][path[k][0]][path[k+1][0]]
                            truck_time_cost += time_cost / data["speed_t"]
                        start_index = None
                        route_info['time'].append(truck_time_cost)
                        truck_time_cost = 0.0


            elif vtype == 'drone': 
                start_index = None
                flag = 0
                for j in range(len(path)):
                    if flag == 0 and path[j][1] == 1:
                        start_index = j
                        flag = 1
                    elif path[j][1] == 3 and start_index is not None:
                        for k in range(start_index, j):
                            #에너지
                            edge_weight = data["edge_km_d"][path[k][0]][path[k+1][0]]
                            energy_consumption += edge_weight * data["energy_kwh/km_d"]
                            #시간
                            time_cost = data["edge_km_d"][path[k][0]][path[k+1][0]]
                            drone_time_cost += time_cost / data["speed_d"]
                        flag = 0
                        start_index = None
                        route_info['time'].append(drone_time_cost)
                        drone_time_cost = 0.0

        grouped_paths = {k: [item['time'] for item in g] for k, g in groupby(divided_routes, key=lambda x: x['vid'])}

        waiting_time = {k: [abs(x - y) for x, y in zip(*v)] for k, v in grouped_paths.items() if len(v) == 2}

        total_sum = sum(sum(values) for values in waiting_time.values())

        return energy_consumption + total_sum
    
    @property
    def cost(self):
        """
        Alias for objective method. Used for plotting.
        """
        return self.objective()
    

    def find_route(self, customer):
       
        for route in self.routes['route']:
            if customer in route['path']:
                return route
            
        raise ValueError(f"Solution does not contain customer {customer}.")
    