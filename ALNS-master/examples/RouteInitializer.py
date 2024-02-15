import numpy as np
import random

from MultiModalState import *

class RouteInitializer:
    def __init__(self, data, k, l, max_drone_mission):
        self.data = data
        self.k = k
        self.l = l
        self.max_drone_mission = max_drone_mission

    def neighbors_init_truck(self, customer):
        locations = np.argsort(self.data["edge_km_t"][customer])
        return locations[locations != 0]

    def validate_truck_routes(self, truck_routes):
        for route in truck_routes:
            consecutive_zeros = sum(1 for loc in route if loc == 0)
            if consecutive_zeros > 2:
                raise ValueError("Unable to satisfy demand with the given number of trucks!!")

    def nearest_neighbor_init_truck(self):
        truck_init_routes = [[] for _ in range(self.data["num_t"])]
        unvisited = set(range(1, self.data["dimension"]))

        while unvisited:
            for i in range(self.data["num_t"]):
                route = [0]
                route_demands = 0

                while unvisited:
                    current = route[-1]
                    neighbors = [nb for nb in self.neighbors_init_truck(current) if nb in unvisited]
                    nearest = neighbors[0]

                    if route_demands + self.data["demand"][nearest] > self.data["capacity_t"]:
                        break

                    route.append(nearest)
                    unvisited.remove(nearest)
                    route_demands += self.data["demand"][nearest]

                route.append(0)
                truck_init_routes[i].extend(route[0:])

        self.validate_truck_routes(truck_init_routes)

        return {
            'num_t': len(truck_init_routes),
            'num_d': 0,
            'route': [{'vtype': 'truck', 'vid': f't{i + 1}', 'path': path} for i, path in enumerate(truck_init_routes)]
        }
    
    def init_truck(self):
        # nearest_neighbor_init_truck() 메서드 호출
        truck_init_routes = self.nearest_neighbor_init_truck()
        init_truck=[]
        # 각 경로를 튜플로 변환
        for route in truck_init_routes['route']:
            tuple_route = [(node, 0) for node in route['path']]
            init_truck.append(tuple_route)
        
        return MultiModalState(init_truck)
        
        

    def makemakemake(self, state):
        empty_list = []

        for route_index, route_info in enumerate(state['route']):
            self.depot_end = len(route_info['path']) - 1
            self.SERVICE = 0
            self.CATCH = 0
            self.only_drone_index = []
            self.fly_node_index = []
            self.catch_node_index = []
            self.subroutes = []
            self.generate_subroutes(route_info['path'])
            diclist = self.dividing_route(self.route_tuples(route_info['path']), route_index)

            empty_list.extend(diclist)

        return MultiModalState(self.combine_paths(empty_list))

    def generate_subroutes(self, each_route):
        while len(self.subroutes) < self.max_drone_mission:
            self.FLY = random.choice(range(self.CATCH, len(each_route)))
            self.SERVICE = self.FLY + self.k
            self.CATCH = self.SERVICE + self.l
            if self.CATCH > self.depot_end:
                break
            subroute = list(range(self.FLY, self.CATCH + 1))
            self.subroutes.append(subroute)
            self.fly_node_index.append(self.FLY)
            self.only_drone_index.append(self.SERVICE)
            self.catch_node_index.append(self.CATCH)

    def route_tuples(self, each_route):
        visit_type = [0] * len(each_route)
        visit_type = [
            1 if index in self.fly_node_index else
            2 if index in self.only_drone_index else
            3 if index in self.catch_node_index else
            0 for index in range(len(visit_type))
        ]
        for i in [
            i for subroute in self.subroutes
            for i in subroute[1:-1] if i not in self.only_drone_index
        ]:
            visit_type[i] = 4
        return list(zip(each_route, visit_type))

    def dividing_route(self, route_with_info, route_index):
        truck_route = [value for value in route_with_info if value[1] != 2]
        drone_route = [value for value in route_with_info if value[1] != 4]

        return [
            {'vtype': 'drone', 'vid': 'd' + str(route_index + 1), 'path': drone_route},
            {'vtype': 'truck', 'vid': 't' + str(route_index + 1), 'path': truck_route},
        ]
        
    
    
    def combine_paths(self, route_data):
        combined_paths = []
        initial_solution = self.nearest_neighbor_init_truck()
        nn_truck_paths = [route['path'] for route in initial_solution['route']]
        
        for i in range(len(route_data)):  # 모든 드론 및 트럭 경로에 대해 반복
            if i % 2 == 0:  # 짝수 인덱스인 경우 드론 경로
                nn_truck_path = nn_truck_paths[i // 2]
                drone_route = route_data[i]['path']
                truck_route = route_data[i + 1]['path']
                
                filled_path = []
                for node, value in drone_route[:-1]:
                    if node not in [point[0] for point in filled_path]:
                        filled_path.append((node, value))
                
                for node, value in truck_route[:-1]:
                    if node not in [point[0] for point in filled_path]:
                        filled_path.append((node, value))
            
                filled_path = sorted(filled_path, key=lambda x: nn_truck_path[:-1].index(x[0]))
                filled_path.append(drone_route[-1])
                combined_paths.append(filled_path)
        
        return combined_paths
