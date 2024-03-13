from RouteGenerator import *
from FileReader import *
from MultiModalState import *

import random

file_reader = FileReader()

vrp_file_path = r'C:\Users\82102\Desktop\examples (2)\examples\data\multi_modal_data.vrp'
sol_file_path = r'C:\Users\82102\Desktop\examples (2)\examples\data\multi_modal_data.sol'


data = file_reader.read_vrp_file(vrp_file_path)

globals
IDLE = 0 # 해당 노드에 드론이 트럭에 업힌 상태의 경우
FLY = 1 # 해당 노드에서 트럭이 드론의 임무를 위해 드론을 날려주는 경우
ONLY_DRONE = 2 # 해당 노드에 드론만이 임무를 수행하는 서비스 노드인 경우
CATCH = 3 # 해당 노드에서 트럭이 임무를 마친 드론을 받는 경우
ONLY_TRUCK = 4 # 해당 노드에서 트럭만이 임무를 수행하는 경우 (드론이 업혀있지 않음)
NULL = None # 해당 노드가 전체 ROUTES에 할당이 안되어 있는 경우 (SOLUTION에서 UNVISITED 한 노드)

class Repair():

    def heavy_truck_repair(self, state, rnd_state):
        heavy_first_repair = MultiModalState(state.routes, state.unassigned)
        routes = heavy_first_repair.routes
        unassigned = heavy_first_repair.unassigned
        condition = lambda x: data["logistic_load"][x[0]] > data["cargo_limit_drone"] 
        
        unassigned_heavy, unassigned = self.extract_and_remain(unassigned, condition)

        while len(unassigned_heavy) > 0:
            customer_heavy = random.choice(unassigned_heavy)
            unassigned_heavy.remove(customer_heavy)
            best_route, best_idx = self.truck_best_insert(customer_heavy, routes)
                
            if best_route is not None and best_idx is not None:
                for i, route in enumerate(routes):
                    if route == best_route:
                        routes[i] = route[:best_idx] + [customer_heavy] + route[best_idx:]
                        self.truck_repair_visit_type_update(routes)

            routes = [route for route in routes if route != [(0, 0), (0, 0)]]

        self.truck_repair_visit_type_update(routes) #최종적으로 visit_type 검사
        self.route_duplication_check(routes)

        state_after_heavy_repair = MultiModalState(routes, unassigned)
        
        if len(state_after_heavy_repair.unassigned) > 0:
            return self.drone_first_truck_second(state_after_heavy_repair, rnd_state)
        else:
            return MultiModalState(routes, unassigned)
    

        #heavy insertion 에서 굳이 drone_first_truck_second 를 고려해주는 이유? -> 어차피 드론으로 갈수 없는거만 차출된거 아닌가?
    
        #state_after_heavy_repair = MultiModalState(routes, unassigned)
        
        #if len(state_after_heavy_repair.unassigned) > 0:
           #return self.drone_first_truck_second(state_after_heavy_repair, rnd_state)
        #else:
            #return MultiModalState(routes, unassigned)

    def drone_first_truck_second(self, state, rnd_state):
        """
        
        """
        # while len(unassigned) > 0:
        #     customer = random.choice(unassigned)
        #     unassigned.remove(customer)
        #     best_route, best_idx = self.drone_best_insert(customer, routes)    
            
        #     if best_route is not None and best_idx is not None:
        #         for i, route in enumerate(routes):
        #             if route == best_route:
        #                 routes[i] = route[:best_idx] + [customer] + route[best_idx:]
        #                 self.drone_repair_visit_type_update(routes)
                                        
        # self.drone_repair_visit_type_update(routes) #최종적으로 visit_type 검사
        # self.route_duplication_check(routes)
        
        # state_after_drone_repair = MultiModalState(routes, unassigned)

        state_after_drone_repair = self.greedy_drone_repair(MultiModalState(state.routes, state.unassigned),rnd_state)

        if len(state_after_drone_repair.unassigned) > 0:
            return self.greedy_truck_repair(state_after_drone_repair,rnd_state)
        else:
            return state_after_drone_repair
    
        # if len(state_after_drone_repair.unassigned)>0:
        #     routes = state_after_drone_repair.routes
        #     unassigned = state_after_drone_repair.unassigned
        #     while len(unassigned) > 0:
        #         customer = random.choice(unassigned)
        #         unassigned.remove(customer)
        #         best_route, best_idx = self.truck_best_insert(customer, routes)
                
        #         if best_route is not None and best_idx is not None:
        #             for i, route in enumerate(routes):
        #                 if route == best_route:
        #                     routes[i] = route[:best_idx] + [customer] + route[best_idx:]
        #                     self.truck_repair_visit_type_update(routes)
                
        #         else:
        #             # routes에 [(0, 0), (0, 0)]이 없으면
        #             if not any(route == [(0, 0), (0, 0)] for route in routes):
        #                 # routes 뒤에 새로운 route 추가
        #                 routes.append([(0, 0), customer, (0, 0)])
        #             else:
        #                 for i, route in enumerate(routes):
        #                     if route == [(0, 0), (0, 0)]:
        #                         # 빈 route에 고객 추가
        #                         routes[i] = [(0, 0), (customer[0],0), (0, 0)]
                                
        #         routes = [route for route in routes if route != [(0, 0), (0, 0)]]
            
        # self.truck_repair_visit_type_update(routes) #최종적으로 visit_type 검사
        # self.route_duplication_check(routes)

        # return MultiModalState(routes, unassigned)
    
    def truck_first_drone_second(self, state, rnd_state):
        """

        """
        truck_first_drone_second_repairs = MultiModalState(state.routes, state.unassigned)
        routes = truck_first_drone_second_repairs.routes
        unassigned = truck_first_drone_second_repairs.unassigned

        while len(unassigned) > 0:
            customer = random.choice(unassigned)
            unassigned.remove(customer)
            best_route, best_idx = self.truck_best_insert(customer, routes)

            if best_route is not None and best_idx is not None:
                for i, route in enumerate(routes):
                    if route == best_route:
                        routes[i] = route[:best_idx] + [customer] + route[best_idx:]
                        self.truck_repair_visit_type_update(routes)
            
            routes = [route for route in routes if route != [(0, 0), (0, 0)]]

        self.truck_repair_visit_type_update(routes)
        self.route_duplication_check(routes)

        state_after_truck_repairs = MultiModalState(routes,unassigned)
        state_after_drone_repairs = MultiModalState(routes,unassigned)

        if len(state_after_truck_repairs.unassigned)>0:
            if len(state_after_drone_repairs.unassigned)>0:
                return self.new_truck_route(state_after_drone_repairs,rnd_state)
            else:
                return state_after_drone_repairs
            
        else:
            return state_after_truck_repairs


    def greedy_truck_repair(self, state, rnd_state):
        truck_repair = MultiModalState(state.routes, state.unassigned)
        routes = truck_repair.routes
        unassigned = truck_repair.unassigned

        while len(unassigned) > 0:
            customer = random.choice(unassigned)
            unassigned.remove(customer)
            best_route, best_idx = self.truck_best_insert(customer, routes)

            if best_route is not None and best_idx is not None:
                for i, route in enumerate(routes):
                    if route == best_route:
                        routes[i] = route[:best_idx] + [customer] + route[best_idx:]
                        self.truck_repair_visit_type_update(routes)
            else:
                # routes에 [(0, 0), (0, 0)]이 없으면
                if not any(route == [(0, 0), (0, 0)] for route in routes):
                    # routes 뒤에 새로운 route 추가
                    routes.append([(0, 0), customer, (0, 0)])
                else:
                    for i, route in enumerate(routes):
                        if route == [(0, 0), (0, 0)]:
                            # 빈 route에 고객 추가
                            routes[i] = [(0, 0), (customer[0],0), (0, 0)]
                            
            routes = [route for route in routes if route != [(0, 0), (0, 0)]]

        self.truck_repair_visit_type_update(routes) #최종적으로 visit_type 검사
        self.route_duplication_check(routes)

        return MultiModalState(routes, unassigned)
    

    
    def greedy_drone_repair(self, state, rnd_state):
        drone_repair = MultiModalState(state.routes, state.unassigned)
        routes = drone_repair.routes
        unassigned = drone_repair.unassigned

        while len(unassigned) > 0:
            customer = random.choice(unassigned)
            unassigned.remove(customer)
            best_route, best_idx = self.drone_best_insert(customer, routes)    
            
            if best_route is not None and best_idx is not None:
                for i, route in enumerate(routes):
                    if route == best_route:
                        routes[i] = route[:best_idx] + [customer] + route[best_idx:]
                        self.drone_repair_visit_type_update(routes)

            routes = [route for route in routes if route != [(0, 0), (0, 0)]]
                                        
        self.drone_repair_visit_type_update(routes) #최종적으로 visit_type 검사
        self.route_duplication_check(routes)

        return MultiModalState(routes, unassigned)
    
    def drone_best_insert(self, customer, routes):
        """
        Finds the best feasible route and insertion idx for the customer.
        Return (None, None) if no feasible route insertions are found.
        """
        best_cost, best_route, best_idx = None, None, None

        for route in routes:   
            for idx in range(1, len(route)):  # depot을 제외한 index를 고려해줘야함
                if self.drone_can_insert(data, customer, route, idx): # 삽입 가능한 경우
                    cost = self.drone_insert_cost(customer, route, idx, routes)
                    if best_cost is None or cost < best_cost:
                        best_cost, best_route, best_idx = cost, route, idx

        return best_route, best_idx

    def drone_randomize_greedy_insert(self, customer, routes):
        best_costs = [None, None, None]
        best_routes = [None, None, None]
        best_idxs = [None, None, None]

        for route in routes:
            for idx in range(1, len(route)):
                if self.drone_can_insert(data, customer, route, idx):  # 삽입 가능한 경우
                    cost = self.drone_insert_cost(customer, route, idx, routes)

                    for i in range(3):
                        if best_costs[i] is None or cost < best_costs[i]:
                            best_costs.insert(i, cost)
                            best_routes.insert(i, route)
                            best_idxs.insert(i, idx)
                            break

        # Randomly select one of the top three costs
        selected_index = random.randint(0, 2)

        return best_routes[selected_index], best_idxs[selected_index]
    
    def new_truck_route(self, state, rnd_state):
        new_truck_state = MultiModalState(state.routes,state.unassigned)
        routes = new_truck_state.routes
        unassigned = new_truck_state.unassigned

        while len(unassigned) > 0:
            customer = random.choice(unassigned)
            unassigned.remove(customer)
            
            if not any(route == [(0, 0), (0, 0)] for route in routes):
                    # routes 뒤에 새로운 route 추가
                    routes.append([(0, 0), customer, (0, 0)])
            else:
                for i, route in enumerate(routes):
                    if route == [(0, 0), (0, 0)]:
                        # 빈 route에 고객 추가
                        routes[i] = [(0, 0), (customer[0],0), (0, 0)]
                            
            routes = [route for route in routes if route != [(0, 0), (0, 0)]]

        self.truck_repair_visit_type_update(routes) #최종적으로 visit_type 검사
        self.route_duplication_check(routes)

        return MultiModalState(routes, unassigned)


    def truck_randomize_greedy_insert(self, customer, routes):
        best_costs = [None, None, None]
        best_routes = [None, None, None]
        best_idxs = [None, None, None]

        for route in routes:
            for idx in range(1, len(route)):
                if self.truck_can_insert(data, customer, route, idx):  # 삽입 가능한 경우
                    cost = self.truck_insert_cost(customer, route, idx, routes)

                    for i in range(3):
                        if best_costs[i] is None or cost < best_costs[i]:
                            best_costs.insert(i, cost)
                            best_routes.insert(i, route)
                            best_idxs.insert(i, idx)
                            break

        # Randomly select one of the top three costs
        selected_index = random.randint(0, 2)

        return best_routes[selected_index], best_idxs[selected_index]
        
    def drone_can_insert(self, data, customer, route, idx): 
        
        # 해당 노드의 Landing spot의 availability가 1이 아닐 때 False
        if data["availability_landing_spot"][customer[0]] == 0:
            return False

        # 해당 고객의 Drone delivery preference가 1이 아닐 때 False
        if data["customer_drone_preference"][customer[0]] == 0:
            return False
        
        new_route = route[:idx] + [customer] + route[idx:]
        self.drone_repair_visit_type_update_route(new_route)
        
        drone_path = [value for value in new_route if value[1] != ONLY_TRUCK]  
        truck_path = [value for value in new_route if value[1] != ONLY_DRONE]
        
        if len(new_route)-2 > data['demand_t']:
            return False

        # 해당 고객들의 logistic_load가 cargo_limit_drone을 넘으면 False
        flag = 0
        total_load = 0
        for i in range(0,len(drone_path)):
            if flag == 0 and drone_path[i][1] == ONLY_DRONE:
                start_index = i
                flag = 1
                j = i + 1
                while flag == 1 and j < len(drone_path):
                    if drone_path[j][1] == ONLY_DRONE:
                        flag = 1
                    elif drone_path[j][1] != ONLY_DRONE:
                        flag = 0
                        end_index = j - 1
                    j += 1
                    
                for k in range(start_index, end_index + 1):
                    total_load += data["logistic_load"][drone_path[k][0]]
                if total_load > data["cargo_limit_drone"]:
                    return False
                total_load = 0

        # max system duration 넘으면 false 수정버전
        # Multimodalstate의 calculate_time_per_route()는 단일 route를 인풋으로 받아서 그 라우트의 total_time_list와 waiting_time_list를 반환
        if MultiModalState.calculate_time_per_route(self, new_route)[0][-1] > data["maximum_system_duration"]:
            return False
        
        # max waiting time 넘으면 false 수정버전
        if any(element > data["max_waiting_time"] for element in MultiModalState.calculate_time_per_route(self, new_route)[1]):
            return False
        
        
        #에너지 충전 및 소모로 인한 배터리 잔량에 대해 고려(드론의 ONE path로 고려(0,1,2,3만 존재))
        drone_current_kwh = data["battery_kwh_d"]
        
        drone_distance = 0
        for i in range(len(drone_path)):
            flag = 0
            if drone_path[i][1] == IDLE: #배터리 충전 계산
                drone_idle_distance = 0
                start_index = i
                j = i + 1
                while j < len(drone_path):
                    if drone_path[j][1] != IDLE:
                        end_index = j
                        break
                    j += 1
                    
                for k in range(start_index,end_index+1):
                    drone_idle_distance += data["edge_km_d"][drone_path[k-1][0]][drone_path[k][0]]
                drone_idle_time = drone_idle_distance / data["speed_d"]
                    
                if drone_current_kwh < data["battery_kwh_d"]: # 만땅이 아니라면
                    drone_current_kwh += (data["charging_kw_d"]) * (drone_idle_time/60) #kwh 단위 고려
                    if drone_current_kwh > data["battery_kwh_d"]:
                        drone_current_kwh = data["battery_kwh_d"] #충전해도 최대 배터리 양은 넘을 수 없음
                
                
            elif drone_path[i][1] == FLY and flag == 0: #배터리 소모 계산
                start_index = i
                j = i + 1
                while j < len(drone_path):
                    if drone_path[j][1] == CATCH:
                        end_index = j
                        flag = 1
                        break
                    j += 1

                for k in range(start_index,end_index):
                    drone_distance += data["edge_km_d"][drone_path[k][0]][drone_path[k+1][0]]
                
                energy_consumption = drone_distance * data["energy_kwh/km_d"]
                drone_current_kwh -= energy_consumption
                
                if drone_current_kwh < data["battery_kwh_d"] * (data["min_soc_d"]/100): #총 배터리의 5퍼센트 이상이 유지되어야 FLY
                    return False

        return True
        
    def drone_insert_cost(self, customer, route, idx, routes): #드론 경로도 고려되어있으므로 분할 후, MM State.object() 적용
        """
        Computes the insertion cost for inserting customer in route at idx.
        """
        
        new_route = route[:idx] + [customer] + route[idx:]
        routes = [new_route] + [r for r in routes if r != route]  # 선택된 route만을 새로운 route로 대체하여 리스트에 수정
        
        self.drone_repair_visit_type_update(routes)
        self.route_duplication_check(routes)

        return MultiModalState(routes).new_objective()
    

    
    def truck_best_insert(self, customer, routes):
        """
        Finds the best feasible route and insertion idx for the customer.
        Return (None, None) if no feasible route insertions are found.
        """
        best_cost, best_route, best_idx = None, None, None

        for route in routes:   
            for idx in range(1,len(route)): #depot을 제외한 index를 고려해줘야함
                if self.truck_can_insert(data, customer, route, idx): #트럭이 insert 될 수 있는 조건
                    cost = self.truck_insert_cost(customer, route, idx, routes)

                    if best_cost is None or cost < best_cost:
                        best_cost, best_route, best_idx = cost, route, idx

        return best_route, best_idx
    
    
    def truck_can_insert(self, data, customer, route, idx): 
        new_route = route[:idx] + [customer] + route[idx:]
        self.truck_repir_visit_type_update_route(new_route)

        truck_path = [value for value in new_route if value[1] != ONLY_DRONE]

        total_logistic_load = 0
        for customer in new_route:
            total_logistic_load += data["logistic_load"][customer[0]]

        if total_logistic_load > data["cargo_limit_truck"]:
            return False
        
        if len(new_route)-2 > data['demand_t']:
            return False

        # max system duration 넘으면 false 수정버전
        # Multimodalstate의 calculate_time_per_route()는 단일 route를 인풋으로 받아서 그 라우트의 total_time_list와 waiting_time_list를 반환
        if MultiModalState.calculate_time_per_route(self, new_route)[0][-1] > data["maximum_system_duration"]:
            return False
        
        # max waiting time 넘으면 false 수정버전
        if any(element > data["max_waiting_time"] for element in MultiModalState.calculate_time_per_route(self, new_route)[1]):
            return False
        
        # 에너지 충전 및 소모로 인한 배터리 잔량에 대해 고려(트럭의 ONE path로 고려(0,1,4,3 만 존재))
        truck_current_kwh = data["battery_kwh_t"]
        truck_distance = 0
        truck_idle_distance = 0
        start_index = 0
        flag = 0
        truck_drive_only = True
        truck_drive_with_drone = False
        
        for idx, customer in enumerate(truck_path):
            last_condition = (idx == len(truck_path) - 1) and truck_drive_with_drone
            if (customer[1] == CATCH and truck_drive_only) or last_condition:
                ### 이 인덱스까지 모두 더해준다
                end_index = idx
                for k in range(start_index, end_index):
                    truck_distance += data["edge_km_t"][truck_path[k][0]][truck_path[k+1][0]]
                energy_consumption = truck_distance * data["energy_kwh/km_t"]
                truck_current_kwh -= energy_consumption

                truck_drive_only = False
                truck_drive_with_drone = True
                start_index = idx
            elif customer[1] == FLY and truck_drive_with_drone: 
                ### 이 인덱스까지 모두 더해준다
                end_index = idx
                for k in range(start_index, end_index):
                    truck_idle_distance += data["edge_km_t"][truck_path[k][0]][truck_path[k+1][0]]
                energy_consumption = truck_idle_distance * data["energy_kwh/km_t"]
                truck_idle_time = truck_idle_distance / data["speed_t"]

                truck_current_kwh -= energy_consumption
                truck_current_kwh -= (data["charging_kw_d"]) * (truck_idle_time/60) #우리는 시간이 분이니까 KW애 나누기60

                truck_drive_with_drone = False
                truck_drive_only = True
                start_index = idx

            if truck_current_kwh < data["battery_kwh_t"] * (data["min_soc_t"]/100): #총 배터리의 5퍼센트 이상이 유지되어야 FLY
                return False

        return True


    def truck_insert_cost(self, customer, route, idx, routes): #드론 경로도 고려되어있으므로 분할 후, MM State.object() 적용
        """
        Computes the insertion cost for inserting customer in route at idx.
        """
        
        new_route = route[:idx] + [customer] + route[idx:]
        routes = [new_route] + [r for r in routes if r != route]  # 선택된 route만을 새로운 route로 대체하여 리스트에 수정
        
        self.truck_repair_visit_type_update(routes)
        self.route_duplication_check(routes)

        return MultiModalState(routes).new_objective()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def drone_repair_visit_type_update(self,routes):
        """
        visit_type update 함수
        """

        for route in routes:
            for j in range(1,len(route)):
                if route[j][1] is None:
                    route[j] = (route[j][0], ONLY_DRONE)
                    k = j - 1  # 현재 노드의 이전 노드부터 시작
                    while k >= 0:
                        if route[k][1] is not None and route[k][1] is not ONLY_DRONE and route[k][1] is not ONLY_TRUCK:  # 이전 노드가 None이 아닌 경우
                            if route[k][1] == IDLE:
                                route[k] = (route[k][0], FLY)
                            elif route[k][1] == FLY:
                                route[k] = (route[k][0], FLY)
                            elif route[k][1] == CATCH:
                                route[k] = (route[k][0], FLY)   
                            break  # 노드의 상태를 설정했으므로 루프를 종료합니다.
                        k -= 1  # k를 감소하여 이전 노드로 이동
                        
                    l = j + 1
                    while l < len(route):
                        if route[l][1] is not None and route[l][1] is not ONLY_DRONE and route[l][1] is not ONLY_TRUCK:
                            if route[l][1] == IDLE:
                                route[l] = (route[l][0], CATCH)
                            elif route[l][1] == FLY:
                                route[l] = (route[l][0], FLY)
                            elif route[l][1] == CATCH:
                                route[l] = (route[l][0], CATCH)   
                            break  # 노드의 상태를 설정했으므로 루프를 종료합니다.
                        l += 1  # k를 감소하여 이전 노드로 이동
                        
        return routes
    
    def drone_repair_visit_type_update_route(self,route):
        for j in range(1,len(route)):
                if route[j][1] is None:
                    route[j] = (route[j][0], ONLY_DRONE)
                    k = j - 1  # 현재 노드의 이전 노드부터 시작
                    while k >= 0:
                        if route[k][1] is not None and route[k][1] is not ONLY_DRONE and route[k][1] is not ONLY_TRUCK:  # 이전 노드가 None이 아닌 경우
                            if route[k][1] == IDLE:
                                route[k] = (route[k][0], FLY)
                            elif route[k][1] == FLY:
                                route[k] = (route[k][0], FLY)
                            elif route[k][1] == CATCH:
                                route[k] = (route[k][0], FLY)   
                            break  # 노드의 상태를 설정했으므로 루프를 종료합니다.
                        k -= 1  # k를 감소하여 이전 노드로 이동
                        
                    l = j + 1
                    while l < len(route):
                        if route[l][1] is not None and route[l][1] is not ONLY_DRONE and route[l][1] is not ONLY_TRUCK:
                            if route[l][1] == IDLE:
                                route[l] = (route[l][0], CATCH)
                            elif route[l][1] == FLY:
                                route[l] = (route[l][0], FLY)
                            elif route[l][1] == CATCH:
                                route[l] = (route[l][0], CATCH)   
                            break  # 노드의 상태를 설정했으므로 루프를 종료합니다.
                        l += 1  # k를 감소하여 이전 노드로 이동
                        
        return route
        
    
    def truck_repair_visit_type_update(self,routes):
        """
        visit_type update 함수
        """
        for route in routes:
            for j in range(1,len(route)-1):
                if route[j][1] is None and route[j-1][1] == IDLE:
                    route[j] = (route[j][0], IDLE)
                elif route[j][1] is None and route[j-1][1] == FLY:
                    route[j] = (route[j][0], ONLY_TRUCK)
                elif route[j][1] is None and route[j-1][1] == ONLY_DRONE:
                    route[j] = (route[j][0], ONLY_TRUCK)
                elif route[j][1] is None and route[j-1][1] == CATCH:
                    route[j] = (route[j][0], IDLE)
                elif route[j][1] is None and route[j-1][1] == ONLY_TRUCK:
                    route[j] = (route[j][0], ONLY_TRUCK)
                                
        return routes
    
    def truck_repir_visit_type_update_route(self,route):
        for j in range(1,len(route)-1):
            if route[j][1] is None and route[j-1][1] == IDLE:
                route[j] = (route[j][0], IDLE)
            elif route[j][1] is None and route[j-1][1] == FLY:
                route[j] = (route[j][0], ONLY_TRUCK)
            elif route[j][1] is None and route[j-1][1] == ONLY_DRONE:
                route[j] = (route[j][0], ONLY_TRUCK)
            elif route[j][1] is None and route[j-1][1] == CATCH:
                route[j] = (route[j][0], IDLE)
            elif route[j][1] is None and route[j-1][1] == ONLY_TRUCK:
                route[j] = (route[j][0], ONLY_TRUCK)
                            
        return route
    
    def unassigned_check(self, routes, unassigned):
            
        for node_id in range(1, data["dimension"]):
            is_in_routes = any(node_id == node[0] for route in routes for node in route)
            if not is_in_routes and (node_id, None) not in unassigned:
                unassigned.append((node_id, None))
                
        return unassigned
    
    def route_duplication_check(self, routes):
        for i, route in enumerate(routes):
            for j in range(i + 1, len(routes)):
                if route == routes[j]:
                    routes[j] = [(0, 0), (0, 0)]
    
        return routes
    
    def extract_and_remain(self, original_list, condition):
        # 새로운 리스트 추출
        new_list = [x for x in original_list if condition(x)]
        
        # 추출한 요소를 제외한 리스트 유지
        remaining_list = [x for x in original_list if x not in new_list]
        
        # 결과 반환
        return new_list, remaining_list