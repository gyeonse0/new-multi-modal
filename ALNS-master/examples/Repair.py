from RouteGenerator import *
from FileReader import *
from MultiModalState import *

import random

file_reader = FileReader()

vrp_file_path = r'C:\Users\82102\Desktop\ALNS-master\examples\data\multi_modal_data.vrp'
sol_file_path = r'C:\Users\82102\Desktop\ALNS-master\examples\data\multi_modal_data.sol'

data = file_reader.read_vrp_file(vrp_file_path)

globals
IDLE = 0 # 해당 노드에 드론이 트럭에 업힌 상태의 경우
FLY = 1 # 해당 노드에서 트럭이 드론의 임무를 위해 드론을 날려주는 경우
ONLY_DRONE = 2 # 해당 노드에 드론만이 임무를 수행하는 서비스 노드인 경우
CATCH = 3 # 해당 노드에서 트럭이 임무를 마친 드론을 받는 경우
ONLY_TRUCK = 4 # 해당 노드에서 트럭만이 임무를 수행하는 경우 (드론이 업혀있지 않음)
NULL = None # 해당 노드가 전체 ROUTES에 할당이 안되어 있는 경우 (SOLUTION에서 UNVISITED 한 노드)

class Repair():
    """
    1. 원래 state에서 Destroy된 부분을 우선 greedy하게 truck route로 메꾸기 -> (Sacramento, 2019)
    2. 트럭으로 채워서 만든 전체 state에 대하여 random한 sortie를 새로 계산

    아직 can_insert (feasibility) 고려안하고 무조건 TRUE인 상태
    """
    
    def greedy_drone_repair(self, state, rnd_state):
        """
        drone repair operator 함수
        greedy_drone_repair : depot뒤 ~ depot앞 idx에 unassigned 노드를 ''ONLY_DRONE''으로 끼워보고 best한 cost 발생시키는 routes 반환
        """
        repairss = MultiModalState(state.routes,state.unassigned)
        routes = repairss.routes
        unassigned = repairss.unassigned
        
        while len(unassigned) > 0:
            customer = random.choice(unassigned)
            unassigned.remove(customer)
            best_route, best_idx = self.drone_best_insert(customer, routes)    
            
            if best_route is not None and best_idx is not None:
                for i, route in enumerate(routes):
                    if route == best_route:
                        routes[i] = route[:best_idx] + [customer] + route[best_idx:]
                        self.drone_repair_visit_type_update(routes)
            else:
                self.greedy_truck_repair(MultiModalState(routes,unassigned),rnd_state) #드론으로 갈수 없는게 있으면 트럭으로 수행하도록
                                        
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
                if self.drone_can_insert(data, customer, route, routes, idx): # 삽입 가능한 경우
                    cost = self.drone_insert_cost(customer, route, idx, routes)
                    if best_cost is None or cost < best_cost:
                        best_cost, best_route, best_idx = cost, route, idx

        return best_route, best_idx

    
        
    def drone_can_insert(self, data, customer, route, routes, idx): 
        
        #new_route = route[:idx] + [customer] + route[idx:]
        #routes = [new_route] + [r for r in routes if r != route]  # 선택된 route만을 새로운 route로 대체하여 리스트에 수정
        #self.drone_repair_visit_type_update(routes)
        #self.route_duplication_check(routes)
        
        # 해당 고객의 logistic_load가 cargo_limit_drone 보다 클 때 False
        if data["logistic_load"][customer[0]] > data["cargo_limit_drone"]:
            return False
        
        # 해당 노드의 Landing spot의 availability가 1이 아닐 때 False
        if data["availability_landing_spot"][customer[0]] == 0:
            return False

        # 해당 고객의 Drone delivery preference가 1이 아닐 때 False
        if data["customer_drone_preference"][customer[0]] == 0:
            return False
        
        # 해당 노드에 배송 예정인 Item code가 "드론 배송불가"이면 False

        # CATCH 노드 (visit_type == 3)에 동시에 도착하도록 제어해줘야함

        # 최종적으로 Depot에 도착했을 때 걸린 시간이 time limit 보다 크면 False

        # 손님 개개인의 time window -> version 2

        return True



    def drone_insert_cost(self, customer, route, idx, routes): #드론 경로도 고려되어있으므로 분할 후, MM State.object() 적용
        """
        Computes the insertion cost for inserting customer in route at idx.
        """
        
        new_route = route[:idx] + [customer] + route[idx:]
        routes = [new_route] + [r for r in routes if r != route]  # 선택된 route만을 새로운 route로 대체하여 리스트에 수정
        
        self.drone_repair_visit_type_update(routes)
        self.route_duplication_check(routes)

        return MultiModalState(routes).objective()
    
    
    
    
    
    
    
    
    
    
    def greedy_truck_repair(self, state, rnd_state):
        """
        truck repair operator
        greedy_truck_repair : depot뒤 ~ depot앞 idx에 unassigned 노드를 ''IDLE/ONLY_TRUCK''으로 끼워보고 best한 cost 발생시키는 routes 반환
        """
        repairs = MultiModalState(state.routes,state.unassigned)
        routes = repairs.routes
        unassigned = repairs.unassigned
        
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
                            routes[i] = [(0, 0), customer, (0, 0)]
                
            
        self.truck_repair_visit_type_update(routes) #최종적으로 visit_type 검사
        self.route_duplication_check(routes)

        return MultiModalState(routes, unassigned)
    
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
        return True


    def truck_insert_cost(self, customer, route, idx, routes): #드론 경로도 고려되어있으므로 분할 후, MM State.object() 적용
        """
        Computes the insertion cost for inserting customer in route at idx.
        """
        
        new_route = route[:idx] + [customer] + route[idx:]
        routes = [new_route] + [r for r in routes if r != route]  # 선택된 route만을 새로운 route로 대체하여 리스트에 수정
        
        self.truck_repair_visit_type_update(routes)
        self.route_duplication_check(routes)

        return MultiModalState(routes).objective()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def drone_repair_visit_type_update(self,routes):
        """
        visit_type update 함수
        """
        
        for i, route in enumerate(routes):
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
    
    def truck_repair_visit_type_update(self,routes):
        """
        visit_type update 함수
        """
        
        for i, route in enumerate(routes):
            for j in range(1,len(route)):
                for j in range(1,len(route)-1):
                    if route[j][1] is None:
                        k = j - 1  # 현재 노드의 이전 노드부터 시작
                        if k >= 0:
                            if route[k][1] == IDLE:
                                route[j] = (route[j][0], IDLE)
                            elif route[k][1] == FLY:
                                route[j] = (route[j][0], ONLY_TRUCK)
                            elif route[k][1] == ONLY_DRONE:
                                route[j] = (route[j][0], ONLY_TRUCK)
                            elif route[k][1] == CATCH:
                                route[j] = (route[j][0], IDLE)
                            elif route[k][1] == ONLY_TRUCK:
                                route[j] = (route[j][0], ONLY_TRUCK)
                            
                        l = j + 1
                        if l < len(route):
                            if route[l][1] == IDLE:
                                route[j] = (route[j][0], IDLE)
                            elif route[l][1] == ONLY_DRONE:
                                route[j] = (route[j][0], ONLY_TRUCK) 
                            elif route[l][1] == CATCH:
                                route[j] = (route[j][0], ONLY_TRUCK)
                            elif route[l][1] == ONLY_TRUCK:
                                route[j] = (route[j][0], ONLY_TRUCK)
                                
        return routes
    
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
    
    
            
            
    
    