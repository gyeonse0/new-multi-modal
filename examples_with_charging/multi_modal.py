import copy
import time
import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd

from FileReader import *
from SolutionPlotter import *
from RouteInitializer import *
from RouteGenerator import *
from Destroy import *
from Repair import *
from MultiModalState import *

SEED = 1234
rnd_state = np.random.RandomState(None)

vrp_file_path = r'C:\Users\User\OneDrive\바탕 화면\examples\data\multi_modal_data.vrp'
sol_file_path = r'C:\Users\User\OneDrive\바탕 화면\examples\data\multi_modal_data.sol'

file_reader = FileReader()
data = file_reader.read_vrp_file(vrp_file_path)
bks = file_reader.read_sol_file(sol_file_path)

Rep = Repair()
destroyer = Destroy()
plotter = SolutionPlotter(data)

initializer = RouteInitializer(data)
initial_truck = initializer.init_truck()
plotter.plot_current_solution(initial_truck,name="Init Solution(Truck NN)")

destroy_operators = [destroyer.random_removal, destroyer.can_drone_removal,destroyer.high_cost_removal] #여기에 오퍼레이터 추가
repair_operators = [Rep.drone_first_truck_second, Rep.truck_first_drone_second, Rep.heavy_truck_repair] #여기에 오퍼레이터 추가
destroy_counts = {destroyer.__name__: 0 for destroyer in destroy_operators}
repair_counts = {repairer.__name__: 0 for repairer in repair_operators}
destroy_scores = {destroyer.__name__: [] for destroyer in destroy_operators}
repair_scores = {repairer.__name__: [] for repairer in repair_operators}

destroy_probabilities = [0.33, 0.34, 0.33]  # 각각의 파괴 연산자에 대한 확률(성능기반의 score가 아니라, 확률만 고려함으로써 더욱 랜덤성 부여)
repair_probabilities = [0.6, 0.1, 0.3]  # 각각의 수리 연산자에 대한 확률

init = initial_truck
runtime_seconds = 1800

# 초기 설정
start_temperature = 100
end_temperature = 0.01
step = 0.1

current_states = []  # 상태를 저장할 리스트
objectives = []  # 목적 함수 값을 저장할 리스트

# 초기 온도 설정
temperature = start_temperature
start_time = time.time()

while time.time() - start_time < runtime_seconds:
    if time.time() - start_time == 0:
        #처음 에는 init을 기반으로 수행
        selected_destroy_operator = np.random.choice(destroy_operators, p=destroy_probabilities)
        selected_repair_operator = np.random.choice(repair_operators, p=repair_probabilities)

        destroyed_state = selected_destroy_operator(init, rnd_state)
        repaired_state = selected_repair_operator(destroyed_state, rnd_state)
        
        current_states.append(repaired_state)
        objective_value = MultiModalState(repaired_state).new_objective()
        objectives.append(objective_value)

        d_idx = destroy_operators.index(selected_destroy_operator)
        r_idx = repair_operators.index(selected_repair_operator)

        destroy_counts[destroy_operators[d_idx].__name__] += 1
        repair_counts[repair_operators[r_idx].__name__] += 1

    else:
        # 파괴 및 수리 연산자 확률에 따라 랜덤 선택(select)
        selected_destroy_operator = np.random.choice(destroy_operators, p=destroy_probabilities)
        selected_repair_operator = np.random.choice(repair_operators, p=repair_probabilities)
        # 선택된 연산자를 사용하여 상태 업데이트
        destroyed_state = selected_destroy_operator(current_states[-1].copy(), rnd_state)
        repaired_state = selected_repair_operator(destroyed_state, rnd_state)
        
        # 이전 objective 값과 비교하여 수락 여부 결정(accept)
        if np.exp((MultiModalState(current_states[-1]).new_objective() - MultiModalState(repaired_state).new_objective()) / temperature) >= rnd.random():
            current_states.append(repaired_state)
            objective_value = MultiModalState(repaired_state).new_objective()
            objectives.append(objective_value)
        else:
            # 이전 상태를 그대로 유지
            current_states.append(current_states[-1])
            objectives.append(MultiModalState(current_states[-1]).new_objective())

        # 온도 갱신
        temperature = max(end_temperature, temperature - step)

        #오퍼레이터 수 계산
        d_idx = destroy_operators.index(selected_destroy_operator)
        r_idx = repair_operators.index(selected_repair_operator)

        destroy_counts[destroy_operators[d_idx].__name__] += 1
        repair_counts[repair_operators[r_idx].__name__] += 1

min_objective = min(objectives)
min_index = objectives.index(min_objective)

print("\nBest Objective Value:",MultiModalState(current_states[min_index]).new_objective())
print("Best Solution:",MultiModalState(current_states[min_index]).routes)
print("Iteration #:",min_index)
pct_diff = 100 * (MultiModalState(current_states[min_index]).new_objective() - init.new_objective()) / init.new_objective()
print(f"This is {-(pct_diff):.1f}% better than the initial solution, which is {init.new_objective()}.")

plotter.plot_current_solution(current_states[min_index])

plt.figure(figsize=(10, 6))
plt.plot(objectives, label='Current Objective')
plt.plot(np.minimum.accumulate(objectives), color='orange', linestyle='-', label='Best Objective')

plt.title('Progress of Objective Value')
plt.xlabel('Iteration(#)')
plt.ylabel('Objective Value(Kwh)')
plt.grid(True)
plt.legend()
plt.show()

print("\nDestroy Operator Counts(#):")
for name, count in destroy_counts.items():
    print(f"{name}: {count}")
print("\nRepair Operator Counts(#):")
for name, count in repair_counts.items():
    print(f"{name}: {count}")