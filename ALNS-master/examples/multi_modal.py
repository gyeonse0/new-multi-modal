from types import SimpleNamespace
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

from alns import ALNS
from alns.accept import SimulatedAnnealing
from alns.select import RouletteWheel
from alns.stop import MaxIterations

SEED = 1234
rnd_state = np.random.RandomState(None)

vrp_file_path = r'C:\Users\these\Downloads\new-multi-modal-main\new-multi-modal-main\ALNS-master\examples\data\multi_modal_data.vrp'
sol_file_path = r'C:\Users\these\Downloads\new-multi-modal-main\new-multi-modal-main\ALNS-master\examples\data\multi_modal_data.sol'

file_reader = FileReader()
data = file_reader.read_vrp_file(vrp_file_path)
bks = file_reader.read_sol_file(sol_file_path)

### RouteInitializer 클래스 instance 
initializer = RouteInitializer(data, k=2, l=1, max_drone_mission=4)
initial_solution = initializer.nearest_neighbor_init_truck()
initial_truck = initializer.init_truck()

#print("\nonly truck's(NN)", initial_truck)


current_route = initializer.makemakemake(initial_solution)
### currnet route 정보 출력 debugging code
#print("\nCurrent's", current_route)
#print("Current Objective cost :",MultiModalState(current_route).objective())


### SolutionPlotter 클래스 instance
plotter = SolutionPlotter(data)
### route 플러팅 시각화 debugging code
#plotter.plot_current_solution(initial_truck,name="Init Solution(NN/Truck)")
#plotter.plot_current_solution(current_route,name="Multi_Modal Solution")


### Destroy 클래스 debugging code
destroyer = Destroy()
destroyed_route1 = destroyer.random_removal(current_route,rnd_state)
##print("\nrandom removal's", destroyed_route1)
### Destroy 플러팅 시각화 debugging code
#plotter.plot_current_solution(destroyed_route1,name="Random Removal1")

### Repair 클래스 debugging code
Rep = Repair()
repaired_route1 = Rep.drone_first_truck_second(destroyed_route1,rnd_state) # greedy_drone_repair로 변경가능
#print("\nRepair's", repaired_route1)
### Repair 플러팅 시각화 debugging code
#plotter.plot_current_solution(repaired_route1,name="drone Repair1")

destroyed_route2 = destroyer.random_removal(repaired_route1,rnd_state)
#print("\nrandom removal's", destroyed_route2)
### Destroy 플러팅 시각화 debugging code
#plotter.plot_current_solution(destroyed_route2,name="Random Removal2")

repaired_route2 = Rep.drone_first_truck_second(destroyed_route2,rnd_state) # greedy_drone_repair로 변경가능
#print("\nRepair's", repaired_route2)
### Repair 플러팅 시각화 debugging code
#plotter.plot_current_solution(repaired_route2,name="drone Repair2")
"""
class Feasibility:
    
    #heuristics/ALNS part 에서 우리가 설정한 제약조건을 만족하는지 checking하는 클래스
    #return 형식 : Ture/False
    
    def function():
        return True,False
"""

Rep = Repair()
destroyer = Destroy()
plotter = SolutionPlotter(data)

#ALNS MAIN CODE
alns = ALNS(rnd.RandomState(SEED))
alns.add_destroy_operator(destroyer.random_removal)
alns.add_repair_operator(Rep.drone_first_truck_second)
#alns.add_repair_operator(Rep.truck_first_drone_second)
alns.add_repair_operator(Rep.heavy_insertion_repair)
alns.add_repair_operator(Rep.greedy_truck_repair)

init = initializer.makemakemake(initial_solution)

select = RouletteWheel(scores=[25, 0, 0, 0],
                       decay=0.8,
                       num_destroy=1,
                       num_repair=3)
accept = SimulatedAnnealing(start_temperature=1000,
                            end_temperature=0.01,
                            step=0.1,
                            method="exponential")
stop = MaxIterations(10000) #iteration을 5000번 까지 수행하도록 설정

result = alns.iterate(init, select, accept, stop)

solution = result.best_state
objective = solution.objective() #objective 갈아끼우려면 plotter와 repair에 insert_cost 변경
pct_diff = -(100 * (objective - 7) / 7) #일단 우리는 best_known_solution을 정확히 모르니까, initial cost를 7로 설정

plotter.plot_current_solution(solution, 'Simple ALNS')
print("\nALNS's : ",solution)

print(f"Best heuristic objective is {objective}.")
print(f"This is {pct_diff:.1f}%  better than the initial solution, which is 7.")

_, ax = plt.subplots(figsize=(12, 6))
result.plot_objectives(ax=ax)
ax.set_xlim(right=10000)  # x 축 범위를 5000으로 제한
ax.set_xticks(np.arange(0, 10001, 500))  # x 축의 눈금을 0부터 5000까지 100 간격으로 설정
plt.tight_layout()
plt.show()

result.plot_operator_counts()
plt.show()
