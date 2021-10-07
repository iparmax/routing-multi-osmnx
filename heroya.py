### Heroya Scheduling algorithm ### 

# Required Libraries
import osmnx as ox
import osmnx.distance as distance
import networkx as nx
import pandas as pd
import numpy as np
import geopandas as gpd
import matplotlib.pyplot as plt
from shapely.geometry import Point
import matplotlib.animation as animation
import time as tm

# Functions
def proj_points(coordinates):
    """Projects GPS (WGS84) coordinates to local UTM grid."""
    lats = coordinates[:,0]
    lngs = coordinates[:,1]
    points_list = [Point((lng, lat)) for lat, lng in zip(lats, lngs)]
    points = gpd.GeoSeries(points_list, crs='epsg:4326')
    points_proj = points.to_crs(graph.graph['crs'])
    return points_proj

def most_frequent(List):
    """Finds the most frequent number on a given list."""
    counter = 0
    num = List[0]
    for i in List:
        curr_frequency = List.count(i)
        if(curr_frequency> counter):
            counter = curr_frequency
            num = i
    return num

def find_exit(node):
    """Finds node of closest gate (by measure of time) to exit from current node."""
    min_length= np.inf
    for i in enter_nodes:
        length = nx.shortest_path_length(G=graph, source=node, target=i, weight='time')
        if min_length >= length:
            min_length = length
            exit_node = i
    return exit_node

def find_gate(node_in,node_final):
    """Finds node of gate that is on the shortest path trajectory."""
    for i in enter_nodes:
        route = nx.shortest_path(graph, source=node_in, target=node_final, weight='time')
        for k in route:
            if k == i:
                return i 

def sample_agents():
    agents = []
    regions = len(truck_origins)
    facilities = len(truck_destinations)
    for i in range(trucks):
        agent = {}
        choice = np.random.randint(low=1,high = regions+1)
        agent['id'] = i
        agent['origin_region'] = chr(choice+96).upper()
        agent['origin_node'] = origin_nodes[choice-1]
        choice = np.random.randint(low=0,high = facilities)
        agent['target_destination'] = export_modes[choice]
        agent['dest_node'] = dest_nodes[choice]
        time_range = np.random.randint(low=100*trucks,high = 700*trucks)
        agent['window'] = time_range
        agents.append(agent)
    return agents


# Global Variables
global trucks
global graph
global export_modes
global gates
global truck_origins
global truck_destinations
global enter_nodes
global origin_nodes
global dest_nodes
global off_load
global run_time

# Numpy Seed for consistency of results and runtime of experiment
np.random.seed(2)
run_time = 5


# Global Variables Initialization
trucks = 5
graph = ox.load_graphml("data\\heroya_project_big.graphml.xml")
export_modes = ["TRAIN","PORT"]
gates = proj_points(pd.read_csv("data\\gates.csv").values)
truck_origins = proj_points(pd.read_csv("data\\truck_origins.csv").values)
truck_destinations = proj_points(pd.read_csv("data\\truck_destinations.csv").values)
enter_nodes = [x for x in distance.nearest_nodes(graph, gates.x,gates.y)]
origin_nodes = [x for x in distance.nearest_nodes(graph, truck_origins.x,truck_origins.y)]
dest_nodes = [x for x in distance.nearest_nodes(graph, truck_destinations.x,truck_destinations.y)]
off_load = 500 # Seconds of time to offload

# Starting depot position
cav_orig_depot = proj_points(pd.read_csv("data\\depot.csv").values)
cav_depot = cav_orig_depot
depot_node = distance.nearest_nodes(graph, cav_depot.x,cav_depot.y)[0]

# Add edge attributes to graph
graph = ox.add_edge_speeds(graph)
graph = ox.add_edge_travel_times(graph)

# Generate Agents
agents = sample_agents()

# Initialize Algorithm

# Outer Loop List initialization
constraints = [] 
all_selection = []
all_times = []
all_windows = []
all_taxi_begin = []
all_truck_begin = []
all_cav_begin = []
all_time_of_exit = []
all_cav_exit_escort = []
all_cav_trajectory = []
all_agents_trajectory = []
all_agents_leave = []
all_true_order = []
all_cav_total = []

# Outer Loop Flag initialization
factor = True
start = tm.time()

# Count feasible solutions Found
counter = 0

# Outer Loop to find optimal solution
while factor:

    # Inner Loop List initialization
    selection = []
    times = []
    windows = []
    taxi_begin = []
    truck_begin = []
    cav_begin = []
    time_of_exit = []
    cav_exit_escort = []
    cav_trajectory = []
    agents_trajectory = []
    agents_leave = []
    true_order = []
    agents_id = [d['id'] for d in agents if 'id' in d]
    not_exited = [d['id'] for d in agents if 'id' in d]

    # Inner Loop Flag Initialization
    scrap_selection = False

    # Inner Loop Constant Initialization
    prob = 1/(counter+1)
    cav_total = 0
    extra_time = 0
    depot_node = distance.nearest_nodes(graph, cav_orig_depot.x,cav_depot.y)[0]

    # Inner Loop to find feasible solution
    while not_exited != [] or agents_id != []:
        
        # Select next agent
        if agents_id != []:
            pick = np.random.choice(agents_id)

            # Pick next agent based on frequency of an agents in constraints
            if constraints != []:
                pick_list =  [d['agent'] for d in constraints if 'agent' in d]
                pick_true = [x for x in pick_list if x not in selection]
                if pick_true!= [] and np.random.rand()<=prob:
                    pick = most_frequent(pick_true)        

            # Calculate routes if this pick does not violate a constraint
            agent = [x for x in agents if x['id'] == pick][0]
            if {'agent':pick,'selection':selection} not in constraints:              
                enter_gate = find_gate(agent['origin_node'],agent['dest_node'])
                cav_route = nx.shortest_path(G=graph, source=depot_node, target=enter_gate, weight='travel_time')
                cav_time = round(sum(ox.utils_graph.get_route_edge_attributes(graph, cav_route, "travel_time")))
                route_to_gate = nx.shortest_path(G=graph, source=agent['origin_node'], target=enter_gate, weight='travel_time')
                time_to_gate = round(sum(ox.utils_graph.get_route_edge_attributes(graph, route_to_gate, "travel_time")))
                if selection == []:
                    truck_begin.append(max(0,cav_time-time_to_gate))
                    cav_begin.append(max(0,time_to_gate-cav_time))
                    taxi_begin.append((max(0,time_to_gate-cav_time))+cav_time)
                else:
                    truck_begin.append(times[-1]+extra_time+cav_time-time_to_gate)
                    cav_begin.append(times[-1]+extra_time)
                    taxi_begin.append(times[-1]+extra_time+cav_time)        
                route_to_dest =  nx.shortest_path(G=graph, source=enter_gate, target=agent['dest_node'], weight='travel_time')
                time_to_dest = round(sum(ox.utils_graph.get_route_edge_attributes(graph, route_to_dest, "travel_time")))
                cav_trajectory.append(cav_route[:-1]+route_to_dest)
                agents_trajectory.append(route_to_gate+route_to_dest)
                times.append(taxi_begin[-1]+extra_time+time_to_dest)
                depot_node = agent['dest_node'] # New depot node
                selection.append(agent['id'])
                true_order.append(agent['id'])
                windows.append(agent['window'])
                agents_id.remove(agent['id'])
                cav_total += cav_time + time_to_dest
            else:
                scrap_selection = True
        
        # Guide exits based on FIFO if all agents served
        if agents_id == []:
            while True:
                for idx,a in enumerate(selection):
                    if len(times) == trucks and a in not_exited:
                        agent_to_exit = a
                        break
                exit_agent = [x for x in agents if x['id'] == agent_to_exit][0]
                cav_route = nx.shortest_path(G=graph, source=depot_node, target=exit_agent['dest_node'], weight='travel_time')     
                cav_time = round(sum(ox.utils_graph.get_route_edge_attributes(graph, cav_route, "travel_time")))
                exit_gate = find_exit(agent['dest_node'])
                route_to_exit = nx.shortest_path(G=graph, source=exit_agent['dest_node'], target=exit_gate, weight='travel_time')
                time_to_exit = round(sum(ox.utils_graph.get_route_edge_attributes(graph, route_to_exit, "travel_time")))
                agents_trajectory.append(route_to_exit)
                true_order.append(exit_agent['id']+trucks)
                cav_trajectory.append(cav_route[:-1]+route_to_exit)
                depot_node = exit_gate
                last_time = max(time_of_exit + times)
                cav_exit_escort.append(last_time)
                time_of_exit.append(last_time+cav_time+time_to_exit)
                not_exited.remove(exit_agent['id'])
                cav_total += cav_time + time_to_exit
                if not_exited == []:
                    break 

        # Initiate exit of a truck waiting more than the offload time
        flag_to_exit = False
        for idx,agent_id in enumerate(selection):
            if times[-1]-times[idx] >= off_load and agent_id in not_exited and agents_id !=[]:
                flag_to_exit = True
                agent_to_exit = agent_id
                break
            
        if flag_to_exit and not_exited!=[]:
            exit_agent = [x for x in agents if x['id'] == agent_to_exit][0]
            cav_route = nx.shortest_path(G=graph, source=depot_node, target=exit_agent['dest_node'], weight='travel_time')
            cav_time = round(sum(ox.utils_graph.get_route_edge_attributes(graph, cav_route, "travel_time")))
            enter_gate = find_gate(agent['origin_node'],agent['dest_node'])
            route_to_exit = nx.shortest_path(G=graph, source=exit_agent['dest_node'], target=enter_gate, weight='travel_time')
            time_to_exit = round(sum(ox.utils_graph.get_route_edge_attributes(graph, route_to_exit, "travel_time")))
            agents_trajectory.append(route_to_exit)
            true_order.append(exit_agent['id']+trucks)
            cav_trajectory.append(cav_route[:-1]+route_to_exit)
            depot_node = enter_gate
            time_of_exit.append(times[-1]+cav_time+time_to_exit)
            cav_exit_escort.append(times[-1])
            not_exited.remove(exit_agent['id'])
            extra_time = cav_time+time_to_exit
            cav_total += cav_time + time_to_exit

        # Basic Constraint violation - Time windows
        if windows !=[]:
            if windows[-1] < times[-1]:
                scrap_selection = True
                constraints.append({'agent':selection[-1],'selection':selection[:-1]})

        # Find a new selection from the start
        if scrap_selection == True:
            break
    
        if len(true_order) == trucks*2:
            counter+=1
            print(counter,true_order)

        if agents_id == [] and not_exited == []:
            all_selection.append(selection)
            all_cav_total.append(cav_total)
            all_times.append(times)
            all_windows.append(windows)
            all_cav_begin.append(cav_begin)
            all_truck_begin.append(truck_begin)
            all_taxi_begin.append(taxi_begin)
            all_time_of_exit.append(time_of_exit)
            all_cav_exit_escort.append(cav_exit_escort)
            all_cav_trajectory.append(cav_trajectory)
            all_agents_trajectory.append(agents_trajectory)
            all_agents_leave.append(agents_leave)
            all_true_order.append(true_order)
            constraints.append({'agent':selection[-1],'selection':selection[:-1]})

        end = tm.time()
        if end-start >=run_time and len(all_selection) >=1:
            factor = False
            min_idx = all_cav_total.index(min(all_cav_total))
            selection_final = all_selection[min_idx]
            windows_final = all_windows[min_idx]
            times_final = all_times[min_idx]
            cav_begin_final = all_cav_begin[min_idx]
            truck_begin_final = all_truck_begin[min_idx]
            taxi_begin_final = all_taxi_begin[min_idx]
            time_of_exit_final = all_time_of_exit[min_idx]
            cav_exit_escort_final = all_cav_exit_escort[min_idx]
            cav_trajectory_final = all_cav_trajectory[min_idx]
            agents_trajectory_final = all_agents_trajectory[min_idx]
            agents_leave_final = all_agents_leave[min_idx]
            true_order_final = all_true_order[min_idx]
            cav_total_final = all_cav_total[min_idx]

# Print Results
minus = min(truck_begin+cav_begin)
cav_exit_escort_final = [int(num)-minus for num in cav_exit_escort_final]
times_final = [int(num)-minus for num in times_final]
truck_begin_final = [int(num)-minus for num in truck_begin_final]
cav_begin_final = [int(num)-minus for num in cav_begin_final]
time_of_exit_final = [int(num)-minus for num in time_of_exit_final]
taxi_begin_final = [int(num)-minus for num in taxi_begin_final]

print(f"Order of Entry and Exit is : {selection_final}")
print(f"Truck Begins route to entrance (seconds) : {truck_begin_final}")
print(f"CAV begins route to entrance (seconds): {cav_begin_final}")
print(f"Taxi begins at (seconds): {taxi_begin_final}")
print(f"Truck & CAV arrive at destination (seconds) : {times_final}")
print(f"Truck must be at destination before (seconds) : {windows_final}")
print(f"CAV begins route to offloaded truck (seconds) : {cav_exit_escort_final}")
print(f"Truck Exits (seconds) : {time_of_exit_final}")
print(f"Total time spent by CAV (seconds) : {cav_total_final}")

### Visualization Process ###

cav_cum = []
idx = []
for i,j in zip(agents_trajectory_final,cav_trajectory_final):
    cav_cum = cav_cum  + j
    idx.append(len(cav_cum)-len(i))

for j,index in enumerate(idx):
    if index != min(idx):
        for i in range(index - min(idx)):
            agents_trajectory_final[j] = [agents_trajectory_final[j][0]] + agents_trajectory_final[j]

for i in range(abs(min(idx))):
    cav_cum = [cav_cum[0]]+cav_cum

agents = []
for idx1,i in enumerate(true_order_final):
    for idx2,j in enumerate(true_order_final) :
        if j == i+ trucks and i < trucks :
            route1 = agents_trajectory_final[idx1]
            route2 = agents_trajectory_final[idx2]
            c = []
            for k in range(len(route2)):
                if k <len(route1):
                    c.append(route1[k])
                else:
                    c.append(route2[k])
            agents.append(c)

routes = []
routes.append(cav_cum)
routes = routes + agents

route_coorindates = []
for route in routes:
    points = []
    for node_id in route:
        x = graph.nodes[node_id]['x']
        y = graph.nodes[node_id]['y']
        points.append([x, y])
    route_coorindates.append(points)
    
n_routes = len(route_coorindates)
max_route_len = max([len(x) for x in route_coorindates])

# Prepare the layout
fig, ax = ox.plot_graph(graph, node_size=0, edge_linewidth=0.5, show=False, close=False,bgcolor='#f7f5fa') # network
ax.scatter(graph.nodes[dest_nodes[0]]['x'],graph.nodes[dest_nodes[0]]['y'], marker = "x",label = "Train Station",s=10**2)
ax.scatter(graph.nodes[dest_nodes[1]]['x'],graph.nodes[dest_nodes[1]]['y'], marker = "x",label = "Port",s=10**2)

scatter_list = []

# Plot the first scatter plot
for j in range(n_routes):
    if j !=0:
        scatter_list.append(ax.scatter(route_coorindates[j][0][0],
                                   route_coorindates[j][0][1],
                                   label=f'Truck {j}', 
                                   alpha=.75,
                                   s=5**2))
    else:
        scatter_list.append(ax.scatter(route_coorindates[j][0][0], 
                                   route_coorindates[j][0][1], 
                                   label=f'CAV ', 
                                   alpha=.75,
                                   marker = "*",
                                   s=10**2))

    
plt.legend(frameon=False)

def animate(i):
    """Animate scatter plot (car movement)
    
    Args:
        i (int) : Iterable argument. 
    
    Returns:
        None
        
    """
    for j in range(n_routes):
        try:
            x_j = route_coorindates[j][i][0]
            y_j = route_coorindates[j][i][1]
            scatter_list[j].set_offsets(np.c_[x_j, y_j])
        except:
            scatter_list[j].set_offsets(np.c_[0, 0])

# Make the animation
anim = animation.FuncAnimation(fig, animate,frames=max_route_len)
writergif = animation.PillowWriter(fps=5)
anim.save(f'trucks_trajectory_{trucks}.gif',writer=writergif)