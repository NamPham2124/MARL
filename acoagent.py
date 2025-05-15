import numpy as np
import random
from collections import defaultdict, deque
from scipy.optimize import linear_sum_assignment

class ACOAgent:
    def __init__(self):
        self.n_robots = 0
        self.state = None
        
        # ACO Parameters
        self.alpha = 1.0   # Pheromone weight
        self.beta = 2.0    # Heuristic weight
        self.rho = 0.1     # Evaporation rate
        self.Q = 100.0     # Pheromone deposit constant
        self.phi = 0.1     # Exploration probability
        self.max_ants = 20
        
        # Navigation system
        self.pheromone_map = defaultdict(float)
        self.path_cache = defaultdict(deque)
        self.task_assignments = {}
        self.obstacle_map = []
        self.robot_positions = {}
        self.packages = {}

    def init_agents(self, state):
        self.state = state
        self.n_robots = len(state['robots'])
        self.obstacle_map = state['map']
        self._parse_environment(state)
        self._assign_initial_tasks()

    def get_actions(self, state):
        self.state = state
        self._parse_environment(state)
        
        if not self.path_cache:
            self._update_task_assignments()
            self._generate_navigation_paths()
            
        actions = []
        for rid in range(self.n_robots):
            if self.path_cache[rid]:
                action = self.path_cache[rid].popleft()
            else:
                action = ('S', '-1')
            actions.append(action)
            
        return actions

    def _parse_environment(self, state):
        # Parse robot positions (x, y)
        self.robot_positions = {
            rid: (pos[0], pos[1]) for rid, pos in enumerate(state['robots'])
        }
        
        # Parse packages with structure: (id, start_x, start_y, target_x, target_y, status, deadline)
        self.packages = {}
        for pkg in state['packages']:
            if len(pkg) >= 7:
                self.packages[pkg[0]] = {
                    'start': (pkg[1], pkg[2]),
                    'target': (pkg[3], pkg[4]),
                    'status': 'pending' if pkg[5] == 0 else 'in_transit',
                    'deadline': pkg[6]
                }

    def _assign_initial_tasks(self):
        robot_ids = list(range(self.n_robots))
        package_ids = [pid for pid, pkg in self.packages.items() 
                      if pkg['status'] == 'pending']
        
        if not package_ids:
            return
            
        cost_matrix = np.zeros((len(robot_ids), len(package_ids)))
        for ridx in robot_ids:
            start_pos = self.robot_positions[ridx]
            for pidx, pid in enumerate(package_ids):
                end_pos = self.packages[pid]['start']
                path = self._find_aco_path(start_pos, end_pos)
                cost_matrix[ridx][pidx] = len(path) if path else 1e6
                
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        self.task_assignments = {
            robot_ids[r]: package_ids[c] for r, c in zip(row_ind, col_ind)
        }

    def _update_task_assignments(self):
        # Reassign tasks considering current positions and package status
        available_robots = [
            rid for rid in range(self.n_robots)
            if rid not in self.task_assignments or not self.path_cache[rid]
        ]
        
        pending_packages = [
            pid for pid, pkg in self.packages.items()
            if pkg['status'] == 'pending' and pid not in self.task_assignments.values()
        ]

        if available_robots and pending_packages:
            cost_matrix = np.zeros((len(available_robots), len(pending_packages)))
            for ridx, rid in enumerate(available_robots):
                start_pos = self.robot_positions[rid]
                for pidx, pid in enumerate(pending_packages):
                    end_pos = self.packages[pid]['start']
                    path = self._find_aco_path(start_pos, end_pos)
                    cost_matrix[ridx][pidx] = len(path) if path else 1e6

            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            for r, c in zip(row_ind, col_ind):
                self.task_assignments[available_robots[r]] = pending_packages[c]

    def _find_aco_path(self, start, end):
        best_path = None
        min_length = float('inf')
        
        for _ in range(self.max_ants):
            path = [start]
            current = start
            
            for _ in range(100):  # Max path length
                if current == end:
                    break
                    
                neighbors = self._get_valid_neighbors(current)
                if not neighbors:
                    break
                    
                next_node = self._select_next_node(current, neighbors, end)
                path.append(next_node)
                current = next_node
                
            if current == end and len(path) < min_length:
                best_path = path
                min_length = len(path)
                
        if best_path:
            self._update_pheromone_trail(best_path)
            
        return best_path

    def _get_valid_neighbors(self, pos):
        x, y = pos
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(self.obstacle_map) and 0 <= ny < len(self.obstacle_map[0]):
                if self.obstacle_map[nx][ny] == 0:
                    neighbors.append((nx, ny))
        return neighbors

    def _select_next_node(self, current, neighbors, end):
        if random.random() < self.phi:
            return random.choice(neighbors)
            
        probs = []
        total = 0.0
        
        for neighbor in neighbors:
            pheromone = self.pheromone_map[(current, neighbor)] + 1e-6
            heuristic = 1 / (1 + abs(neighbor[0]-end[0]) + abs(neighbor[1]-end[1]))
            weight = (pheromone**self.alpha) * (heuristic**self.beta)
            probs.append(weight)
            total += weight
            
        if total == 0:
            return random.choice(neighbors)
            
        rand = random.uniform(0, total)
        cumulative = 0
        for i, weight in enumerate(probs):
            cumulative += weight
            if rand <= cumulative:
                return neighbors[i]
        return neighbors[-1]

    def _update_pheromone_trail(self, path):
        for edge in self.pheromone_map:
            self.pheromone_map[edge] *= (1 - self.rho)
            
        if path:
            delta = self.Q / len(path)
            for i in range(len(path)-1):
                a, b = path[i], path[i+1]
                self.pheromone_map[(a, b)] += delta
                self.pheromone_map[(b, a)] += delta * 0.5

    def _generate_navigation_paths(self):
        for rid, pid in self.task_assignments.items():
            if rid not in self.robot_positions or pid not in self.packages:
                continue
                
            current_pos = self.robot_positions[rid]
            package = self.packages[pid]
            
            # Path to package pickup
            pickup_path = self._find_aco_path(current_pos, package['start'])
            if pickup_path:
                actions = self._convert_path_to_actions(pickup_path)
                actions.append(('P', str(pid)))
                
                # Path to delivery
                delivery_path = self._find_aco_path(package['start'], package['target'])
                if delivery_path:
                    actions += self._convert_path_to_actions(delivery_path)
                    actions.append(('D', str(pid)))
                    
                self.path_cache[rid] = deque(actions)

    def _convert_path_to_actions(self, path):
        actions = []
        for i in range(len(path)-1):
            current = path[i]
            next_pos = path[i+1]
            dx = next_pos[0] - current[0]
            dy = next_pos[1] - current[1]
            
            if dx == 1:
                actions.append(('D', '-1'))
            elif dx == -1:
                actions.append(('U', '-1'))
            elif dy == 1:
                actions.append(('R', '-1'))
            elif dy == -1:
                actions.append(('L', '-1'))
        return actions
