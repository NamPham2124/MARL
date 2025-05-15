# priorityagent.py
# -*- coding: utf-8 -*-
import heapq
from collections import deque

class PriorityAgent:
    def __init__(self):
        self.initialized = False
        self.n_rows = 0
        self.n_cols = 0
        self.time_step = 0
        self.robots = {}          # {id: {'pos': (x,y), 'carrying': pkg_id}}
        self.packages = {}        # {id: {'start': (x,y), 'target': (x,y), 'deadline': int, 'status': str}}
        self.task_heap = []       
        self.path_cache = {}      # {robot_id: deque(path)}
        self.robots_order = []    
        self.assigned_pkgs = set()
        self.robot_states = {}    
        self.map = []             
        self.failed_assignments = set()
        self.path_memory = {}

    def init_agents(self, state):
        self.time_step = state['time_step']
        self.map = [row.copy() for row in state['map']]
        self.n_rows = len(self.map)
        self.n_cols = len(self.map[0]) if self.n_rows > 0 else 0
        self._parse_robots(state['robots'])
        self._parse_packages(state['packages'])
        self._build_priority_queue()
        self.initialized = True

    def _parse_robots(self, robots):
        # Khởi tạo lại các từ điển để tránh dữ liệu cũ
        self.robots_order = list(range(len(robots)))
        self.robots = {}
        self.path_cache = {}
        self.robot_states = {}
        
        for idx in range(len(robots)):
            robot = robots[idx]
            # Xử lý giá trị carrying mặc định (-1 -> None)
            carrying = robot[2] if len(robot) > 2 and robot[2] != -1 else None
            
            self.robots[idx] = {
                'pos': (robot[0], robot[1]),
                'carrying': carrying
            }
            # Khởi tạo path cache và trạng thái cho mọi robot
            self.path_cache[idx] = deque()
            self.robot_states[idx] = 'idle'

    def _parse_packages(self, packages):
        self.packages = {}
        for pkg in packages:
            pkg_id = pkg[0]
            self.packages[pkg_id] = {
                'start': (pkg[1], pkg[2]),
                'target': (pkg[3], pkg[4]),
                'deadline': pkg[6],
                'status': 'pending'
            }

    def _build_priority_queue(self):
        self.task_heap = []
        for pkg_id, pkg in self.packages.items():
            if pkg['status'] == 'pending' and pkg_id not in self.assigned_pkgs:
                heapq.heappush(self.task_heap, (-pkg['deadline'], pkg_id))

    def get_actions(self, state):
        if not self.initialized:
            self.init_agents(state)
        
        actions = {}
        for rid in self.robots_order:
            move_action, package_action = self._get_robot_action(rid)
            actions[rid] = (move_action, package_action)
        return actions

    def _get_robot_action(self, robot_id):
        # Kiểm tra path cache
        if not self.path_cache[robot_id]:
            self._assign_package(robot_id)
        
        # Lấy action từ path cache
        if self.path_cache[robot_id]:
            return self.path_cache[robot_id].popleft(), None
        return ('stay', None)

    def _find_path(self, start, end):
        # Triển khai thuật toán tìm đường (ví dụ BFS)
        # Giả lập path đơn giản cho mục đích demo
        path = []
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        
        if dx != 0:
            direction = 'D' if dx > 0 else 'U'
            path += [direction] * abs(dx)
        if dy != 0:
            direction = 'R' if dy > 0 else 'L'
            path += [direction] * abs(dy)
            
        return deque(path)

    def _assign_package(self, robot_id):
        if self.task_heap:
            _, pkg_id = heapq.heappop(self.task_heap)
            pkg = self.packages[pkg_id]
            
            # Tạo path từ robot hiện tại -> start -> target
            robot_pos = self.robots[robot_id]['pos']
            pickup_path = self._find_path(robot_pos, pkg['start'])
            deliver_path = self._find_path(pkg['start'], pkg['target'])
            
            # Kết hợp các hành động
            full_path = pickup_path
            full_path.append('P')  # Nhặt package
            full_path += deliver_path
            full_path.append('D')  # Trả package
            
            self.path_cache[robot_id] = full_path
            self.assigned_pkgs.add(pkg_id)
            self.robot_states[robot_id] = 'busy'

if __name__ == "__main__":
    agent = PriorityAgent()
