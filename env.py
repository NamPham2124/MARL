import numpy as np

class Robot:
    def __init__(self, position):
        self.position = position
        self.has_package = False  # Thêm trạng thái mang hàng
        self.carrying = 0 
        self.target = None        # Thêm mục tiêu hiện tại
        self.current_package = None


class Package: 
    def __init__(self, start, start_time, target, deadline, package_id): 
        self.start = start
        self.start_time = start_time
        self.target = target
        self.deadline = deadline
        self.picked = False       # Thêm trạng thái đã nhặt
        self.package_id = package_id
        self.status = 'None' # Possible statuses: 'waiting', 'in_transit', 'delivered'

class Environment: 

    def __init__(self, map_file, max_time_steps = 100, n_robots = 5, n_packages=20,
             move_cost=-0.01, delivery_reward=10., delay_reward=1., 
             seed=2025): 
        """ Initializes the simulation environment. :param map_file: Path to the map text file. :param move_cost: Cost incurred when a robot moves (LRUD). :param delivery_reward: Reward for delivering a package on time. """ 
        self.map_file = map_file
        self.grid = self.load_map()
        self.n_rows = len(self.grid)
        self.n_cols = len(self.grid[0]) if self.grid else 0 
        self.move_cost = move_cost 
        self.delivery_reward = delivery_reward 
        self.delay_reward = delay_reward
        self.t = 0 
        self.robots = [] # List of Robot objects.
        self.packages = [] # List of Package objects.
        self.total_reward = 0

        self.n_robots = n_robots
        self.max_time_steps = max_time_steps
        self.n_packages = n_packages

        self.rng = np.random.RandomState(seed)
        self.reset()
        self.done = False
        self.state = None

    def load_map(self):
        """
        Reads the map file and returns a 2D grid.
        Assumes that each line in the file contains numbers separated by space.
        0 indicates free cell and 1 indicates an obstacle.
        """
        grid = []
        with open(self.map_file, 'r') as f:
            for line in f:
                # Strip line breaks and split into numbers
                row = [int(x) for x in line.strip().split(' ')]
                grid.append(row)
        return grid
    
    def is_free_cell(self, position):
        """
        Checks if the cell at the given position is free (0) or occupied (1).
        :param position: Tuple (row, column) to check.
        :return: True if the cell is free, False otherwise.
        """
        r, c = position
        if r < 0 or r >= self.n_rows or c < 0 or c >= self.n_cols:
            return False
        return self.grid[r][c] == 0

    def add_robot(self, position):
        """
        Adds a robot at the given position if the cell is free.
        :param position: Tuple (row, column) for the robot's starting location.
        """
        if self.is_free_cell(position):
            robot = Robot(position)
            self.robots.append(robot)
        else:
            raise ValueError("Invalid robot position: must be on a free cell not occupied by an obstacle or another robot.")

    def reset(self):
        """
        Resets the environment to its initial state.
        Clears all robots and packages, and reinitializes the grid.
        """
        self.t = 0
        self.robots = []
        self.packages = []
        self.total_reward = 0
        self.done = False
        self.state = None

        # Reinitialize the grid
        #self.grid = self.load_map(sel)
        # Add robots and packages
        tmp_grid = np.array(self.grid)
        for i in range(self.n_robots):
            # Randomly select a free cell for the robot
            position, tmp_grid = self.get_random_free_cell(tmp_grid)
            self.add_robot(position)
        
        N = self.n_rows
        list_packages = []
        for i in range(self.n_packages):
            # Randomly select a free cell for the package
            start = self.get_random_free_cell_p()
            while True:
                target = self.get_random_free_cell_p()
                if start != target:
                    break
            
            to_deadline = 10 + self.rng.randint(N/2, 3*N)
            if i <= min(self.n_robots, 20):
                start_time = 0
            else:
                start_time = self.rng.randint(1, self.max_time_steps)
            list_packages.append((start_time, start, target, start_time + to_deadline ))

        list_packages.sort(key=lambda x: x[0])
        for i in range(self.n_packages):
            start_time, start, target, deadline = list_packages[i]
            package_id = i+1
            self.packages.append(Package(start, start_time, target, deadline, package_id))

        return self.get_state()
    
    def get_state(self):
        """
        Returns the current state of the environment.
        The state includes the positions of robots and packages.
        :return: State representation.
        """
        selected_packages = []
        for i in range(len(self.packages)):
            if self.packages[i].start_time == self.t:
                selected_packages.append(self.packages[i])
                self.packages[i].status = 'waiting'

        state = {
            'time_step': self.t,
            'map': self.grid,
            'robots': [(robot.position[0] + 1, robot.position[1] + 1,
                        robot.carrying) for robot in self.robots],
            'packages': [(package.package_id, package.start[0] + 1, package.start[1] + 1, 
                          package.target[0] + 1, package.target[1] + 1, package.start_time, package.deadline) for package in selected_packages]
        }
        return state
        

    def get_random_free_cell_p(self):
        """
        Returns a random free cell in the grid.
        :return: Tuple (row, col) of a free cell.
        """
        free_cells = [(i, j) for i in range(self.n_rows) for j in range(self.n_cols) \
                      if self.grid[i][j] == 0]
        i = self.rng.randint(0, len(free_cells))
        return free_cells[i]


    def get_random_free_cell(self, new_grid):
        """
        Returns a random free cell in the grid.
        :return: Tuple (row, col) of a free cell.
        """
        free_cells = [(i, j) for i in range(self.n_rows) for j in range(self.n_cols) \
                      if new_grid[i][j] == 0]
        i = self.rng.randint(0, len(free_cells))
        new_grid[free_cells[i][0]][free_cells[i][1]] = 1
        return free_cells[i], new_grid

    
    def step(self, actions):
        """
        Advances the simulation by one timestep.
        :param actions: A list where each element is a tuple (move_action, package_action) for a robot.
            move_action: one of 'S', 'L', 'R', 'U', 'D'.
            package_action: '1' (pickup), '2' (drop), or '0' (do nothing).
        :return: The updated state, reward for this step, done flag, and info dict.
        """
        r = 0
        if len(actions) != len(self.robots):
            raise ValueError("The number of actions must match the number of robots.")

        # -------- Process Movement --------
        proposed_positions = []
        old_pos = {}
        next_pos = {}

        for i, robot in enumerate(self.robots):
            move, pkg_act = actions[i]
            new_pos = self.compute_new_position(robot.position, move)
            # Kiểm tra vị trí mới có hợp lệ không (trong bản đồ và không phải vật cản)
            if not self.valid_position(new_pos):
                new_pos = robot.position  # Nếu không hợp lệ thì giữ nguyên vị trí
            proposed_positions.append(new_pos)
            old_pos[robot.position] = i
            next_pos[new_pos] = i

        moved_robots = [0] * len(self.robots)
        computed_moved = [0] * len(self.robots)
        final_positions = [None] * len(self.robots)
        occupied = {}

        while True:
            updated = False
            for i in range(len(self.robots)):
                if computed_moved[i] != 0:
                    continue

                pos = self.robots[i].position
                new_pos = proposed_positions[i]
                can_move = False

                if new_pos not in old_pos:
                    can_move = True
                else:
                    j = old_pos[new_pos]
                    if (j != i) and (computed_moved[j] == 0):
                        continue
                    can_move = True

                if can_move:
                    if new_pos not in occupied:
                        occupied[new_pos] = i
                        final_positions[i] = new_pos
                        computed_moved[i] = 1
                        moved_robots[i] = 1
                        updated = True
                    else:
                        # Nếu vị trí bị chiếm, robot giữ nguyên vị trí cũ
                        new_pos = pos
                        occupied[new_pos] = i
                        final_positions[i] = pos
                        computed_moved[i] = 1
                        moved_robots[i] = 0
                        updated = True

                if updated:
                    break

            if not updated:
                break

        for i in range(len(self.robots)):
            if computed_moved[i] == 0:
                final_positions[i] = self.robots[i].position

        # Cập nhật vị trí robot và tính chi phí di chuyển
        for i, robot in enumerate(self.robots):
            move, pkg_act = actions[i]
            if move in ['L', 'R', 'U', 'D'] and final_positions[i] != robot.position:
                r += self.move_cost
            robot.position = final_positions[i]

        # -------- Process Package Actions --------
        for i, robot in enumerate(self.robots):
            move, pkg_act = actions[i]

            # Khởi tạo thuộc tính carrying và has_package nếu chưa có
            if not hasattr(robot, 'carrying'):
                robot.carrying = 0
            if not hasattr(robot, 'has_package'):
                robot.has_package = False

            # Nhặt hàng
            if pkg_act == '1':
                if robot.carrying == 0:
                    for j in range(len(self.packages)):
                        pkg = self.packages[j]
                        if pkg.status == 'waiting' and pkg.start == robot.position and pkg.start_time <= self.t:
                            robot.carrying = pkg.package_id
                            robot.has_package = True
                            pkg.status = 'in_transit'
                            pkg.picked = True
                            break

            # Thả hàng
            elif pkg_act == '2':
                if robot.carrying != 0:
                    package_id = robot.carrying
                    pkg = self.packages[package_id - 1]
                    if robot.position == pkg.target:
                        pkg.status = 'delivered'
                        if self.t <= pkg.deadline:
                            r += self.delivery_reward
                        else:
                            r += self.delay_reward
                        robot.carrying = 0
                        robot.has_package = False

        # Tăng bước thời gian
        self.t += 1

        self.total_reward += r

        done = False
        infos = {}

        if self.check_terminate():
            done = True
            infos['total_reward'] = self.total_reward
            infos['total_time_steps'] = self.t

        return self.get_state(), r, done, infos


        return self.get_state(), r, done, infos
    
    def check_terminate(self):
        if self.t == self.max_time_steps:
            return True
        
        for p in self.packages:
            if p.status != 'delivered':
                return False
            
        return True

    def compute_new_position(self, position, move):
        """
        Computes the intended new position for a robot given its current position and move command.
        """
        r, c = position
        if move == 'S':
            return (r, c)
        elif move == 'L':
            return (r, c - 1)
        elif move == 'R':
            return (r, c + 1)
        elif move == 'U':
            return (r - 1, c)
        elif move == 'D':
            return (r + 1, c)
        else:
            return (r, c)

    def valid_position(self, pos):
        """
        Checks if the new position is within the grid and not an obstacle.
        """
        r, c = pos
        if r < 0 or r >= self.n_rows or c < 0 or c >= self.n_cols:
            return False
        if self.grid[r][c] == 1:
            return False
        return True

    def render(self):
        """
        A simple text-based rendering of the map showing obstacles and robot positions.
        Obstacles are represented by 1, free cells by 0, and robots by 'R'.
        """
        # Make a deep copy of the grid
        grid_copy = [row[:] for row in self.grid]
        for i, robot in enumerate(self.robots):
            r, c = robot.position
            grid_copy[r][c] = 'R%i'%i
        for row in grid_copy:
            print('\t'.join(str(cell) for cell in row))
        

if __name__=="__main__":
    env = Environment('map.txt', 10, 2, 5)
    state = env.reset()
    print("Initial State:", state)
    print("Initial State:")
    env.render()

    # Agents
    # Initialize agents
    from greedyagent import GreedyAgents as Agents
    agents = Agents()   # You should define a default parameters here
    agents.init_agents(state) # You have a change to init states which can be used or not. Depend on your choice
    print("Agents initialized.")
    
    # Example actions for robots
    list_actions = ['S', 'L', 'R', 'U', 'D']
    n_robots = len(state['robots'])
    done = False
    t = 0
    while not done:
        actions = agents.get_actions(state) 
        state, reward, done, infos = env.step(actions)
    
        print("\nState after step:")
        env.render()
        print(f"Reward: {reward}, Done: {done}, Infos: {infos}")
        print("Total Reward:", env.total_reward)
        print("Time step:", env.t)
        print("Packages:", state['packages'])
        print("Robots:", state['robots'])

        # For debug purpose
        t += 1
        if t == 100:
            break
    