from collections import deque

class SimpleBFSAgent:
    def __init__(self, env):
        self.env = env
        self.paths = [None] * env.n_robots  # lưu đường đi cho từng robot

    def bfs(self, start, goal, grid):
        """Tìm đường đi ngắn nhất từ start đến goal trên grid bằng BFS"""
        n_rows, n_cols = len(grid), len(grid[0])
        queue = deque()
        queue.append((start, []))
        visited = set()
        visited.add(start)

        while queue:
            current, path = queue.popleft()
            if current == goal:
                return path  # trả về danh sách các bước đi (các vị trí)

            for dx, dy, action in [(-1, 0, 'U'), (1, 0, 'D'), (0, -1, 'L'), (0, 1, 'R')]:
                nx, ny = current[0] + dx, current[1] + dy
                if 0 <= nx < n_rows and 0 <= ny < n_cols:
                    if grid[nx][ny] == 0 and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append(((nx, ny), path + [action]))
        return []  # không tìm được đường

    def act(self, state):
        """
        state: trạng thái hiện tại của môi trường, bạn lấy thông tin cần thiết từ đây
        Trả về danh sách hành động cho từng robot ['L', 'R', 'U', 'D', 'S', '1', '2']
        """
        actions = []
        grid = self.env.grid

        for i, robot in enumerate(self.env.robots):
            if robot.carrying == 0:
                # Robot chưa mang hàng, đi tới điểm lấy hàng (giả sử lấy gói đầu tiên chưa lấy)
                package = None
                for p in self.env.packages:
                    if p.status == 'waiting':
                        package = p
                        break
                if package is None:
                    # Không còn gói hàng cần lấy, đứng yên
                    actions.append('S')
                    continue
                goal = package.start
            else:
                # Robot đang mang hàng, đi tới điểm giao hàng
                package = None
                for p in self.env.packages:
                    if p.status == 'in_transit' and p.package_id == robot.carrying:
                        package = p
                        break
                if package is None:
                    actions.append('S')
                    continue
                goal = package.target

            start = robot.position
            path = self.bfs(start, goal, grid)
            if len(path) == 0:
                # Không tìm được đường, đứng yên
                actions.append('S')
            else:
                # Lấy bước đi đầu tiên trong path làm hành động
                actions.append(path[0])

        return actions
