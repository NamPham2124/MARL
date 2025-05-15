from env import Environment, Robot, Package
from gui import run_simulation              # Đảm bảo bạn đã tạo file gui.py chứa hàm run_simulation
from greedyagent import GreedyAgents as Agents  # Tên class đúng

# Lưới bản đồ: 0 là ô trống, 1 là tường
grid = [
    [0, 0, 0, 0, 1],
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 0, 0, 0, 0],
]

# Vị trí khởi tạo robot
robots = [Robot(position=(0, 0)), Robot(position=(4, 4))]

# Gói hàng và thời gian/đích đến của chúng
packages = [
    Package(start=(0, 2), start_time=0, target=(3, 4), deadline=10, package_id=1),
    Package(start=(2, 0), start_time=0, target=(0, 4), deadline=15, package_id=2)
]

# Khởi tạo môi trường
env = Environment(map_file="map4.txt", max_time_steps=100, n_robots=2, n_packages=20)



# Dùng agent Greedy
agent = Agents()

# Chạy mô phỏng với giao diện
run_simulation(env, agent)
