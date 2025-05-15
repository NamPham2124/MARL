import tkinter as tk

CELL_SIZE = 50
DELAY = 0.5  # thời gian delay giữa mỗi bước (giây)

ROBOT_COLORS = ['#3498db', '#2ecc71', '#9b59b6', '#e67e22', '#1abc9c']

def draw_grid(canvas, grid):
    rows = len(grid)
    cols = len(grid[0])
    for i in range(rows):
        for j in range(cols):
            color = "white" if grid[i][j] == 0 else "black"
            canvas.create_rectangle(
                j*CELL_SIZE, i*CELL_SIZE,
                (j+1)*CELL_SIZE, (i+1)*CELL_SIZE,
                fill=color, outline="gray"
            )

def draw_packages(canvas, packages, current_time):
    for pkg in packages:
        if (not hasattr(pkg, 'picked') or not pkg.picked) and \
           (not hasattr(pkg, 'status') or pkg.status == 'waiting') and \
           current_time >= pkg.start_time:
            x, y = pkg.start
            canvas.create_rectangle(
                y*CELL_SIZE + 15, x*CELL_SIZE + 15,
                y*CELL_SIZE + CELL_SIZE - 15, x*CELL_SIZE + CELL_SIZE - 15,
                fill="green", outline="#27ae60", width=2
            )
            time_left = max(pkg.deadline - current_time, 0)
            canvas.create_text(
                y*CELL_SIZE + CELL_SIZE/2, x*CELL_SIZE + CELL_SIZE/2,
                text=str(time_left), fill="white", font=('Arial', 10, 'bold')
            )

def draw_robots(canvas, robots):
    for idx, robot in enumerate(robots):
        x, y = robot.position
        has_package = getattr(robot, 'has_package', False) or getattr(robot, 'carrying', 0) != 0
        color = "#e74c3c" if has_package else ROBOT_COLORS[idx % len(ROBOT_COLORS)]

        canvas.create_oval(
            y*CELL_SIZE + 10, x*CELL_SIZE + 10,
            y*CELL_SIZE + CELL_SIZE - 10, x*CELL_SIZE + CELL_SIZE - 10,
            fill=color, outline="#2c3e50", width=2
        )

        canvas.create_text(
            y*CELL_SIZE + CELL_SIZE/2, x*CELL_SIZE + CELL_SIZE/2,
            text=str(idx+1), fill="white", font=('Arial', 12, 'bold')
        )

        if has_package and getattr(robot, 'target', None) is not None:
            tx, ty = robot.target
            canvas.create_polygon(
                ty*CELL_SIZE + CELL_SIZE/2, tx*CELL_SIZE + 5,
                ty*CELL_SIZE + CELL_SIZE - 5, tx*CELL_SIZE + CELL_SIZE/2,
                ty*CELL_SIZE + CELL_SIZE/2, tx*CELL_SIZE + CELL_SIZE - 5,
                ty*CELL_SIZE + 5, tx*CELL_SIZE + CELL_SIZE/2,
                fill="#f1c40f", outline="#f39c12", width=2
            )

def run_simulation(env, agent, max_steps=None):
    agent.init_agents(env.get_state())

    root = tk.Tk()
    root.title("Delivery Simulation")

    # Tạo frame chính chứa canvas và phần reward
    main_frame = tk.Frame(root)
    main_frame.pack()

    # Canvas bên trái
    canvas = tk.Canvas(main_frame,
                       width=len(env.grid[0]) * CELL_SIZE,
                       height=len(env.grid) * CELL_SIZE)
    canvas.grid(row=0, column=0, padx=10, pady=10)

    # Frame bên phải để hiển thị reward
    info_frame = tk.Frame(main_frame)
    info_frame.grid(row=0, column=1, sticky='n', padx=10, pady=10)

    # Label tiêu đề
    reward_title = tk.Label(info_frame, text="Reward", font=('Arial', 16, 'bold'))
    reward_title.pack(pady=(0,10))

    # Label hiển thị giá trị reward
    reward_label = tk.Label(info_frame, text="0", font=('Arial', 14))
    reward_label.pack()

    step = 0

    def update_step():
        nonlocal step
        if max_steps is not None and step >= max_steps:
            return

        canvas.delete("all")
        draw_grid(canvas, env.grid)
        draw_packages(canvas, env.packages, env.t)
        draw_robots(canvas, env.robots)
        root.update()

        actions = agent.get_actions(env.get_state())
        env.step(actions)

        # Cập nhật reward (giả sử env.reward có sẵn)
        current_reward = getattr(env, 'reward', 0)
        reward_label.config(text=f"{current_reward}")

        step += 1
        root.after(int(DELAY * 1000), update_step)

    update_step()
    root.mainloop()
