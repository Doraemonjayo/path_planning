import PathPlanner
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

dt = 0.02

# マウスの座標で点を移動させる関数
def on_move(event):
    global robot

    if event.inaxes and event.button == 1:  # イベントがグラフ領域内で発生した場合

        robot = (event.xdata, event.ydata)

        # 描画を更新
        # plt.draw()

def update(frame):
    global robot

    v = path.calcVelocity(robot)

    arrow_v_forward.set_offsets(robot)
    arrow_v_forward.set_UVC(*path.v_forward)
    arrow_v_lateral.set_offsets(robot)
    arrow_v_lateral.set_UVC(*path.v_lateral)
    arrow_v.set_offsets(robot)
    arrow_v.set_UVC(*v)

    robot = (robot[0] + v[0] * dt, robot[1] + v[1] * dt)

    r.set_data(*robot)

    return arrow_v_forward, arrow_v_lateral, arrow_v, r

path = PathPlanner.Path(((0,0),(10,0),(10,5),(0,5),(0,10),(10,10)), 5, 5)

x = []
y = []

for _path in path.path:
    for t in np.arange(0, _path.t1, 0.1):
        p = _path.t_to_p(t)
        if p is not None:
            x.append(p[0])
            y.append(p[1])

robot = (0,0)

fig, ax = plt.subplots()

# グラフの描画
ax.plot(x, y, 'r', lw = 2)
arrow_v_forward = ax.quiver(0, 0, 1, 1, angles='xy', scale_units='xy', scale=1, color='r')
arrow_v_lateral = ax.quiver(0, 0, 1, 1, angles='xy', scale_units='xy', scale=1, color='g')
arrow_v = ax.quiver(0, 0, 1, 1, angles='xy', scale_units='xy', scale=1, color='b')
r, = ax.plot(0, 0, 'k.', ms = 20)
plt.xlabel("x")               # x軸のラベル
plt.ylabel("y")               # y軸のラベル
plt.title("Path")         # タイトル
plt.axis('equal')
plt.legend()                   # 凡例
plt.grid(True)                 # グリッド線

fig.canvas.mpl_connect('motion_notify_event', on_move)

ani = FuncAnimation(fig, update, interval=dt * 1000, blit=True)

plt.show()    