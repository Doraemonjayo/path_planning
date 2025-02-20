import PathPlanner2
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
    arrow_v_forward.set_UVC(*path.forwardVelocity)
    arrow_v_lateral.set_offsets(robot)
    arrow_v_lateral.set_UVC(*path.lateralVelocity)
    arrow_v.set_offsets(robot)
    arrow_v.set_UVC(*v)

    robot = (robot[0] + v[0] * dt, robot[1] + v[1] * dt)

    r.set_data(*robot)

    return arrow_v_forward, arrow_v_lateral, arrow_v, r

# std::vector<Vector2d> points = {
#             Vector2d(0, 0),
#             // Vector2d(0, 0),

#             Vector2d(1, 0),
#             Vector2d(1, 1),
#             Vector2d(0, 1),

#             Vector2d(-3.112, TCF * 1.362), // スタートゾーン
#             Vector2d(-3.112, TCF * (1.362 - 0.7)), // スタートゾーンからでた箇所
#             Vector2d(-0.157 - 1.562 - 0.038 - 0.474, TCF * -0.6), // かご回収手前
#             Vector2d(-0.157 - 1.562 - 0.038 - 0.474, TCF * -1.362), // かご回収
#             Vector2d(-0.157 - 1.562 - 0.038 - 0.474 + 0.001, TCF * -0.6 + 0.001), // かご回収手前
#             Vector2d(3.112, TCF * 0.6),
#             Vector2d(3.112, TCF * 1.362),
#             Vector2d(0, 0),
# };

v = 2

path = PathPlanner2.Path((
    ((0, 0), v), 
    ((1, 0), v), 
    ((1, 1), v), 
    ((0, 1), v), 
    ((-3.112, 1.362), v), 
    ((-3.112, 1.362 - 0.7), v), 
    ((-0.157 - 1.562 - 0.038 - 0.474, -0.6), v), 
    ((-0.157 - 1.562 - 0.038 - 0.474, -1.362), 0),
    ((-0.157 - 1.562 - 0.038 - 0.474 + 0.1, -0.6 + 0.1), v),
    ((3.112, 0.6), v),
    ((3.112, 1.362), v / 2),
    ((0, 0), 0),
    ), v, 20, 5)
# path = PathPlanner2.Path((((0, 0), 10), ((10, 0), 10), ((10, 5), 10), ((0, 5), 10), ((0, 10), 10), ((10, 10), 0)), 10, 100, 25)

x = []
y = []

for _path in path.paths:
    for t in np.arange(0, _path.duration, 0.01):
        p = _path.timeToPoint(t)
        if p is not None:
            x.append(p[0])
            y.append(p[1])

robot = (0,0)

fig, ax = plt.subplots()

# グラフの描画
ax.plot(x, y, 'r', lw = 2)
arrow_v_forward = ax.quiver(0, 0, 1, 1, angles='xy', scale_units='xy', scale=1, color='r', alpha=0.5)
arrow_v_lateral = ax.quiver(0, 0, 1, 1, angles='xy', scale_units='xy', scale=1, color='g', alpha=0.5)
arrow_v = ax.quiver(0, 0, 1, 1, angles='xy', scale_units='xy', scale=1, color='b', alpha=0.5)
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