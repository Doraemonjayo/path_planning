import PathPlanner
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import numpy as np
import platform

dt = 0.02
num_robots = 10  # ロボットの数
num_frames = 300  # 録画するフレーム数

# 録画するかどうかのフラグ
record_video = False  # Trueにすると録画、Falseにすると録画しない

# ロボットの状態を保持するリスト
robots = [(np.random.uniform(-5, 15), np.random.uniform(-5, 15)) for _ in range(num_robots)]

def reset_positions():
    """ロボットをランダムな位置に再配置 (-5～15の範囲)"""
    global robots
    robots = [(np.random.uniform(-5, 15), np.random.uniform(-5, 15)) for _ in range(num_robots)]

def on_key(event):
    """スペースキーを押すとロボットの位置を再配置"""
    if event.key == ' ':
        reset_positions()
        print("Robots reset to new random positions.")

def update(frame):
    global robots

    # 各ロボットの位置を更新
    for i, robot in enumerate(robots):
        v = path.calcVelocity(robot)
        robots[i] = (robot[0] + v[0] * dt, robot[1] + v[1] * dt)

        # ロボットの位置を更新
        robot_points[i].set_data(*robots[i])

    return robot_points

path = PathPlanner.Path(((0, 0), (10, 0), (10, 5), (0, 5), (0, 10), (10, 10)), 10, 100)

x = []
y = []

# Pathの全ての点を描画用に取得
for _path in path.path:
    for t in np.arange(0, _path.length, 0.1):
        p = _path.t_to_p(t)
        if p is not None:
            x.append(p[0])
            y.append(p[1])

fig, ax = plt.subplots()

# グラフの描画
ax.plot(x, y, 'r', lw=2)

# 各ロボットの描画要素を初期化
robot_points = [ax.plot(0, 0, 'k.', ms=10)[0] for _ in range(num_robots)]

plt.xlabel("x")               # x軸のラベル
plt.ylabel("y")               # y軸のラベル
plt.title("Path")             # タイトル
plt.axis('equal')
plt.legend()                  # 凡例
plt.grid(True)                # グリッド線

# 描画範囲を設定 (-5～15)
ax.set_xlim(-5, 15)
ax.set_ylim(-5, 15)

# キーボード入力イベントを登録
fig.canvas.mpl_connect('key_press_event', on_key)

# OS判定して動画ライターを選択
if platform.system() == 'Windows':
    # WindowsでFFmpegWriterを使うためにffmpegをインストールしている必要があります
    writer = FFMpegWriter(fps=int(1 / dt))
else:
    # LinuxでもFFMpegWriterを使用
    writer = FFMpegWriter(fps=int(1 / dt))

# アニメーション作成
ani = FuncAnimation(fig, update, frames=num_frames, interval=dt * 1000, blit=True)

# 録画フラグがTrueの場合にのみ動画を保存
if record_video:
    ani.save("robots_animation.mp4", writer=writer)
    # 動画保存完了メッセージ
    print("Animation saved as 'robots_animation.mp4'.")

# プロットを表示
plt.show()
