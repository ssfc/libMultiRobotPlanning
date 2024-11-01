#!/usr/bin/env python3

from matplotlib import animation
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection

import argparse
import math
import matplotlib
# matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as manimation
import numpy as np
import yaml


Colors = ['orange']  #, 'blue', 'green']


class Animation:
    # 构造函数接受 map 和 schedule 参数，分别用于提供地图信息和路径调度信息。
    def __init__(self, _map, _schedule):
        self.map = _map
        self.schedule = _schedule

        # 计算地图的纵横比 aspect
        aspect = _map["map"]["dimensions"][0] / _map["map"]["dimensions"][1]

        # 创建了一个无边框的图形 fig
        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        # 将绘图区 ax 设置为长宽比例相等。
        self.ax = self.fig.add_subplot(111, aspect='equal')
        # 随后调整子图的边距，让绘图区完全填充画布：
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()

        # 根据 map 数据中定义的地图尺寸，定义了边界范围
        # create boundary patch
        x_min = -0.5
        y_min = -0.5
        x_max = _map["map"]["dimensions"][0] - 0.5
        y_max = _map["map"]["dimensions"][1] - 0.5

        # self.ax.relim()
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)
        # self.ax.set_xticks([])
        # self.ax.set_yticks([])
        # plt.axis('off')
        # self.ax.axis('tight')
        # self.ax.axis('off')

        # 遍历障碍物信息，将每个障碍物位置绘制为红色的方块。
        self.patches.append(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='none', edgecolor='red'))
        for o in _map["map"]["obstacles"]:
            x, y = o[0], o[1]
            self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='red', edgecolor='red'))

        # create agents:
        self.T = 0
        # 为每个代理的目标位置绘制半透明方块表示，颜色基于预定义的 Colors 数组。
        # draw goals first
        for d, i in zip(_map["agents"], range(0, len(_map["agents"]))):
            if "goal" in d:
                goals = [d["goal"]]
            if "potentialGoals" in d:
                goals = [goal for goal in d["potentialGoals"]]
            for goal in goals:
                self.patches.append(
                    Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5, facecolor=Colors[i % len(Colors)],
                              edgecolor='black', alpha=0.5))

        # 创建一个代表代理的圆形对象，并将其起始位置设为 start 坐标。
        for d, i in zip(_map["agents"], range(0, len(_map["agents"]))):
            name = d["name"]
            self.agents[name] = Circle((d["start"][0], d["start"][1]), 0.3, facecolor=Colors[i % len(Colors)],
                                       edgecolor='black')
            self.agents[name].original_face_color = Colors[i % len(Colors)]
            self.patches.append(self.agents[name])
            # 通过 self.T 保存调度中最后一个时间步数。
            self.T = max(self.T, _schedule["schedule"][name][-1]["t"])
            # 在代理起始位置显示名称标注。
            self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], name.replace('agent', ''))
            self.agent_names[name].set_horizontalalignment('center')
            self.agent_names[name].set_verticalalignment('center')
            self.artists.append(self.agent_names[name])

        # self.ax.set_axis_off()
        # self.fig.axes[0].set_visible(False)
        # self.fig.axes.get_yaxis().set_visible(False)

        # self.fig.tight_layout()

        # 指定 animate_func 和 init_func 作为每帧的更新和初始化函数，并设置帧数为 T + 1。每帧以 100 毫秒的间隔更新。
        self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                                            init_func=self.init_func,
                                            frames=int(self.T + 1) * 10,
                                            interval=100,
                                            blit=True)

    # speed：控制视频播放的速度。
    def save(self, file_name, speed):
        self.anim.save(
            file_name,
            "ffmpeg",  # 指定使用 ffmpeg 编码器保存视频。
            fps=10 * speed,  # 设置视频的帧率
            dpi=200),  # 设置分辨率
        # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    def show(self):
        plt.show()  # 将当前创建的 matplotlib 图形窗口显示出来。

    # 在开始动画前将所有的图形元素（如补丁和文本）添加到绘图区域中
    def init_func(self):
        # self.patches 是一个包含矩形和圆形等图形元素的列表，表示障碍物、边界、目标位置以及其他可视化对象。
        # 遍历 self.patches 中的每个图形元素，并将其添加到 ax（绘图区域）中，以便在绘图时显示。
        for p in self.patches:
            self.ax.add_patch(p)
        # self.artists 是一个包含艺术元素的列表，通常为文本元素，用于展示每个 agent 的名称或其他信息。
        # 遍历 self.artists 中的每个艺术元素（文本）并将其添加到 ax 中。
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, i):
        # 遍历 self.schedule["schedule"] 中每个 agent 的路径。
        for agent_name in self.schedule["schedule"]:
            agent = schedule["schedule"][agent_name]
            pos = self.get_state(i / 10, agent)  # 获取当前时间步 i / 10 下 agent 的位置。
            p = (pos[0], pos[1])  # 将位置数据转换为坐标元组 p。
            self.agents[agent_name].center = p  # 更新每个 agent 的 center 属性，以更改其在画布中的位置。
            self.agent_names[agent_name].set_position(p)  # 将文本标签与 agent 的当前位置对齐。

        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        # check agent-agent collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)  # agent 的坐标位置（以 NumPy 数组形式）
                pos2 = np.array(d2.center)
                # 如果两个 agent 的距离小于 0.7，则认为发生碰撞。
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')  # 将碰撞的两个 agent 标记为红色。
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {})".format(i, j))
        # 返回包含所有图形元素和文本标签的列表，以便在 FuncAnimation 的每一帧中刷新图形。
        return self.patches + self.artists

    # 在给定时间 t 获取 agent 在路径 d 上的插值位置
    def get_state(self, t, d):

        # 找到第一个时间大于或等于 t 的位置。
        idx = 0
        while idx < len(d) and d[idx]["t"] < t:
            idx += 1

        # t 小于 d 中的第一个时间戳，返回第一个位置 [d[0]["x"], d[0]["y"]]。
        if idx == 0:
            return np.array([float(d[0]["x"]), float(d[0]["y"])])
        # t 位于 d[idx - 1] 和 d[idx] 之间，计算插值。
        elif idx < len(d):
            posLast = np.array([float(d[idx - 1]["x"]), float(d[idx - 1]["y"])])
            posNext = np.array([float(d[idx]["x"]), float(d[idx]["y"])])
        # t 超出最后一个时间戳，返回最后一个位置 [d[-1]["x"], d[-1]["y"]]。
        else:
            return np.array([float(d[-1]["x"]), float(d[-1]["y"])])

        # 计算时间间隔。
        dt = d[idx]["t"] - d[idx - 1]["t"]
        # 归一化时间 t 到 [0, 1] 范围。
        t = (t - d[idx - 1]["t"]) / dt
        # 根据 t 的比例在 posLast 和 posNext 之间进行线性插值。
        pos = (posNext - posLast) * t + posLast

        # 返回 t 时刻的插值位置。
        return pos


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map")  # 输入的地图文件
    parser.add_argument("schedule", help="schedule for agents")  # 包含 agent 移动时间表的文件。
    parser.add_argument('--video', dest='video', default=None,
                        help="output video file (or leave empty to show on screen)")
    parser.add_argument("--speed", type=int, default=1, help="speedup-factor")  # 指定动画播放的加速倍数
    args = parser.parse_args()

    with open(args.map) as map_file:
        map = yaml.safe_load(map_file)  # 将地图文件加载为字典对象

    with open(args.schedule) as states_file:
        schedule = yaml.safe_load(states_file)  # 将时间表文件加载为字典对象。

    animation = Animation(map, schedule)  # 创建 Animation 实例，将地图和时间表数据传入，设置动画对象。

    if args.video:
        animation.save(args.video, args.speed)
    else:
        animation.show()

# Run on windows platform:
# python visualize.py ../benchmark/32x32_obst204/map_32by32_obst204_agents10_ex1.yaml output.yaml