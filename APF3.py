import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class APF:
    def __init__(self, start, goal, obstacles, side, Katt, Krep, step_size, Dgoal):
        self.start = start
        self.q_goal = np.array(goal, dtype=float)
        self.q_obj = obstacles
        self.side = side
        self.rho0 = self.create_rho()
        self.Katt = Katt
        self.Krep = Krep
        self.step_size = step_size
        self.Dgoal = Dgoal
        self.q_robot = np.array(start, dtype=float)

    def create_rho(self):
        rho = []
        for i in range(len(self.side)):
            rho.append(max(self.side[i][0], self.side[i][1]))
        rho = 1 * np.array(rho, dtype=float)
        return rho

    def dist(self, q):
        x_robot = self.q_robot[0]
        y_robot = self.q_robot[1]
        x = q[0]
        y = q[1]
        return np.linalg.norm([x_robot - x, y_robot - y])

    def Fatt(self):
        return self.Katt * (self.q_goal - self.q_robot)

    def Frep(self, q, rho):
        r = self.dist(q)
        return ((self.Krep / r ** 3) * (1 / r - 1 / rho) * (self.q_robot - q) * abs(
            self.q_robot - self.q_goal) ** 4) - (4 / 2) * (self.Krep * (1 / r - 1 / rho) ** 2) * (
                    (self.q_robot - self.q_goal) ** 3)

    def calculate_rep(self):
        sum = 0
        for i, q in enumerate(self.q_obj):
            if self.dist(q) < self.rho0[i]:
                sum += self.Frep(q, self.rho0[i])
        return sum

    def no_collision(self, point):
        for i in range(len(self.q_obj)):
            xmin = self.q_obj[i][0] - 0.5 * self.side[i][0]
            xmax = self.q_obj[i][0] + 0.5 * self.side[i][0]
            ymin = self.q_obj[i][1] - 0.5 * self.side[i][1]
            ymax = self.q_obj[i][1] + 0.5 * self.side[i][1]
            if point[0] > xmin and point[0] < xmax and point[1] > ymin and point[1] < ymax:
                return 0
        return 1

    def Jitter(self, theta_prev, delta_theta):
        x = self.q_robot[0] + 0.2 * self.step_size * np.cos(theta_prev + 0.5 * delta_theta)
        y = self.q_robot[1] + 0.2 * self.step_size * np.sin(theta_prev + 0.5 * delta_theta)
        if self.no_collision([x, y]):
            self.q_robot[0] = x
            self.q_robot[1] = y
        else:
            self.q_robot[0] = self.q_robot[0] + 0.2 * self.step_size * np.cos(theta_prev + delta_theta)
            self.q_robot[1] = self.q_robot[1] + 0.2 * self.step_size * np.sin(theta_prev + delta_theta)

    def dynamics(self):
        theta_prev = 0
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.grid()
        scat = ax.scatter([], [], color='blue')
        ax.plot(self.q_robot[0], self.q_robot[1], marker='o')
        ax.plot(self.q_goal[0], self.q_goal[1], marker='o')
        rectangles = [None] * len(self.q_obj)
        sign = np.ones(len(self.q_obj))
        i = 0
        while (1):
            i += 1
            if not self.no_collision(self.q_robot):
                print("collision")
                break
            if self.dist(self.q_goal) < self.Dgoal:
                print("APF3", i - 1)
                break
            Fatt = self.Fatt()
            Frep = self.calculate_rep()
            F = Frep + Fatt
            theta = np.arctan2(F[1], F[0])
            delta_theta = theta - theta_prev
            if i == 1 and abs(delta_theta) > np.pi / 2:
                self.Jitter(theta_prev, delta_theta)
            else:
                a = np.array([np.cos(theta), np.sin(theta)])
                self.q_robot += a * self.step_size

            theta_prev = theta
            scat.set_offsets([self.q_robot[0], self.q_robot[1]])

            if i % 10 == 0 or i == 1:
                for j in range(len(self.q_obj)):
                    if rectangles[j]:
                        rectangles[j].remove()
                for j in range(len(self.q_obj)):
                    if i == 1 and j % 2 == 0:
                        sign[j] = 1
                    elif i == 1:
                        sign[j] = -1
                    if (self.q_obj[j][1] + 0.5 * self.side[j][1] > 100):
                        sign[j] = -1
                    if (self.q_obj[j][1] - 0.5 * self.side[j][1] < 0):
                        sign[j] = 1
                    self.q_obj[j][1] += sign[j] * self.step_size * 2

                    rectangles[j] = patches.Rectangle(((self.q_obj[j][0] - 0.5 * self.side[j][0], self.q_obj[j][1] - 0.5 *
                                                    self.side[j][1])), self.side[j][0], self.side[j][1], facecolor='grey')

                    ax.add_patch(rectangles[j])

            plt.pause(0.05)

#start, goal, obstacles, rho0, Katt, Krep, step_size, Dgoal

side = [[5, 20], [15, 15], [20, 10], [15, 5], [10, 15], [20, 15], [15, 15]]
obstacles = [[29, 12], [35, 69], [69, 47], [33, 25], [32, 37], [68, 74], [62, 94]]


abf = APF([0,0], [100,100], obstacles, side, 0.5, 0.4, 0.5, 2)
abf.dynamics()

