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
        self.radius = 15*self.step_size


    def create_rho(self):
        rho = []
        for i in range(len(self.side)):
            rho.append(max(self.side[i][0], self.side[i][1]))
        rho = 0.9 * np.array(rho, dtype=float)
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
        rh = self.dist(self.q_goal)
        a = (self.Krep/r**3)*(1/r - 1/rho)*(self.q_robot - q)*(rh**4)/(1+rh**4)
        b = -(4/2)*(self.Krep*(1/r - 1/rho)**2)*(self.q_robot-self.q_goal)*(rh**2)/((1+rh**4)**2)
        return a + b

    def calculate_rep(self):
        sum = 0
        for i, q in enumerate(self.q_obj):
            if self.dist(q) < self.rho0[i]:
                sum += self.Frep(q, self.rho0[i])
        return sum

    def no_collision(self, point,r):
        for i in range(len(self.q_obj)):
            xmin = self.q_obj[i][0] - 0.5 * self.side[i][0]-r
            xmax = self.q_obj[i][0] + 0.5 * self.side[i][0]+r
            ymin = self.q_obj[i][1] - 0.5 * self.side[i][1]-r
            ymax = self.q_obj[i][1] + 0.5 * self.side[i][1]+r
            if point[0]>xmin and point[0]<xmax and point[1]>ymin and point[1]<ymax:
                return 0
        return 1

    def Jitter(self, theta_prev, delta_theta):
            x = self.q_robot[0] + 0.2 * self.step_size * np.cos(theta_prev + 0.5*delta_theta)
            y = self.q_robot[1] + 0.2 * self.step_size * np.sin(theta_prev + 0.5 * delta_theta)
            if self.no_collision([x,y],0):
                self.q_robot[0] = x
                self.q_robot[1] = y
            else:
                self.q_robot[0] = self.q_robot[0] + 0.2 * self.step_size * np.cos(theta_prev + delta_theta)
                self.q_robot[1] = self.q_robot[1] + 0.2 * self.step_size * np.sin(theta_prev + delta_theta)


    def dynamics(self):
        global_goal = self.q_goal
        theta_prev = 0
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.grid()
        scat = ax.scatter([], [], color='blue')
        ax.plot(self.q_robot[0], self.q_robot[1], marker='o')
        ax.plot(global_goal[0], global_goal[1], marker='o')
        rectangles = [None]*len(self.q_obj)
        sign = np.ones(len(self.q_obj))
        i = 0
        j = 0
        self.q_goal = self.select_sub_target(global_goal)
        while(1):
            if j==13 or (math.isclose(self.q_robot[0],self.q_goal[0],abs_tol=self.step_size) and math.isclose(self.q_robot[1],self.q_goal[1],abs_tol=self.step_size)):
                self.q_goal = self.select_sub_target(global_goal)
                j=0
            i += 1
            j+=1
            if not self.no_collision(self.q_robot,0):
                print("collision",self.q_robot)
                break
            if self.dist(global_goal) < self.Dgoal:
                print("We have reached the goal point")
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


            if i%15==0 or i==1:
                for j in range(len(self.q_obj)):
                    if rectangles[j]:
                        rectangles[j].remove()
                for j in range(len(self.q_obj)):
                    if i==1 and j%2==0:
                        sign[j] = 1
                    elif i==1:
                        sign[j] = -1
                    if (self.q_obj[j][1] + 0.5 * self.side[j][1] > 100):
                        sign[j] = -1
                    if (self.q_obj[j][1] - 0.5 * self.side[j][1] < 0):
                        sign[j] = 1
                    self.q_obj[j][1] += sign[j] * self.step_size*2

                    rectangles[j] = patches.Rectangle(((self.q_obj[j][0] - 0.5 * self.side[j][0], self.q_obj[j][1] - 0.5 *
                                                    self.side[j][1])), self.side[j][0], self.side[j][1], facecolor='grey')

                    ax.add_patch(rectangles[j])

            plt.pause(0.05)


    def select_sub_target(self, global_goal):
        radius = self.radius
        angles = np.linspace(0,2*np.pi,80)
        points = [self.q_robot + radius * np.array([np.cos(theta), np.sin(theta)]) for theta in angles]
        h = []
        for point in points:
            if self.no_collision(point,self.step_size*4):
                r = np.linalg.norm(point-global_goal)
            else:
                r = float("inf")
            h.append(r)
        p = points[np.argmin(h)]
        return p



#start, goal, obstacles, rho0, Katt, Krep, step_size, Dgoal

side = [[10, 10], [10, 5], [15, 20], [5, 10], [15, 10], [10, 15]]
obstacles = [[41, 99], [25, 17], [66, 46], [27, 30], [24, 16], [53, 36]]



abf = APF([0,0], [100,100], obstacles, side, 0.5, 20000, 0.5, 2)
path_points = abf.dynamics()



