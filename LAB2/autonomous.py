import numpy as np
import math
import matplotlib.pyplot as plt
import simple_pid as pid


def normalizeAngle(angle):
    """
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


class car:
    def __init__(self, x, y, theta, phi, length, dt):
        print("Initializing....")
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.length = length
        self.dx = 0
        self.dy = 0
        self.dtheta = 0
        self.dphi = 0
        self.trjaectory_x = []
        self.trjaectory_x.append(self.x)
        self.trjaectory_y = []
        self.trjaectory_y.append(self.y)
        self.trjaectory_theta = []
        self.trjaectory_theta.append(self.theta)
        self.dxvector = []
        self.dyvector = []
         

    def controlled(self, current_postion_x, current_position_y, current_theta, true_x, true_y, true_theta, dt):
        """
        :param current_postion_x:   (float) vehicle's x-coordinate [m]
        :param current_position_y:  (float) vehicle's y-coordinate [m]
        :param true_x:              (float vector) path's x-coordinates [m]
        :param true_y:              (float vector) path's y-coordinates [m]
        :param dt:                  (float) time step [s]
        :return v:                  (float) vehicle's speed [m/s]
        :return ws:                 (float) vehicle's steering angle speed [rad/s]
        """
        kv=0.03
        ks=100
        kl=1

        error_world=[true_x-current_postion_x,true_y-current_position_y, true_theta-current_theta]

        error_bot=np.dot(np.array([[math.cos(current_theta)  ,  math.sin(current_theta),     0],
                                    [-math.sin(current_theta),  math.cos(current_theta),     0],
                                    [0                       ,             0           ,     1]]),
                                    error_world)

        v=error_bot[0]*kv
        ws=error_bot[1]*kl+error_bot[2]*ks

        print("v",v)
        print("ws",ws)
                
        return v, ws

    def path_set(self, path_x, path_y, dt):
        """
        :param x:                   (float vector) path's x-coordinates [m]
        :param y:                   (float vector) path's y-coordinates [m]
        :param GPS_x:               (float vector (?) ) vehicle's x-coordinate [m]
        :param GPS_y:               (float vector (?) ) vehicle's y-coordinate [m]
        :return ws:                 (float) vehicle's steering angle speed [rad/s]
        """
        max_phi = math.radians(30)    # Vehicle's steering limits [rad]

        theta = [0]*len(path_x)
        velocity = [0]*len(path_x)
        phi = [0]*len(theta)

        for i in range(len(path_x)-1):
            theta[i] = math.atan2(path_y[i+1]-path_y[i], path_x[i+1]-path_x[i])
            velocity[i] = math.sqrt(
                (path_x[i+1]-path_x[i])**2 + (path_y[i+1]-path_y[i])**2)/dt
        theta[-1] = math.atan2(path_y[1]-path_y[-1], path_x[1]-path_x[-1])
        velocity[-1] = math.sqrt((path_x[1]-path_x[-1])
                                 ** 2 + (path_y[1]-path_y[-1])**2)/dt

        theta_deg = [math.degrees(x) for x in theta]
        #print("theta", theta_deg)
        #print("velocity", velocity)

        for i in range(len(theta)-1):
            phi[i] = math.atan2(
                (normalizeAngle(theta[i+1]-theta[i]))/dt * self.length, velocity[i])
        phi[-1] = math.atan2((theta[1]-theta[-1]) * self.length, velocity[-1])

        phi_deg = [math.degrees(x) for x in phi]
        #print("phi", phi_deg)

        # for i in range(len(phi)):
        #     phi[i] = normalizeAngle(phi[i])
        #     phi[i] = -max_phi if phi[i] < - \
        #         max_phi else max_phi if phi[i] > max_phi else phi[i]
        # norm_phi_deg = [math.degrees(x) for x in phi]
        # print("normalized phi", norm_phi_deg)
        ws = []
        for i in range(1,len(phi)):
            ws.append((phi[i]- phi[i-1])/dt)
        ws.append((phi[0]- phi[-1])/dt)
        #print("ws", ws)

        return velocity, ws  # NOT phi , it should be ws

    def update(self, v, ws, dt):
        self.dphi = ws
        # self.phi = ws
        self.phi += self.dphi*dt
        self.dtheta = v*math.tan(self.phi)/self.length
        self.theta += self.dtheta*dt
        self.dx = v*math.cos(self.theta)
        self.dy = v*math.sin(self.theta)
        self.x += self.dx*dt
        self.y += self.dy*dt
        self.trjaectory_x.append(self.x)
        self.trjaectory_y.append(self.y)
        self.dxvector.append(self.dx)
        self.dyvector.append(self.dy)
        self.trjaectory_theta.append(self.theta)

    def plot(self, color):
        plt.plot(self.trjaectory_x, self.trjaectory_y, color)
        for i in range(len(self.trjaectory_x)-1):
            plt.arrow(self.trjaectory_x[i], self.trjaectory_y[i], self.dxvector[i],
                      self.dyvector[i], head_width=0.1, head_length=0.5, color='red')
        plt.savefig('trajectory.png')


def main():
    x = [200, 300, 400, 400, 300, 200, 100, 100, 200]
    y = [100, 100, 200, 300, 400, 400, 300, 200, 100]
    dt = 1
    ratio = 100

    x_aux=[0]*((len(x)-1)*(ratio))
    y_aux=[0]*((len(y)-1)*(ratio))
    theta_aux=[0]*((len(x)-1)*(ratio))

    for i in range(len(x)-1):
        for j in range(ratio):
            x_aux[i*ratio+j] = x[i]+(x[i+1]-x[i])*j/ratio
            y_aux[i*ratio+j] = y[i]+(y[i+1]-y[i])*j/ratio
            theta_aux[i*ratio+j] = math.atan2(y_aux[i*ratio+j]-y_aux[i*ratio+j-1], x_aux[i*ratio+j]-x_aux[i*ratio+j-1])

    print("x_aux", x_aux)
    print("y_aux", y_aux)

    Auto = car(x[0], y[0], 0, 0, 2.46, dt)

    actual_x = [200, 300, 402, 400, 302, 201,  90, 100, 201]
    actual_y = [100, 101, 201, 300, 390, 380, 301, 180, 101]

    actual_x_aux = [0]*len(actual_x)*ratio
    actual_y_aux = [0]*len(actual_y)*ratio
    actual_theta_aux = [0]*len(actual_x)*ratio

    for i in range(len(actual_x)-1):
        for j in range(ratio):
            actual_x_aux[i*ratio+j] = actual_x[i]+(actual_x[i+1]-actual_x[i])*j/ratio
            actual_y_aux[i*ratio+j] = actual_y[i]+(actual_y[i+1]-actual_y[i])*j/ratio
            actual_theta_aux[i*ratio+j] = math.atan2(actual_y_aux[i*ratio+j]-actual_y_aux[i*ratio+j-1], actual_x_aux[i*ratio+j]-actual_x_aux[i*ratio+j-1])
    actual_x_aux[-1] = actual_x[0]
    actual_y_aux[-1] = actual_y[0]

    print("Setting path....")
    velocity, ws = Auto.path_set(x_aux, y_aux, dt)
    print("velocity", velocity)
    print("ws", ws)

    print("Updating....")
    for i in range(len(ws)-1):
        print()
        print("Step", i)
        #after the first iteration the controller is called to resolve any error
        if(i!=0):
            controlv, controlws=Auto.controlled(Auto.x,Auto.y,Auto.theta, x_aux[i-1], y_aux[i-1], theta_aux[i-1], dt)
            ws[i] += controlws
            velocity[i] += controlv
        Auto.update(velocity[i], ws[i], dt)
        print("Position calculated", i, "Theta - ",Auto.theta,"X-", Auto.x,"Y- ",Auto.y)
        Auto.x = actual_x_aux[i]
        Auto.y = actual_y_aux[i]
        Auto.theta = actual_theta_aux[i]
        print("Position wanted", i, "Theta - ",theta_aux[i],"X-", x_aux[i],"Y- ", y_aux[i])
        print("Position actual", i, "Theta - ", Auto.theta, "X-", Auto.x, "Y- ", Auto.y)
    print("Plotting....")
    Auto.plot('b')

    # print("Updating....")
    # for i in range(len(ws)):
    #     Auto.update(velocity[i], ws[i], dt)
    #     print("Trajectory", i, "Theta - ",Auto.trjaectory_theta[i],"X-", Auto.trjaectory_x[i],"Y- ",Auto.trjaectory_y[i])
    # print("Plotting....")
    # Auto.plot('b')

if __name__ == '__main__':
    main()
