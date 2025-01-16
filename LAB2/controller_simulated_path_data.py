import numpy as np
import math
import matplotlib.pyplot as plt
import simple_pid as pid
import cv2 as cv


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
    def __init__(self, x, y, theta, phi, length, kv, ks, kl, dt, v, ws):
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
        self.trajectory_x = []
        self.trajectory_x.append(self.x)
        self.trajectory_y = []
        self.trajectory_y.append(self.y)
        self.trajectory_theta = []
        self.trajectory_theta.append(self.theta)
        self.dxvector = []
        self.dyvector = []
        self.kv = kv
        self.ks = ks
        self.kl = kl
        self.last_x = x
        self.last_y = y
        self.last_theta = theta
        self.last_phi = 0
        self.last_v = v
        self.last_ws = ws

    def controlled(self, current_postion_x, current_position_y, current_theta, true_x, true_y, true_theta):
        """
        :param current_postion_x:   (float) vehicle's x-coordinate [m]
        :param current_position_y:  (float) vehicle's y-coordinate [m]
        :param true_x:              (float vector) path's x-coordinates [m]
        :param true_y:              (float vector) path's y-coordinates [m]
        :param dt:                  (float) time step [s]
        :return v:                  (float) vehicle's speed [m/s]
        :return ws:                 (float) vehicle's steering angle speed [rad/s]
        """

        error_world=[true_x-current_postion_x,true_y-current_position_y, true_theta-current_theta]

        error_bot=np.dot(np.array([[math.cos(current_theta)  ,  math.sin(current_theta),     0],
                                    [-math.sin(current_theta),  math.cos(current_theta),     0],
                                    [0                       ,             0           ,     1]]),
                                    error_world)

        v=math.sqrt(error_bot[0]**2+error_bot[1]**2)*self.kv
        ws=(error_bot[1])*self.kl+(error_bot[2])*self.ks
        print("v",v)
        print("ws",ws)
                
        return v, ws

    def update(self, v, ws, dt):

        self.phi = self.last_phi + (ws + self.last_ws) * dt
        
        if self.phi > np.pi/6:
            self.phi = np.pi/6
        if self.phi < -np.pi/6:
            self.phi = -np.pi/6
        else:
            self.phi = self.phi

        self.theta = self.last_theta + (np.tan(self.phi)*v/self.length + np.tan(self.phi)*self.last_v/self.length) * dt
        
        self.dx = v * np.cos(self.theta)
        self.x = self.last_x + (self.dx)* dt
        
        self.dy = v * np.sin(self.theta)
        self.y = self.last_y + (self.dy)* dt

        self.trajectory_x.append(self.x)
        self.trajectory_y.append(self.y)
        self.trajectory_theta.append(self.theta)
        self.dxvector.append(self.dx)
        self.dyvector.append(self.dy)

        self.last_x = self.x 
        self.last_y = self.y
        self.last_theta = self.theta
        self.last_phi = self.phi
        self.last_v = v
        self.last_ws = ws

   
    def plot(self, color):
        plt.plot(self.trajectory_x, self.trajectory_y, color)
        # for i in range(len(self.trajectory_x)-1):
        #     plt.arrow(self.trajectory_x[i], self.trajectory_y[i], self.dxvector[i],
        #               self.dyvector[i], head_width=0.1, head_length=0.5, color='red')
        plt.savefig('trajectory.png')


def main():
    #Comes from the path planning
    x = [ 0, 10, 20, 30, 40, 50, 60, 70, 80]
    # x= [66.59041972,  66.50485799,  66.41302225,  66.31634621,  66.2162636,
    #     66.11420815,  66.01161721,  65.90997289,  65.81078768 , 65.71419313,
    #     65.61432803,  65.50386382,  65.38791225,  65.29282671 , 65.24164645,
    #     65.18326569 , 65.01266429,  64.69453123,  64.52729269 , 64.90503664,
    #     65.98105261  ,67.46616943,  69.03548474,  70.54375877,  71.98603654,
    #     73.38033227,  74.84292575,  76.52024909,  78.41227259,  80.22915485,
    #     81.66699622 , 82.76319507,  83.84122667,  85.17480772 , 86.71273411,
    #     88.2921625  , 89.80807697,  91.27899027,  92.73845883 , 94.20437821,
    #     95.68055545 , 97.16920768,  98.66599847, 100.16415636, 101.65874561,
    #     103.14906226, 104.6350823,  106.11866717, 107.60349841 ,109.09259374,
    #     110.58413362, 112.07435663, 113.56179241, 115.05067264 ,116.54577796,
    #     118.04401746, 119.53414617, 121.00820408, 122.48437837, 123.99217437,
    #     125.5314534 , 127.0224169,  128.37675089, 129.63813327 ,130.99696528,
    #     132.59670649, 134.15556572, 135.1925149,  135.43443772, 135.21486224,
    #     134.96885538, 134.87674273, 134.81493618, 134.65851405, 134.42273864,
    #     134.19370945, 134.02541905, 133.86928876, 133.65899029, 133.41624649,
    #     133.27554711, 133.35580741, 133.56208221, 133.68304056 ,133.57319887,
    #     133.3151135,  133.0386923,  132.82457827, 132.68566347, 132.62342406,
    #     132.57296907, 132.43053982, 132.13510345, 131.78504352, 131.51401785,
    #     131.37767642, 131.31642495, 131.26138971, 131.143697   ,130.89447309]
    y = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    # y= [139.18255108, 137.92466149, 136.56461692, 135.12922928, 133.64531049,
    #     132.13967246, 130.6376468,  129.14618053, 127.65977634, 126.17298581,
    #     124.68159315, 123.18182684, 121.67862333, 120.19178775, 118.73847912,
    #     117.27946933, 115.73452849, 114.07674701, 112.53446471, 111.40941368,
    #     110.84192658, 110.67576656, 110.72556673, 110.84718449, 110.92866623,
    #     110.88133474, 110.7310032 , 110.53974441, 110.34519132, 110.13659024,
    #     109.89908266, 109.64479198, 109.40845788, 109.2214679 , 109.09274704,
    #     109.02332191, 108.98680758, 108.89838466, 108.67248572, 108.35289981,
    #     108.09978462, 108.04197616, 108.09459149, 108.09346528, 107.96571871,
    #     107.84873809, 107.90143007, 108.11135852, 108.30067989, 108.31263158,
    #     108.17282975, 107.98028376, 107.81280807, 107.69508993, 107.64163339,
    #     107.61597588, 107.52885943, 107.31374244, 107.0837113 , 107.02973625,
    #     107.21527068, 107.36078207, 107.13925641, 106.48855539, 105.64097885,
    #     104.82435475, 104.11193701, 103.50443999, 102.95037529, 102.24798168,
    #     101.17476084,  99.71413631,  98.09520238,  96.53881006,  95.07316563,
    #     93.62911799,  92.15581914,  90.6622755 ,  89.16890647,  87.6845004,
    #     86.2029497 ,  84.71790116,  83.22979187,  81.74275445,  80.25976004,
    #     78.77970154,  77.30058612,  75.81957798,  74.33268209,  72.83686378,
    #     71.33977185,  69.85531185,  68.39158413,  66.93498345,  65.46710292,
    #     63.980219  ,  62.48239109,  60.98294944,  59.49122431,  58.01654593]

    plt.plot(x, y, 'y')

    #some parameters
    ratio = 100
    primary_time_per_iteration = 1
    dt = primary_time_per_iteration/ratio
    ws = 0
    velocity = 0
    kv=1.5/primary_time_per_iteration
    ks=40
    kl= 1

    #interpolate the path with more points
    x_aux=[0]*((len(x)-1)*(ratio))
    y_aux=[0]*((len(y)-1)*(ratio))
    theta_aux=0
    for i in range(len(x)-1):
        for j in range(ratio):
            x_aux[i*ratio+j] = x[i]+(x[i+1]-x[i])*j/ratio
            y_aux[i*ratio+j] = y[i]+(y[i+1]-y[i])*j/ratio
    x_aux[-1] = x[-1]
    y_aux[-1] = y[-1]

    #print("x_aux", x_aux)                    v       ws
    #print("y_aux", y_aux)                    ex  etheta ey
    #   (self, x,    y,   theta, phi, length, kv  ,   ks    , kl  ,   dt, v,       ws):
    #Car initialization with what the controller needs to do
    Auto = car(x[0], y[0]+1, math.atan2(y_aux[1]-y[0], x_aux[1]-x[0])    , 0,  2.46,  kv ,  ks , kl  ,   dt, velocity, ws)
    SSE_control=0
    print("Updating....")
    for i in range(len(x_aux)-1):
        #theta_aux is the angle we want the car to face in the next step 
        theta_aux = math.atan2(y_aux[i+1]-Auto.y, x_aux[i+1]-Auto.x)
        #output of the controller
        velocity, ws = Auto.controlled(Auto.x, Auto.y, Auto.theta, x_aux[i], y_aux[i], theta_aux)
        #update the car position either with a model or with the real car and the sensors
        Auto.update(velocity, ws, dt)
        #actualize the values of the last step
        Auto.last_v=velocity
        Auto.last_ws=ws
        #error measure for log purposes
        SSE_control += math.sqrt((Auto.x-x_aux[i])**2+(Auto.y-y_aux[i])**2)

    print("SSE_control",SSE_control)
    print("MSE_control", SSE_control/len(x_aux))

    print("Plotting....")
    Auto.plot('b')

if __name__ == '__main__':
    main()
