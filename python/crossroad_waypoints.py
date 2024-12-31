import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.interpolate import interp1d
# from scipy.misc import comb
import bezier
from Bezier import Bezier
from scipy.interpolate import splrep,splev, CubicSpline
from scipy.special import binom
import pathlib
import os
from glob import glob


class CrossroadWaypoints(object):

    def __init__(self):
        self.num_points_per_curve = 100
        self.subset_size = 4     # Calculate the number of subsets

        # self.loadWaypoints('waypoints.txt')
        # self.plotWaypoints()
        # self.loadPath('./path_data/5agents_roadshoulder/')
        # self.plotPath('./path_data/testPresidentCar7AgentsSmall/')
        # self.saveFitPath('./path_data/testPresidentCar7AgentsSmall/fit/')

        # self.plotPath('./path_data/iros_final_paper_demo/test4AgentsSmall_straight2teams/')
        # self.saveFitPath('./path_data/iros_final_paper_demo/test4AgentsSmall_straight2teams/fit/')

        # self.plotPath('./path_data/iros_final_paper_demo/test4AgentsSmall_straight4teams/')
        # self.saveFitPath('./path_data/iros_final_paper_demo/test4AgentsSmall_straight4teams/fit/')

        # self.plotPath('./path_data/iros_final_paper_demo/test8AgentsSmall_turnstraight4teams/')
        # self.saveFitPath('./path_data/iros_final_paper_demo/test8AgentsSmall_turnstraight4teams/fit/')

        # self.plotPath('./path_data/iros_final_paper_demo/testPresidentCar7AgentsSmall/')
        # self.saveFitPath('./path_data/iros_final_paper_demo/testPresidentCar7AgentsSmall/fit/')

        # # moving obstacles
        # self.plotPath('./path_data/test4AgentsSmall_straight2teams_movingobs1/')
        # self.saveFitPath('./path_data/test4AgentsSmall_straight2teams_movingobs1/fit/')

        # self.plotPath('./path_data/test4AgentsSmall_straight2teams_movingobs3/')
        # self.saveFitPath('./path_data/test4AgentsSmall_straight2teams_movingobs3/fit/')

        # self.plotPath('./path_data/testPresidentCar7AgentsSmall_movingobs1/')
        # self.saveFitPath('./path_data/testPresidentCar7AgentsSmall_movingobs1/fit/')

        self.plotPath('./path_data/test8AgentsSmall_turnstraight4teams_movingobs1/')
        self.saveFitPath('./path_data/test8AgentsSmall_turnstraight4teams_movingobs1/fit/')

    def loadWaypoints(self,path):
        if os.path.isfile(path):
            x_list = []
            y_list = []
            file = open(path, "r")
            for eachLine in file:
                x, y = eachLine.split()
                x_list.append(float(x))
                y_list.append(float(y))
                

            wp_x = np.array(x_list)
            wp_y = np.array(y_list)

            self.wp = np.vstack((wp_x,wp_y))
            for i in range(len(self.wp[0])):
                print('[', self.wp[0][i],'', self.wp[1][i], ']')
        else:
            print("Not valid path")

    def plotWaypoints(self):
        fig = plt.figure(figsize=(12,10))
        axs = fig.add_subplot(111) 

        axs.scatter(self.wp[0], self.wp[1],marker='.')
        axs.axis('equal')
        axs.set(xlim=(-100,100))
        plt.show()
        
    
    def loadPath(self,path):
        # allpaths_name = pathlib.Path(path)
        # self.allpaths_list = list(allpaths_name.glob("*.txt"))

        # self.allpaths_list = glob(os.path.join(path, '*.txt'))
        self.allpaths_list = [f.name for f in os.scandir(path) if f.name.endswith('.txt') and f.is_file()]
        # print(self.allpaths_list)

        self.x_list = []
        self.y_list = []
        self.a_list = []
        for i in np.arange(len(self.allpaths_list)):
            # if self.allpaths_list[i] == "costs.txt":
            #     continue
            path_agent = path + self.allpaths_list[i]

            file = open(path_agent)
            x_path = []
            y_path = []
            a_path = []
            for eachLine in file:
                a_old = (float(eachLine.split()[2]) - 2)*45
                break

            file = open(path_agent)
            for eachLine in file:
                x = float(eachLine.split()[0]) * 3.5 - 108.3
                
                y_ori = float(eachLine.split()[1])
                # sign_y = np.sign(y_ori - 17.5)
                # y =  (20.6 - sign_y*4) - (y_ori - 17.5 - sign_y*0.5)* 3.5
                sign_y = np.sign(y_ori - 18)
                y =  (20.6 - sign_y*4) - (y_ori - 18 - sign_y*1)* 3.5


                a = (float(eachLine.split()[2]) - 2)*45
                if abs(a-a_old)>90 and a > a_old:
                    a = a -360
                elif abs(a-a_old)>90 and a < a_old:
                    a = a + 360    
                a_old = a
                x_path.append(x)
                y_path.append(y)
                a_path.append(a)

            x_path = np.array(x_path)
            y_path = np.array(y_path)
            a_path = np.array(a_path)

            self.x_list.append(x_path)
            self.y_list.append(y_path)
            self.a_list.append(a_path)

        # return self.x_list,self.y_list,self.a_list

    def fitPath(self):
        # # p = []
        # # for i in np.arange(len(self.x_list)):
        # #     p.append([self.x_list[i], self.y_list[i]])
        # self.p = np.asfortranarray([self.x_list, self.y_list])
        # print(self.p)

        # curve = bezier.Curve(self.p, degree=len(self.x_list)-1)

        # self.x_fit = []
        # self.y_fit = []
        # t = np.linspace(0, 1, 101)
        # for i in np.arange(len(t)):
            
        #     p_fit = curve.evaluate(float(i))
        #     self.x_fit.append(p_fit[0])
        #     self.y_fit.append(p_fit[1])

        # t = np.arange(len(self.x_list))
        # nodes_tx = np.asfortranarray([t,self.x_list])
        # nodes_ty = np.asfortranarray([t,self.y_list])
        # nodes_ta = np.asfortranarray([t,self.a_list])
        # curve_tx = bezier.Curve(nodes_tx, degree=len(self.x_list)-1)
        # curve_ty = bezier.Curve(nodes_ty, degree=len(self.y_list)-1)
        # curve_ta = bezier.Curve(nodes_ta, degree=len(self.a_list)-1)
        # t_fit = np.linspace(0, 1, 1001)
        # self.x_fit = []
        # self.y_fit = []
        # self.a_fit = []

        # for i in np.arange(len(t_fit)):
        #     tx_fit = curve_tx.evaluate(float(i))
        #     self.x_fit.append(tx_fit[1])
        #     ty_fit = curve_ty.evaluate(float(i))
        #     self.y_fit.append(ty_fit[1])
        #     ta_fit = curve_ta.evaluate(float(i))
        #     self.a_fit.append(ta_fit[1])

        # bazier_xy = Bezier()
        # p_xy = []
        # for i in np.arange(len(self.x_list)):
        #     p_xy.append([self.x_list[i], self.y_list[i]])
        # bazier_xy.points = np.array(p_xy)
        # b_xy = bazier_xy.evaluate_bezier(1000)
        # self.x_fit = b_xy[:,0]
        # self.y_fit = b_xy[:,1]

        # bazier_ta = Bezier()
        # p_ta = []
        # for i in np.arange(len(self.a_list)):
        #     p_ta.append([i, self.a_list[i]])
        # bazier_ta.points = np.array(p_ta)
        # b_ta = bazier_ta.evaluate_bezier(1000)
        # self.a_fit = b_ta[:,1]

        # t = np.arange(len(self.x_list))
        # t_fit = np.linspace(t[0], t[-1], num=1000, endpoint=True)
        # f_tx = interp1d(t, self.x_list, kind='cubic')
        # self.x_fit = f_tx(t_fit)
        # f_ty = interp1d(t, self.y_list, kind='cubic')
        # self.y_fit = f_ty(t_fit)
        # f_ta = interp1d(t, self.a_list, kind='cubic')
        # self.a_fit = f_ta(t_fit)

        # # B- spline fitting
        # self.x_fit = []
        # self.y_fit = []
        # self.a_fit = []

        # path_num = len(self.x_list)        
        # for i in np.arange(path_num):
        #     t = np.arange(len(self.x_list[i]))
        #     t_fit = np.linspace(t[0], t[-1], num=1000, endpoint=True)

        #     f_tx = splrep(t, self.x_list[i])
        #     self.x_fit.append(np.array(splev(t_fit,f_tx)))

        #     f_ty = splrep(t, self.y_list[i])
        #     self.y_fit.append(np.array(splev(t_fit,f_ty)))
            
        #     f_ta = splrep(t, self.a_list[i])
        #     self.a_fit.append(np.array(splev(t_fit,f_ta)))

        # #  Cubic spline
        # self.x_fit = []
        # self.y_fit = []
        # self.a_fit = []

        # path_num = len(self.x_list)        
        # for i in np.arange(path_num):
        #     t = np.arange(len(self.x_list[i]))
        #     t_fit = np.linspace(t[0], t[-1], num=1000, endpoint=True)

        #     f_tx = CubicSpline(t, self.x_list[i],bc_type='natural')
        #     self.x_fit.append(np.array(f_tx(t_fit)))

        #     f_ty = CubicSpline(t, self.y_list[i], bc_type='natural')
        #     self.y_fit.append(np.array(f_ty(t_fit)))
            
        #     f_ta = CubicSpline(t, self.a_list[i], bc_type='natural')
        #     self.a_fit.append(np.array(f_ta(t_fit)))

        # ## Piecewise fitting
        #  Cubic spline
        self.x_fit = []
        self.y_fit = []
        self.a_fit = []

        path_num = len(self.x_list)  # Define the number of data points in each subset
        # self.subset_size = 6      # Calculate the number of subsets
        for i in np.arange(path_num):
            t = np.arange(len(self.x_list[i]))
            num_subsets = len(self.x_list[i]) // (self.subset_size -1)
            
            # Initialize arrays to hold Bezier control points
            bezier_control_points_tx = []
            bezier_control_points_ty = []
            bezier_control_points_ta = []

            # Iterate over each subset and compute Bezier control points
            for j in range(num_subsets):
                if j == num_subsets-1:
                    subset_t = t[j * (self.subset_size-1): len(self.x_list[i])]
                    # print('subset_t', subset_t)

                    subset_x = self.x_list[i][j * (self.subset_size-1): len(self.x_list[i])]
                    subset_y = self.y_list[i][j * (self.subset_size-1): len(self.x_list[i])]
                    subset_a = self.a_list[i][j * (self.subset_size-1): len(self.x_list[i])]
                else:
                    subset_t = t[j * (self.subset_size-1): (j + 1) * (self.subset_size-1)+1]
                    # print('subset_t', subset_t)

                    subset_x = self.x_list[i][j * (self.subset_size-1): (j + 1) * (self.subset_size-1)+1]
                    subset_y = self.y_list[i][j * (self.subset_size-1): (j + 1) * (self.subset_size-1)+1]
                    subset_a = self.a_list[i][j * (self.subset_size-1): (j + 1) * (self.subset_size-1)+1]


                # Fit a Bezier curve through the subset of data points
                control_points_tx = np.column_stack((subset_t, subset_x))
                bezier_control_points_tx.append(control_points_tx)

                control_points_ty = np.column_stack((subset_t, subset_y))
                bezier_control_points_ty.append(control_points_ty)

                control_points_ta = np.column_stack((subset_t, subset_a))
                bezier_control_points_ta.append(control_points_ta)

            # Generate points for visualization
           
            t_vals = np.linspace(0, 1, self.num_points_per_curve)

            # Evaluate and plot the piecewise Bezier curves
            x_fit=np.array([])
            y_fit=np.array([])
            a_fit=np.array([])
            i = 1
            for control_points in bezier_control_points_tx:
                x_fit_subset = np.array([self.mybezier(tt, control_points) for tt in t_vals])[:, 1]
                if i<len(bezier_control_points_tx):
                    x_fit_subset = x_fit_subset[:-1] 
                x_fit = np.append(x_fit,x_fit_subset)
                i +=1
            xxxx = np.array(x_fit).flatten()    
            self.x_fit.append(xxxx)
            
            i = 1
            for control_points in bezier_control_points_ty:
                y_fit_subset = np.array([self.mybezier(tt, control_points) for tt in t_vals])[:, 1]
                if i<len(bezier_control_points_ty):
                    y_fit_subset = y_fit_subset[:-1]
                y_fit= np.append(y_fit, y_fit_subset)
                i +=1
            self.y_fit.append(np.array(y_fit).flatten())

            i = 1
            for control_points in bezier_control_points_ta:
                a_fit_subset = np.array([self.mybezier(tt, control_points) for tt in t_vals])[:, 1]
                if i<len(bezier_control_points_ta):
                    a_fit_subset = a_fit_subset[:-1]
                a_fit = np.append(a_fit,a_fit_subset)
                i +=1
            self.a_fit.append(np.array(a_fit).flatten())



        # # Polynomial fitting
        # t = np.arange(len(self.x_list))
        # t_fit = np.linspace(t[0], t[-1], num=1000, endpoint=True)
        # ploy_tx = np.polyfit(t, self.x_list, 7) 
        # p_tx = np.poly1d(ploy_tx)
        # self.x_fit=p_tx(t_fit)
        # ploy_ty = np.polyfit(t, self.y_list, 7) 
        # p_ty = np.poly1d(ploy_ty)
        # self.y_fit=p_ty(t_fit)
        # ploy_ta = np.polyfit(t, self.a_list, 7) 
        # p_ta = np.poly1d(ploy_ta)
        # self.a_fit=p_ta(t_fit)

    def mybezier(self, t, control_points):
        n = len(control_points) - 1
        return sum([binom(n, i) * (1 - t)**(n - i) * t**i * P for i, P in enumerate(control_points)])



    def saveFitPath(self,path):
        self.fitPath()

        for i in np.arange(len(self.allpaths_list)):
            f = open(path+self.allpaths_list[i], 'w')
            for j in range(len(self.x_fit[i])):
                f.write(str(self.x_fit[i][j]) + '    ' + str(self.y_fit[i][j]) + 
                            '    ' + str(self.a_fit[i][j]) + 
                            '\n')
            f.close()   

        # fig = plt.figure(figsize=(12,10))
        # axs = fig.add_subplot(111) 
        # axs.plot(self.x_fit,self.y_fit)
        # axs.axis('equal')
        # axs.grid('on')
        # plt.show()
        path_num = len(self.x_fit)
        fig, axs = plt.subplots(2, int(np.ceil(path_num/2)))
        for i in np.arange(path_num):
            axs[int(i/np.ceil(path_num/2)),i%int(np.ceil(path_num/2))].plot(self.x_fit[i],self.y_fit[i])
            axs[int(i/np.ceil(path_num/2)),i%int(np.ceil(path_num/2))].axis('equal')
        plt.show()
        


    def plotPath(self,path):
        self.loadPath(path)
        path_num = len(self.x_list)

        # fig = plt.figure(figsize=(12,10))
        # axs = fig.add_subplot(111) 
        # axs.plot(self.x_list,self.y_list)
        # axs.axis('equal')
        # axs.grid('on')
        # plt.show()
        
        fig, axs = plt.subplots(2, int(np.ceil(path_num/2)))
        for i in np.arange(path_num):
            print('agent name', self.allpaths_list[i])
            # print('agent x', self.x_list[i])
            # print('agent y', self.y_list[i])

            axs[int(i/np.ceil(path_num/2)),i%int(np.ceil(path_num/2))].plot(self.x_list[i],self.y_list[i])
            axs[int(i/np.ceil(path_num/2)),i%int(np.ceil(path_num/2))].axis('equal')
            axs[int(i/np.ceil(path_num/2)),i%int(np.ceil(path_num/2))].title.set_text(self.allpaths_list[i])

        plt.show()
        


    
# ----------------------------------------------- #
    def B_nx(self, n, i, x):
        if i > n:
            return 0
        elif i == 0:
            return (1-x)**n
        elif i == 1:
            return n*x*((1-x)**(n-1))
        return self.B_nx(n-1, i, x)*(1-x)+self.B_nx(n-1, i-1, x)*x

    def get_value(self, p, param):
        sumx = 0.
        sumy = 0.
        length = len(p)-1
        for i in range(0, len(p)):
            print('iii',i)
            sumx += (self.B_nx(length, i, param) * p[i][0])
            sumy += (self.B_nx(length, i, param) * p[i][1])
        return sumx, sumy

    def get_bezier(self, p,t):
        xx = [0] * len(t)
        yy = [0] * len(t)
        for i in range(0, len(t)):
            print('x[i]=', t[i])
            a, b = self.get_value(p, t[i])
            xx[i] = a
            yy[i] = b
            print('xx[i]=', xx[i])
        return xx, yy




#        27.8
#        24.4
#        16.4
#        12.8
# 
# -88.6       -52.2  -48.8  -45.3 -41.8

#        27.8  28.1
#        24.4  24.6
#        16.4  16.6
#        12.8  13.1
# 
# -88.6       -52.3  -48.8  -45.3 -41.8
# -66.3                                      -27.8


if __name__ == '__main__':
    try:
        CrossroadWaypoints()
    except KeyboardInterrupt:
        print(' - Exited by user.')

