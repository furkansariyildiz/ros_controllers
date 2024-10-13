#!/usr/bin/env python3


import matplotlib.pyplot as plt
import csv



class Plotter:
    def __init__(self):
        self.x_result_ = []
        self.y_result_ = []
        self.x_desired_ = []
        self.y_desired_ = []


    
    def read_csv(self, result_path_filename, desired_path_filename):
        with open(result_path_filename, 'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            next(plots)
            for row in plots:
                print("row[0]: ", row[0])
                print("row[1]: ", row[1])
                self.x_result_.append(float(row[0]))  
                self.y_result_.append(float(row[1]))

        with open(desired_path_filename, 'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            next(plots)
            for row in plots:
                print("row[0]: ", row[0])
                print("row[1]: ", row[1])
                self.x_desired_.append(float(row[0]))  
                self.y_desired_.append(float(row[1]))

        # Plot result x and y positions
        plt.plot(self.x_result_, self.y_result_, label='Result Path', color='blue')

        # Plot desired x and y positions
        plt.plot(self.x_desired_, self.y_desired_, label='Desired Path', color='red')

        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.title('Desired and Result Position')
        plt.legend()
        plt.grid(True)
        plt.show()




if __name__ == '__main__':
    plotter = Plotter()
    plotter.read_csv('/home/furkan/controller_ws/src/ros_controllers/results/result_pure_pursuit.csv', '/home/furkan/controller_ws/src/ros_controllers/results/desired_pure_pursuit.csv')

