#!/usr/bin/env python3


import matplotlib.pyplot as plt
import csv



class Plotter:
    def __init__(self):
        self.x_result_ = []
        self.y_result_ = []
        self.x_desired_ = []
        self.y_desired_ = []
        self.error_ = []
        self.time_ = []


    
    def read_csv(self, result_path_filename, desired_path_filename, error_filename):
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

        with open(error_filename, 'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            next(plots)
            for index, row in enumerate(plots):
                print("row[0]: ", row[0])
                self.error_.append(float(row[0]))
                self.time_.append(float(index))

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

        plt.plot(self.time_, self.error_, label='Error', color='blue')
        plt.xlabel('Time')
        plt.ylabel('Error')
        plt.title('Error')
        plt.legend()
        plt.grid(True)
        plt.show()




if __name__ == '__main__':
    plotter = Plotter()
    # plotter.read_csv('/home/furkan/controller_ws/src/ros_controllers/results/result_pid.csv', '/home/furkan/controller_ws/src/ros_controllers/results/desired_pid.csv')
    plotter.read_csv('/home/locint/ros_controllers_ws/src/ros_controllers/results/result_stanley.csv', '/home/locint/ros_controllers_ws/src/ros_controllers/results/desired_stanley.csv', '/home/locint/ros_controllers_ws/src/ros_controllers/results/error_stanley.csv')
    # plotter.read_csv('/home/furkan/controller_ws/src/ros_controllers/results/result_pure_pursuit.csv', '/home/furkan/controller_ws/src/ros_controllers/results/desired_pure_pursuit.csv')

