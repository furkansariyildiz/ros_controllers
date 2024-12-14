#!/usr/bin/env python3


import matplotlib.pyplot as plt
import csv



class Plotter:
    def __init__(self):
        self.x_result_ = []
        self.y_result_ = []
        self.x_desired_ = []
        self.y_desired_ = []
        self.continuous_error_ = []
        self.continuous_time_ = []
        self.discrete_error_ = []
        self.discrete_time_ = []



    
    def read_csv(self, result_path_filename, desired_path_filename, continuous_error_filename, discrete_error_filename=None):
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

        with open(continuous_error_filename, 'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            next(plots)
            for index, row in enumerate(plots):
                print("row[0]: ", row[0])
                self.continuous_error_.append(float(row[0]))
                self.continuous_time_.append(float(index))

        with open(discrete_error_filename, 'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            next(plots)
            for index, row in enumerate(plots):
                print("row[0]: ", row[0])
                self.discrete_error_.append(float(row[0]))
                self.discrete_time_.append(float(index))

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

        # Plot continuous error
        plt.plot(self.continuous_time_, self.continuous_error_, label='Continuous Error', color='blue')
        plt.xlabel('Time')
        plt.ylabel('Error')
        plt.title('Continuous Error')
        plt.legend()
        plt.grid(True)
        plt.show()

        # Plot discrete error
        plt.plot(self.discrete_time_, self.discrete_error_, label='Discrete Error', color='blue')
        plt.xlabel('Time')
        plt.ylabel('Error')
        plt.title('Discrete Error')
        plt.legend()
        plt.grid(True)
        plt.show()



if __name__ == '__main__':
    plotter = Plotter()
    # plotter.read_csv('/home/furkan/controller_ws/src/ros_controllers/results/result_pid.csv', '/home/furkan/controller_ws/src/ros_controllers/results/desired_pid.csv', '/home/furkan/controller_ws/src/ros_controllers/results/continuous_error_pid.csv', '/home/furkan/controller_ws/src/ros_controllers/results/discrete_error_pid.csv')
    plotter.read_csv('/home/furkan/controller_ws/src/ros_controllers/results/result_stanley.csv', '/home/furkan/controller_ws/src/ros_controllers/results/desired_stanley.csv', '/home/furkan/controller_ws/src/ros_controllers/results/continuous_error_stanley.csv', '/home/furkan/controller_ws/src/ros_controllers/results/discrete_error_stanley.csv')
    # plotter.read_csv('/home/furkan/controller_ws/src/ros_controllers/results/result_pure_pursuit.csv', '/home/furkan/controller_ws/src/ros_controllers/results/desired_pure_pursuit.csv', '/home/furkan/controller_ws/src/ros_controllers/results/continuous_error_pure_pursuit.csv', '/home/furkan/controller_ws/src/ros_controllers/results/discrete_error_pure_pursuit.csv')
    # plotter.read_csv('/home/furkan/controller_ws/src/ros_controllers/results/result_mpc.csv', '/home/furkan/controller_ws/src/ros_controllers/results/desired_mpc.csv', '/home/furkan/controller_ws/src/ros_controllers/results/continuous_error_mpc.csv', '/home/furkan/controller_ws/src/ros_controllers/results/discrete_error_mpc.csv')
