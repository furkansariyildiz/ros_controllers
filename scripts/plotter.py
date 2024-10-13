#!/usr/bin/env python3


import matplotlib.pyplot as plt
import csv



class Plotter:
    def __init__(self):
        self.time = []
        self.position = []
        self.velocity = []
        self.effort = []


    
    def read_csv(self, file_path):
        with open(file_path, 'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            for row in plots:
                self.time.append(float(row[0]))
                self.position.append(float(row[1]))
                self.velocity.append(float(row[2]))
                self.effort.append(float(row[3]))

        plt.plot(self.time, self.position, label='position')
        plt.xlabel('time')
        plt.ylabel('position')
        plt.title('Position vs Time')
        plt.show()




if __name__ == '__main__':
    plotter = Plotter()
    plotter.read_csv('joint_states.csv')

