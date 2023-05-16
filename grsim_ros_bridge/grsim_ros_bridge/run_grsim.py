#! /usr/bin/env python3
import os

#TODO: Change the path to the grSim executable
GR_SIM_PATH =  "/home/ubuntu/ros_ws/src/grSim/bin/grSim"

def main(args = None):
    os.system(GR_SIM_PATH)

if __name__ == '__main__':
    main()