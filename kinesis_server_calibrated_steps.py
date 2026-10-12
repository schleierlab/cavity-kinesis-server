# -*- coding: utf-8 -*-
"""
Created on Thu Oct 04 17:53:55 2018

@author: Quantum Engineer
"""

import sys
import time
import zprocess
import numpy as np
import matplotlib.pyplot as plt
import os
import clr
from device_server import DeviceServer
import h5py

sys.path.append(r"C:\Program Files\Thorlabs\Kinesis")
clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.InertialMotorCLI")
from Thorlabs.MotionControl.DeviceManagerCLI import * # pyright: ignore
from Thorlabs.MotionControl.KCube.InertialMotorCLI import * # pyright: ignore


class KCubeController:
    def __init__(self, serial_number, num_channels, forward_step_size=100, reverse_scale_factors=[]):
        DeviceManagerCLI.BuildDeviceList() # pyright: ignore
        serial_numbers = DeviceManagerCLI.GetDeviceList() # pyright: ignore
        print(f"Found serial numbers of conneccted controllers: {serial_numbers}")
        self.device = KCubeInertialMotor.CreateDevice(serial_number) # pyright: ignore
        self.device.Connect(serial_number)
        self.device.StartPolling(250)
        device_info = self.device.GetDeviceInfo()
        print(f"Connected to device {device_info.Name} with serial number {serial_number}")
        self.max_move = 3000
        self.channels = 1 + np.arange(num_channels)
        self.forward_step_size = forward_step_size

        if reverse_scale_factors:
            self.reverse_scale_factors = reverse_scale_factors
            assert len(self.reverse_scale_factors) == len(self.channels), "reverse scale factors array has incorrect length"
        else:
            self.reverse_scale_factors = np.ones_like(self.channels)

        # Real positions is the position we calibrate
        time.sleep(1.8)
        self.real_positions = self.get_pico_positions()
        print("Real Position", self.real_positions)


    def get_pico_positions(self):
        self.pico_positions = [self.device.GetPosition(i) for i in self.channels]
        return self.pico_positions
    
    def get_real_positions(self):
        return self.real_positions

    def move_to_positions(self, positions):
        # TODO move in steps
        assert len(positions) == len(self.channels), "position array has incorrect length"
        print(positions)
        for channel, position, current_pos, reverse_factor in zip(self.channels, positions, self.real_positions, self.reverse_scale_factors):
            if position != current_pos:
                if np.abs(position - current_pos) <= self.max_move:
                    while position != current_pos:

                        if position > current_pos:
                            step = min(self.forward_step_size, position - current_pos)
                            real_step = step
                        else:
                            step = max(-self.forward_step_size, position - current_pos) * reverse_factor
                            real_step = step / reverse_factor
                        step = int(step)
                        self.device.MoveBy(channel, step, 2000) # I think the timeout is in milliseconds
                        self.real_positions[channel-1] += real_step
                        current_pos = self.real_positions[channel-1]

                else:
                    raise Exception(f"Requested move from {current_pos} to {position} exceeds maximum move size of {self.max_move}.")
        self.get_pico_positions()
        print("Real positions", self.real_positions)
        print("Pico positions", self.pico_positions)
    def set_maximum_move(self, max_move):
        self.max_move = max_move

class KinesisServer(DeviceServer):
    def __init__(self, variable_names, serial_numbers, port, max_move_names, reverse_scale_factor_list):
        super().__init__(port)
        print("Starting Kinesis Server")
        self.serials = serial_numbers
        self.variables = variable_names
        self.controllers = [KCubeController(i, len(j), reverse_scale_factors=rsfs) for i, j, rsfs in zip(self.serials, self.variables, reverse_scale_factor_list)]
        self.max_move_names = max_move_names
        print("Started Kinesis Server")

    def transition_to_buffered(self, h5_filepath):
        """
        Get positions from h5_filepath
        Get positions from picomotors
        If positions are different, move picomotors
        """
        with h5py.File(h5_filepath) as f:
            desired_positions = [
                self.get_desired_positions(i, f) for i in self.variables
            ]
            max_moves = [f["globals"].attrs[i] for i in self.max_move_names]

        print(f"Desired positions are: {desired_positions}")
        real_positions = [i.real_positions for i in self.controllers]
        print(f"Real positions are: {real_positions}")
        print(f"Maximum allowed moves are: {max_moves}")

        assert len(max_moves) == len(desired_positions), "Maximum move array length must be same length as controller array length"
        for controller, positions, max_move in zip(self.controllers, desired_positions, max_moves):
            print(f"Moving to desired position for controller {controller}")
            controller.set_maximum_move(max_move)
            controller.move_to_positions(positions)
        print("transition to buffered")

    def get_desired_positions(self, variables, hf):
        desired_positions = [hf["globals"].attrs[i] for i in variables]
        return desired_positions

    def transition_to_static(self, h5_filepath):
        """To be overridden by subclasses. Do any post processing after a
        shot, eg computing optical depth, fits, displaying images, saving
        images and results to the h5 file, returning cameras to an idle
        state."""
        #actual_positions = [i.get_actual_positions() for i in self.controllers]
        #print(f"Actual positions: {actual_positions}")
        print("transition to static")

    def abort(self):
        """To be overridden by subclasses. Return cameras and any other state
        to one in which transition_to_buffered() can be called again. abort()
        will be called if there was an exception in either
        transition_to_buffered() or transtition_to_static(), and so should
        ideally be written to return things to a sensible state even if those
        methods did not complete. Like any cleanup function, abort() should
        proceed to further cleanups even if earlier cleanups fail. As such it
        should make liberal use of try: except: blocks, so that an exception
        in performing one cleanup operation does not stop it from proceeding
        to subsequent cleanup operations"""
        print("abort")


serials = ["97100395", "97100364", "97100362"]
global_names = [["Tweezers_Picomotor_1",
                 "Tweezers_Picomotor_2",
                 "Tweezers_Picomotor_3",
                 "Tweezers_Picomotor_4"],
                 ["MOT_CH1", "MOT_CH2", "MOT_CH3", "MOT_CH4"],
                 ["MOT_CH5",
                 "MOT_CH6",
                 "Raman_Picomotor_X",
                 "Raman_Picomotor_Y"]]

reverse_scale_names = [
    ["Tweezers_Picomotor_1_RSF", "Tweezers_Picomotor_2_RSF", "Tweezers_Picomotor_3_RSF", "Tweezers_Picomotor_4_RSF"], 
    ["MOT_CH1_RSF", "MOT_CH2_RSF", "MOT_CH3_RSF", "MOT_CH4_RSF"],
    ["MOT_CH5_RSF", "MOT_CH6_RSF", "Raman_Picomotor_X_RSF", "Raman_Picomotor_Y_RSF"]
]

# Maybe along with global_names should be a dictionary so it's easier to correspond?
reverse_scale_factors=[  ## moving picomotor backwards should have larger steps than forward
    [1, 1, 1, 1],
    [1, 2.0, 1.7, 1.8], 
    [1.1, 2.3, 1, 1], #Calibrated Ch 5
]


max_names = ["Tweezers_Picomotor_Max", "MOT_Picomotor_Max", "MOT_Picomotor_Max"]
port = 7426
assert len(serials) == len(global_names), "List of globals must be the same length as serial numbers"
kserver = KinesisServer(global_names, serials, port, max_names, reverse_scale_factors)
kserver.shutdown_on_interrupt()
