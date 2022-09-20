#!/usr/bin/env python3

# run this script, please:
# use python3 and install pip3 by sudo apt install python3-pip
# upgrade pip3: pip3 install --upgrade pip
# install ur_rtde: pip3 install ur_rtde

# or make install the rtde with c++ pyline using build method:
# https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html#build

## ! go to ur panel to start remote control in setting

# if simulation mode, run: sudo bash /home/wangzi/URFiles/ursim-3.15.3.106223/start-ursim.sh 

from rtde_io import RTDEIOInterface as RTDEIO
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import RTDEControlInterface as RTDEControl
import time
import struct
import numpy as np
from copy import deepcopy


UR_IP_ADDRESS = "127.0.0.1"


class state_jointctl():
    def __init__(self):
        self.ur_jointctl = RTDEControl(UR_IP_ADDRESS)

    def set_TargetJoint(self, target_joint_q, is_Noblocked):
        return self.ur_jointctl.moveJ(target_joint_q, 1.05, 1.4, is_Noblocked)

    def set_TargetTCP(self, target_pose, is_Noblocked):
        return self.ur_jointctl.moveL(target_pose, 1.25, 1.5, is_Noblocked)

class state_ioctl():
    def __init__(self):
        self.ur_ioctl = RTDEIO(UR_IP_ADDRESS)

    def set_DigitalOutput(self, pin_id, pin_level):
        return self.ur_ioctl.setStandardDigitalOut(pin_id, pin_level)

    def set_ConfigurableDigitalOutput(self, pin_id, pin_level):
        return self.ur_ioctl.setConfigurableDigitalOut(pin_id, pin_level)

    def set_ToolDigitalOutput(self, pin_id, pin_level):
        return self.ur_ioctl.setToolDigitalOut(pin_id, pin_level)

    def set_AnalogVotageOutput(self, pin_id, pin_level_percent):
        return self.ur_ioctl.setAnalogOutputVoltage(pin_id, pin_level_percent)

    def set_AnalogCurrentOutput(self, pin_id, pin_level_percent):
        return self.ur_ioctl.setAnalogOutputCurrent(pin_id, pin_level_percent)

class state_receive():
    def __init__(self):
        self.ur_receive = RTDEReceive(UR_IP_ADDRESS)

    def get_ActuralTCPPose(self):
        # Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz),
        # where rx, ry and rz is a rotation vector representation of 
        # the tool orientation
        return self.ur_receive.getActualTCPPose()

    def get_ActuraljointQuan(self):
        # Actual joint positions
        return self.ur_receive.getActualQ()

    def get_inputPinsState(self): 
        # Current state of the digital inputs. 0-7: Standard, 8-15: Configurable, 16-17: Tool
        bits = self.ur_receive.getActualDigitalInputBits()
        return {'con_input': bits[0:8], 'dig_input': bits[8:16], 'too_input': bits[16:18]}

    def get_outputPinsState(self): 
        # Current state of the digital inputs. 0-7: Standard, 8-15: Configurable, 16-17: Tool
        bits = self.ur_receive.getActualDigitalOutputBits()
        str_byte = "{:018b}".format(bits)
        return str_byte
        # return {'dig_inout': bits, 'con_input': bits[8:16], 'too_input': bits[16:18]}


def main():
    print("python script for ur5 is working now...")

    urReceive = state_receive()
    urJointctl = state_jointctl()
    urIoctl = state_ioctl()
    
    angles = np.linspace(0, 2*np.pi, 36, endpoint=False)
    Pose_initial = urReceive.get_ActuralTCPPose()
    Pose_initial = [0.162, 0.527, 0.515, -0.264, 2.778, 0.0184]

    for ang in angles:
        ## get tcp pose
        print('current Pose_ur:', urReceive.get_ActuralTCPPose())
        
        scale_rad = 0.10
        Pose_ur = deepcopy(Pose_initial)
        Pose_ur[1] += scale_rad * np.sin(ang)
        Pose_ur[2] += scale_rad * np.cos(ang)
        urJointctl.set_TargetTCP(Pose_ur, True)
        time.sleep(0.2)


if __name__ == "__main__":
    main()