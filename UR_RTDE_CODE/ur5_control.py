#!/usr/bin/env python3

# run this script, please:
# use python3 and install pip3 by sudo apt install python3-pip
# upgrade pip3: pip3 install --upgrade pip
# install ur_rtde: pip3 install ur_rtde

# if simulation mode, run: sudo bash /home/wangzi/URFiles/ursim-3.15.3.106223/start-ursim.sh 

from rtde_io import RTDEIOInterface as RTDEIO
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import RTDEControlInterface as RTDEControl
import time
import struct

UR_IP_ADDRESS = "10.113.130.25"

class state_jointctl():
    def __init__(self):
        self.ur_jointctl = RTDEControl(UR_IP_ADDRESS)

    def set_TargetJoint(self, target_joint_q, is_Noblocked):
        return self.ur_jointctl.moveJ(target_joint_q, 1.05, 1.4, is_Noblocked)

    def set_TargetTCP(self, target_pose, is_Noblocked):
        return self.ur_jointctl.moveJ(target_pose, 0.25, 0.5, is_Noblocked)

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
    
    index_loop = 0
    index_loop_max = 4

    Analog_percent = 0.0
    Analog_percent_max = 0.5

    joint_array_2 = [0.0,  0.010, 0.0,  -0.010]
    joint_array_3 = [0.010, 0.0, -0.010,  0.0]

    Pin_array_0 = [False,  True, False,  True]
    Pin_array_1 = [False,  True, False,  True]
    Pin_array_2 = [False,  True, False,  True]
    Pin_array_3 = [True, False,  True,  False]

    RobotStateCtl = True
    while(RobotStateCtl):
        index_loop += 1
        if index_loop >= index_loop_max:
            index_loop = 0

        Analog_percent += 0.10
        if Analog_percent >= Analog_percent_max:
            Analog_percent = 0

        # get joint space pos
        JointPos = urReceive.get_ActuraljointQuan()
        print('current JointPos:', JointPos)
        JointPos[1] += joint_array_2[index_loop]
        JointPos[2] += joint_array_3[index_loop]
        print('target JointPos:', JointPos)
        # set joint space pos
        urJointctl.set_TargetJoint(JointPos, False)

        # output output state to monite pin state
        outputPinState = urReceive.get_outputPinsState()
        print('outputPinState: ', outputPinState)
        # set Analog_percent output
        # print(Analog_percent)
        # urIoctl.set_AnalogVotageOutput(0, 0)
        # urIoctl.set_AnalogVotageOutput(1, Analog_percent)
        
        urIoctl.set_DigitalOutput(0, Pin_array_0[index_loop])
        urIoctl.set_DigitalOutput(1, Pin_array_1[index_loop])
        # urIoctl.set_DigitalOutput(0, Pin_array_2[index_loop])
        # urIoctl.set_DigitalOutput(1, Pin_array_3[index_loop])
        input("Press Enter to continue...")
        # time.sleep(5)


if __name__ == "__main__":
    main()




