import numpy as np
import csv
import os
import matplotlib.pyplot as plt
from scipy import signal


# Takes in output motor speed in counts/s
# motor_speed =  pos - pos_prev/delta_time
# G_P_dot(s) = (lambda/T)/(s_1/T)

# lambda = max(motor_speed)/max(input)
# T = the time at which this is true = 0.632 * steady_state_value

delta_time = 2000 # Micro Seconds
delta_time = 2000/1e6 # Conv to seconds



def store_recorded_data(path):
    pos1,pos2,in1,in2 = [],[],[],[]

    with open(path,'r') as file:
        reader = csv.reader(file)

        for row in reader:
            pos1.append(float(row[0]))  # Store values from the first column
            pos2.append(float(row[1]))  # Store values from the second column
            in1.append(float(row[2]))  # Store values from the third column
            in2.append(float(row[3]))  # Store values from the fourth column
    pos1 = np.array(pos1)
    pos2 = np.array(pos2)
    in1 = np.array(in1)
    in2 = np.array(in2)
    return pos1,pos2,in1,in2




def get_time_arr(arr):
    arrLen = np.size(arr)
    time = np.linspace(0,arrLen * delta_time,arrLen)
    return time

def getMotorSpeedArr(pos):
    motor_speed_arr = np.array([])
    for i,x in enumerate(pos):
        if i == 0:
            motor_speed_arr = np.append(motor_speed_arr,0)
        else:
            
            motor_speed_arr = np.append(motor_speed_arr,(pos[i] - pos[i-1])/delta_time)

    return motor_speed_arr

def getMaxMotorSpeed(motor_speed_arr): 
    return np.max(motor_speed_arr)
    

def getLambda(maxMotorSpeed, maxInput):
    return maxMotorSpeed/maxInput



def getTimeConstantT(steadyStateValue, motor_speed_arr, time_arr):
    threshold = 0.632 * steadyStateValue
    T_index = np.where(motor_speed_arr >= threshold)[0][0]
    T = time_arr[T_index]
    return T


def getIdealResponse(Lambda,T,inputArr,delta_time=delta_time):
    numerator = [Lambda/T]
    denominator = [1,1/T]
    system = signal.TransferFunction(numerator,denominator,dt=delta_time)
    tout, output = signal.dlsim(system,inputArr)
    return output

def getControlConsants(T,Lambda,Ts): # Ts Desired closed-loop settling time, T settling time
    w_cl = 7.55/Ts
    kp = (3*w_cl**2 * T)/Lambda
    ki = (w_cl**3 * T)/Lambda
    kd = (3 * w_cl * T-1)/(Lambda)

    return kp, ki, kd
file_name = "recorded_data.csv"
dirname = os.path.dirname(__file__)
path = os.path.join(dirname,file_name)


pos1,pos2,in1,in2 = store_recorded_data(path)
time = get_time_arr(pos1)
# An error is recording before 1000 means I have to do this
time = time[1000:]
pos1 = pos1[1000:]
in1 = in1[1000:]
motor_speed_arr1 = getMotorSpeedArr(pos1)
max_motor_speed1 = getMaxMotorSpeed(motor_speed_arr1)
maxInput1 = getMaxMotorSpeed(in1)
Lambda1 = getLambda(max_motor_speed1,maxInput1)
# Lambda = 22.5
T1 = getTimeConstantT(max_motor_speed1,motor_speed_arr1,time)
print(T1)
idealMotorSpeed1 = getIdealResponse(Lambda1,T1,in1)
# T = 0.29
kp1,ki1,kd1 = getControlConsants(T1,Lambda1,Ts=0.35)
print(f'{kp1 = }, {ki1 = }, {kd1 = }')


pos2 = pos2[1000:]
in2 = in2[1000:]
motor_speed_arr2 = getMotorSpeedArr(pos2)
max_motor_speed2 = getMaxMotorSpeed(motor_speed_arr2)
maxInput2 = getMaxMotorSpeed(in2)
Lambda2 = getLambda(max_motor_speed2,maxInput2)
# Lambda = 22.5
T2 = getTimeConstantT(max_motor_speed2,motor_speed_arr2,time)
idealMotorSpeed2 = getIdealResponse(Lambda2,T2,in2)

# T = 0.29
kp2,ki2,kd2 = getControlConsants(T2,Lambda2,Ts=0.35)
print(f'{kp2 = }, {ki2 = }, {kd2 = }')

# plt.plot(time,motor_speed_arr1)
# plt.show()
fig,(ax1,ax2,ax3) = plt.subplots(3,1)
ax1.plot(time,motor_speed_arr1, '-',label = "recorded" )
ax1.plot(time,idealMotorSpeed1, '-',label = 'ideal')
ax1.legend()

ax2.plot(time,motor_speed_arr2)
ax2.plot(time,idealMotorSpeed2)

ax3.plot(time,in1)



plt.show()