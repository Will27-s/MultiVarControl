import numpy as np



# Takes in output motor speed in counts/s
# motor_speed =  pos - pos_prev/delta_time
# G_P_dot(s) = (lambda/T)/(s_1/T)

# lambda = max(motor_speed)/max(input)
# T = the time at which this is true = 0.632 * steady_state_value

delta_time = 2000 # Micro Seconds
delta_time = 2000/1e6 # Conv to seconds


pos = np.array([]) # Position data
pos = np.random.random(300)


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
            print(i)
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


def getControlConsants(T,Lambda,Ts): # Ts Desired closed-loop settling time, T settling time
    w_cl = 7.55/Ts
    kp = (3*w_cl**2 * T)/Lambda
    ki = (w_cl**3 * T)/Lambda
    kd = (3 * w_cl * T-1)/(Lambda)

    return kp, ki, kd


time = get_time_arr(pos)
motor_speed_arr = getMotorSpeedArr(pos)
max_motor_speed = getMaxMotorSpeed(motor_speed_arr)
maxInput = 18
Lambda = getLambda(max_motor_speed,maxInput)
Lambda = 22.5
T = getTimeConstantT(max_motor_speed,motor_speed_arr,time)
T = 0.29
kp,ki,kd = getControlConsants(T,Lambda,Ts=0.35)
print('kp: ',kp,'ki: ',ki,'kd: ',kd)