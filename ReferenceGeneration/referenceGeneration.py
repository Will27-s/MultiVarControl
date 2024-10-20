import numpy as np
import matplotlib.pyplot as plt
import os

# Parameters
L1 = 160  #mm?
L2 = 75   #mm?
time_taken = 1 #seconds
time_step = 2000 #micro seconds
time_step = 2000 * 1e-6 # Convert to seconds

# CONSTS
encoder_steps_per_revolution = 131.25 * 16  
counts_per_degree = encoder_steps_per_revolution / 360  

def forwardKinematics(th1,th2,L1,L2):
    x = L1 * np.cos(np.rad2deg(th1)) + L2 * np.cos(np.rad2deg(th1+th2))
    y = L1 * np.sin(np.rad2deg(th1)) + L2 * np.sin(np.rad2deg(th1+th2))
    
    return x,y

#Circle Case
xc = L1/2
yc = L1+L2/2
xc,yc = forwardKinematics(45,45,L1,L2) # Using forward kinematics to define centre position
circle_radius = 49    



def inverseKinematics(x,y,L1,L2):
    costh2 = (x**2 + y**2-L1**2-L2**2)/(2 * L1 * L2)
    sinth2 = np.sqrt(1-costh2**2)
    theta2 = np.arctan2(sinth2,costh2)
    
    A = x
    B = L1 + (L2 * costh2)
    C = -L2 * sinth2

    w = (C-np.sqrt(C**2 + B**2 - A**2))/(A+B)
    theta1 = 2 * np.arctan(w)
    return theta1,theta2

def convert_degrees_to_encoder_counts(theta):
    return np.ceil(theta * counts_per_degree)

def circleGeneration(L1,L2,circle_radius,time_taken,time_step):
    theta1 = np.array([])
    theta2 = np.array([])
    motor_counts1 = np.array([])
    motor_counts2 = np.array([])

    N = int(np.ceil(time_taken/time_step))
    angles = np.linspace(0, 2 * np.pi, N)
    for angle in angles:
        x = xc + circle_radius * np.cos(angle)
        y = xc + circle_radius * np.sin(angle)
        th1, th2 = inverseKinematics(x,y,L1,L2)
        e1 = convert_degrees_to_encoder_counts(np.rad2deg(th1))
        e2 = convert_degrees_to_encoder_counts(np.rad2deg(th2))
        
        theta1 = np.append(theta1,th1)
        theta2 = np.append(theta2,th2)

        motor_counts1 = np.append(motor_counts1,e1)
        motor_counts2 = np.append(motor_counts2,e2)


    return motor_counts1, motor_counts2

# TODO simulate using the motor counts info to recreate circle

#File saving
def save_file(motor_counts,relative_path):
    dirname = os.path.dirname(__file__)
    path = os.path.join(dirname,relative_path)
    if not os.path.exists(path):
        os.mkdir(path)

    motor_counts = ', '.join(map(str, motor_counts))
    with open(path, 'w') as f:
        f.write(motor_counts)


def plot_motor_position(motor_counts1,motor_counts2):
    time_length = len(motor_counts1)
    fig, ax1 = plt.subplots()
    time = np.arange(0,time_length*time_step,time_step)
    ax1.plot(time, motor_counts1,'.-',markersize=2)
    ax1.plot(time,motor_counts2)
    ax1.set_xlabel('Time(s)')
    ax1.set_ylabel('Encoder Counts')
    plt.show()

path1 = './circle/motor1.txt'
path2 = './circle/motor2.txt'
motor_counts1, motor_counts2 = circleGeneration(L1,L2,circle_radius,time_taken,time_step)
plot_motor_position(motor_counts1,motor_counts2)
save_file(motor_counts1,path1)
save_file(motor_counts2,path2)

# Future: Have a generation function that takes a shape as an input and auto saves it to the correct file