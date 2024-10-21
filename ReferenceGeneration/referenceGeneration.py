import numpy as np
import matplotlib.pyplot as plt
import os

# Parameters
L1 = 160  #mm?
L2 = 75   #mm?
time_taken = 5 #seconds
time_step = 2000 #micro seconds
time_step = time_step * 1e-6 # Convert to seconds

# CONSTS
encoder_steps_per_revolution = 131.25 * 16  
counts_per_degree = encoder_steps_per_revolution / 360  

def forwardKinematics(th1,th2,L1,L2):
    x = L1 * np.cos(np.rad2deg(th1)) + L2 * np.cos(np.rad2deg(th1+th2))
    y = L1 * np.sin(np.rad2deg(th1)) + L2 * np.sin(np.rad2deg(th1+th2))
    
    return x,y

#Circle Case
x_centre = L1/2
y_centre = L1+L2/2
x_centre,y_centre = forwardKinematics(45,45,L1,L2) # Using forward kinematics to define centre position
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

def getNumberofTimeSteps(time_taken,time_step):
    return int(np.ceil(time_taken/time_step))

def circleGeneration2DoF(L1,L2,circle_radius,time_taken,time_step):
    theta1 = np.array([])
    theta2 = np.array([])
    motor_counts1 = np.array([])
    motor_counts2 = np.array([])

    N = int(np.ceil(time_taken/time_step))
    angles = np.linspace(0, 2 * np.pi, N)
    for angle in angles:
        x = x_centre + circle_radius * np.cos(angle)
        y = x_centre + circle_radius * np.sin(angle)
        th1, th2 = inverseKinematics(x,y,L1,L2)
        e1 = convert_degrees_to_encoder_counts(np.rad2deg(th1))
        e2 = convert_degrees_to_encoder_counts(np.rad2deg(th2))
        
        theta1 = np.append(theta1,th1)
        theta2 = np.append(theta2,th2)

        motor_counts1 = np.append(motor_counts1,e1)
        motor_counts2 = np.append(motor_counts2,e2)


    return motor_counts1, motor_counts2
def wait_x_seconds_generation(time_to_wait,motor_counts): # Cretes an array that allows the motor to wait at it's current position
    if motor_counts.size == 0:
        final_value = 0
    else: 
        final_value = motor_counts[-1]
    N = getNumberofTimeSteps(time_to_wait,time_step)
    wait_array = np.full(N,final_value)
    motor_counts = np.append(motor_counts,wait_array)
    return motor_counts

def circleGeneration1DoF(motor_counts,sweep_angle=360,time_taken=3,time_step=time_step): # Sweep angle is how much of the circle you want to draw
    N = getNumberofTimeSteps(time_taken,time_step)
    if (motor_counts.size == 0):
        start_position = 0
    else:
        start_position = motor_counts[-1]
    positions = np.linspace(start_position,start_position+(sweep_angle*counts_per_degree),N)
    positions = np.ceil(positions).astype(int)
    motor_counts = np.append(motor_counts,positions)
    return motor_counts

# TODO simulate using the motor counts info to recreate circle

#File saving
def save_file(motor_counts,relative_path):
    dirname = os.path.dirname(__file__)
    path = os.path.join(dirname,relative_path)
    
    motor_counts = motor_counts.astype(int) # Making sure integers before saving
    motor_counts = ', '.join(map(str, motor_counts))
    with open(path, 'w') as f:
        f.write('{')
        f.write(motor_counts)
        f.write('}')

def get_time_array(motor_counts):
    time_length = len(motor_counts)
    time = np.arange(0,time_length*time_step,time_step)

    return time

def plot_motor_position(time,motor_counts1,motor_counts2):
    fig, ax1 = plt.subplots()
    
    ax1.plot(time, motor_counts1, label='Motor1')
    ax1.plot(time,motor_counts2,label='Motor2')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Encoder Counts')

    ax1.legend()

    plt.show()


path1 = r'circle/motor11DoFGeneration.txt'
path2 = r'circle/motor21DoFGeneration.txt'

# Generation

# Circle
'''
motor_counts1, motor_counts2 = circleGeneration2DoF(L1,L2,circle_radius,time_taken,time_step)
motor_counts1 = wait_x_seconds_generation(3,motor_counts1)
motor_counts2 = wait_x_seconds_generation(3,motor_counts2)
motor_counts1, motor_counts2 = circleGeneration2DoF(L1,L2,circle_radius,time_taken,time_step)
'''
# 1DoF Circle
# Wait 2 seconds then Motor 1 circle for 6 seconds, wait 3 seconds motor2 circles for 4 seconds, wait 2 seconds, then both circles for 5 seconds

motor_counts1 = np.array([])
motor_counts2 = np.array([])
motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
motor_counts2 = wait_x_seconds_generation(2,motor_counts2)

motor_counts1 = circleGeneration1DoF(motor_counts1,time_taken=6)
motor_counts2 = wait_x_seconds_generation(6,motor_counts2)

motor_counts1 = wait_x_seconds_generation(3,motor_counts1)
motor_counts2 = wait_x_seconds_generation(3,motor_counts2)

motor_counts1 = wait_x_seconds_generation(4,motor_counts1)
motor_counts2 = circleGeneration1DoF(motor_counts2,time_taken=4)

motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
motor_counts2 = wait_x_seconds_generation(2,motor_counts2)

motor_counts1 = circleGeneration1DoF(motor_counts1,time_taken=5)
motor_counts2 = circleGeneration1DoF(motor_counts2,time_taken=5)







time = get_time_array(motor_counts1)
print(motor_counts1.size,motor_counts2.size)
plot_motor_position(time,motor_counts1,motor_counts2) # Need to close plot window to end script, weird behaviour in VSCode

save_file(motor_counts1,path1)
save_file(motor_counts2,path2)

# Future: Have a generation function that takes a shape as an input and auto saves it to the correct file, also include length of the array