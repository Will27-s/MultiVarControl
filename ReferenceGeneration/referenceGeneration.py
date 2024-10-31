import numpy as np
import matplotlib.pyplot as plt
import os

# Parameters
L1 = 126  #mm?
L2 = 145   #mm?

time_step = 2000 #micro seconds
time_step = time_step * 1e-6 # Convert to seconds

# CONSTS
encoder_steps_per_revolution = 131.25 * 16  
counts_per_degree = encoder_steps_per_revolution / 360  

def forwardKinematics(th1,th2,L1,L2):
    x = L1 * np.cos(np.deg2rad(th1)) + L2 * np.cos(np.deg2rad(th1+th2))
    y = L1 * np.sin(np.deg2rad(th1)) + L2 * np.sin(np.deg2rad(th1+th2))
    
    return x,y

#Circle Case
time_taken = 10 #seconds
x_centre = L1/2
y_centre = L1+L2/2
x_centre,y_centre = forwardKinematics(110,10,L1,L2) # Using forward kinematics to define centre position

circle_radius = 49    

#Square Case
squareLength = 70



def inverseKinematics(x,y,L1=L1,L2=L2):
    costh2 = (x**2 + y**2-L1**2-L2**2)/(2 * L1 * L2)
    sinth2 = np.sqrt(1-costh2**2)
    theta2 = np.arctan2(sinth2,costh2)
    
    A = x
    B = L1 + (L2 * costh2)
    C = -L2 * sinth2

    w = (C+np.sqrt(C**2 + B**2 - A**2))/(A+B)
    theta1 = 2 * np.arctan(w)
    return theta1,theta2

def convert_degrees_to_encoder_counts(theta):
    return np.floor(theta * counts_per_degree)

def convert_encoder_counts_to_degrees(encoder_counts):
    return encoder_counts/counts_per_degree
def getNumberofTimeSteps(time_taken,time_step):
    return int(np.ceil(time_taken/time_step))

def circleGeneration2DoF(L1,L2,circle_radius,time_taken,time_step):
    theta1 = np.array([])
    theta2 = np.array([])
    motor_counts1 = np.array([])
    motor_counts2 = np.array([])

    N = getNumberofTimeSteps(time_taken,time_step)
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

def get_to_x_position(start_position,end_position,time_taken,time_step=time_step):
    N = getNumberofTimeSteps(time_taken,time_step)
    positions = np.linspace(start_position,end_position,N).astype(int)
    return positions

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

def shape_recreation(motor_counts1,motor_counts2):
    x_values,y_values = [0],[0]
    theta_1 = motor_counts1 / counts_per_degree
    theta_2 = motor_counts2 / counts_per_degree
    for i,a in enumerate(motor_counts1):
        x,y = forwardKinematics(theta_1[i],theta_2[i],L1,L2)
        x_values.append(x)
        y_values.append(y)
    x_values = np.array(x_values)
    y_values = np.array(y_values)

    return x_values, y_values

#File saving

def create_header_file(motor_counts1,motor_counts2,relative_path, shape):
    ref_length = motor_counts1.size
    # Create a header file
    header_file_content = """
    #ifndef ARRAYS_H
    #define ARRAYS_H

    const int ref_length = """+ str(ref_length) +""";

    // Array 1
    const int ref1"""+ "_"+ shape + """[] = {""" + ", ".join(map(str, motor_counts1.astype(int))) + """};

    // Array 1
    const int ref2"""+ "_"+ shape + """[] = {""" + ", ".join(map(str, motor_counts2.astype(int))) + """};

    #endif  // ARRAYS_H
    """
    dirname = os.path.dirname(__file__)
    path = os.path.join(dirname,"../2DofRobot",relative_path)
    with open(path,"w") as file:
        file.write(header_file_content)

def save_file(motor_counts,relative_path,shape):
    dirname = os.path.dirname(__file__)
    path = os.path.join(dirname,shape,relative_path)
    print(dirname)
    print(path)
    
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

def getSquareCoords(squareCentre_x,squareCentre_y,squareLength=squareLength):
    x1 = squareCentre_x + squareLength/2
    y1 = squareCentre_y + squareLength/2
    x2 = x1 - squareLength
    y2 = y1
    x3 = x2
    y3 = y2 - squareLength
    x4 = x3 + squareLength
    y4 = y3
    x5 = x1
    y5 = y1
    x_coords = np.array([x1,x2,x3,x4,x5])
    y_coords = np.array([y1,y2,y3,y4,y5])
    return x_coords,y_coords

def squareReferenceGeneration(x_coords,y_coords,time_taken):
    motor_counts1square = np.array([])
    motor_counts2square = np.array([])

    for i, val in enumerate(x_coords[:-1]):
        xStart, yStart, xEnd, yEnd = x_coords[i], y_coords[i], x_coords[i+1], y_coords[i+1]
        xPositionArray = get_to_x_position(xStart,xEnd,time_taken/4)
        yPositionArray = get_to_x_position(yStart,yEnd,time_taken/4)
        motor1EncoderValues = np.array([])
        motor2EncoderValues = np.array([])
        for xPos,yPos in zip(xPositionArray,yPositionArray):
            th1, th2 = inverseKinematics(xPos,yPos)
            en1 = convert_degrees_to_encoder_counts(np.rad2deg(th1))
            en2 = convert_degrees_to_encoder_counts(np.rad2deg(th2))
            motor1EncoderValues = np.append(motor1EncoderValues,en1)
            motor2EncoderValues = np.append(motor2EncoderValues,en2)
        motor_counts1square = np.append(motor_counts1square,motor1EncoderValues)
        motor_counts2square = np.append(motor_counts2square,motor2EncoderValues)
    return motor_counts1square,motor_counts2square



# Generation

# Square
x_centre,y_centre = forwardKinematics(10,90,L1,L2)
x_coords, y_coords = getSquareCoords(x_centre,y_centre,squareLength)
print('xcoords', x_coords)
motor_counts1_start = convert_degrees_to_encoder_counts(90)
motor_counts2_start = convert_degrees_to_encoder_counts(0)


motor_counts1 = np.array([motor_counts1_start])
motor_counts2 = np.array([motor_counts2_start])

motor_counts1square, motor_counts2square = squareReferenceGeneration(x_coords,y_coords,8)


motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
motor_counts2 = wait_x_seconds_generation(2,motor_counts2)

# Get's to the start position of the circle from completely straight
motor_counts1 = np.append(motor_counts1,get_to_x_position(motor_counts1_start,motor_counts1square[0],time_taken=3))
motor_counts2 = np.append(motor_counts2,get_to_x_position(motor_counts2_start,motor_counts2square[0],time_taken=3))

motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
motor_counts2 = wait_x_seconds_generation(2,motor_counts2)

# Adds circle to arrays
motor_counts1 = np.append(motor_counts1,motor_counts1square)
motor_counts2 = np.append(motor_counts2,motor_counts2square)

motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
motor_counts2 = wait_x_seconds_generation(2,motor_counts2)
# Returns to the start position
motor_counts1 = np.append(motor_counts1,get_to_x_position(motor_counts1[-1],motor_counts1_start,time_taken=3))
motor_counts2 = np.append(motor_counts2,get_to_x_position(motor_counts2[-1],motor_counts2_start,time_taken=3))

motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
motor_counts2 = wait_x_seconds_generation(2,motor_counts2)


# Circle

# motor_counts1_start = convert_degrees_to_encoder_counts(90)
# motor_counts2_start = convert_degrees_to_encoder_counts(0)


# motor_counts1 = np.array([motor_counts1_start])
# motor_counts2 = np.array([motor_counts2_start])

# motor_counts1circle, motor_counts2circle = circleGeneration2DoF(L1,L2,circle_radius,2,time_step)
# print(convert_encoder_counts_to_degrees(motor_counts1circle[0]),convert_encoder_counts_to_degrees(motor_counts2circle[0]))

# motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
# motor_counts2 = wait_x_seconds_generation(2,motor_counts2)


# # Get's to the start position of the circle from completely straight
# motor_counts1 = np.append(motor_counts1,get_to_x_position(motor_counts1_start,motor_counts1circle[0],time_taken=3))
# motor_counts2 = np.append(motor_counts2,get_to_x_position(motor_counts2_start,motor_counts2circle[0],time_taken=3))

# motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
# motor_counts2 = wait_x_seconds_generation(2,motor_counts2)

# # Adds circle to arrays
# motor_counts1 = np.append(motor_counts1,motor_counts1circle)
# motor_counts2 = np.append(motor_counts2,motor_counts2circle)

# motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
# motor_counts2 = wait_x_seconds_generation(2,motor_counts2)
# # Returns to the start position
# motor_counts1 = np.append(motor_counts1,get_to_x_position(motor_counts1[-1],motor_counts1_start,time_taken=3))
# motor_counts2 = np.append(motor_counts2,get_to_x_position(motor_counts2[-1],motor_counts2_start,time_taken=3))

# motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
# motor_counts2 = wait_x_seconds_generation(2,motor_counts2)

# 1DoF Circle
# Wait 2 seconds then Motor 1 circle for 6 seconds, wait 3 seconds motor2 circles for 4 seconds, wait 2 seconds, then both circles for 5 seconds

'''
motor_counts1 = np.array([motor_counts1_start])
motor_counts2 = np.array([motor_counts2_start])

motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
motor_counts2 = wait_x_seconds_generation(2,motor_counts2)

motor_counts1 = circleGeneration1DoF(motor_counts1,time_taken=6,sweep_angle=90)
motor_counts2 = wait_x_seconds_generation(6,motor_counts2)

motor_counts1 = wait_x_seconds_generation(3,motor_counts1)
motor_counts2 = wait_x_seconds_generation(3,motor_counts2)

motor_counts1 = wait_x_seconds_generation(4,motor_counts1)
motor_counts2 = circleGeneration1DoF(motor_counts2,time_taken=4,sweep_angle=90)

motor_counts1 = wait_x_seconds_generation(2,motor_counts1)
motor_counts2 = wait_x_seconds_generation(2,motor_counts2)

# motor_counts1 = circleGeneration1DoF(motor_counts1,time_taken=5)
# motor_counts2 = circleGeneration1DoF(motor_counts2,time_taken=5)
'''

# Check if arrays are equal length
print(motor_counts1.size,motor_counts2.size)
print("motor1 and motor2 are equal: ",motor_counts1.size==motor_counts2.size)

path1 = r'motor12DoFGeneration.txt'
path2 = r'motor22DoFGeneration.txt'

save_file(motor_counts1,path1,'circle')
save_file(motor_counts2,path2,'circle')

header_path = '2DoFCircleReferenceSignals.h'
create_header_file(motor_counts1,motor_counts2,header_path, 'circle')

# Plotting
time = get_time_array(motor_counts1)
plot_motor_position(time,motor_counts1,motor_counts2) # Need to close plot window to end script, weird behaviour in VSCode

x_recreated, y_recreated = shape_recreation(motor_counts1,motor_counts2)
plt.plot(x_recreated,y_recreated, '.')
plt.grid()
plt.axis('equal')
plt.show()

plt.plot(x_coords,y_coords)
plt.grid()
plt.axis('equal')
plt.show()