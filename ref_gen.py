import numpy as np

# Parameters
L1 = 160  
L2 = 49   

encoder_steps_per_revolution = 131.25 * 16  
counts_per_degree = encoder_steps_per_revolution / 360  
N = 1000
angles = np.linspace(0, 2 * np.pi, N)


#Circle Case
xc = L1
yc = 0  
r = 49    



motor1_counts = []
motor2_counts = []
theta1_list = []
theta2_list = []

for angle in angles:
    x = xc + r * np.cos(angle)
    y = yc + r * np.sin(angle)

    # Inverse Kinematics 
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    D = np.clip(D, -1.0, 1.0)
    theta2 = np.arccos(D) 

    
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    theta1_list.append(theta1)
    theta2_list.append(theta2)
    
theta1_array = np.unwrap(np.array(theta1_list))
theta2_array = np.unwrap(np.array(theta2_list))

theta1_deg = np.degrees(theta1_array)
theta2_deg = np.degrees(theta2_array)

motor1_counts = theta1_deg * counts_per_degree
motor2_counts = theta2_deg * counts_per_degree
motor1_counts = np.round(motor1_counts).astype(int)
motor2_counts = np.round(motor2_counts).astype(int)


#File saving
motor1_counts = ', '.join(map(str, motor1_counts))
motor2_counts = ', '.join(map(str, motor2_counts))
with open('motor1.txt', 'w') as f:
    f.write(motor1_counts)
with open('motor2.txt', 'w') as f:
    f.write(motor2_counts)
