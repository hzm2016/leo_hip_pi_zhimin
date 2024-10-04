import csv
import matplotlib.pyplot as plt
from math import *

def deg2rad(deg):
    rad = deg*pi/180.0
    return rad

# Load CSV data manually
file_path = '163515.csv'  # Replace with your actual CSV file path

# Initialize lists for each column
time = []
L_torque = []
R_torque = []
L_cmd = []
R_cmd = []
R_IMU_angle = []
L_IMU_angle = []
L_IMU_vel = []
R_IMU_vel = []
L_encoder = []
R_encoder = []
L_encoder_vel = []
R_encoder_vel = []
R_est_torque  = []

# Read the CSV file
with open(file_path, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    
    # Skip header row
    next(csvreader)
    
    # Extract data row by row
    for row in csvreader:
        time.append(float(row[0]))  # 'now' is the time
        L_cmd.append(float(row[1]))
        R_cmd.append(float(row[2]))
        L_torque.append(float(row[3]))
        R_torque.append(float(row[4]))
        L_IMU_angle.append(float(row[5]))
        R_IMU_angle.append(float(row[6]))
        L_IMU_vel.append(float(row[7]))
        R_IMU_vel.append(float(row[8]))
        L_encoder.append(float(row[9]))
        R_encoder.append(float(row[10]))
        L_encoder_vel.append(float(row[11]))
        R_encoder_vel.append(float(row[12]))
        
        #R_est_torque.append(0.35*6.4*9.81*sin(deg2rad(float(row[12]))))

# Create subplots
fig, axs = plt.subplots(3, 2, figsize=(10, 8))

#R_est_torque = 0.35*6.4*9.81*sin(R_encoder)

# Plotting the data
axs[0, 0].plot(time, L_cmd, label='L Cmd', color='red')
axs[0, 0].plot(time, L_torque, label='R Torque', color='blue')
axs[0, 0].set_title('Left')
axs[0, 0].legend()

axs[0, 1].plot(time, R_cmd, label='R Cmd', color='red')
axs[0, 1].plot(time, R_torque, label='R Torque', color='blue')
axs[0, 1].set_title('Right')
axs[0, 1].legend()

axs[1, 0].plot(time, L_IMU_angle, label='L IMU Angle', color='red')
axs[1, 0].plot(time, R_IMU_angle, label='R IMU Angle', color='blue')
axs[1, 0].set_title('Angular Position (IMU)')
axs[1, 0].legend()

axs[1, 1].plot(time, L_encoder, label='L Encoder', color='red')
axs[1, 1].plot(time, R_encoder, label='R Encoder', color='blue')
axs[1, 1].set_title('Angular Position (encoder)')
axs[1, 1].legend()

axs[2, 0].plot(time, L_IMU_vel, label='L', color='red')
axs[2, 0].plot(time, R_IMU_vel, label='R', color='blue')
axs[2, 0].set_title('Angular Velocity (IMU)')
axs[2, 0].legend()

axs[2, 1].plot(time, L_encoder_vel, label='L', color='red')
axs[2, 1].plot(time, R_encoder_vel, label='R', color='blue')
axs[2, 1].set_title('Angular Velocity (encoder)')
axs[2, 1].legend()

# Formatting the plots
for ax in axs.flat:
    ax.set(xlabel='Time (s)', ylabel='Value')
    ax.grid(True)

# Adjust layout for better spacing
plt.tight_layout()

# Display the plot
plt.show()
