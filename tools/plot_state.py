import sys
import math
import matplotlib.pyplot as plt
import numpy as np

def quaternion_to_euler(quaternion):
    # Extract the components of the quaternion
    x, y, z, w = quaternion

    # Calculate the roll, pitch, and yaw angles
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    
    # Unwrap the yaw angle
    # # Convert roll, pitch, and yaw from radians to degrees
    # roll_deg = math.degrees(roll)
    # pitch_deg = math.degrees(pitch)
    # yaw_deg = math.degrees(yaw_unwrapped)

    return [roll, pitch, yaw]

def main():
    # Check if the file path is provided as a command line argument
    if len(sys.argv) < 2:
        print("Please provide the file path as a command line argument.")
        return
    
    file_path = sys.argv[1]

    # Read data from the text file
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            values = line.split()
            data.append([float(value) for value in values])

    # Extract position and Euler angles from the data
    timestamps = [row[0] for row in data]
    residual = [[row[1], row[2], row[3], row[4], row[5], row[6], row[7], row[8], row[9], row[10], 
                 row[11], row[12], row[13], row[14], row[15], row[16], row[17], row[18], row[19], row[20], row[21]] for row in data]
      
    # Plot residual
    fig, ax = plt.subplots(4, 2, sharex=True)
    # Increase the size of the figure window
    fig.set_size_inches(12, 5)

    # Plot residual components
    ax[0, 0].plot(timestamps, [res[0] for res in residual], label='pos_x')
    ax[0, 0].plot(timestamps, [res[1] for res in residual], label='pos_y')
    ax[0, 0].plot(timestamps, [res[2] for res in residual], label='pos_z')
    ax[0, 0].legend()

    ax[1, 0].plot(timestamps, [res[3] for res in residual], label='vel_x')
    ax[1, 0].plot(timestamps, [res[4] for res in residual], label='vel_y')
    ax[1, 0].plot(timestamps, [res[5] for res in residual], label='vel_z')
    ax[1, 0].legend()
    
    ax[2, 0].plot(timestamps, [res[6] for res in residual], label='ori_x')
    ax[2, 0].plot(timestamps, [res[7] for res in residual], label='ori_y')
    ax[2, 0].plot(timestamps, [res[8] for res in residual], label='ori_z')
    ax[2, 0].legend()

    ax[0, 1].plot(timestamps, [res[9] for res in residual], label='gyro_bias_x')
    ax[0, 1].plot(timestamps, [res[10] for res in residual], label='gyro_bias_y')
    ax[0, 1].plot(timestamps, [res[11] for res in residual], label='gyro_bias_z')
    ax[0, 1].legend()

    ax[1, 1].plot(timestamps, [res[12] for res in residual], label='acc_bias_x')
    ax[1, 1].plot(timestamps, [res[13] for res in residual], label='acc_bias_y')
    ax[1, 1].plot(timestamps, [res[14] for res in residual], label='acc_bias_z')
    ax[1, 1].legend()
    
    ax[2, 1].plot(timestamps, [res[15] for res in residual], label='mag_bias_x')
    ax[2, 1].plot(timestamps, [res[16] for res in residual], label='mag_bias_y')
    ax[2, 1].plot(timestamps, [res[17] for res in residual], label='mag_bias_z')
    ax[2, 1].legend()

    ax[3, 0].plot(timestamps, [res[18] for res in residual], label='g_bias_x')
    ax[3, 0].plot(timestamps, [res[19] for res in residual], label='g_bias_y')
    ax[3, 0].legend()

    ax[3, 1].plot(timestamps, [res[20] for res in residual], label='g_bias_z')
    ax[3, 1].legend()

    fig.suptitle(file_path)

    plt.legend()
    plt.show()



if __name__ == "__main__":
    main()