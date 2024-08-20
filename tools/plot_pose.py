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
    positions = [[row[1], row[2], row[3]] for row in data]
    quaternions = [[row[4], row[5], row[6], row[7]] for row in data]

    # Convert quaternions to Euler angles
    euler_angles = []
    for quaternion in quaternions:
        ea_ = quaternion_to_euler(quaternion)
        euler_angles.append(ea_)

    # Unwrap the yaw angle
    unwrapped_yaw = np.unwrap([angle[2] for angle in euler_angles])

    # Update the yaw angle in the euler_angles list
    for i in range(len(euler_angles)):
        euler_angles[i][0] = math.degrees(euler_angles[i][0])
        euler_angles[i][1] = math.degrees(euler_angles[i][1])
        euler_angles[i][2] = math.degrees(unwrapped_yaw[i])
    
    # # Plot position
    # fig, ax = plt.subplots(3, 1, sharex=True)
    # ax[0].plot(timestamps, [pos[0] for pos in positions], label='pos_x')
    # ax[1].plot(timestamps, [pos[1] for pos in positions], label='pos_y')
    # ax[2].plot(timestamps, [pos[2] for pos in positions], label='pos_z')
    # ax[0].set_ylabel('Position X')
    # ax[1].set_ylabel('Position Y')
    # ax[2].set_ylabel('Position Z')
    # ax[2].set_xlabel('Timestamp')
    # plt.legend()
    # plt.show()

    # # Plot Euler angles
    # fig, ax = plt.subplots(3, 1, sharex=True)
    # ax[0].plot(timestamps, [angle[0] for angle in euler_angles], label='roll')
    # ax[1].plot(timestamps, [angle[1] for angle in euler_angles], label='pitch')
    # ax[2].plot(timestamps, [angle[2] for angle in euler_angles], label='yaw')
    # ax[0].set_ylabel('Roll')
    # ax[1].set_ylabel('Pitch')
    # ax[2].set_ylabel('Yaw')
    # ax[2].set_xlabel('Timestamp')
    # plt.legend()
    # plt.show()

    # Plot position and Euler angles in one window
    fig, ax = plt.subplots(3, 2, sharex=True)
    # Increase the size of the figure window
    fig.set_size_inches(12, 5)

    # Plot position
    ax[0, 0].plot(timestamps, [pos[0] for pos in positions], label='pos_x')
    ax[1, 0].plot(timestamps, [pos[1] for pos in positions], label='pos_y')
    ax[2, 0].plot(timestamps, [pos[2] for pos in positions], label='pos_z')
    ax[0, 0].set_ylabel('Position X')
    ax[1, 0].set_ylabel('Position Y')
    ax[2, 0].set_ylabel('Position Z')

    # Plot Euler angles
    ax[0, 1].plot(timestamps, [angle[0] for angle in euler_angles], label='roll')
    ax[1, 1].plot(timestamps, [angle[1] for angle in euler_angles], label='pitch')
    ax[2, 1].plot(timestamps, [angle[2] for angle in euler_angles], label='yaw')
    ax[0, 1].set_ylabel('Roll')
    ax[1, 1].set_ylabel('Pitch')
    ax[2, 1].set_ylabel('Yaw')

    ax[2, 0].set_xlabel('Timestamp')
    ax[2, 1].set_xlabel('Timestamp')

    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()