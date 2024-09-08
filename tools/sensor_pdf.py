import sys
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm

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

def pdf(data):
    mean = np.mean(data)
    std_dev = np.std(data)

    print(f"噪声均值: {mean}")
    print(f"噪声标准差: {std_dev}")

    # 绘制直方图和高斯分布拟合曲线
    plt.hist(data, bins=10, density=True, alpha=0.6, color='g')

    # 拟合高斯分布
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = norm.pdf(x, mean, std_dev)
    plt.plot(x, p, 'k', linewidth=2)
    title = "Fit results: mean = %.2f,  std_dev = %.2f" % (mean, std_dev)
    plt.title(title)

    plt.show()

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

    pdf(positions[0])
    pdf(positions[1])
    pdf(positions[2])

if __name__ == "__main__":
    main()