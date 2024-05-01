import serial
import numpy as np
import open3d as o3d
import math
import time



totalDataPoints = []
scans = 15  # We have 15 scans
measurements = 64  # Each scan consists of 64 measurements


for k in range(scans):
    #Initialize the serial port
    s = serial.Serial(port='COM3', baudrate=115200, timeout=10)
    print("Opening: " + s.name)

    data = []

    #Reset the buffers of the UART port to delete the remaining data in the buffers
    s.reset_output_buffer()
    s.reset_input_buffer()

    #Wait for the user's signal to start the program
    input("press enter to start")
    #Send the character 's' to MCU via UART
    #This will signal MCU to start the transmission
    s.write(b's')  # Assuming 's' is encoded as bytes
    for i in range(measurements):
        x = s.readline()
        data_str = x.decode().strip()  # Remove leading/trailing whitespace and newline characters
        if data_str.isdigit():  # Check if the string contains only digits
            if (int(data_str) > 4000):
                data.append(4000)
                print("Measurement too large, appending 4000")
            else:
                data.append(int(data_str))
            print(x.decode().strip())
        
    
    #Close the port
    s.close()
    print("Closing: " + s.name)
   


    
    #Write data to file 
    for i in range(len(data)):  # Use the length of data to iterate
        X_mm = data[i] * math.sin(math.radians(5.625*i)) 
        Y_mm = data[i] * math.cos(math.radians(5.625*i))
        Z_mm = k * 400
        totalDataPoints.append((X_mm,Y_mm,Z_mm))
        print(X_mm,Y_mm,Z_mm)

with open('projectscan.xyz', 'w') as f:
    for X_mm, Y_mm, Z_mm in totalDataPoints:
        f.write(f"{X_mm:f} {Y_mm:f} {Z_mm:f}\n")
f.close()
pcd = o3d.io.read_point_cloud("projectscan.xyz", format="xyz")



#Lets see what our point cloud data looks like graphically       
o3d.visualization.draw_geometries([pcd])


points = np.asarray(totalDataPoints)

# Create lines connecting consecutive points within a scan
lines_within_scan = []
for scan in range(scans):
    for i in range(measurements - 1):
        # Connect point i to i+1 within this scan
        index = scan * measurements + i
        lines_within_scan.append([index, index + 1])

# Optionally, if you want to connect the last point of the scan to the first one
for scan in range(scans):
    start_index = scan * measurements
    end_index = (scan + 1) * measurements - 1
    lines_within_scan.append([end_index, start_index])

# Create lines connecting corresponding points between scans
lines_between_scans = []
for scan in range(scans - 1):
    for i in range(measurements):
        # Connect point i of this scan to point i of the next scan
        current_index = scan * measurements + i
        next_index = (scan + 1) * measurements + i
        lines_between_scans.append([current_index, next_index])

# Combine the lines
all_lines = lines_within_scan + lines_between_scans

# Creating the LineSet
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(all_lines)
)

# Visualizing the point cloud with lines
o3d.visualization.draw_geometries([line_set])  # Visualize the line set


