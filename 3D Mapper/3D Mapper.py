

import serial
import numpy as np
from math import *
import open3d as o3d

#written by Brandon English
#400395427
#englisb

s = serial.Serial('COM3', 115200, timeout = 10)
                            
print("Opening: " + s.name)

while True:
    # reset the buffers of the UART port to delete the remaining data in the buffers
    s.reset_output_buffer()
    s.reset_input_buffer()

    # wait for user's signal to start the program
    print("Press J1 to start")
    d = '' 
    while d.isdigit() == False:
        r = s.readline()                    
        d = r.decode()                      # Decodes byte input from UART into string 
        d = d[0:-2]    
    # send the character 's' to MCU via UART
    # This will signal MCU to start the transmission

    f = open("tof_radar.xyz", "w")    #create a new file for writing


    stop = 0
    deg = 0
    z = 0
    # recieve coordinates from UART of MCU

    for i in range(24):
        if deg == 2 * pi:
            deg = 0
        d = '' 
        while d.isdigit() == False:
            r = s.readline()                    
            d = r.decode()                      # Decodes byte input from UART into string 
            d = d[0:-2]                         # Removes carriage return and newline from string
            if d == "Stop":
                print("Stopping Transmission... Erasing Data... Return to Start")
                stop = 1
                break
        if stop == 1:
            break
        x =  float(d) * cos(deg)                   
        y =  float(d) * sin(deg)
        f.write(str(x) + ' ')
        f.write(str(y) + ' ')
        f.write(str(z) + '\n')
        deg = deg + pi / 4
        if (i + 1) % 8 == 0:
            z += 0
        print("ye")

    f.close()   #there should now be a file containing 24 vertex coordinates
           
    if stop == 0:
        #Read the test data in from the file we created        
        print("Now reading: Location H point cloud data (pcd)")
        pcd = o3d.io.read_point_cloud("tof_radar.xyz", format="xyz")

        #Lets see what our point cloud data looks like numerically       
        print("The PCD array:")
        print(np.asarray(pcd.points))

        #OK, good, but not great, lets add some lines to connect the vertices
        #   For creating a lineset we will need to tell the package which vertices need connected
        #   Remember each vertex actually contains one x,y,z coordinate

        plane = []
        for i in range(24):
            plane.append([i])

        #Define coordinates to connect lines in each yz slice        
        lines = []  
        for i in range(23):
            lines.append([plane[i], plane[i+1]])
            if i == 7 or i == 15:
                lines.pop()

        #Define coordinates to connect lines between current and next yz slice        
        for i in range(16):
            lines.append([plane[i], plane[i+8]])
            
         
        lines.append([plane[0], plane[7]])
        lines.append([plane[8], plane[15]])
        lines.append([plane[16], plane[23]])

        line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))

        o3d.visualization.draw_geometries([line_set])

    print("Close the serial port with s.close()")
