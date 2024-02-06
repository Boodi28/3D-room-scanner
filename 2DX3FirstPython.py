# Student name: Abdalla Mahdy
# Student number: 400411114
# MacID: mahdya

import serial
import math

s = serial.Serial('COM4', 115200)

s.open
s.reset_output_buffer()
s.reset_input_buffer()

f = open("3Dpoints.xyz", "w")
step = 0
x = 0  # Initial x-displacement (mm)
increment = 250  # X-displacement steps (mm)

num_inc = int(input("Enter a number:"))
count = 0

while count < num_inc:
    raw = s.readline()
    data = raw.decode("utf-8")  # Decode byte input from UART into string
    data = data[0:-2]  # Remove carriage return and newline from string
    if data.isdigit():  # Check if string is a digit
        angle = (step / 512) * 2 * math.pi  # Obtain angle based on motor rotation
        r = int(data)
        y = r * math.cos(angle)  # Calculate y
        z = r * math.sin(angle)  # Calculate z
        print(y)
        print(z)
        f.write("{} {} {}\n".format(x, y, z))  # Write data to .xyz file
        step += 32
    if step == 512:  # Reset number of steps after a full rotation is completed and increment x and count
        step = 0
        x += increment
        count += 1
    print(data)
f.close()  # Close file when done so data saves
