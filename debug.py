
import serial
import matplotlib.pyplot as plt


while(1):
	try:
		ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
		set_point = []
		roll_ctrl = []
		plane_angle = []
		other = []
		plt.ion()
		fig = plt.figure()
		break;
	except (serial.serialutil.SerialException):
		pass
		
try:	
	while(1):
		if ser.in_waiting > 0:
			plt.clf()
			next_line = ser.readline()
			data_str = next_line.decode('UTF-8')
			str_data = data_str.split(",")
			set_point.append(float(str_data[0]))
			roll_ctrl.append(float(str_data[1]))
			plane_angle.append(float(str_data[2]))
			other.append(float(str_data[3]))
		
except:
	plt.plot(set_point)
	plt.plot(roll_ctrl)
	plt.plot(plane_angle)
	plt.plot(other)
	plt.legend(['set_point', 'roll_ctrl', 'plane_angle', 'other'])
	plt.show()
	temp = input()











