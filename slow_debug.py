
import serial
import matplotlib.pyplot as plt


while(1):
	try:
		ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
		data = [[]]
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
			for i in range(len(str_data)):
				data[i].append(float(str_data[i]))
				plt.plot(data[i])
			plt.show()
			plt.pause(0.0001)
			
		
except:
	raise(Exception)
	#pass










