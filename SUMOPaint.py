# Program: Gps location provider service SUMO
# Goal: Allow android user to connect to a Sumo simulation and input live location data into the simulation
# Author: Michael Hackwill
# Date: 19th October 2021
# Tested and works with SUMO 1.9.2

import os, sys
import traci
import multiprocessing
import numpy as np
from multiprocessing import Process, Value, Array
import socket
import time


#creates the server, handles incoming calls and subsequent user requests
def server(data,lat,lng,acc,spd,hdn):
	# size of buffer and backlog
	buffer = 2048 # value should be a relatively small power of 2, e.g. 4096
	backlog = 1

	# create a socket
	server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # AF_INET = IPv4 socket family; SOCK_STREAM = TCP socket type

	# bind the socket to an address and port
	host = '127.0.0.1' # localhost
	port = 8080 # reserve a port for the service (i.e. a large number less than 2^16); the call will fail if some other application is already using this port number on the same machine
	server_socket.bind((host, port)) # binds the socket to the hostname and port number

	# listen for incoming connections
	server_socket.listen(backlog)

	while True: # infinite loop 1
		client_socket, address = server_socket.accept() # passively accept TCP client connections; the call returns a pair of arguments: client is a new Socket object used to communicate with the client and address is the address of the client


		#ERROR HANDLING

		# record client connection time (as seen from the server)
		start_time = time.strftime('%d %b %Y at %H:%M:%S')
		init_time = str(start_time) # convert connection time to a string
		print('Made a connection with', address, 'on', init_time + '.')

		while True: # infinite loop 2
			incoming = client_socket.recv(buffer).decode('UTF-8') # receive client data into buffer
			if incoming=='end':
				print('Client is no longer providing location.')
				break
			start_time = time.strftime('%d %b %Y at %H:%M:%S')
			init_time = str(start_time)
			lata,lnga,acca,spda,hdna=Locationconstruct(incoming)
			lat.value = lata
			lng.value = lnga
			acc.value = acca
			spd.value = spda
			hdn.value = hdna



# main program
if __name__ == '__main__':

	# constants
	endSim = 1800000 # the simulation will be permitted to run for a total of endSim milliseconds; 1800000 milliseconds = 30 minutes
	timeout = 1 # a floating point number specified in seconds to control simulation timestep

	# initialisations
	step = 0 # time step
	d = Value('d', 0.0) # 'd' is a string containing a type code as used by the array module (where 'd' is a floating point number implemented in double in C) and 0.0 is an initial value for 'd'
	latitude=Value('d',-37.91541476)
	longitude=Value('d',145.14014268)
	accuracy=Value('d',20)
	speed=Value('d',10)
	heading=Value('d',0)
	print()
	print('===========================')
	print('Beginning the main program.')
	print('===========================')
	print()
	print("Connecting to SUMO via TraCI.")
	print()
	# import TraCI (to use the library, the <SUMO_HOME>/tools directory must be on the python load path)
	if 'SUMO_HOME' in os.environ:
		tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
		sys.path.append(tools)
	else:
		sys.exit("Please declare environment variable 'SUMO_HOME'.")

	# compose the command line to start SUMO-GUI
	sumoBinary = "/Program Files (x86)/Eclipse/Sumo/bin/sumo-gui"
	sumoCmd = [sumoBinary, "-S", "-c", "SUMOPaint.sumo.cfg"]
	#netOffset=[-334889.52,4198727.67];
	convBoundary='0.00,-0.00,2522.23,2285.53'
	origBoundary='145.105390,-37.921406,145.151814,-37.900672'
	projParameter="+proj=utm +zone=55 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
	# start the simulation and connect to it with the python script
	traci.start(sumoCmd)
	i=0;

	thread = Process(target=server, args=(d,latitude,longitude,accuracy,speed,heading)) # represents a task (i.e. the server program) running in a subprocess
	print("Launching the server.")
	thread.start()
	print("The server has been launched.")
	xprev=0
	yprev=0

	while step < endSim:
		thread.join(timeout) # implicitly controls the speed of the simulation; blocks the main program either until the server program terminates (if no timeout is defined) or until the timeout occurs

		print('Time step [s]: {}'.format(step/1000))
		print()
		x2, y2 = traci.simulation.convertGeo(longitude.value, latitude.value, fromGeo=True)
		start_time = time.strftime('%d %b %Y at %H:%M:%S')
		init_time = str(start_time)
		print('Location sent to SUMO')
		print('lat:',str(latitude.value),'lng:',str(longitude.value),'Time:',init_time)

		edgeID=traci.vehicle.getRoadID("veh66")
		#headingfix=90-np.arctan2(y2-yprev,x2-xprev)*180/np.pi
		#if headingfix<0:
			#headingfix=headingfix+360
		traci.vehicle.moveToXY("veh66",edgeID,0,x2,y2,angle=heading.value,keepRoute=0)
		xprev=x2
		yprev=y2
		#traci.vehicle.setSpeedMode("veh66", 0)
		#vehicle.setSpeed(speed[-1])

		# go to the next time step
		step += 1000 # in milliseconds

		traci.simulationStep()

	print("Shutting the server down.")
	thread.terminate()
	print("Closing the main program. Goodbye.")
	traci.close() # close the connection to SUMO








def Locationconstruct(locationStr):
	#string decoder function to convert incoming location string into constituent parts

	#splits incoming string on ',' terms
	arr=locationStr.split(',')

	#takes constituent string components and formats then into floating point values
	latstr=arr[0]
	lngstr=arr[1]
	accstr=arr[2]
	spdstr=arr[3]
	hdnstr=arr[4]
	lat=float(latstr[4:-1])
	lng=float(lngstr[4:-1])
	acc=float(accstr[4:-1])
	spd=float(spdstr[4:-1])
	hdn=float(hdnstr[4:-1])
	return lat, lng, acc, spd, hdn
