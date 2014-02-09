# Low Power WiFi Datalogger 
# Simple TCP Listening Server
# Created by Tony DiCola (tony@tonydicola.com)

# Create a simple server to listen on TCP port 8000, accept any connections 
# and print all data received to standard output (along with a timestamp of
# when the data was received).
# Must be terminated by hitting ctrl-c to kill the process!

from socket import *
import datetime
import sys
import threading


SERVER_PORT = 8000


# Create listening socket
server = socket(AF_INET, SOCK_STREAM)

# Ignore waiting for the socket to close if it's already open.  See the python socket
# doc for more info (very bottom of http://docs.python.org/2/library/socket.html).
server.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

# Listen on any network interface for the specified port
server.bind(('', SERVER_PORT))
server.listen(5)

# Worker thread function to print all data received to standard output until connection closes.
def process_connection(client):
	received = ''
	while True:
		data = client.recv(1024)
		if not data:
			print '{0},{1}'.format(datetime.datetime.utcnow(), received.strip());
			sys.stdout.flush() 
			break
		else:
			received += data;
	client.close()

try:
	# Wait for connections and spawn worker threads to process them.
	while True:
		client, address = server.accept()
		thread = threading.Thread(target=process_connection, args=(client,))
		thread.daemon = True
		thread.start()
except:
	server.close()