# Low Power WiFi Datalogger 
# Arduino Yun-based Datalogger
# Created by Tony DiCola (tony@tonydicola.com)

# Quick and dirty script to connect to an Arduino Yun-based datalogger 
# and retrieve data every minute.  The YunDataLogger class is useful outside
# this script in other tools, such as an IPython notebook that wants to query
# the Yun datalogger.

# Update the configuration below with the Yun's name, username, and password
# before running.  All output is written to standard out.
# Press Ctrl-C to quit at any time.

import select
import paramiko
import sys
import time


class YunDatalogger(object):
    def __init__(self, address='arduino.local', username='root', password=''):
        # Create an SSH connect to the Arduino Yun
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(address, username=username, password=password)
        # Connect to the Yun's console
        self.conin, self.conout, self.conerr = self.client.exec_command('telnet localhost 6571')
    
    def _clear_output(self):
        # Remove all the pending data to be read from the console output.
        self.conin.write('X\n')
        self.conin.flush()
        while self.conout.channel.recv_ready():
            data = self.conout.read(1)
    
    def sample_csv(self, seconds, sample_freq_ms):
        num_samples = int(seconds*(1000.0/sample_freq_ms))
        # Read a number of samples made at the specified sample frequency (in milliseconds)
        # First clear the pending data from the console.
        self._clear_output()
        # Tell the Yun to start sending data at the specified frequency.
        self.conin.write('S{0:d}\n'.format(sample_freq_ms))
        self.conin.flush()
        # Collect the results
        results = []
        for i in range(num_samples):
            results.append(self.conout.readline().strip())
        # Stop the sampling and clear pending console data.
        self._clear_output()
        # Return the results as a string in CSV format
        return '\n'.join(results)
    
    def sample_csv_single(self):
        results = self.sample_csv(1, 1000)
        return results.strip()

    def close(self):
        self.client.close()


SAMPLE_FREQUENCY_SECONDS = 60.0
YUN_ADDRESS = 'arduino.local'
YUN_USERNAME = 'root'
YUN_PASSWORD = 'password'


# When run as a script at the command line, grab a sample from the Yun datalogger
# a the specified frequency and write it to standard output.
if __name__ == '__main__':
    while True:
        logger = YunDatalogger(address=YUN_ADDRESS,
                               username=YUN_USERNAME,
                               password=YUN_PASSWORD)
        print logger.sample_csv_single()
        sys.stdout.flush()
        logger.close()
        time.sleep(SAMPLE_FREQUENCY_SECONDS)
