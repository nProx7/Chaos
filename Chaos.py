from math import e
from tkinter import SE
import serial.tools.list_ports
from collections import Counter
import multiprocessing
import platform
import shutil
import serial
import time
import csv
import os
import re

# vid and pid of the teensy
dev_vid = '16c0'
dev_pid = '0483'

BAUD_RATE = 115200

# returns the device from the serial number    
def find_port(serial_number):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.serial_number == serial_number:
            return port.device
    return ''

# returns a list of serial_numbers matching the vid and pid of the MCO
def find_ports():
    ports = serial.tools.list_ports.comports()
    ids = []
    for port in ports:
        if port.vid == int(dev_vid, 16) and port.pid == int(dev_pid, 16):
            ids.append(port.serial_number)
    return ids
    
def move_file(source_path, dest_path):
    try:
        shutil.move(source_path, dest_path)
        print(f"Trial Data moved to {dest_path}")
    except Exception as e:
        print(f"Error occurred: {e}")

# error thrown on timeout for recieve_comm()
class CleanResponseError(Exception):
    def __init__(self, message="Failed to generate clean response"):
        self.message = message
        super().__init__(self.message)

class SerialCommunication:
    def __init__(self, serial_number, baud_rate = BAUD_RATE):
        while True:
            try: 
                # find the port at the given serial_number
                self.port = find_port(serial_number) 
                self.baud_rate = baud_rate 
                
                # initiate a Serial connection with the given port
                self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
                
                # display if connected
                print("Connected to", self.ser.name) 
                break
            
            except serial.SerialException as e: 
                # display error message and pause before reconnecting
                print("Error: Could not open serial port", e)
                time.sleep(1)
                                    
    def send_command(self, command): 
        # send the command to the teensy
        self.ser.write(command.encode())
        
        # return received if command is not requesting data
        if not ('?' in command):
            return "received"
        
        # sets longest time the device will wait on one pass
        self.ser.timeout = .15
        
        # response can be either a + or - integer or float
        pattern = re.compile(r'^\-?\d{1,11}(\.\d+)?$')
        
        # counts the number of valid responses
        counter = Counter()
        
        # buffer to hold response from device
        response = ''
        
        # quit reading after 5 seconds
        runtime = time.time_ns()
        while time.time_ns() - runtime < 5e9:
            try: 
                # read any data available and append to response
                response += self.ser.read(self.ser.in_waiting).decode()
                
                # continue if nothing was read
                if not response:
                    continue
                
                # remove \r characters and create an array of strings by the newline character 
                temp = response.replace('\r', '').split('\n')
                
                for value in temp:
                    # increment counter if valid response 
                    if pattern.match(value):
                        counter[value] += 1
                        
                # if the counter is not empty return the most common element
                if counter:
                    return counter.most_common(1)[0][0]
                
                # keep only the last string in temp if no valid input is read
                response = temp[-1]
                
            except Exception as e:
                # display error message without breaking the loop
                print(e)
                
    def receive_comm(self, timeout = .15):
        # valid input is +/-#### +/-#### (nan or +/-######.#...)
        pattern = re.compile(r'^(\-?\d{1,4}\s){1,2}(nan|\-?\d{1,6}(\.\d+))$')

        # counts the number of valid responses
        counter = Counter()
        
        # buffer to hold response from device
        response = ''
        
        # quit reading after 1 second
        runtime = time.time_ns()
        while time.time_ns() - runtime < timeout * 1e9:
            # read data from teensy and append to response
            response += self.ser.read(1).decode()
            
            # continue if no response
            if not response:
                continue
           
            # remove \r characters and create an array of strings by the newline character 
            temp = response.replace('\r', '').split('\n')
            
            for value in temp:
                # increment counter of middle element if valid response
                if pattern.match(value):
                    counter[value.split(' ')[1]] += 1
                    
            # return the most commmon response if counter is not empty
            if counter:
                return counter.most_common(1)[0][0]
            
            # keep only the last string in temp if no valid input is read
            response = temp[-1]
            
        raise CleanResponseError
            
    # close the serial connection
    def close_connection(self): 
        self.ser.close()
        
def trial(serial_number):
    # connect to the teensy
    ser_comm = SerialCommunication(serial_number)
    
    # stop the REPT function and securely close connection
    ser_comm.send_command("REPT0")
    ser_comm.close_connection()
    
    # re-connect to the teensy
    ser_comm = SerialCommunication(serial_number)
    
    # display frequency(FREQ), amplitude(AMPL), position, device_time
        # turn the coil on and start the REPT function
    commands = ["FREQ?", "AMPL?", "POSN?", "TIME?", "COIL1", "REPT1"]  
    for command in commands: 
        response = ser_comm.send_command(command) 
        print(command, "->", response)
    
    FREQ_INC, AMPL_INC = 1, 3       # the rate in which FREQ and AMPL is incremented
    FREQ_BASE, AMPL_BASE = 5, 100   # the lowest value FREQ and AMPL can be
    FREQ_TH, AMPL_TH = 1500, 1000   # the highest value FREQ and AMPL can be
    FREQ_DIV = 1e3                  # number to divide FREQ by | determines if FREQ increments by .01 or .001
    
    # starting value of FREQ and AMPL
    FREQ, AMPL = FREQ_BASE, AMPL_BASE
    
    # current trial information is sent to currTrial folder
    file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "currTrial")
    
    # create currTrial folder if it doesn't exist
    if not os.path.exists(file_path):
        os.makedirs(file_path)
        
    while True:
        # change teensy FREQ and AMPL and display message
        print(f"AMPL{AMPL}->{ser_comm.send_command(f'AMPL{AMPL}')}")       
        print(f"FREQ{FREQ/FREQ_DIV}->{ser_comm.send_command(f'FREQ{FREQ/float(FREQ_DIV)}')}")

        # naming convention : SERIAL_NUMBER AMPL### ####FREQ### m-d-Y H,M.csv
        file_name = f"{serial_number} AMPL{AMPL} {int(FREQ_DIV)}FREQ{FREQ}" 
        file_name += f' {time.strftime("%m-%d-%Y %H,%M", time.localtime())}.csv'
        
        # open the file as a csv
        with open(os.path.join(file_path, file_name), 'a', newline='') as file:
            writer = csv.writer(file)
            
            # write data key if applicable
            if os.path.getsize(os.path.join(file_path, file_name)) == 0:
                writer.writerow(["time(s)", "position"])
            
            bound = 0                   # dynamic bound : increments if encoder overflows
            DELTA = 4096                # angle spans [-2047, 2048] allows read position to remain smooth
            trial_time_hr = 45/3600     # total time per trial in hours
            last_pos_data = 0           # stores the last position
            
            # end trial after {trial_time_hr} hours
            start_time = time.time_ns()
            while time.time_ns() - start_time <= trial_time_hr * 3.6e12: 
                try:
                    # get the position from the REPT function
                    position_data_point = int(ser_comm.receive_comm()) 

                    try:
                        # decrement or increment bound if position drops by over 3500 
                        if abs(position_data_point + DELTA*bound - last_pos_data) > 3500:
                            bound -= 1 if ((position_data_point + DELTA*bound - last_pos_data) > 0) else -1 #bound -= (position_data_point+DELTA*bound-last_pos_data)/abs(position_data_point+DELTA*bound-last_pos_data) 
                    except: 
                        # bound is bound
                        bound = bound  
                        
                    # write [time(s), position] to csv
                    writer.writerow([(time.time_ns() - start_time)/10**9, position_data_point+DELTA*bound])
                    
                    # make dynamic_position the new bound
                    last_pos_data = position_data_point+DELTA*bound
    
                except Exception as e:
                    # display the exception | likely CleanResponseError
                    print(e)
                    
                    # remind device REPT should be on 
                    ser_comm.send_command("REPT1")
                    
                    # remind device of its FREQ and AMPL
                    ser_comm.send_command("FREQ"+str(FREQ/FREQ_DIV))
                    ser_comm.send_command("AMPL"+str(AMPL))
                    continue
                
        # completed trials go to the Completed folder under its serial number
        dest_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Completed") # "/home/nprox7/Desktop/DataCollector/Completed/"
        dest_path = os.path.join(dest_path, serial_number)
        
        # create files if they don't exist 
        if not os.path.exists(dest_path):
            os.makedirs(dest_path)
            
        # move csv to Completed folder
        move_file(os.path.join(file_path, file_name), dest_path)
        
        # increment trial FREQ by FREQ_INC
        FREQ += FREQ_INC
        
        # on FREQ over/under-flow, set FREQ to max/min value, increment trial AMPL,
            # and change the direction to increment
        if FREQ > FREQ_TH or FREQ < FREQ_BASE:
            FREQ = FREQ_TH if FREQ > FREQ_TH else FREQ_BASE
            AMPL += AMPL_INC
            FREQ_INC *= -1
            
        # on AMPL over/under-flow, set AMPL to max/min value and swap direction to increment
        if AMPL > AMPL_TH or AMPL < AMPL_BASE:
            AMPL = AMPL_TH if AMPL > AMPL_TH else AMPL_BASE
            AMPL_INC *= -1
            
    # close the connection if it excapes | it never does
    ser_comm.close_connection()
    
def main():
    devices = {} # set of devices: accessed via serial number
    
    while True:
        current_ids = find_ports() # returns an array of serial numbers from all connected MCOs
        
        # iterate through the serial numbers and compare to devices
        for serial_number in current_ids:
            if serial_number not in devices:
                # run trial on new core with new serial_number as argument
                p = multiprocessing.Process(target=trial, args=(serial_number,))
                p.start()
                
                # display trial information
                print(f'trial started for MCO: {serial_number}')
                
                # index the device in devices
                devices[serial_number] = p
        
        # check for new devices every 5 seconds
        time.sleep(5)
    

if __name__ == "__main__":
    main()