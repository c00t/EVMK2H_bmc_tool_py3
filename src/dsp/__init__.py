# ------------------------------------------------------------------------------
#
# Copyright (c) 2013 Texas Instruments Incorporated - http://www.ti.com
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
#
# BMC Tool
#
# version 1.0 -- GUI which allows a user to select the appropriate COM port,
# select a text file to parse, and execute a series of i2c commands to update
# UCD modules on the XTCIEVMK2X.
#
# -------------------------------------------------------------------------------

import sys
import serial
import time
import os
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
import string
from serial.tools import list_ports

# GLOBALS
# Version Differences
bmc_ver = '1.0.1'
command_checks = (
    'Sent',  # 1.0.1.X
    'Read results:'  # 1.0.2.X
)
error_checks = (
    'Error',  # 1.0.1.X
    'The following errors occurred on I2C operations number'  # 1.0.2.X
)
pre_command_list = (
    ['gpio d xxxx_x0xx', 'delay 2000', 'gpio d xxxx_xxx0', 'delay 2000'],  # 1.0.1.X
    ['hwdbg cmd i2c', 'shutdown pwr', 'delay 2000']  # 1.0.2.X
)
command_check = ''
error_check = ''
pre_commands = []

# UCD version commands
ucd_version_commands = (
    'i2c 1.68 9b r r r r r r r r r r',
    'i2c 1.4e 9b r r r r r r r r r r',
    'i2c 1.34 9b r r r r r r r r r r'
)

ucd_versions = [
    'UNKNOWN',  # 104
    'UNKNOWN',  # 78
    'UNKNOWN'  # 52
]

PORT_TIMEOUT = 0.1
commands = []
tk_root = tk.Tk()


def getOldVerResults(ser):
    cont = True
    result = ''
    line = ''
    while cont:
        b = ser.read()
        if b:
            try:  # Python3 reads bytes, decode to string
                b = b.decode("utf-8") #or other appropriate encoding. consider errors='ignore' or errors='replace' for robustness
            except AttributeError:
                    pass
            if b == '\b':
                line = line[:-1]
            elif b == '\n':

                log.write(line + '\n')
                if 'Receiving data from I2C... Received. Data:' in line:
                    ascii_value = int(line[43:], base=16)
                    if 31 < ascii_value < 127:
                        character = chr(ascii_value)
                        if character in string.printable:
                            result += character
                line = ''
            elif b != '\r':
                line += b
        else:

            log.write(line + "\n")
            line = ''
            cont = False
    return result


def getNewVerResults(ser):


    cont = True
    result = ''
    line = ''
    while cont:
        b = ser.read()
        if b:
            try: 
                 b = b.decode("utf-8") 
            except AttributeError:
                pass
            if b == '\b':
                line = line[:-1]
            elif b == '\n':

                log.write(line + "\n")
                if 'Read results:' in line:

                    bytes_list = line.split(' ')[5:]
                    for byte_str in bytes_list:
                        if '0x' not in byte_str:
                            continue

                        character = chr(int(byte_str, base=16))
                        if character in string.printable:
                            result += character
                line = ''
            elif b != '\r':
                line += b
        else:

            log.write(line + "\n")
            line = ''
            cont = False
    return result


def get_ucd_versions():
    port = list_var.get()
    try:
        ser = serial.Serial(port, 115200, timeout=PORT_TIMEOUT)
    except serial.SerialException:
        messagebox.showerror("ERROR", "Could not open serial port")
        return False
    get_version(ser)
    global ucd_versions
    global bmc_ver
    i2c_enable = 'hwdbg cmd i2c'
    for x in i2c_enable:
        ser.write(x.encode()) #encode to send bytes data over serial port
        time.sleep(0.01)
    ser.write(b'\r\n') #send bytes
    iter = 0
    for command in ucd_version_commands:

        for x in command:
            ser.write(x.encode())
            time.sleep(0.01)
        ser.write(b'\r\n')
        if bmc_ver == '1.0.1':

            ucd_versions[iter] = getOldVerResults(ser)
        else:

            ucd_versions[iter] = getNewVerResults(ser)
        print(ucd_versions[iter])
        iter += 1

    ser.close()


def delay_command(str):
    words = str.split(' ')
    print("Delaying " + words[1] + " ms...")
    time.sleep(float(words[1]) / 1000)


def get_version(ser):
    global command_check
    global error_check
    global pre_commands
    global bmc_ver
    ser.write(b'ver\r\n')
    cont = True
    line_str = ''
    i = 0
    while cont:
        b = ser.read()
        if b:
            try:
                 b = b.decode("utf-8") 
            except AttributeError:
                pass

            if b == '\b':
                line_str = line_str[:-1]
            elif b == '\n':
                log.write(line_str + "\n")
                if '1.0.1.' in line_str:
                    bmc_ver = '1.0.1'
                    i = 0
                elif '1.0.2' in line_str:
                    bmc_ver = '1.0.2'
                    i = 1
                line_str = ''
            elif b != '\r':
                line_str += b
        else:
            log.write(line_str + "\n")
            line_str = ''
            cont = False
    command_check = command_checks[i]
    error_check = error_checks[i]
    pre_commands = pre_command_list[i]
    return i


def open_file():
    inFile = filedialog.askopenfile(parent=tk_root, title='Please choose a file')
    if inFile: #check if a file was actually selected
        temp = inFile.name
        inFile.close()
        entry.delete(0, tk.END)
        entry.insert(0, temp)


def parse_file(str):
    command_file = open(str, 'r')
    print("Opened file " + command_file.name)
    if command_file: # Check if file opened successfully
        for line in command_file:
            if line[0] != '#':
                line = line.replace('\t', ' ')
                if 'I2CWrite' in line and len(line) < 256:
                    shifted_addr = int(line[9:11], 16)
                    address = shifted_addr >> 1
                    line = line.replace((hex(shifted_addr)[2:]).upper(), (hex(address)[2:]).upper(), 1)
                    line = line.replace('I2CWrite ', 'i2c 1.')
                    commands.append(line)
                if 'Pause' in line:
                    words = line.split(' ')
                    commands.append('delay ' + words[1])
    print("Finished parsing and adding commands")
    command_file.close()


def list_serial_ports():

    return [port.device for port in list_ports.comports()]




def execute_precommands(ser):
    for command in pre_commands:
        if 'delay' in command:
            delay_command(command)
            continue
        sys.stdout.write("Sending command: " + command + "\r\n")
        for x in command:
            ser.write(x.encode())
            time.sleep(0.01)
        ser.write(b'\r\n')
        # Read back result, line by line, checking for errors
        cont = True
        line_str = ''
        while cont:
            b = ser.read()
            if b:
               try:
                 b = b.decode("utf-8") 
               except AttributeError:
                pass
               if b == '\b':
                 line_str = line_str[:-1]

               elif b== '\n':
                   log.write(line_str + '\n')
                   line_str = ''
               elif b != '\r':
                   line_str += b
            else:
               log.write(line_str + '\n')
               line_str = ''
               cont = False




def execute_commands():
    # Get the selected port and try to open it
    port = list_var.get()
    fileStr = entry.get()
    if not fileStr:
        messagebox.showerror("ERROR", "No File Selected")
        return 1
    try:
        ser = serial.Serial(port, 115200, timeout=PORT_TIMEOUT)
    except serial.SerialException:

        messagebox.showerror("ERROR", "Could not open serial port")
        return 1
    # Get version, parse input file, and execute pre-commands
    get_version(ser)
    parse_file(fileStr)
    execute_precommands(ser)
    # Iterate through commands, send to BMC and check for errors
    error_count = 0
    for command in commands:
        if 'delay' in command:
            delay_command(command)
            continue
        error_count = 0
        keep_sending = True
        while keep_sending and error_count < 10:
            keep_sending = False
            error_occurred = False
            sys.stdout.write("Sending command: " + command + "\r\n")

            for x in command:
                ser.write(x.encode())
                time.sleep(0.01)
            ser.write(b'\r\n')
            # Read back result, line by line, checking for errors
            cont = True
            line_str = ''
            succeeded = False
            timeout_count = 0
            while cont:
                b = ser.read()
                if b:                  
                   try:
                       b = b.decode("utf-8") 
                   except AttributeError:
                       pass
                   if b == '\b':
                       line_str = line_str[:-1]
                   elif b == '\n':
                         if command_check in line_str and not error_occurred:
                             print("I2C command success\r\n")
                             succeeded = True
                         if error_check in line_str and not error_occurred:
                             print("Error occured")
                             keep_sending = True
                             error_count += 1
                             error_occurred = True
                         log.write(line_str + "\n")
                         line_str = ''
                   elif b != '\r':
                        line_str += b

                elif succeeded or timeout_count > 10 :  #no data received hence check for success status
                    if timeout_count > 10 :
                        messagebox.showerror("ERROR", "An error has occurred with the BMC, please power cycle the device and try again.")
                        error_count = 10
                    log.write(line_str + '\n')
                    line_str = ''
                    cont = False
                else:
                   timeout_count += 1

                    
        else: #loop ends normally hence error count should be <10
             if error_count >= 10:

                messagebox.showerror("ERROR", "Could not send command " + command)
                break
    if error_count < 10:
        messagebox.showinfo("Complete", "Finished sending all commands")
    ser.close()
    return True


# Open Log file to save BMC output
log = open('bmc_log.txt', 'w', encoding="utf-8") #open file in utf-8 encoding for wider character support
log.write('Beginning BMC command script\n')

# Get Port Selection List
port_list = list_serial_ports()
if not port_list:
    messagebox.showerror("Error", "No serial ports found!")
    exit()

list_var = tk.StringVar(value=port_list[0]) # Use StringVar for dropdown
dropdown = tk.OptionMenu(tk_root, list_var, *port_list)


exec_button = tk.Button(tk_root, text="Execute", command=execute_commands)
open_button = tk.Button(tk_root, text="Open", command=open_file)
ver_button = tk.Button(tk_root, text="Get Versions", command=get_ucd_versions)
entry = tk.Entry(tk_root)

dropdown.pack(side=tk.TOP)
exec_button.pack(side=tk.BOTTOM)
entry.pack(side=tk.LEFT)
open_button.pack(side=tk.LEFT)
ver_button.pack(side=tk.LEFT)

tk_root.mainloop()

log.close()

def main() -> str:
    return "End of application"
