#
# MIT Haystack Observatory
# Ben Welchman 06-23-2025
#

# --------------------------
#
# List of Functions:
#
#   open_port                
#   log_telemetry
#   ctrlc
#   print_help
#   parse_command_line
#   reduce
#   xor
#   eval_packet
#   add_cksum
#   write_max
#   get_reg_states
#   update_state
#   main_loop
#
# --------------------------

import serial
import time
import pathlib
import datetime
import signal
import sys
import argparse
import numpy as np

uart_port = '/dev/ttyUSB0'
uart_baud = 921600
telem_tag = '$COM,'

G_INITIALIZED = False

# Default register states for MAX7317ATE+ expanders
#
# !! FOR VLA EXPERIMENT 2025 EXT ANTENNA DEFAULT CHANGED !!
#
#                   P 0 1 2 3 4 5 6 7 8 9 
CTRL_MAIN_DEFAULT   = [1,1,1,1,0,0,0,1,1,0]     # OLD DEFAULT = [1,1,1,1,0,0,0,1,1,1]

#                   P 0 1 2 3 4 5 6 7 8 9 
CTRL_TX_DEFAULT    = [0,1,1,0,0,0,0,0,0,0]

#                   P 0 1 2 3 4 5 6 7 8 9 
CTRL_RX_DEFAULT    = [0,1,1,1,0,0,0,0,0,0]  


if not G_INITIALIZED:
  g_ctrl_main = CTRL_MAIN_DEFAULT.copy()

  g_ctrl_tx1 = CTRL_TX_DEFAULT.copy()
  g_ctrl_tx2 = CTRL_TX_DEFAULT.copy()

  g_ctrl_rx1 = CTRL_RX_DEFAULT.copy()
  g_ctrl_rx2 = CTRL_RX_DEFAULT.copy()
  g_ctrl_rx3 = CTRL_RX_DEFAULT.copy()
  g_ctrl_rx4 = CTRL_RX_DEFAULT.copy()

  g_reg_map_tx = {1: g_ctrl_tx1,
                  2: g_ctrl_tx2}

  g_reg_map_rx = {1: g_ctrl_rx1,
                  2: g_ctrl_rx2,
                  3: g_ctrl_rx3,
                  4: g_ctrl_rx4}
  
  G_INITIALIZED = True

def open_port(port, timeout):
  uart = serial.Serial(port, uart_baud, timeout=timeout)
  return uart

def log_telemetry(nmea_string):
  
  print("Message received: ", nmea_string)

def ctrlc(signal, frame):

        sys.exit(0)

def print_help():
  msg = "\nUsage:\n\n"
  msg = msg + ">> afe_service.py  [-h]  [-a <0/1>]  [-i <channel> <0,1>]  [t]\n\n"
  msg = msg + "  -h, --help         Show this help message and exit\n"
  msg = msg + "  -a, --antenna      GNSS Antenna Select <0/1> (Internal/External)\n"
  msg = msg + "  -i, --inputrf      RF Input select <channel> (1,2,3,4) <0/1/> (Internal/External)\n"
  msg = msg + "  -t, --telemetry    Log and print current telemetry\n"
  print(msg)

def parse_command_line():
    
    error_flag = False
    #
    # Note:
    #       Using the the same argparse variable for the 
    #       TCP port and port for serial device and baud rate 
    #       will work until both need to be supported at the same time
    #
    parser = argparse.ArgumentParser(add_help=False)

    parser.add_argument("-h", "--help", action="store_true", dest="help_flag")

    parser.add_argument("-a", "--antenna", choices=["0","1"])
   
    parser.add_argument("-i", "--inputrf", nargs=2, choices=["0","1","2","3","4"]) 

    parser.add_argument("-t", "--telemetry", action="store_true")


    (args,unknowns) = parser.parse_known_args()

    if args.help_flag:
      print_help()

    if (len(unknowns) != 0):
      print("Unknown options:", unknowns)
      print_help()
      error_flag = True

    return error_flag, args

def reduce(function, iterable, initializer=None):
    it = iter(iterable)
    if initializer is None:
        value = next(it)
    else:
        value = initializer
    for element in it:
        value = function(value, element)
    return value

def xor(a,b):
  return a^b

def eval_packet(packet,gen_cksm_f=False,debug_f=False):
     err_code = 0 
     ascii_checksum = 0 
     calculated_checksum = 0
     verbose_f = False

     if (debug_f):
       print("packet:", packet)
     # endif debug

     dollar_cnt = packet.count("$")
     star_cnt = packet.count("*")

     if (packet[0] == '$'):
       if (dollar_cnt != star_cnt):               # lost data
         if (verbose_f):
           print("err: be", dollar_cnt, star_cnt)   # be = begin end
         # endif extra info
         err_code = -1
       else:
         if (debug_f):
           pass #print("number of sentences in data:",dollar_cnt,"len=",len(packet))
         # endif
     else:                                       # pre-checked, should not happen
       print("data does not begin with '$'", dollar_cnt, star_cnt) 
       err_code = -2
     # end else error

     if (err_code == 0):
       packet = packet.strip("$\n")
       
       nmeadata, ascii_checksum = packet.split("*",1)
       if (debug_f):
           print("nmeadata=", nmeadata)
       # endif

       checksum = None
       if (not gen_cksm_f):              # checksum present, don't generate
         try:
           checksum = int(ascii_checksum,16)
         except Exception as eobj:       # encountered noise instead of checksum
           if (debug_f):
             print("err: ck(1)")             # ck = checksum
           # endif
           err_code = -3
         # end except
       else:
         pass   # no checksum to extract
       # else no checksum 
     # endif not error
      
     if (err_code == 0):
       calculated_checksum = reduce(xor, (ord(s) for s in nmeadata), 0)
       if (checksum == calculated_checksum):
         if (debug_f):
           print("success,checksum=",hex(checksum),                                   
                                     hex(calculated_checksum),
                                     ascii_checksum)
         # endif debug
       else:
         if (not gen_cksm_f):   # compare generated vs read
           err_code = -4
           if (debug_f):
             print("err: ck(2)") 
           # endif 
           if (debug_f or verbose_f):
             print("The NMEA cksum != calc cksum:(bin,calc,read)",
                                       hex(checksum),                                   
                                       hex(calculated_checksum),
                                       ascii_checksum)
           # endif debug
         else:
           checksum = calculated_checksum
           if (debug_f):
             print("synthetic checksum=",hex(checksum))
           # endif debug
         # end else use manufactured checksum
       # end else bad checksum
     # endif parse passes checks  
     return err_code, ascii_checksum, calculated_checksum

def add_cksum(pkt_in):

  packet_out = ""

  err_code, dck, cck = eval_packet(pkt_in,gen_cksm_f=True,debug_f=False)
  if (err_code != 0):
    err_f = True
  # endif

  ck_hex = hex(cck)             # to string
  ck_split = ck_hex.split('x')  # split at hex identifier
  ck_str = ck_split[1].upper()  # upper case
  if ( len(ck_str) == 1):       # if single digit
    ck_str = "0" + ck_str       # then prefix with '0', ex: "0A"
  # endif single digit
    
  packet_out = pkt_in + ck_str

  return packet_out

def write_max(block, channel, index, bit):

  msg_draft = "$PMIT"

  if block == "main":
    msg_draft += "MAX"

  elif block == "tx":
    msg_draft += "XT"
    msg_draft += str(channel)

  elif block == "rx":
    msg_draft += "XR"
    msg_draft += str(channel)

  else:
    return 0
  
  msg_draft += "," + str(index) + "," + str(bit) + "*"
  msg = add_cksum(msg_draft)
  uart.write(msg.encode())
  print("writing nmea msg: " + msg)


  while 1:
    line = uart.readline()
    if line:
        print(line.decode().strip())
        return



def get_reg_states():
   
  main_reg = []
  
  tx1_reg = []
  tx2_reg = []

  rx1_reg = []
  rx2_reg = []
  rx3_reg = []
  rx4_reg = []

  line = None

  msg_draft = "$PMITMA?*"
  msg = add_cksum(msg_draft)
  uart.write(msg.encode())

  while line is None:
    line = uart.readline()
    if line:
      line = line.decode()
      print(line)
      for i in range(14, 33, 2):
        main_reg.append(line[i])

  line = None

  msg_draft = "$PMITXT1?*"
  msg = add_cksum(msg_draft)
  uart.write(msg.encode())

  while line is None:
    line = uart.readline()
    if line:
      line = line.decode()
      for i in range(14, 33, 2):
        tx1_reg.append(line[i])

  line = None

  msg_draft = "$PMITXT2?*"
  msg = add_cksum(msg_draft)
  uart.write(msg.encode())

  while line is None:
    line = uart.readline()
    if line:
      line = line.decode()
      for i in range(14, 33, 2):
        tx2_reg.append(line[i])

  line = None
  
  msg_draft = "$PMITXR1?*"
  msg = add_cksum(msg_draft)
  uart.write(msg.encode())

  while line is None:
    line = uart.readline()
    if line:
      line = line.decode()
      for i in range(14, 33, 2):
        rx1_reg.append(line[i])

  line = None

  msg_draft = "$PMITXR2?*"
  msg = add_cksum(msg_draft)
  uart.write(msg.encode())

  while line is None:
    line = uart.readline()
    if line:
      line = line.decode()
      for i in range(14, 33, 2):
        rx2_reg.append(line[i])

  line = None

  msg_draft = "$PMITXR3?*"
  msg = add_cksum(msg_draft)
  uart.write(msg.encode())

  while line is None:
    line = uart.readline()
    if line:
      line = line.decode()
      for i in range(14, 33, 2):
        rx3_reg.append(line[i])

  line = None

  msg_draft = "$PMITXR4?*"
  msg = add_cksum(msg_draft)
  uart.write(msg.encode())

  while line is None:
    line = uart.readline()
    if line:
      line = line.decode()
      for i in range(14, 33, 2):
        rx4_reg.append(line[i])

  reg_list = main_reg, tx1_reg, tx2_reg, rx1_reg, rx2_reg, rx3_reg, rx4_reg

  return(np.stack(reg_list))

def update_state(args):

  if args.antenna:
    antenna_int = int(args.antenna)
    write_max("main", -1, 5, antenna_int)

  if args.inputrf:
    channel, bit = args.inputrf
    channel_int = int(channel)
    bit_int = 1 - int(bit)
    write_max("rx", channel_int, 1, bit_int)

def main_loop():

  reg_states = get_reg_states()
  print("Reg States:")
  print(reg_states)

  while True:
    line = uart.readline()
    if line:
      print(line.decode().strip())
  
  #telemetry = get_telemetry()

  #log_telemetry(reg_state, telemetry)

  #print(telemetry)

if __name__ == '__main__':

    err_f = False

    signal.signal(signal.SIGINT, ctrlc) # for control c

    try:
      uart = open_port(uart_port, 1)
      print("UART initialized at", uart_port)
    except:
      print("Failed to initialize UART")
      sys.exit()

    error_flag, args = parse_command_line() # parse command line options

    if (error_flag):
      sys.exit()

    print(args)

    if (len(sys.argv) > 1):
      update_state(args)

    main_loop()



    #print(g_ctrl_afe)
    #print(g_ctrl_rx1)