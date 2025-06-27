#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
  afec,ds.py    - afe test harness, based on pimcmds3.py
  mit haystack observatory
  rps 1/27/2025
"""
#
#-------------------------------------------------------------------------
#
"""

  The afe command set:
    use -h for basic help
    use -? to print extra help (arparser may be used differently per command)

classes/functions:
   SystemCmd                   - gets underlying error response on sys.command
   class UserDefinedException  - trying to clean up/wrap error handling
   class AlarmException        - part of non-blocking kbhit
   alarmHandler                - part of non-blocking kbhit
   signal_handler              - ^C handler
   reduce                      - fancy way to apply function to each byte in packet
   xor                         - xor operator as a function
   eval_packet                 - calculate nmea checksum, versus received checksum
   add_cksum                   - calculate nmea checksum, add to packet without checksum
   String2List                 - does what it says
   char_dump                   - uart diagnostic
   hex_print                   - uart diagnostic
   getFiles                    - more generic glob
   isInlist                    - returns true if index found
   indexInlist                 - returns true if index found and also index
   countX                      - returns count
   getch                       - blocking/nonblocking read of stdin 
   history_input               - history for arrow keys
   print_help                  - help for commands
   getUnixTime                 - gmtime plus gmtime in pieces
   getUnixTimeString           - gm time to formatted gm time
   append_file                 - saved responses to file
   parse_command_line          - command line parameters
   parse_afe_send              - parse afe send command line parameters
   parse_settime               - parse afe set time command line parameters
   parse_TelemRate             - parse afe set rate command line parameters
   parse_setmag                - parse afe setmag command line parameters
   parse_setimu                - you should get the picture by now
   parse_setmax                -
   parse_script                - parse afe script command line parameters
   utcNow                      - time utility
   class afecmds:
     __init__                  - init
     read_accumulate           - part of afe_send, listens until other side goes quiet
     afe_read                  - read_accumulate followed by buffer clear
     afe_send                  - afe send
     device_send               - afe device driver wrapper
     ace_send                  - generic afe send to device
     isConnected               - generic, is serial or ether comms connected?
     Cleanup                   - background task cleanup before comms close
     myClose                   - generic, allows for serial or ether comms close
     myConnect                 - generic, serial or ether comms connect
     select_dev                - offers user choice 
     init_ser                  - initialize serial comms
     print_retry               - work around for blocking i/o
     close_ser                 - close serial comms
     getSerialData             - read serial port
     myConnectSocket           - ethernet connect
     myConnectUsart            - usart connect
     start_threads             - wraps start for background tasks
     run_script_once           - run script once
     do_script_cmd             - handle the script command 
     afeHandler                - calls read_accumulate inside thread, handles response
     send_time_query           - send NMEA time parameters query
     setTime                   - read OS' date, set 432 date and time, readback, compare
     verifyTime                - reads time, compares against this machine's time
     setTelemRate              - sets the telemetry periodic rate
     setMag                    - set the magnetometer
     setImu                    - set the imu
     setMax                    - set the MAX7317 chip (7 different MAX chips)
     enable_logging            - enable logging
     process_line              - vector for commands
     uart_listen               - socket_listen removed, see KeysightPowerMeter.py
     main_loop
 class ListenThread
      __init__
      run
      pause                     - pause listening
      resume                    - resume listening
      read                      - read listen buffer -> 
      clear                     - clear listen buffer
      shutdown
 class peThread
      __init__
      run
      is_paused
      pause                     - pause 
      resume                    - resume 
      shutdown
"""
#
#-------------------------------------------------------------------------
#
USE_DRF = False             # simplifies porting 

import signal               # for ^C and non-blocking kbhit
import sys
import os
import socket               # TCP/IP
import serial               # usart
import glob                 # finds files 
import time                 # timestamps and timeouts
import datetime as dt       # date formatting
import calendar             # time conversion to match dt
from dateutil import tz     # time zone info
import argparse             # command line parsing
import threading
from threading import Lock  # lock
import traceback            # for debugging
import shlex                # custom parsing 
import logging   
import subprocess           # better than os.system()
import termios              # for non-blocking terminal io
import fcntl                # for non-blocking terminal io
import platform             # what version of python is this?
import select               # used in getch
#
# the python version test is increasingly irrelevant
# due to almost universal use of version 3 but may come back 
# into relevance because of future language changes
#
version = platform.python_version()    # checking for python3
print("python version:",version)
#
#-------------------------------------------------------------------------
#
#              GLOBALS
#
#-------------------------------------------------------------------------
#
USE_SOCKET      = False     # True for Socket, False for Usart
BAUD_RATE       = 9600
DEFAULT_WAIT    = 2         # listen wait on the outside context
MAX_TRIES       = 800      # listen wait on the inside context, flushes partials on timeout
LABEL_SZ        = 32
CMDSTR_SZ       = 128

TELEM_RATE_DEF    = 1     # DEF=default, MIN, MAX should stay constant
MAG_CCR_MIN       = 50
MAG_CCR_MAX       = 400
MAG_CCR_DEF       = 200
MAG_UPDR_MIN      = 146
MAG_UPDR_MAX      = 159
MAG_UPDR_DEF      = 150

g_response_f = False  # issue with sharing live data between objects
                                  
afeObj = None               # handle of instantiation of afe class 


g_history = []              # up/down arrow
g_hist_idx  = -1
g_verbose   = False

g_odrList = ["ODR_OFF", "ACC_1_6_HZ_ULP", "GYR_6_5_HZ_LP",  "ODR_12_5_HZ_LP", "ODR_26_HZ_LP",
                        "ODR_52_HZ_NP",  "ODR_104_HZ_NP",  "ODR_208_HZ_HP",   "ODR_416_HZ_HP", 
                        "ODR_833_HZ_HP", "ODR_1_66_KHZ_HP","ODR_3_33_KHZ_HP", "ODR_6_66_KHZ_HP"]


#
#-------------------------------------------------------------------------
#
#              FUNCTIONS and CLASSES
#
#-------------------------------------------------------------------------
#
# note: this did not work, can delete
#
class ErrorCatchingArgumentParser(argparse.ArgumentParser):
    def exit(self, status=0, message=None):
        if status:
            raise Exception(f'Exiting because of an error: {message}')
        exit(status)
# end ErrorCatchingArgumentParser
#
#------------------------------------------------------------------------------------
#
def SystemCmd(cmd,verbose_f=False,shell_f=True):
  err_f = False
  output_str  = ""

  if (verbose_f):
    print("cmd=",cmd)
  try:
     theOutput = subprocess.check_output(cmd,stderr=subprocess.STDOUT,shell=shell_f)
  except Exception as eobj:
     print("Exception: cmd: %s"%(cmd),eobj)
     eobj_output = repr(eobj)
     if (eobj_output != ""):
       print(eobj_output)
     err_f = True
     output_str = eobj_output
  else:
    output_str = theOutput.decode("utf-8")  # new for python3, else b'<text>' 
    if (verbose_f):
      print(output_str)
    # end if 
  return err_f, output_str
# end SystemCmd
#
#-------------------------------------------------------------------------
#
# frank prefers exceptions rather than going back up the call stack
# however exceptions are very much like gotos disfavored by djykstra
#
class UserDefinedException(Exception):
     def __init__(self, value):
         self.value = value
     def __str__(self):
         return repr(self.value)
# end UserDefinedException
#
#-------------------------------------------------------------------------
#
class AlarmException(Exception): #  used in non-blocking kbhit
    pass
# end AlarmException
#
#-------------------------------------------------------------------------
#
def alarmHandler(signum, frame):  #  used in non-blocking kbhit
    raise AlarmException
# end alarmHandler
#
#---------------------------------------------------------
#
def getFiles(srcPath,globList,all_f=True,startSec=None,endSec=None,):
  fullList = []
  retList = []
  ii = 0
  verbose_f = True
  while ii < len(globList):
    fullList = fullList + glob.glob(srcPath+globList[ii])
    ii = ii + 1
  # end while
  if (all_f):
     retList = fullList
  else:
    ii = 0
    jj = 0
    shortList = []
    now = time.time()
  
    while (ii < len(combinedList)):
      fn = combinedList[ii]
      modTime = os.path.getmtime(fn)      # time in seconds
      if ((modTime >= startSec) and (modTime < endSec)):
         shortList = shortList + [fn]
         jj = jj + 1
      # endif
      ii = ii + 1
    #end while
    retList = shortList
  # end else 
  if (len(retList)==0):
    if (verbose_f):
      print("No files found at:",srcPath,"using filter:",globList)
  return retList
# end getFiles
#
#-------------------------------------------------------------------------
#
def signal_handler(signal, frame):
        global afeObj

        print('You pressed Ctrl+C!')
        if (afeObj != None):            # belt and suspenders
           afeObj.Cleanup()             # belt and suspenders
        sys.exit(0)
# end signal_handler 
#
# iterate function across list
#
def reduce(function, iterable, initializer=None):
    it = iter(iterable)
    if initializer is None:
        value = next(it)
    else:
        value = initializer
    for element in it:
        value = function(value, element)
    return value
# end reduce
#
# -----------------------------------------------------------------
#
# nmea checksum function
#
def xor(a,b):
  return a^b
# end xor
#
# ----------------------------------
#
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
# end eval_packet
#
# -----------------------------------------------------------------
#
def add_cksum(pkt_in,debug_f=False):
  err_f = False
  packet_out = ""
  
  if (debug_f):
    print("pre_checksum=", pkt_in)
  # endif debug
  #
  # calculate nmea checksum, add checksum to packet
  #
  err_code, dck, cck = eval_packet(pkt_in,gen_cksm_f=True,debug_f=debug_f)
  if (err_code != 0):
    err_f = True
  # endif

  if (not err_f):
    ck_hex = hex(cck)             # to string
    ck_split = ck_hex.split('x')  # split at hex identifier
    ck_str = ck_split[1].upper()  # upper case
    if ( len(ck_str) == 1):       # if single digit
      ck_str = "0" + ck_str       # then prefix with '0', ex: "0A"
    # endif single digit
    
    packet_out = pkt_in + ck_str

  # endif not error

  return err_f, packet_out

# end add_cksum
#
#------------------------------------------------------------------------------
#
def String2List(word):
    return [char for char in word]
# end String2List
#
#------------------------------------------------------------------------------
#
def isInlist(inlist,    # the list
             value):    # the item to look for
  try:
    inlist.index(value)
  except Exception:         # any exception is invalid comparison, return false
    return False
  return True

# end isInlist
#
#------------------------------------------------------------------------------
#
def indexInlist(inlist,    # the list
                value):    # the item to look for
  match_f = False          # default
  index   = -1             # default

  try:
    index = inlist.index(value)
  except Exception:         # any exception is invalid comparison
    pass
  else:
    match_f = True

  return match_f, index

# end indexInlist
#
#------------------------------------------------------------------------------
#
def countX(lst, x):
    count = 0
    for ele in lst:
        if (ele == x):
            count = count + 1
    return count
# end countX
#
#------------------------------------------------------------------------
#
# building block for history on up/down arrow
#
# https://stackoverflow.com/questions/510357/python-read-a-single-character-from-the-user
# https://stackoverflow.com/questions/2408560/non-blocking-console-input
#
def getch(block_f=True,poll_cnt=1500):
  fd = sys.stdin.fileno()

  oldterm = termios.tcgetattr(fd)
  newattr = termios.tcgetattr(fd)
  #newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
  #termios.tcsetattr(fd, termios.TCSANOW, newattr)

  newattr[3] = newattr[3] & ~(termios.ECHO | termios.ICANON) # lflags
  newattr[6][termios.VMIN] = 0  # cc
  newattr[6][termios.VTIME] = 0 # cc
  termios.tcsetattr(sys.stdin, termios.TCSAFLUSH, newattr)

  oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
  fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

  try:        
    while (poll_cnt > 0):            
      try:
       if (block_f):
          c = sys.stdin.read(1)    # read is blocking
          if (len(c) != 0):
            break
       else:                       # select read or false
          poll_cnt = poll_cnt - 1
          if (select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])):  
            c = sys.stdin.read(1)
            break
          else:
            c = False
      except IOError: pass
    # end while
  finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
  return c
# end getch
#
#-------------------------------------------------------------------
#
def history_input(prompt):
  global g_history
  global g_hist_idx

  updown_f = False
  updown_dr = 0
  at_top_f = False

  line = ""
  try:
    print("\n"+ prompt, end=" ")  # has taken a hit from thread
  except Exception as eobj:
    pass
     
  while (True):   
   
   inp = getch()              # attempt to read char

   num = ord(inp)             # convert char to number
   if (num == 0x0a):          # end of line     
       break                    # exit while loop

   #print("0x%x "%(num))   # debug

   bs_f = False                          # not backspace
   if (num == 0x08) or (num == 0x7f):    # if backspace
     if (len(line) > 0):                 #   if room to back up
       line = line[:-1]                  #   then back up
       print("\r"+prompt+line+" ",end=" ")       # erase one char
       print("\r"+prompt+line,end=" ")           # reprint line       
       bs_f = True                       # set flag for later
     else:                               # backspace python weirdness
       line = ""
     # end else recreate empty line
   # endif backspace
   
   if (num == 0x1b):         # if escape
     num2 = ord(getch())     #  then check for up / down arrow sequence 
     if (num2 == 0x5b):      #  if 2nd of updown triplet
       num3 = ord(getch())   #    then get last of triplet
       if (num3 == 0x41):    #    if up arrow
         updown_f = True     #      flag updown sequence
         updown_dr = 1       #      flag up
       else: 
         if (num3 == 0x42):  # else down arrow
            updown_f = True  #   flag updown sequence
            updown_dr = -1   #   flag down
         # endif down
       # endif not up
     # endif up or down

     if (updown_f):                        # if up or down
       if (updown_dr >= 0):                #   if up (backwards in history)
         at_top_f = False                  #     top is false (forwards in time)
            
         if (len(g_history) > 0):
           line = g_history[g_hist_idx]    #   read line from history    
           print("%c[2K"%(27),end=" ")   # clear line           
           print("\r" + prompt + g_history[g_hist_idx],end=" ") # print
         # end if history
         g_hist_idx = g_hist_idx - 1             #     move backwards in history
         if (g_hist_idx < 0):
           g_hist_idx = 0                     #     but not before beginning
  
       # endif up 

       else:                              #  else down (forwards in history)  
         g_hist_idx = g_hist_idx + 1           # advance
         if (g_hist_idx >= len(g_history)):    #    if at or above top
           lastLen = len(g_history)
           g_hist_idx = lastLen -1      #    then backup pointer to last
           at_top_f = True                       #    set top flag
           line = ""                             # erase line  
           if (lastLen > 0):                     # if history  
             blankLine = " " * lastLen           # present a blank line
             print( "%c[2K"%(27))                # clear line
             print("\r" + prompt + blankLine + "\r" + prompt,end=" ")
           # endif history exists         
         else:                             
           at_top_f = False                      # clear at-top 
           line = g_history[g_hist_idx]          #    read line from history
           print("%c[2K"%(27),end=" ")         # clear line 
           print("\r" + prompt + g_history[g_hist_idx],end=" ") # from history
         # end else not at top of list

     # endif updown pressed
   # endif escape sequence

   if (updown_f):                 # if using history
    updown_f = False              # then clear history flag
   else: 
     if (not bs_f):               # if not history or backing up
       line = line + inp          #   build line      
       print("\r"+ prompt + line,end=" ")   
     # endif not backspace
   # end else not updown

  # end while                 
  
  if (len(line) > 0):           # don't store blank lines into history
    g_history.append(line)       # append command
    g_hist_idx = len(g_history) -1  # point to line
  # endif something on line
  
  return line
     
# end history_input
#
# -------------------------------------------------------------------------
#
def char_dump(theString):

  ii = 0
  while (ii < len(theString)):
   print( ii, hex(ord(theString[ii])),theString[ii])
   sys.stdout.flush()
   ii = ii + 1
  # end while
# end char_dump
#
# -------------------------------------------------------------------------
#
def hex_print(inList,p_f=True):

  outList = []
  ii = 0
  while (ii < len(inList)):

    outList = outList + [hex(inList[ii])]

    ii = ii + 1

  # end while

  if (p_f):
    print(outList)
  # endif print

  return outList

# end hex_print
#
# -------------------------------------------------------------------------
#
def print_help():

  msg = "\nafe open <ipaddr> <port> [-w<seconds> ] # defaults to 30 sec, -1 = wait forever\n"
  msg = msg + "afe open <dev> <baud> [ -w<seconds> ]\n"
  msg = msg + "afe close\n"

  # -----------------------------------------------------------------------
  # to pull out the options, enter the -h command in the particular command
  # for example:
  #
  # > afe setrate -h
  #
  # ------------------------------------------------------------

  msg = msg + "afe settime [-h] [-i] [-g] [-e] [-p] [-n] [-t [TIME]]\n"
  msg = msg + " optional arguments:\n"
  msg = msg + "  -h, --help                show this help message and exit\n"
  msg = msg + "  -i, --immediate           set time immediate\n"
  msg = msg + "  -g, --gnss                time source gnss (nmea)\n"
  msg = msg + "  -e, --external            time source external (pps, immediate)\n"
  msg = msg + "  -p, --pps                 time source epoch (pps)\n"
  msg = msg + "  -n, --nmea                time epoch (nmea)\n"
  msg = msg + "  -t [TIME], --time [TIME]  UTC time in seconds\n"
  msg = msg + "  -?, --query               query time parameters\n"

  msg = msg + "afe verifytime\n"

  msg = msg + "afe setrate [-h] [-m] [-i] [-hk] [-a] [-s [SEC]]\n"
  msg = msg + " optional arguments:\n"
  msg = msg + " -m, --magnetometer    set magnetometer rate\n"
  msg = msg + " -i, --imu             set imu rate)\n"
  msg = msg + " -hk, --housekeeping   set housekeeping rate\n"
  msg = msg + " -a, --all             set mag, imu, hk to the same rate\n"
  msg = msg + " -s [SEC], --sec [SEC]\n"
  msg = msg + "  -?, --query           query rate parameters\n"

  msg = msg + "afe setmag -h usage: afecmds.py [-h] [-ccr [CYCLE_COUNT]] [-updr [UPDATE_RATE]] [-?]\n"
  msg = msg + " optional arguments:\n"
  msg = msg + "  -h, --help            show this help message and exit\n"
  msg = msg + "  -ccr [CYCLE_COUNT], --cycle_count [CYCLE_COUNT]\n"
  msg = msg + "                        cycle count register range 50 .. 400\n"
  msg = msg + "  -updr [UPDATE_RATE], --update_rate [UPDATE_RATE]\n"
  msg = msg + "                        update rate register range 146 .. 159\n"
  msg = msg + "  -?, --query           query mag parameters\n"

  msg = msg + "afe send \"<string>\" [-h] [-i ITER] [-d DELAY] [-r] [-e EXPECT] [-a APPEND] [-l [LISTEN]]\n"
  msg = msg + " optional arguments:\n"
  msg = msg + "  -h, --help                      show this help message and exit\n"
  msg = msg + "  -i ITER, --iter ITER            iterations.\n"
  msg = msg + "  -d DELAY, --delay DELAY         seconds delay between iterations.\n"
  msg = msg + "  -r, --response                  response expected.\n"
  msg = msg + "  -e EXPECT, --expect EXPECT      expected response.\n"
  msg = msg + "  -a APPEND, --append APPEND      append response to file.\n"
  msg = msg + "  -l [LISTEN], --listen [LISTEN]  listen seconds.\n"
  
  msg = msg + "scr <script-file>\n"
  msg = msg + "exit | quit\n"
  print(msg)

# end print_help 
#
# -------------------------------------------------------------------------
#
def getUnixTime(unixTime=None):

   if (unixTime == None):              # if time not passed in
     unixTime = time.time()            # get timestamp
   # endif time not passed in
   
   tm_tup = time.gmtime(unixTime)      # this is a tuple

   year   = '%04d'%(tm_tup[0])         # extract time pieces
   month  = '%02d'%(tm_tup[1])
   day    = '%02d'%(tm_tup[2])
   hour   = '%02d'%(tm_tup[3])
   minute = '%02d'%(tm_tup[4])
   second = '%02d'%(tm_tup[5])

   
   return (unixTime, year, month, day, hour, minute, second)
# end getUnixTime
#
# -------------------------------------------------------------------------
#
def getUnixTimeString(unixTime=None):
  ts_str = ""
  
  ts_val = getUnixTime(unixTime)
  
  ts_str = '%s-%s-%sT%s:%s:%s' % (
                    ts_val[1],
                    ts_val[2],
                    ts_val[3],
                    ts_val[4],
                    ts_val[5],
                    ts_val[6]) 
  return ts_str
# end getUnixTimeString
#
#-------------------------------------------------------------------
#
def append_file(filename,command,response):
   #
   # open/append file 
   # write timestamp, command, response
   # close file
   #
   err_f = False

   try:
     afd = open(filename, 'a')      # open append
   except IOError:
      emsg= '*** Cannot open: %s ***' % filename
      print(emsg)
      err_f = True
   else:
     ts_str = getUnixTimeString()
     logData = ts_str + ", " + command + ", " + response + "\n"

     try:
       afd.write(logData)           # write file
     except IOError:
       emsg= '*** Cannot write: %s ***' % filename
       print(emsg)
       err_f = True
     else:
       afd.close()                  # close file

   return err_f
# end append_file 
#
#-------------------------------------------------------------------
#
def parse_command_line():
    err_f = False
    #
    # Note:
    #       Using the the same argparse variable for the 
    #       TCP port and port for serial device and baud rate 
    #       will work until both need to be supported at the same time
    #
    parser = argparse.ArgumentParser()

    d_helpStr = "--debug (-d):\t  run in debug mode and not service context."
    parser.add_argument("-d", "--debug",action='store_true', help= d_helpStr)
   
    I_helpStr = "--ipaddr (-I):\t  IP address or path to serial device"
    parser.add_argument("-I", "--ipaddr",help= I_helpStr)

    m_helpStr = "--metadata (-m):  metadata path"
    parser.add_argument("-m", "--metadata", help= m_helpStr)

    P_helpStr = "--port (-P):\t  port number or baud rate"
    parser.add_argument("-P", "--port", type=int, help= P_helpStr)

    s_helpStr = "--script (-s):\t  script file"
    parser.add_argument("-s", "--script", help= s_helpStr)

    v_helpStr = "--verbose (-v):\t  adds debug output and additional detail."  
    parser.add_argument("-v", "--verbose",action='store_true', help=v_helpStr)

    w_helpStr = "--wait (-w):\t  delay between commands"
    parser.add_argument("-w", "--wait", action='store_true', help= w_helpStr)

    extra_helpStr = "--extra_help (-?):\t  the argparser may be used differently per command"
    parser.add_argument("-?", "--extra_help", action='store_true', help= extra_helpStr)

    (args,unknowns) = parser.parse_known_args()

    if (len(unknowns) != 0):
      print("unknown options:", unknowns)
      print()
      print(parser._option_string_actions['--debug'].help)       # 1
      print(parser._option_string_actions['--ipaddr'].help)      # 2
      print(parser._option_string_actions['--metadata'].help)    # 3
      print(parser._option_string_actions['--port'].help)        # 4
      print(parser._option_string_actions['--script'].help)      # 5
      print(parser._option_string_actions['--wait'].help)        # 6
      print(parser._option_string_actions['--verbose'].help)     # 7
      print(parser._option_string_actions['--extra_help'].help)  # 8

      print()
      print_help()
      err_f = True

    return err_f,args

# end parse_command_line
#
#-------------------------------------------------------------------------
#
# ArgumentParser 
#  Note:
#    if for boolean 'store_true', then
#    arg for that param will never return None.
#
# Note:
#    if you want a parameter as a default but also allow passing the option
#    then also const and nargs='?'
#
def parse_afe_send(theLine):
    #
    err_f = False
    exit_f = False
    args = None   # if -h then return args None

    parser = argparse.ArgumentParser() # ErrorCatchingArgumentParser(argparse.ArgumentParser) # argparse.ArgumentParser()

    parser.add_argument("-i", "--iter", type=int, 
                                      default=1, help="iterations.")
                           
    parser.add_argument("-d", "--delay",type=int, default=0, 
                                       help="seconds delay between iterations.")

    parser.add_argument("-r", "--response",action='store_true',
                        help="response expected.")

    parser.add_argument("-e", "--expect",  
                              default="", help="expected response.")

    parser.add_argument("-a", "--append",  
                              default="", help="append response to file.")

    #
    # adding listen without wait argument
    # default delay determine stochastically
    #
    theWait = 3
    parser.add_argument("-l", "--listen",type = int, 
                                         default = theWait,
                                         const = theWait,
                                         nargs='?',
                                         help = "listen seconds.")
    try:
      (args,unknowns) = parser.parse_known_args(theLine)
    except SystemExit as eobj:                            # returned from -h, --help
      exit_f = True     
    except Exception as eobj:                             # haven't seen this path trigger yet
      err_str = "options exception: 1"   # which is why the result will be a raise
      err_code = -1
      eobj_str = repr(eobj)
      errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
      raise UserDefinedException(errDict)  
    else:
        theLast = len(unknowns)
        ii = 0
        while (ii < theLast):
          if (unknowns[ii][0] == '-'):
            print("unknown option=",unknowns[ii])
            err_f = True    # keep going, print all
          ii = ii + 1
        # end while

        if (err_f):
          exit_f = True
    # end else

    return exit_f, args
# end parse_afe_send
#
#-------------------------------------------------------------------------
#
# ArgumentParser 
#  Note:
#    if for boolean 'store_true', then
#    arg for that param will never return None.
#
# Note:
#    if you want a parameter as a default but also allow passing the option
#    then also const and nargs='?'
#
def parse_settime(theLine):
    err_f = False
    args = None   # if -h then return args None
    exit_f = False

    parser = argparse.ArgumentParser()

    parser.add_argument("-i", "--immediate", action='store_true', 
                               help="set time immediate")

    parser.add_argument("-g", "--gnss", action='store_true', 
                               help="time source gnss (nmea)")

    parser.add_argument("-e", "--external", action='store_true', 
                               help="time source external (pps, immediate)")

    parser.add_argument("-p", "--pps", action='store_true', 
                               help="time source epoch (pps)")

    parser.add_argument("-n", "--nmea", action='store_true', 
                               help="time epoch (nmea)")
                         
    parser.add_argument("-t", "--time",type=int, nargs='?',
                               help="UTC time in seconds")

    parser.add_argument("-?", "--query", action='store_true', 
                               help="query parameters") 
    try:
      (args,unknowns) = parser.parse_known_args(theLine)
    except SystemExit as eobj:                            # returned from -h, --help
      exit_f = True     
    except Exception as eobj:                             # haven't seen this path trigger yet
      err_str = "options exception: 3"   # which is why the result will be a raise
      err_code = -1
      eobj_str = repr(eobj)
      errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
      raise UserDefinedException(errDict)  
    else:
        theLast = len(unknowns)
        ii = 0
        while (ii < theLast):
          if (unknowns[ii][0] == '-'):
            print("unknown option=",unknowns[ii])
            err_f = True    # keep going, print all
          ii = ii + 1
        # end while

        if (err_f):
          exit_f = True
    # end else

    return exit_f, args
# end parse_settime
#
#-------------------------------------------------------------------------
#
def parse_TelemRate(theLine):
    global TELEM_RATE_DEF
    """
    afe setrate -m  --magnetometer (t/f)
                -i  --imu          (t/f)
                -hk --housekeeping (t/f)
                -a  --all          (t/f)
                -s  --seconds      (int)
    """
    #
    err_f = False
    exit_f = False
    args = None   # if -h then return args None

    parser = argparse.ArgumentParser()

    parser.add_argument("-m", "--magnetometer", action='store_true', 
                               help="set magnetometer rate")

    parser.add_argument("-i", "--imu", action='store_true', 
                               help="set imu rate)")

    parser.add_argument("-hk", "--housekeeping", action='store_true', 
                               help="set housekeeping rate")

    parser.add_argument("-a", "--all", action='store_true', 
                               help="set mag, imu, hk to the same rate")

    parser.add_argument("-s", "--sec", type=int, nargs='?', const=TELEM_RATE_DEF,
                               help="periodic interval, range 0 (off) .. 60")

    parser.add_argument("-?", "--query", action='store_true', 
                               help="query parameters")                     
    try:
      (args,unknowns) = parser.parse_known_args(theLine)
    except SystemExit as eobj:                            # returned from -h, --help
      exit_f = True     
    except Exception as eobj:                             # haven't seen this path trigger yet
      err_str = "options exception: 5"   # which is why the result will be a raise
      err_code = -1
      eobj_str = repr(eobj)
      errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
      raise UserDefinedException(errDict)  
    else:
        theLast = len(unknowns)
        ii = 0
        while (ii < theLast):
          if (unknowns[ii][0] == '-'):
            print("unknown option=",unknowns[ii])
            err_f = True    # keep going, print all
          ii = ii + 1
        # end while

        if (err_f):
          exit_f = True
    # end else

    return exit_f, args
# end parse_TelemRate
#
#-------------------------------------------------------------------------
#
def parse_setmag(theLine):
    global MAG_CCR_MIN
    global MAG_CCR_MAX
    """
    afe setmag -ccr  --cycle_count  (int)
               -updr  --update_rate  (int)
                -?  --query        (t/f)
    """
    err_f = False
    exit_f = False
    args = None   # if -h then return args None

    parser = argparse.ArgumentParser()

    parser.add_argument("-ccr", "--cycle_count", type=int, nargs='?', const=MAG_CCR_DEF,
                               help="cycle count register range %d .. %d"%(MAG_CCR_MIN, MAG_CCR_MAX))

    parser.add_argument("-updr", "--update_rate", type=int, nargs='?', const=MAG_UPDR_DEF,
                               help="update rate register range %d .. %d"%(MAG_UPDR_MIN, MAG_UPDR_MAX))

    parser.add_argument("-?", "--query", action='store_true', 
                               help="query parameters")

    try:
      (args,unknowns) = parser.parse_known_args(theLine)
    except SystemExit as eobj:                            # returned from -h, --help
      exit_f = True     
    except Exception as eobj:                             # haven't seen this path trigger yet
      err_str = "options exception: 7"   # which is why the result will be a raise
      err_code = -1
      eobj_str = repr(eobj)
      errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
      raise UserDefinedException(errDict)  
    else:
        theLast = len(unknowns)
        ii = 0
        while (ii < theLast):
          if (unknowns[ii][0] == '-'):
            print("unknown option=",unknowns[ii])
            err_f = True    # keep going, print all
          ii = ii + 1
        # end while

        if (err_f):
          exit_f = True
    # end else

    return exit_f,args
# end parse_setmag
#
#-------------------------------------------------------------------------
#
def parse_setimu(theLine):
    global g_odrList
    """
    > afe setimu -a ODR_416_HZ_HP -g ODR_416_HZ_HP -hpa          -ulpa         -lpg
                 --acc           --gyro            --hi_perf_acc --ultralowacc --low_pwr_gyro
                 -?  --query        (t/f)
    """
    err_f = False
    exit_f = False
    args = None   # if -h then return args None
    

    parser = argparse.ArgumentParser()

    parser.add_argument("-a", "--acc",
                               help="acc output data rate, select one of: %s"%(g_odrList))

    parser.add_argument("-g", "--gyro",
                               help="gyro output data rate, select one of: %s"%(g_odrList))

    parser.add_argument("-hpa", "--hi_perf_acc", action='store_true',
                          help="enable accelerometer high performance")

    parser.add_argument("-lpg", "--low_pwr_gyro", action='store_true',
                          help="enable low power gyro")

    parser.add_argument("-ulpa", "--ultralowacc", action='store_true',
                          help="enable ultra low power accelerometer")

    parser.add_argument("-?", "--query", action='store_true',
                               help="query parameters")

    try:
      (args,unknowns) = parser.parse_known_args(theLine)
    except SystemExit as eobj:                            # returned from -h, --help
      exit_f = True     
    except Exception as eobj:                             # haven't seen this path trigger yet
      err_str = "options exception: 7"   # which is why the result will be a raise
      err_code = -1
      eobj_str = repr(eobj)
      errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
      raise UserDefinedException(errDict)  
    else:
        theLast = len(unknowns)
        ii = 0
        while (ii < theLast):
          if (unknowns[ii][0] == '-'):
            print("unknown option=",unknowns[ii])
            err_f = True    # keep going, print all
          ii = ii + 1
        # end while

        if (err_f):
          exit_f = True
    # end else

    return exit_f,args
# end parse_setimu
#
#-------------------------------------------------------------------------
#
def parse_setmax(theLine):
    global g_odrList
    """
    > afe setmax -tx [1|2] -rx[1|2|3|4] -i [0..9] -b 0|1|X|x,...]
                 --tx      --rx         --index   --bits 
                 -?  --query        (t/f)
    """
    err_f = False
    exit_f = False
    args = None   # if -h then return args None
    

    parser = argparse.ArgumentParser()

    parser.add_argument("-tx", "--tx", type=int,                 # absence will set value to None
                         help="MAX tx chip select range 1 .. 2")

    parser.add_argument("-rx", "--rx", type=int,
                         help="MAX rx chip select range 1 .. 4")

    parser.add_argument("-i", "--index", type=int,                # absence will set value to None
                         help="MAX chip bit index range 0 .. 9")

    parser.add_argument("-b", "--bits",
                               help="comma separated bits 0|1|x|X")

    parser.add_argument("-?", "--query", action='store_true',
                               help="query parameters")

    try:
      (args,unknowns) = parser.parse_known_args(theLine)
    except SystemExit as eobj:                            # returned from -h, --help
      exit_f = True     
    except Exception as eobj:                             # haven't seen this path trigger yet
      err_str = "options exception: 7"   # which is why the result will be a raise
      err_code = -1
      eobj_str = repr(eobj)
      errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
      raise UserDefinedException(errDict)  
    else:
        theLast = len(unknowns)
        ii = 0
        while (ii < theLast):
          if (unknowns[ii][0] == '-'):
            print("unknown option=",unknowns[ii])
            err_f = True    # keep going, print all
          ii = ii + 1
        # end while

        if (err_f):
          exit_f = True
    # end else

    return exit_f,args
# end parse_setmax
#
#-------------------------------------------------------------------------
#
def parse_script(theLine):
    err_f = False
    exit_f = False
    args = None   # if -h then return args None

    parser = argparse.ArgumentParser()

    parser.add_argument("-i", "--iterate", type=int, 
                          default=1,help="iterate script, -i-1 = forever")
 
    try:
      (args,unknowns) = parser.parse_known_args(theLine)
    except SystemExit as eobj:                            # returned from -h, --help
      exit_f = True     
    except Exception as eobj:                             # haven't seen this path trigger yet
      err_str = "options exception: 9"   # which is why the result will be a raise
      err_code = -1
      eobj_str = repr(eobj)
      errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
      raise UserDefinedException(errDict)  
    else:
        theLast = len(unknowns)
        ii = 0
        while (ii < theLast):
          if (unknowns[ii][0] == '-'):
            print("unknown option=",unknowns[ii])
            err_f = True
          ii = ii + 1            # keep going, print all
        # end while

        if (err_f):
          exit_f = True
    # end else no error

    return exit_f, args

# end parse_script
#
#-----------------------------------------------------------------------
#
#
# create a utc time to force a testing path that needs the utc time
#
def utcNow(offset=0):
   ts = int(time.time())     # force type long for metadata timestamp
   return ts,dt.datetime.utcfromtimestamp(ts+offset).strftime('%Y-%m-%dT%H-%M-%S')
# end utcNow
#
#-------------------------------------------------------------------
#

class afecmds:

    def __init__(self,ipaddr,port,wait,rootPath,script):
        self.rootPath   = rootPath    # root path to metadata
        self.script     = script
        self.slow_f     = False       # simplifies scripts
        self.connected  = False       # socket connection valid
        self.sockObj    = None        # socket object
        self.sock_wait_f = False      # connect wait forever
        self.sock_wait_value = wait   # connect wait seconds
        self.bk_listen  = None        # background socket listen thread

        self.lfp        = None        # log file handle
        self.logName    = 'afelog'    # log file name
        self.logPath    = './afelog'  # log file path
        self.prompt     = 'afecmd> '  # user prompt
        self.socket_f   = USE_SOCKET  # connect socket versus assort

        #
        # First, init to None, then Overwrite w/values
        #
        self.ipaddr     = None        # ip address
        self.port_value = None        # socket port
        self.serPath    = None        # path to serial device
        self.baud_rate  = BAUD_RATE   # baud rate for serial device

        if (USE_SOCKET):
          self.ipaddr     = ipaddr    # ip address
          self.port_value = port      # socket port
        else:
           self.serPath    = ipaddr   # path to serial device
           self.baud_rate  = port     # baud rate for serial device

        self.ser_init_f    = False    # set when serial initialized
        self.serialDev     = None     # set when initialized 
        self.reply         = None     # packet reply

        self.lock          = Lock()      # attempt to mutex print statements
        self.send          = False       # send to socket
        self.sfp           = None        # script file
        self.quit_f        = False       # quit detected
        self.interactive_f = True        # if True then echo replies from device
        self.rs_first_f    = True        # used to set console mode of 432 
                                         # (suppress prompt from 432)
        self.hdf5_run_f     = None       # state of background hdf5 data collector
        self.hdf5_init      = False      # metadata init
        self.metaObj        = None       # digitalrf metadata object 
        self.metadata_path  = None       # metadata write path
        self.meta_write_first_f = True   # initialize before first write
        self.meta_chmod_first_f = True   # chmod if first write fails
        self.idx_arr            = None   # set before use
        self.ruObj              = None   # redis
        self.loopObj            = None   # loop thread
        self.server_addr        = 6      # 6 = aurora
        self.seq_num            = 1      # av header sequence number  
        #
        #     
        self.success_f = True
        self.startup_script = None
        if ((self.ipaddr != None) or (self.serPath != None)):
          if (g_verbose):
            print("IPADDR=",self.ipaddr)
            print("serial=",self.serPath)
          # endif

          if (USE_SOCKET):
            self.success_f = self.myConnect(self.ipaddr,self.port)
          else:
            if (g_verbose):
              print("baud_rate=",self.baud_rate)
            # endif
            self.success_f = self.myConnect(self.serPath,self.baud_rate)

          if (self.success_f):
            self.success_f = self.start_threads(True)   # listen is part of send
            if (self.success_f):
               if (script != None):
                 self.startup_script = script
          # endif success
        # end if listening on socket
    # end __init__
    #
    # ------------------------------------------------------------------------------
    #
    def read_accumulate(self,wait_until= DEFAULT_WAIT):
      """
      #
      # a call to send is followed by a call to read
      # the far side might not have responded yet
      # so hang on for wait time
      #
      # this function allows for multi line returns
      # so if response is present, check again to see if it is still growing
      # 
      """
      msg = ""   
      if (self.bk_listen == None):        # Cleanup/shutdown leads to this path
        return msg
   
      time_start = time.time()
      
      msg = self.bk_listen.read()         # read does not delete        

      length_start = len(msg)
      while True:                         # loop   
        time_now = time.time()
        if (time_now >= (time_start + wait_until)):
            break                          # timeout break
        #time.sleep(0.1)                   # sleep (slows things down too much)

        msg = self.bk_listen.read()

        if (len(msg) > 0):
          break
        
      # end while                         # end loop

      return msg

    # end read_accumulate
    #
    # ------------------------------------------------------------------------------
    #
    def afe_read(self,wait_until= DEFAULT_WAIT):

     debug_f = False

     resp_in = self.read_accumulate(wait_until=wait_until)   #  read
     if (debug_f):
       print("afe_read(resp_in:)='%s'"%(resp_in))
     # endif debug

     msgList_in = resp_in.split('\n')      #  split by linefeeds, typically "\n#ok\n" 
     msgList_out = []
     ii = 0
     while (ii < len(msgList_in)):   # filter for interesting stuff
        msg_out = msgList_in[ii]
        msg_out = msg_out.strip('\r')
        if (len(msg_out) > 0):
          msgList_out = msgList_out + [msg_out] 
        # endif not empty
        ii = ii + 1                  # next
     # end while

     self.bk_listen.clear()          #  clear for next read
     
     if (debug_f):
       print("afe_read(msgs)=", msgList_out)
     # endif debug

     return msgList_out       

    # end afe_read
    #
    # ------------------------------------------------------------------------------
    #
    """
    # Notes: 
    #   1. code conditional nesting was getting too deep, so code was cut into two,
    #      the cut made at just before I/O selection.
    #   2. the "@" decorator is a static variable.
    #   3. Q: Why do this (the static var)?
    #      A: This program expects a protocol that terminates in a CRLF before
    #         processing responses.
    #         The 432 expects a human, and sends a prompt that does not terminate
    #         in a CRLF. The first command is intended to command the 432 that there 
    #         is no human, don't send a prompt.
    # 
    #
    """
    def afe_send(self,theCmd,ignore_err_f=True):
      status = 0  
      err_f = False
      debug_f = False    # enable to trace behavior   
      resp_str = ""
      exit_f = False

      if (not self.isConnected()):
        print("afe is not connected.")
        status = -1
        err_f = True
        return status, resp_str
      # endif not connected

      theCmd = theCmd.strip()
      #                                # still here?
      if ((len(theCmd) > 0) and (theCmd == "-?") or (theCmd == "--help")):   # likely parse only
         pass
      else: 
        # check for nothing on the line
        if (len(theCmd) == 0)or((len(theCmd) == 2)and(theCmd[0] == '"')and(theCmd[-1] == '"') ):
          print("Error, no command on line")
          status = -1
          return status, resp_str
        elif ( (theCmd[0] != '"') and (theCmd[-1] != '"')):
          theCmd = '"' + theCmd +  '"'         
      # end else 
      #                                # still here?
      try:
        shlex_out = shlex.split(theCmd)
      except Exception as eobj:        # likely no closing quotations
        print("afe_send: shlex exception:",eobj)
        status = -2
        err_f = True
        return status, resp_str
      #                                  # still here? 
         
      try:
        exit_f,args = parse_afe_send(shlex_out) # parse send options
      except Exception as eobj:
        err_f = True
        err_str = "options exception: 11"
        err_code = -3
        eobj_str = repr(eobj)
        errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
        raise UserDefinedException(errDict)
      else:
        # ---------------------------
        #  -h detected or bad option
        # ---------------------------
        if (exit_f):                    # -----------------
          return status, resp_str       #   early return
        # endif                         # -----------------
      # end else
 
      iter = 1
      if (args.iter != None):
        if (args.iter > 1):
          iter = args.iter

      delay =0
      if (args.delay != None):
        if (args.delay > 0):
          delay = args.delay
      #
      # Note: response, listen, and expect are different things
      #       listen means listen (provides an amount of time to listen,
      #       implies but does not require a response)
      #       response means a response is expected but no details on what to expect
      #       expect is the expected response as a string to compare
      #
      #
      response_f = args.response       # read from argparse for this command
      if (response_f):
        g_response_f = True            # sending info to thread
      
      listen_f = False
      listen_wait = 1                  # how many seconds to listen
      if (args.listen != None):        # before timing out
        if (args.listen):
          listen_wait = args.listen
          listen_f = True

      expect_str = ""
      expect_f = False
      if (args.expect != None):
        if (args.expect != ""):
          expect_str = args.expect
          expect_f = True

      append= ""
      append_f = False
      if (args.append != None):
        if (args.append != ""):
          app_fn = args.append
          append_f = True
      
      if (debug_f):
         print("\nappend_f=",append_f,"expect_f=",expect_f,"listen_f=",
               listen_f,"listen_wait=",listen_wait)

      try:                              
        unquoted = shlex_out[0]        # shlex removed embedded quotes
      except Exception as eobj:        # unbalanced quotes
        err_f = True
        err_str = "shlex exception"
        err_code = -4
        eobj_str = repr(eobj)
        errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
        raise UserDefinedException(errDict)

      isquoted = '"' + unquoted + '"'       # restore quotes (for uart xfer)
          
      ii = 0                      
      while (ii < iter):               # iteration option       
         self.bk_listen.clear()
         print("sending:",unquoted )        
         try:
            self.device_send(isquoted)        # vectors to I/O device of interest
         except Exception as eobj:
            err_str = "Exception in device_send()"
            err_code = -6
            eobj_str = repr(eobj)
            errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
            raise UserDefinedException(errDict) 
         # end exception
         #
         #  still here?
         #
         if ((status < 0) and (not ignore_err_f)):
           break
         # endif ignore errors

         if (response_f or expect_f):  # if response expected, then listen
           tmo_f = False
           if (debug_f):
             print("listen_wait=", listen_wait)
           resp_str = self.read_accumulate(listen_wait)
           resp_str = resp_str.strip()      # remove CRLF

           if (resp_str == ""):
              tmo_f = True
           else:
              # -----------------------------------------
              # received from AFE in response to command
              # -----------------------------------------
              print('rx: "%s"'%(resp_str))  
              # -------------------------------------------------
              # test for condition where host (this) rebooted     
              # and ALE is in interactive mode with echo          
              # host's first command (see code above) is to stop the echo
              # the user sometimes forgets that ALE boots this way
              # --------------------------------------------------
              raw_cmd = unquoted.strip()
              cmdLen = len(raw_cmd)
              #print("raw_cmd=", raw_cmd, cmdLen,"resp_str=", resp_str,len(resp_str))
              if (raw_cmd == resp_str[:cmdLen]):
                # -----------------------------------------
                # echo seen in response from AFE 
                # -----------------------------------------
                print("detected echo, turn off interactive mode and retry command")
              # endif
           # end else
           #
           #
           if (tmo_f):   
             err_str = "timeout: response expected" 
             print(err_str)
             if (iter == 1):
               status = -7                    # > 1 iteration allows for retry
               err_code = status
               eobj_str = "" 
               errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
               raise UserDefinedException(errDict)
               # stop on error
             else:
               if (delay > 0):
                 time.sleep(delay)            # delay between sends
               ii = ii + 1                    # next iteration
               continue                       # go to top of loop
           # end if timeout
           #
           # still here? 
           #
           if (not err_f):
             
             if (append_f):
                err_f = append_file(app_fn,isquoted, resp_str)
                if (err_f):
                  status = -7
                  err_code = status
                  eobj_str = "" 
                  err_str = ""
                  errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
                  raise UserDefinedException(errDict)
             if (expect_f):         # compare against returned value  
               #
               # Note:
               #   The 432 returns '#' as first char to prevent responses from being 
               #   interpreted as commands when there are 432s' hooked up to 432's
               #   the problem is '#' is a python comment and can cause all sorts of problems.
               #   Ignore first char on expect.
               #
               if (resp_str[1:] == expect_str):
                  print("Success!")
                  status = 0                  # reset status
                  break                       # success
               else:
                  print("Fail! Unexpected response: %s, expected: %s" %(resp_str,expect_str))
                  status = -9
                  if ((ii + 1) < iter):
                    print("retrying command...")
               # end else unexpected value 
             # end else, no exception
           # end if expect

           g_response_f = False

         # end if expect or response
         ii = ii + 1                          # next
         if ((delay > 0) and (ii < iter)):    # if delay and not last
           time.sleep(delay)                  # then delay before repeat
         # endif delay
      # end while
         
      if status == 0:
         self.send= True

      return status, resp_str
    # end afe_send

    #
    # ------------------------------------------------------------------------------
    #
    def device_send(self, qmsg):
        status = 0
        err_f = False
        slen = len(qmsg)
        smsg = qmsg[1:slen - 1]



        if (self.socket_f):         # if socket, connected?
              if (self.sockObj == None):
                err_f = True
              # endif no socket
        else:                       # else uart, connected?
              if (self.serialDev == None):
                err_f = True
              # endif no serial device
        if (err_f):                 # if not connected
                emsg = 'Not connected.'
                print( emsg)
                if self.lfp != None:
                  logging.error(emsg)
                status = -2
                self.connected = False
        else:                       # else keep going
                self.listen_buf = ''    # erase last response
                if (self.socket_f):     # if socket
                  try:
                      self.sockObj.send(smsg.encode('ascii'))
                  except Exception as eobj:
                    emsg =  repr(eobj) 
                    err_f = True
                    status = -3
                else:                          # else usart
                  smsg = smsg + '\r'           # append cr (EOL)                
                  try:
                    self.serialDev.write(str.encode(smsg))
                  except Exception as eobj:
                    emsg =  repr(eobj) 
                    err_f = True                                 
                # end else usart
 
                if (err_f):             # if error
                    print(str(emsg))
                    if self.lfp != None:
                      logging.error(emsg)
                    emsg = 'Connected?'
                    print(emsg)
                    if self.lfp != None:
                      logging.error(emsg)
                    status = -5

                if ((not err_f) and (status == 0)):
                    self.send= True       # set global flaj
                # endif no error
        # end else keep going

        return status
    # end device_send
    #
    # ------------------------------------------------------------------------------
    #
    def isConnected(self):
        return self.connected
    # end isConnected
    #
    # ------------------------------------------------------------------------------
    #
    def Cleanup(self):
        debug_f = False
        #
        # called before exiting program, shuts down background threads
        #

        print("Cleanup: shutting down afe listener thread")
        if self.bk_listen != None:
           self.bk_listen.shutdown()    # kill socket listen thread
           self.bk_listen = None 
        else:
           pass # print("no afe listener thread to shut down" )
 
        if self.lfp != None:  
           msg = "Cleanup: log file closing"
           if (debug_f):
             print(msg)
           # endif debug
           logging.info(msg)   
           logging.shutdown()    # close logger 
           self.lfp = None
        else:
           if (debug_f):
             print("no logger to shut down")
           # endif debug

        if (self.loopObj != None):
         print("Cleanup: shutting down loop thread")
         self.loopObj.shutdown()
         self.loopObj = None      


        self.myClose()           # close ports and sockets
    # end Cleanup
    #
    # -------------------------------------------------------------------------
    #
    def myClose(self):
        if (self.connected):
          wmsg = 'closing interface to device...'   
          print(wmsg)
          if self.lfp != None:
             logging.info(wmsg)
        if (self.sockObj != None):
          try:                        # in "try" block because socket might be closed
            self.sockObj.close()      # closes socket but does destroy object
          except Exception as eobj:
            pass                      # due to closing a None object
          self.connected = False
          self.sockObj = None         # destroy socket object

        if (self.serialDev != None):   # close serial device
          self.close_ser()
        self.connected = False
    # end myClose
    #
    # -------------------------------------------------------------------------
    #  
    # NOTE: inverted sense, returning success not failure
    # 
    def myConnect(self,ipaddr=None,port_value=None):
      success_f = False

      if (self.socket_f == True):
        success_f = self.myConnectSocket(ipaddr,port_value)
      else:
        success_f = self.myConnectUsart(serPath =ipaddr,baud_rate =port_value)
      # end else 

      return success_f    # NOTE: inverted sense, returning success not failure
    # end myConnect
    #
    #------------------------------------------------------------
    #
    def select_dev(self,print_f=True):
      theList = []
      found_f = False

      theList = theList + glob.glob("/dev/*USB*")
      theList = theList + glob.glob("/dev/*usb*")
      theList = theList + glob.glob("/dev/ttyACM*")
      theList = theList + glob.glob("/dev/ttyUSB*")
      theList = theList + glob.glob("/dev/cu.usb*")
      theList = theList + glob.glob("/dev/ttyGNSS1")
      #
      # if the default port doesn't match the available ports,
      # then set the default to the available.
      #
      if (len(theList)==0):
        print("\n*** No USB ports of appropriate type detected. ***\n")
        print("Please check your serial device and/or usb cable.")
      else:
        #
        # find a match, if not then offer a choice
        #
        found_f =  isInlist(theList,self.serPath)
        #
        # if found, then done
        #
        if (found_f):
          if (print_f):
            print("device:",self.serPath)
        else:
          if (self.serPath == None):
            print("No serial ports selected.")
          else:
            print("Selected port:",self.serPath,"not found.")
            print("In list:", theList)

          if (len(theList)==1):
            print("Using:",theList)
            self.serPath = theList[0]    # only choice
            found_f = True              # signal found
          else:
            print("Please select a port from:",theList)
            ii = 0
            while (ii < len(theList)):
               item = theList[ii]
               print("Set default port to:",item,"?")
               sys.stdout.flush()      # request can occur within thread
               line = history_input("[y],n,q --> ")
               line = line.strip()
               line = line.upper()
               if ((len(line)==0) or 
                   (line[0]=='Y') or 
                   (line[0]=='y') or 
                   (line[0]=='\n')):
                  self.serPath = item      # found it
                  found_f = True            # signal found
                  break
               else:
                 if ((line[0]=='q') or (line[0]=='Q')):
                   break
                 # endif
               # end else
               ii = ii + 1      # next
            # end while list
          # end else more than one to choose
        # end if not found
      # end else check for match in list
      if (found_f):
        #
        # does the user have the privileges to access the device?  
        #
        mode = os.stat(self.serPath)        # returns a structure
        if (mode.st_gid != 20):             # group id, looking for '20', dialouy
          print("User may not have dialout group privileges to access device.")
        # endif group id test
      # endif device found
      
      return found_f,theList
    # end select_dev
    #
    #------------------------------------------------------------
    #
    def init_ser(self,serPath = None,       # if None, then search
                 baud_rate    = None,       # if None -> use default (9600) 
                 verbose_f    = False):     # if False -> only error info to console
                      
      err_f = False
      if (self.quit_f):
        return err_f

      if (verbose_f):
        print("\nInitializing serial port.")

      if (baud_rate != None):
        if (baud_rate != self.baud_rate):       # overwrite 
          self.baud_rate = baud_rate 
      else:
         baud_rate = self.baud_rate
         if (baud_rate == None):
           print("baud rate never set.")
           err_f = True
      
      if (serPath != None):
          self.serPath = serPath              # overwrite
                                             
      #
      # compare given port against available ports
      #
      found_f,theList = self.select_dev(print_f=False)
      if (not found_f):
          print("No available serial ports.")
          err_f = True                         # give up

      if (not err_f):                          # path is in self.serPath    
        self.serialDev = serial.Serial()       # doesn't need path yet
        self.serialDev.baudrate = self.baud_rate
        self.serialDev.port = self.serPath     # open at path
        try:
          self.serialDev.open()
        except Exception as eobj:
          print( "init_ser() Exception:",eobj)
          print( "Unable to open serial port:",self.serPath)
          self.serialDev = None                 # cleanup
          self.ser_init_f = False               # clear flag
          err_f = True

      if (not err_f):
        #
        # if this line is reached then next test should never fail
        # belts and suspenders
        #
        if (not self.serialDev.isOpen()):
           err_f = True
           print( "Serial port opened but test for open failed.",self.serPath)
      # endif not error
      #
    
      if (not err_f):
        self.ser_init_f = True
        self.connected = True
      # endif no error
         
      return err_f
    # end init_ser  
    #
    #------------------------------------------------------
    #
    def close_ser(self,digi_init_f=False):
      if (self.serialDev != None):
        try:
          self.serialDev.close()     # clean close (or not)
        except Exception as eobj:
          pass
      self.serialDev = None          # re-init object   
      self.ser_init_f = False        # force reinit of serial port
    # end close_ser
    #
    #------------------------------------------------------
    #
    """
      intent: retry when print interferes w/ tasked print
              try printing again
    """
    def print_retry(self,rcnt,theString):
       retry = rcnt
      
       while(retry > 0):           # I/O blocking
         try:
           print(theString) 
         except Exception as eobj:
           retry = retry -1
         else:
           break
       # end while
       #
    # end print_retry
    #
    #------------------------------------------------------------
    """
    #
    # intent: put this routine into a thread
    #
    #
    """
    def getSerialData(self):      
      bcnt =0               # count of bytes received
      tries = MAX_TRIES     # how long to keep trying
      err_f = False         # set true on error
      status = 0            # <0 indicates error, >0 indicates data read
      msg = ""              # collects pieces for count of tries
      debug_enter_f = False # prints each time routine is entered and exited
      debug_data_f  = False # prints each char as it arrives 
      #
      if (self.serialDev == None):     # if not initialized then initialize
         err_f = self.init_ser(serPath = self.serPath,
                                        baud_rate = self.baud_rate)       
         if (err_f):                   # prints if error 
           status = -1               # code allows for recovery, if possible
      # endif

      if (not err_f):
        while ((tries > 0) and (err_f == False)):
          if (debug_enter_f):               # debugging gevent lockup
            print("entering serialDev waiting")
            sys.stdout.flush()
          # endif 
          try:
            bcnt = self.serialDev.in_waiting
          except Exception as eobj:
            err_f = True
            #
            # if "IOError: [Errno 6] Device not configured" 
            # then (possibly) unplugged 
            #
            estr = repr(eobj)          # convert object to string
            if (estr.find('Device not configured') >=0):
               status = -2
               self.ser_init_f = False
            else:
              print("Exception in serialDev:",eobj)
              # traceback.print_exc(file=sys.stdout)  # details of error
              #
              # the above traceback is commented out because on close
              # the exception is a side-effect of closing
              #
              status = -3
            # end exception
          else: 
            if (bcnt == 0):  
              #
              pass
              
            # endif no data       
            else:    # data is present
              
              piece_str = ""        # init decoded piece_byte
              try:
                piece_byte = self.serialDev.read(bcnt)
              except Exception as eobj:
                print("Exception in serialDev:",eobj)
                err_f = True
                status = -4
                break
                #
                # this has been the exception error message:
                """
                 'device reports readiness to read but returned no data '
                  serial.serialutil.SerialException: device reports readiness
                  to read but returned no data 
                    possibly means device disconnected or multiple access on port
                """
                #
              try:
                piece_str = piece_byte.decode("utf-8")
              except Exception as eobj:
                datum = ""          # init not decoded as hex
                try:
                  datum = hex(ord(piece_byte))
                except Exception as eobj2:
                   noiseList = list(piece_byte)
                   print("binary rx:",piece_byte)     # hex
                   print("binary as list:", noiseList)  # decimal
                   #        
                   # "During handling of the above exception, another exception occurred"
                   #
                   # Noise (such as disconnecting and reconnecting power) could set  
                   # the most significant bit, which will raise a utf-8 exception
                   #
                else:
                  
                   pass
              else:
                
                if (debug_data_f):
                  char_dump(piece_str)
                  sys.stdout.flush()
                # endif debug
              # end else successful decode
              msg = msg + piece_str
              status = 1;                     # positive value flags data present
            
            if (debug_enter_f):               # debugging gevent lockup
              print("exited serialDev waiting" )
              sys.stdout.flush()    

          # end else no exception from inWaiting
         
          tries = tries -1        

        # end while

        return status, msg

    # end getSerialData
    #
    # -------------------------------------------------------------------------
    #
    def myConnectUsart(self,serPath=None,baud_rate=None):
      err_f = False      # set true on error
      success_f = False  # set true on no errors
      verbose_f = g_verbose   # set true for debugging
   
      if (self.quit_f):
        return err_f

      #
      # redis experiment
      #
      #if (self.baudRate >= 115200):
      #  import redisUtilities as ru
      #  self.ruObj = ru.redisWrapper()
      #  return err_f
      # endf redis vector

      if (serPath == None):  
        serPath = self.serPath      # if none, use default
        
      if (baud_rate == None):       # if none, use default 
        baud_rate = self.baud_rate    

      if (not self.ser_init_f):             # initialize the serial port, if needed
        err_f = self.init_ser(serPath,baud_rate,verbose_f=verbose_f)
        if (not err_f):
          print("Opened serial port: %s, baud rate: %s."%(self.serPath,baud_rate))
          success_f = True         
        # endif no error
      # endif not initialized
      else:                       # already initialized
         success_f = True
      # end else already initialized

      return success_f

    # end myConnectUsart
    #
    # -------------------------------------------------------------------------
    #
    def myConnectSocket(self,ipaddr=None,port_value=None):
        success_f = False
        if (ipaddr == None):  
           ipaddr = self.ipaddr  # use default
        if (port_value == None):
           port_value = self.port_value

        self.sockObj = socket.socket(socket.AF_INET,
                                   socket.SOCK_STREAM)  # socket object

        if (not self.sock_wait_f) or (self.sock_wait_value != -1):
           self.sockObj.setblocking(0)
                    
        wmsg= 'connecting...' + str(ipaddr) + ' '+ str(port_value) 
        wmsg= wmsg+', waiting ' + str(self.sock_wait_value)+' sec'
        print(wmsg)
        if self.lfp != None:
           logging.info(wmsg)
        time_start = time.time()
        while 1:
            try:
                self.sockObj.connect((ipaddr,port_value))  # note (()) form              
            except Exception as eobj:
                if not self.sock_wait_f or self.sock_wait_value != -1:
                   time_now = time.time()
                   if time_now >= time_start + self.sock_wait_value:
                      emsg =  repr(eobj) 
                      emsg1 = 'socket connect failed:'+str(emsg)
                      print(emsg1)
                      if self.lfp != None:
                         logging.error(emsg1)
                      emsg2 = 'timeout'
                      print(emsg2)
                      if self.lfp != None:
                         logging.error(emsg2)
                      break
                else:
                   emsg =  repr(eobj) 
                   emsg1 = 'socket connect failed:'+str(emsg)
                   print(emsg1)
                   if self.lfp != None:
                      logging.error(emsg1)
                   break
            else:
              success_f = True
              self.ipaddr = ipaddr          # override default
              self.port_value = port_value
              self.connected = True
              break

        return success_f
    # end myConnectSocket
    #
    # -------------------------------------------------------------------------
    #
    def start_threads(self,listen_f):

      self.listen_f = listen_f
      success_f = False                    # set on open success

      if ((self.ipaddr != None) and (self.port_value != None)):
        success_f = self.myConnect(self.ipaddr,self.port_value)         
      else:
        if ((self.serPath != None) and (not USE_SOCKET)):
          success_f = self.myConnect(self.serPath,self.baud_rate)   # default baud rate
        else:
          emsg= '\nif socket then <address> and <port> expected, '
          emsg = emsg + 'else if USB then <device> and <baudrate> '
          print(emsg)
          if self.lfp != None:
            logging.error(emsg)
          
          if (not USE_SOCKET):
                                     # this can still work
             err_f = self.init_ser() # initialize asks user for params
             if (not err_f):         # if not error, then success
                success_f = True

        # end else no input params
      #
      # end else check for partial input params
      #
      if (success_f):
        if (g_verbose):
          print("Starting listen thread, listen_f=", listen_f)
        # endif
        #
        # sensitive to race conditions xx
        #
        self.bk_listen = listenThread(self,1,listen_f)   # Listen for N sec burst, initally 5
        self.bk_listen.start()
      else:
        print("Error: listen thread NOT started")

      return success_f
      
    # end start_threads
    #
    # -------------------------------------------------------------------------
    #
    def is_comment(self,line):

       comment_f = False

       line = line.lstrip(' ')  # clean left

       if (len(line)==0):      # check for blank line.
          comment_f = True
       else:
         if (line[0] == '\n') or (line[0] == '#'):     # check for line feed
           comment_f = True
        
       return comment_f

    # end is_comment
    #
    # -------------------------------------------------------------------------
    #
    def run_script_once(self,filename,user_f):
      status = 0
      print_comment = False
      print_script   = True    # script has lock, slows things down

      try:
          self.sfp = open(filename, 'r')
      except IOError:
          emsg= '*** Cannot open: %s ***' % filename
          print(emsg)
          if self.lfp != None:
             logging.error(emsg)
          status = -5
          return status 

      resp_err_cnt = 0
      line = self.sfp.readline()  # first line
      lnum = 1  # count lines

      print("\nPress any key followed by\'return\' to pause the script\n")

      while line != '':  # while data in file
        sz = len(line)
        if line[sz - 1] == '\n':  # remove lf
             line = line[0:sz - 1]

        if (self.is_comment(line) and not print_comment):
            pass
        else:
            if (print_script):
              self.lock.acquire()        # lock prevents print interleaving
              print('%d:' % lnum, line )  # logged during execution 
              self.lock.release()        # unlock
            # end if print script
        # end else print the line
        lnum = lnum + 1
                                            # send occurs here
        status = self.process_line(line)  # process via recursion

        # print("line=",line,"status=",status)

        if self.quit_f:      
            break
        # endif quit in script

        self.send = False # init/re-init

        if (user_f):   # user pause to allow user to exit the script          
           res = getch(False)            # pause for user interaction
           if (type(res) is bool):
                pass                    # did not press key             
           else:
              # print("res=",hex(ord(res)),"ii=",ii)
              status = -1
           # end else
         # endif user available for user interaction

        if (status < 0):
          if (user_f):
            print("Continue? (Y, N (quit script), Q (quit program)) > ",end=" ")
            sys.stdout.flush() 

            line = getch(True)
            print("%c"%(line[0]))
            line = line.upper()
            line.strip()
            if len(line) == 0:
               return status
            else:
                if line[0] == 'Y':
                    pass
                elif line[0] == 'N':
                    break               # stop script
                elif line[0] == 'Q':
                    self.quit_f = True
                    return status       # stop program
                else:
                     break  # stop script
                #end else
            # end else decode user option
          else:              # else no pause
            return status
          # end else no pause
        # end if error

        line = self.sfp.readline()  # next line

        if (len(line) == 0):         # end of file
          break
        # end if end of file
                    
      # end while
      self.sfp.close()
      self.sfp = None  # flags closed

      self.lock.acquire()
      print('script complete')
      self.lock.release()
                
      if self.lfp != None:
        logging.info('script complete')

      return status

    # end run_script_once
    #
    # -------------------------------------------------------------------------
    #
    def do_script_cmd(self,line,kidx,user_f=True):
      status = 0
      err_f = False
      exit_f = False

      param1 = line[kidx:]
      param1 = param1.lstrip(' ')
      param1 = param1.rstrip(' ')
      param1 = param1.rsplit(' ')  # remove extra params
      param1 = param1[0]
      sfilenm = param1

      if sfilenm.rfind('.') == -1:
          sfilenm = sfilenm + '.txt'
      #
      # pull off options
      #
      try:
        shlex_out = shlex.split(line)
      except Exception as eobj:        # likely no closing quotations
        err_str = "hlex exception"
        err_code = -1
        eobj_str = repr(eobj)
        errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
        raise UserDefinedException(errDict)
      #                                # still here? 
      try:       
        exit_f,args = parse_script(shlex_out) # parse send options
      except Exception as eobj:
        err_f = True
        err_str = "options exception: 12"
        err_code = -3
        eobj_str = repr(eobj)
        errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
        raise UserDefinedException(errDict)
      else:
        # ----------------------------
        #  -h detected or bad option
        # ----------------------------
        if (exit_f):          # -----------------
          return status       #   early return
        # endif               # -----------------
      # end else
      
      forever = False
      iterate = 1
      if (args.iterate != None):
        iterate = args.iterate
        if (iterate == -1):       # check for forever
          iterate = 1
          forever = True
        else:
          if (iterate <= 0):      # check for bad input
             iterate = 1 

      while (True):      
        cmd = 'print("Starting script. Iteration=",end=" ")'
        eval(cmd)
        
        if (forever):
           print( "Forever")
        else:
           print("i=",iterate)  # note: this print might be hard to find

        status = self.run_script_once(sfilenm,user_f)

        if (not user_f):         # if started by invocation, then one and done
          self.Cleanup()  
          sys.exit()  
         # end if command line invocation

        if (self.quit_f):        # user quit on command error 
          break

        if (status < 0):         # script error
          break

        if (not forever):
          iterate = iterate - 1    # decrement loop counter
          if (iterate <= 0):
            break
                                 # check for user quit here
        res = history_input("press any key followed by\'return\' to exit > ")

        if (res != ''):
           break
      # end while
   
      return status

    # end do_script_cmd
    #
    #-----------------------------------------------------------------------
    #
    # afeHandler is the message handler called from the listen thread
    #
    def afeHandler(self):
      global g_verbose

      err_f = False
      verbose_f = g_verbose
      metaDict = {}

      msg = self.read_accumulate()            # read listen task buffer
      if (len(msg) > 1):               # if something there
        if (verbose_f):                #   if verbose
          msg = msg.strip()
          print('rx: "%s"'%(msg))
        # endif verbose
        self.bk_listen.clear()         #   clear for next read

      # endif something sent from 432

    # end afeHandler
    #
    # -------------------------------------------------------------------------
    #
    def verifyTime(self,iter=1):
     global g_response_f

     status = 0
     err_f = False
     resp = ""

     if (not self.isConnected()):
       print("verifyTime: afe is not connected.")
       status = -1
       err_f = True
       return status
     # endif  

     #
     # design:
     #
     #  1. read time from afe 
     #  2. read time.time()
     #  3. compare
     #
     # note: there will be a delay in the round-trip
     #
     # note: the command string is sent thru a parser
     #       to pull out command line options
     #       to allow looping
     #
     # note: call is in try/except block because tmo
     #       raises an exception
     #   
     try:
       status, resp = self.afe_send("\"eval('time.time()')\" -r -l -i%d"%(iter))
     except Exception as eobj:
       print("Exception in verifyTime:",eobj)
       status = -1
       err_f = True
     else:
       if (resp != ""):
         rxTime = int(resp)
         print("rxTime=",rxTime,"nowTime=",int(time.time()) )
       # endif 
     # end else no exception

     return status

    # end verifyTime
    #
    # -------------------------------------------------------------------------
    #
    def wrapped_send(self,msg_in):
      err_f = False
      status = 0
      #
      # afe_send recognizes options separate from the message
      # the message must have explicit quotes to separate
      # the message from the options
      #
      msg_out = '"' + msg_in + '"'   # match the api of afe_send()
      try:
        status, resp = self.afe_send(msg_out)
      except Exception as eobj:
        print("Exception in afe_send: 2,", eobj) # disambiguate
        err_f = True
        status = -10   # afe_send() err code ranges to -9
      # endif
      return err_f,status
    # end wrapped_send
    #
    # -------------------------------------------------------------------------
    #
    def send_time_query(self):
      err_f = False
      debug_f = True
      status = 0

      pre_cksum_str = "$PMITTP?*"   # TP? = Time Parameter Query

      err_f, packet_out = add_cksum(pre_cksum_str)

      if (err_f):
        print("Error: send_time_query(): add_cksum()")
        status = -1
      # endif
      else:
        if (debug_f):
          print("packet_out=", packet_out)
        # endif debug
        #
        err_f, err_code = self.wrapped_send(packet_out)
        if (err_f):
          status = -10 + err_code # return code ranges from -1 to -10
        # endif
      # endif not error
      #
      if (err_f):
        print("No command knowingly sent to AFE")
      # endif

      return status

    # end send_time_query
    #
    # -------------------------------------------------------------------------
    #
    """
     $PMITTSG - set time source gnss
     $PMITTSE - set time source external
     $PMITTEP - set time epoch pps
     $PMITTEN - set time epoch nmea
     $PMITTEI - set time epoch immediate
     $PMITTP? - query time parameter
    """
    def setTime(self, theCmd):
      err_f = False        # error flag
      status = 0           # return error code
      debug_f = True
      exit_f = False
      base_str = ""        # initializer
      time_str = ""        # initializer
      pre_cksum_str =""    # initializer
     
      if (not self.isConnected()):
        print("setTime: afe is not connected.")
        status = -1
        err_f = True
        return status
      # endif    
     
      try:
        shlex_out = shlex.split(theCmd)
      except Exception as eobj:        # likely no closing quotations
        print("setTime: shlex exception:",eobj)
        err_f = True
        status = -1
        return status
      # endif                               # still here? 
         
      try:
        exit_f, args = parse_settime(shlex_out) # parse send options
      except Exception as eobj:
        err_f = True
        err_str = "options exception: 13"
        err_code = -3
        eobj_str = repr(eobj)
        errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
        raise UserDefinedException(errDict)
      else:
        # ---------------------------
        #  -h detected or bad option
        # ---------------------------
        if (exit_f):          # -----------------
          return status       #   early return
        # endif               # -----------------
      # end else
      
      query_f     = args.query           # -----------------------
      if (query_f):                      # time query, early return
        return self.send_time_query()    # -----------------------
      # endif query 

      gnss_f      = args.gnss
      external_f  = args.external
      pps_f       = args.pps
      nmea_f      = args.nmea
      immediate_f = args.immediate
      theTime     = args.time         # converted to integer by argparse
      query_f     = args.query

      if (debug_f):
        print("theTime=",theTime)     # not sure what happens with -t and no option after
      # endif debug

      contradiction_f = False
      if ((gnss_f and immediate_f) or (gnss_f and pps_f)):  # contradictions
        contradiction_f = True
        theCase = -1
      elif ((nmea_f and immediate_f) or (nmea_f and pps_f)): # more contradictions
        contradiction_f = True
        theCase = -2
      elif (external_f and (theTime == None)):
        contradiction_f = True
        theCase = -3
      elif ( (gnss_f and (theTime != None)) or (nmea_f and (theTime != None)) ):
        contradiction_f = True
        theCase = -4
      # end elseif
     
      if (contradiction_f):
        print("setTime: contradiction in options detected. case:",theCase)
        err_f = True
        status = -2
      else:
        if ((not gnss_f) and (not nmea_f)):
          print("\nsetting time to:",theTime)
          if (immediate_f):
            base_str = "$PMITTEI"   # external immediate
          elif (pps_f):
            base_str = "$PMITTEP"   # external pps
          elif (external_f):
             base_str = "$PMITTSE"  # external (likely need to revisit)
          else:                     # time w/o option
             base_str = "$PMITTSE"  #   (likely need to revisit)
          # end else
        # endif time set option check
        elif (gnss_f or nmea_f):
          print("\nsetting time to NMEA")
          if (gnss_f):
             base_str = "$PMITTSG"
          elif (nmea_f):
             base_str = "$PMITTEN"
          # end elif

        # end elif gnss or nmea
        #
        time_str = theTime
        if (theTime == None):                # if time not passed in then
           time_str = str(int(time.time()))  #   use host clock
        else:                                # else
           time_str = str(theTime)           #   convert int time to string
        # end else passed-in time 
        
        pre_cksum_str = base_str + "," + time_str + "*"
        err_f, packet_out = add_cksum(pre_cksum_str)
        if (err_f):
          print("Error: setTime(): add_cksum()")
          status = -3
        # endif
      # end else
      if (not err_f):
        if (debug_f):
          print("packet_out=", packet_out)
        # endif debug
        #
        err_f, err_code = self.wrapped_send(packet_out)
        if (err_f):
          status = -10 + err_code # return code ranges from -1 to -10
        # endif
      # endif not error
      #
      if (err_f):
        print("No command knowingly sent to AFE")
      # endif
      #
      #
      return status
      #
    # end setTime
    #
    # -------------------------------------------------------------------------
    #
    """
      PMITRT  - telemetry rate
      PMITRM  - telemetry rate magnetometer
      PMITRI  - telemetry rate imu
      PMITRA  - telemetry rate set all
    """
    def setTelemRate(self,theCmd):
      global TELEM_RATE_DEF

      err_f = False
      exit_f = False
      status = 0
      debug_f = False
      base_str = ""        # initializer
      time_str = ""        # initializer
      pre_cksum_str =""    # initializer
     
      if (not self.isConnected()):
        print("setTelemRate: afe is not connected.")
        status = -1
        err_f = True
        return status
      # endif    
      
      try:
        shlex_out = shlex.split(theCmd)
      except Exception as eobj:        # likely no closing quotations
        print("setTelemRate: shlex exception:",eobj)
        err_f = True
        status = -1
        return status
      # endif                               # still here? 
         
      try:
        exit_f,args = parse_TelemRate(shlex_out) # parse send options
      except Exception as eobj:
        err_f = True
        err_str = "options exception: 14"
        err_code = -2
        eobj_str = repr(eobj)
        errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
        raise UserDefinedException(errDict)
      else:
        # ----------------------------
        #  -h detected or bad option
        # ----------------------------
        if (exit_f):          # -----------------
          return status       #   early return
        # endif               # -----------------
      # end else
      
      mag_f   = args.magnetometer
      imu_f   = args.imu
      hk_f    = args.housekeeping
      all_f   = args.all
      sec_int = args.sec
      query_f = args.query
      if (debug_f):
        print("\nmag_f=", mag_f)  # the way the prompt sits needs \n to start
        print("imu_f=", imu_f)
        print("hk_f=", hk_f)
        print("all_f=", all_f)
        print("sec_int=", sec_int)
        print("query_f=",query_f)
      # endif debug

      if (sec_int == None):
        sec_int = TELEM_RATE_DEF
      # endif period not set

      if ((sec_int < 0) or (sec_int > 60)):
         status = -3
         print("Error: seconds must be >= 0 or <= 60")
         print("No command sent to AFE")
         return status
      # endif early return on error  

      sec_str = str(sec_int)   

      if ((not mag_f) and (not imu_f) and (not hk_f) and (not all_f) and (not query_f)):
         status = -4
         print("No flags set")
         print("No command sent to AFE")
         return status
      # endif no rate selected 

      if (query_f):
        base_str = "$PMITR?"                                # query
        pre_cksum_str = base_str + "*"
        err_f, packet_out = add_cksum(pre_cksum_str)
        if (err_f):
          print("Error: setTelemRate():add_cksum()")
          status = -5
        else:
          if (debug_f):
            print("packet_out=", packet_out)
          # endif debug
          err_f, err_code = self.wrapped_send(packet_out)   # send 
          if (err_f):
            status = -10 + err_code # return code ranges from -1 to -10
          # endif error
        # end else not error
      elif (all_f):
        base_str = "$PMITRA"                                # all 
        pre_cksum_str = base_str + "," + sec_str + "*"
        err_f, packet_out = add_cksum(pre_cksum_str)
        if (err_f):
          print("Error: setTelemRate():add_cksum()")
          status = -6
        else:
          if (debug_f):
            print("packet_out=", packet_out)
          # endif debug
          err_f, err_code = self.wrapped_send(packet_out)     # send 
          if (err_f):
            status = -20 + err_code # return code ranges from -1 to -10
          # endif error
        # end else not error
      else:              # if not all then check other flags
        sent_f = False
        if (mag_f):
          base_str = "$PMITRM"                                # mag rate
          pre_cksum_str = base_str + "," + sec_str + "*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str)
          #
          if (err_f):
            print("Error: setTelemRate():add_cksum()")
            status = -7
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            err_f, err_code = self.wrapped_send(packet_out)   # send 
            if (err_f):
              status = -30 + err_code # return code ranges from -1 to -10
            # endif error
            sent_f = True
          # end else not error
        # endif mag
        #
        if (hk_f):
          base_str = "$PMITRT"                                # telemetry rate
          pre_cksum_str = base_str + "," + sec_str + "*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str)
          #
          if (err_f):
            print("Error: setTelemRate():add_cksum()")
            status = -8
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            if (sent_f):
              time.sleep(0.5)  # pause before multiple sends
            # endif sent
            err_f, err_code = self.wrapped_send(packet_out)   # send 
            if (err_f):
              status = -40 + err_code # return code ranges from -1 to -10
            # endif error
            sent_f = True
          # end else not error
        # endif hk
        #
        if (imu_f):
          base_str = "$PMITRI"                                # imu rate
          pre_cksum_str = base_str + "," + sec_str + "*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str)
          #
          if (err_f):
            print("Error: setTelemRate():add_cksum()")
            status = -9
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            if (sent_f):
              time.sleep(0.5)  # pause before multiple sends
            # endif sent
            err_f, err_code = self.wrapped_send(packet_out)   # send 
            if (err_f):
              status = -50 + err_code # return code ranges from -1 to -10
            # endif error
          # end else not error
        # endif imu
        sent_f = True
      # end else not all
      #
      if (err_f):
        print("No command knowingly sent to AFE")
      # endif
      #
      return status

    # end setTelemRate
    #
    # -------------------------------------------------------------------------
    #
    def setMag(self,theCmd):
      global MAG_CCR_MIN      # !!! move these to self.__init___() !!!
      global MAG_CCR_MAX       
      global MAG_CCR_DEF       
      global MAG_UPDR_MIN     
      global MAG_UPDR_MAX      
      global MAG_UPDR_DEF     

      status = 0
      err_f = False
      debug_f = True
      base_str = ""        # initializer
      mag_str = ""        # initializer
      pre_cksum_str =""    # initializer
      exit_f = False
     
      if (not self.isConnected()):
        print("setTelemRate: afe is not connected.")
        status = -1
        err_f = True
        return status
      # endif   

      try:
        shlex_out = shlex.split(theCmd)
      except Exception as eobj:        # likely no closing quotations
        print("setTelemRate: shlex exception:",eobj)
        err_f = True
        status = -2
        return status
      # endif                               # still here? 
        
      try:
        exit_f,args = parse_setmag(shlex_out) # parse send options
      except Exception as eobj:
        err_f = True
        err_str = "options exception: 15"
        err_code = -3
        eobj_str = repr(eobj)
        errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
        raise UserDefinedException(errDict)
      else:
        # ---------------------------
        #  -h detected or bad option
        # ---------------------------
        if (exit_f):          # -----------------
          return status       #   early return
        # endif               # -----------------
      # end else

      ccr_int = args.cycle_count

      updr_int = args.update_rate

      query_f = args.query

      default_f = False

      if (query_f):
        if ((ccr_int != None) or (updr_int != None)):
           print("query detected, ignoring other options")
      else:
        if ((ccr_int == None) and (updr_int == None)):
           default_f = True
           print("No options detected, setting magnetometer defaults")
           ccr_int = MAG_CCR_DEF
           updr_int = MAG_UPDR_DEF
        else:
           if (((ccr_int == None) and (updr_int != None)) or ((ccr_int != None) and (updr_int == None))):
              err_f = True
              status = -4
              print("Error, both -ccr and -updr must be set or not set as a pair")
           # endif missmatched options
        # end else not defaults
      # end else not query

      if ((not err_f) and query_f):
        base_str = "$PMITMG?"                              # mag query
        pre_cksum_str = base_str + "*"
        #
        err_f, packet_out = add_cksum(pre_cksum_str)
        #
        if (err_f):
          print("Error: setMag(): add_cksum()")
          status = -2
        else:
          if (debug_f):
            print("packet_out=", packet_out)
          # endif debug
          err_f, err_code = self.wrapped_send(packet_out)   # send mag rate
          if (err_f):
            status = -10 + err_code # return code ranges from -1 to -10
          # endif error
        # end else not error
      # endif not error and yes query
      #
      if ((not err_f) and (not query_f)):
        if ((ccr_int >= 50) and (ccr_int <= 400)):
          pass
        else:
          status = -5
          err_f = True
          print("Error: setMag(): ccr not in range:",ccr_int,"high: %d, low: %d"%(MAG_CCR_MAX,
                                                                                  MAG_CCR_MIN))
        # end else out of range
        # 
        if (not err_f):
          if ((updr_int >= 146) and (updr_int <= 159)):
            pass
          else:
            err_f = True
            status = -6
            print("Error: setMag(): updr_int not in range:", updr_int,"high: %d, low: %d"%(MAG_UPDR_MAX,
                                                                                           MAG_UPDR_MIN))
          # end else out of range
        # endif not error
        #
        if (not err_f):
          base_str = "$PMITMGS"                              # mag setting
          pre_cksum_str = base_str + "," + str(ccr_int) + "," + str(updr_int) + "*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str)
          #
          if (err_f):
            print("Error: setMag(): add_cksum()")
            status = -7
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            err_f, err_code = self.wrapped_send(packet_out)   # send mag rate
            if (err_f):
              status = -20 + err_code # return code ranges from -1 to -10
            # endif error
          # end else not error
        # endif not error
      # endif not error and  not query
      #
      if (err_f):
        print("No command knowingly sent to AFE")
      # endif
      #
      return status

    # end setMag
    #
    # -------------------------------------------------------------------------
    #
    def setImu(self,theCmd):
      global g_odrList     

      status = 0
      err_f = False
      debug_f = True
      base_str = ""        # initializer
      mag_str = ""        # initializer
      pre_cksum_str =""    # initializer
      exit_f = False
     
      if (not self.isConnected()):
        print("setTelemRate: afe is not connected.")
        status = -1
        err_f = True
        return status
      # endif   

      try:
        shlex_out = shlex.split(theCmd)
      except Exception as eobj:        # likely no closing quotations
        print("setTelemRate: shlex exception:",eobj)
        err_f = True
        status = -2
        return status
      # endif                               # still here? 
        
      try:
        exit_f,args = parse_setimu(shlex_out) # parse send options
      except Exception as eobj:
        err_f = True
        err_str = "options exception: 15"
        err_code = -3
        eobj_str = repr(eobj)
        errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
        raise UserDefinedException(errDict)
      else:
        # ---------------------------
        #  -h detected or bad option
        # ---------------------------
        if (exit_f):          # -----------------
          return status       #   early return
        # endif               # -----------------
      # end else

      acc_lbl = args.acc 

      gyro_lbl = args.gyro
  
      acc_perf_f = args.hi_perf_acc

      gyr_lpwr_f = args.low_pwr_gyro

      acc_ulp_f = args.ultralowacc

      if (acc_ulp_f and acc_perf_f):
        print("Error: can't select both ultralow and high perf for acc")
        err_f = True
        status = -4
        return status
      # endif can't do selection

      acc_perf_lbl = "0"
      if (acc_perf_f):
        acc_perf_lbl = "1"
      # endif 

      acc_ulp_lbl = "0"
      if (acc_ulp_f):
        acc_ulp_lbl = "1"
      # endif 


      gyr_lpwr_lbl = "0"
      if (gyr_lpwr_f):
        gyr_lpwr_lbl = "1"
      # endif 

      query_f = args.query

      default_f = False

      if ((not err_f) and query_f):
        if ((acc_lbl != None) or (gyro_lbl != None)):
           print("query detected, ignoring other options")
        # endif other params check
        base_str = "$PMITIM?"                              # mag query
        pre_cksum_str = base_str + "*"
        #
        err_f, packet_out = add_cksum(pre_cksum_str)
        #
        if (err_f):
          print("Error: setMag(): add_cksum()")
          status = -5
        else:
          if (debug_f):
            print("packet_out=", packet_out)
          # endif debug
          err_f, err_code = self.wrapped_send(packet_out)   # send mag rate
          if (err_f):
            status = -10 + err_code # return code ranges from -1 to -10
          # endif error
        # end else not error
      # endif not error and yes query
      #
      if ((not err_f) and (not query_f)):
        #
        # parameter verification
        #
        acc_f = isInlist(g_odrList, acc_lbl)
        gyr_f = isInlist(g_odrList, gyro_lbl)
        if ((not acc_f) or (not gyr_f)):
           err_f = True
           status = -6

           if (not gyr_f):
             print("label: %s not found in list: %s"%(gyro_lbl, g_odrList))
           # endif label check fails

           if (not acc_f):
             print("label: %s not found in list: %s"%(acc_lbl, g_odrList))
           # endif label check fails
        else:
          base_str = "$PMITIMU"                              
          pre_cksum_str1 = base_str + "," + acc_lbl + "," + gyro_lbl +  "," + acc_perf_lbl 
          pre_cksum_str2 = pre_cksum_str1  + "," + acc_ulp_lbl + "," + gyr_lpwr_lbl + "*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str2)
          #
          if (err_f):
            print("Error: setMag(): add_cksum()")
            status = -7
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            err_f, err_code = self.wrapped_send(packet_out)   # send mag rate
            if (err_f):
              status = -20 + err_code # return code ranges from -1 to -10
            # endif error
          # end else not error
        # endif not error
      # endif not error and  not query
      #
      if (err_f):
        print("No command knowingly sent to AFE")
      # endif
      #
      return status

    # end setImu
    #
    # -------------------------------------------------------------------------
    #
    """
      misc:
       0 - TRIG_TX_SRC_SEL
       1 - TRIG_RX_SRC_SEL
       2 - EXT_TX_TRIG_ENABLE
       3 - EXT_RX_TRIG_ENABLE
       4 - NOT_USED
       5 - EBIAS_EN
       6 - GPIO_SPI_TEST
       7 - PPS_SOURCE_SEL
       8 - REF_SOURCE_SEL
       9 - REF_SOURCE_SEL

      tx:
       0 - NOT USED
       1 - TX_BLANK_SEL
       2 - FILTER_BYPASS_SEL
       3 - AMP_BYPASS_SEL
       4 - NOT USED
       5 - NOT USED
       6 - NOT USED
       7 - NOT USED
       8 - NOT USED
       9 - CH_SPI_TET

      rx:
       0 - CHAN_BIAS_EN
       1 - INT_RF_TRIG_SEL
       2 - FILTER_BYPASS_SEL
       3 - AMP_BYPASS_SEL
       4 - ATTEN_C1
       5 - ATTEN_C2
       6 - ATTEN_C4
       7 - ATTEN_C8
       8 - ATTEN_C16
       9 - CH_SPI_TET
    """
    def setMax(self,theCmd):
      global g_odrList     

      status = 0
      err_f = False
      debug_f = False
      exit_f = False
     
      if (not self.isConnected()):
        print("setTelemRate: afe is not connected.")
        status = -1
        err_f = True
        return status
      # endif   

      try:
        shlex_out = shlex.split(theCmd)
      except Exception as eobj:        # likely no closing quotations
        print("setTelemRate: shlex exception:",eobj)
        err_f = True
        status = -2
        return status
      # endif                               # still here? 
        
      try:
        exit_f,args = parse_setmax(shlex_out) # parse send options
      except Exception as eobj:
        err_f = True
        err_str = "options exception: 15"
        err_code = -3
        eobj_str = repr(eobj)
        errDict = {'err_code': err_code, 'err_str': err_str, 'eobj_str': eobj_str}
        raise UserDefinedException(errDict)
      else:
        # ---------------------------
        #  -h detected or bad option
        # ---------------------------
        if (exit_f):          # -----------------
          return status       #   early return
        # endif               # -----------------
      # end else

      query_f   = args.query   # 1
      tx_val    = args.tx      # 2
      rx_val    = args.rx      # 3
      bit_str   = args.bits    # 4
      bit_index = args.index   # 5

  
      if (debug_f):
        print("query_f=", query_f)     # 1
        print("tx_val=", tx_val)       # 2
        print("rx_val=", rx_val)       # 3
        print("bit_str=", bit_str)     # 4
        print("bit_index=", bit_index) # 5
      # endif debug

      # --------------------
      # set default options
      # --------------------

      if (not query_f):
        if (bit_index == None):
           bit_index = 0
        # endif option not set
      # endif not query

      # ---------------------------------
      # options checks with early return
      # ---------------------------------

      if ((tx_val != None) and ((tx_val <= 0) or (tx_val > 2))):
         status = -4
         print("Error: bad tx, must be in range 1..2 or not used")
         print("No command sent to AFE")
         return status
      # endif bad logics test

      if ((rx_val != None) and ((rx_val <= 0) or (rx_val > 4))):
         status = -5
         print("Error: bad rx, must be in range 1..4 or not used")
         print("No command sent to AFE")
         return status
      # endif bad logics test

      if ( (tx_val != None) and (rx_val != None)):
         status = -6
         print("Can't command with both tx and rx option set")
         print("No command sent to AFE")
         return status
      # endif bad logics test
     
      if ( (bit_str == None) and (not query_f)):
         status = -8
         print("Can't command with bit string missing")
         print("No command sent to AFE")
         return status
      # endif bad logics test

      if ( (bit_str != None) and query_f):
         status = -9
         print("Can't query with bitlist or bit_index option set")
         print("No command sent to AFE")
         return status
      # endif bad logics test 

      if ( (bit_index != None) and ((bit_index < 0) or (bit_index > 9))):
         status = -10
         print("bit_index option out of range (0..9): %d"%(bit_index))
         print("No command sent to AFE")
         return status
      # endif bad logics test 

      if (bit_str != None):
        bitList = bit_str.split(",")  
        cmded_bits = len(bitList) + bit_index   
        if (cmded_bits > 10):
          status = -11
          print("Error: bit list (%d) plus bit index (%d)  > total settable bits (10)"%(len(bitList), 
                                                                                        bit_index))
          print("No command sent to AFE")
          return status
        # endif bad logics test
        else:
          ii = 0
          while (ii < len(bitList)):
            aa = bitList[ii]
            if ((aa == '0') or (aa == '1') or (aa == 'x') or (aa == 'X')):
              pass
            else:
              status = -12
              print("Error: bit list valid chars are comma separated '0' '1' 'X' or 'x', found:",aa)
              print("No command sent to AFE")
              return status
            # end else 
            ii = ii + 1
          # end while
        # end else bad logics test
      # endif data  in bit_str

      # -----------------------------
      # query command
      # -----------------------------
      if (query_f):
        if ((tx_val == None) and (rx_val == None)):
          base_str = "$PMITMA?"                           # max misc query
          pre_cksum_str = base_str + "*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str)
          #
          if (err_f):
            print("Error: setMax(): add_cksum()")
            status = -13
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            err_f, err_code = self.wrapped_send(packet_out)   # send packet
            if (err_f):
              status = -20 + err_code # return code ranges from -1 to -10
            # endif error
          # end else not error
        # endif not tx, rx query
        elif (tx_val != None):
          base_str = "$PMITXT%d?"%(tx_val)               # max tx query
          pre_cksum_str = base_str + "*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str)
          #
          if (err_f):
            print("Error: setMax(): add_cksum()")
            status = -14
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            err_f, err_code = self.wrapped_send(packet_out)   # send packet
            if (err_f):
              status = -40 + err_code # return code ranges from -1 to -10
            # endif error
          # end else not error
        elif (rx_val != None):
          base_str = "$PMITXR%d?"%(rx_val)               # max rx query
          pre_cksum_str = base_str + "*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str)
          #
          if (err_f):
            print("Error: setMax(): add_cksum()")
            status = -15
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            err_f, err_code = self.wrapped_send(packet_out)   # send packet
            if (err_f):
              status = -50 + err_code # return code ranges from -1 to -10
            # endif error
          # end else not error
           
      # -----------------------------
      # bits command
      # -----------------------------
      if (not query_f):
        if ((tx_val == None) and (rx_val == None)):
          base_str = "$PMITMAX"                           # max misc bits
          pre_cksum_str = base_str + "," + str(bit_index) + "," + bit_str +"*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str)
          #
          if (err_f):
            print("Error: setMax(): add_cksum()")
            status = -16
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            err_f, err_code = self.wrapped_send(packet_out)   # send packet
            if (err_f):
              status = -60 + err_code # return code ranges from -1 to -10
            # endif error
          # end else not error
        # endif not tx, rx query
        elif (tx_val != None):
          base_str = "$PMITXT%d"%(tx_val)               # max tx bits
          pre_cksum_str = base_str + "," + str(bit_index) + "," + bit_str + "*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str)
          #
          if (err_f):
            print("Error: setMax(): add_cksum()")
            status = -17
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            err_f, err_code = self.wrapped_send(packet_out)   # send packet
            if (err_f):
              status = -70 + err_code # return code ranges from -1 to -10
            # endif error
          # end else not error
        elif (rx_val != None):
          base_str = "$PMITXR%d"%(rx_val)               # max rx bits
          pre_cksum_str = base_str + "," + str(bit_index) + "," + bit_str + "*"
          #
          err_f, packet_out = add_cksum(pre_cksum_str)
          #
          if (err_f):
            print("Error: setMax(): add_cksum()")
            status = -18
          else:
            if (debug_f):
              print("packet_out=", packet_out)
            # endif debug
            err_f, err_code = self.wrapped_send(packet_out)   # send packet
            if (err_f):
              status = -80 + err_code # return code ranges from -1 to -10
            # endif error
          # end else not error      

      # endif query
 
      return status

    # end setMax
    #
    # -------------------------------------------------------------------------
    #
    def enable_logging(self,lfn):
       status =  0
       logging.basicConfig(filename= lfn,level=logging.DEBUG,
                    format='%(asctime)s %(message)s',
                    filemode='a',
                    datefmt='%m-%d-%YT%I:%M:%S')

       logging.Formatter.converter = time.gmtime

       self.lfp = True
       print("logging enabled: %s"%(lfn))
       logging.info("log file opened")
       #
       # you can turn logging on but can't turn logging off
       #
       return status
    # end enable_logging
    #
    # -------------------------------------------------------------------------
    #
    def process_line(self, line):
        self.quit_f = False
        status = 0
        err_f = False
        port_value = None
        debug_f = False   # dump parameters and exit
        
        if len(line) == 0:
            return status

        if line == '\n':  # skip empty lines
            return status

        param = line.lstrip(' ')

        if (len(param) == 0):
          return status

        if param[0] == '#':  # comment
            return status

        if self.lfp != None:  # if logger,then log
            logging.info('cmd: '+line)

        keyword = '?'                            # *** ? ***
        kidx = len(keyword)  # index past keyword
        if param[0:kidx] == keyword:
          print_help()
          return status
        #
        # The following are of the form: afe <cmd>
        #
        cmd_handled = False
        keyword = 'afe'                  # *** afe ***
        kidx = len(keyword)  # index past keyword
        if param[0:kidx] == keyword:
            ipaddr = ''
            port_value = None
            wait_value = 30               # default wait
            param1 = param[kidx:]
            kidx = kidx + 1
            param1 = param1.lstrip(' ')
            #print("options=",param1)
            if param1[0:4] == 'open':     #  *** afe open *** 
                cmd_handled = True
                if self.isConnected():
                    #print('already open') # not an error
                    return status

                param2 = param1[kidx:]
                param2 = param2.lstrip(' ')
                space_idx = param2.find(' ')
                #print("\nparam2=",param2,"space_idx=",space_idx)
                if (space_idx < 0):          # input params not on command line
                   self.start_threads(True)    # if previously opened, reopen
                                               #    using saved params 
                else:                          # grab params
                    ipaddr = param2[0:space_idx]
                    param3 = param2[space_idx:]
                    param3 = param3.lstrip(' ')
                    space_idx = param3.find(' ')
                    wait_f = False
                    if (space_idx < 0):
                        port_value = param3
                    else:
                        port_value = param3[0:space_idx]
                        param4 = param3[space_idx:]
                        param4 = param4.lstrip(' ')
                        wait_f = True

                    if wait_f:
                        keyword = '-w'
                        kidx = len(keyword)
                        if param4[0:kidx] == keyword:
                            param5 = param4[kidx:]
                            wait_string = param5
                            wait_value = wait_string.lstrip(' ')
                            try:
                                wait_value = int(wait_string)
                            except Exception as eobj:
                                emsg= 'wait value must be an integer:%s'\
                                     % wait_string, eobj
                                print(emsg)
                                if self.lfp != None:
                                  logging.error(emsg)
                                status = -2
                                return status
                        else:
                            emsg = "unrecognized: \'%s\'" % param4
                            print(emsg)
                            if self.lfp != None:
                              logging.error(emsg)
                        # endif looking for  wait else not found
                    # endif need to look for wait
                    try:
                        port_value = int(port_value)
                    except Exception as eobj:
                        emsg='port must be integer: %s'%(port_value)
                        print(emsg)
                        if self.lfp != None:
                          logging.error(emsg)
                        status = -3
                        return status
                    #
                    if (port_value != None):                     
                      self.port = port_value
                      print("self.port_value=",port_value)
                      print("self.baud_rate=",self.baud_rate)
                    # endif
                    self.sock_wait_f = wait_f
                    self.sock_wait_value = wait_value                    
                    #
                    # also called from thread if drop detected
                    #            
                    if (USE_SOCKET):          # socket or usb?
                      self.ipaddr = ipaddr
                      self.port   = port
                    else:
                      self.serPath   = ipaddr
                      self.baud_rate = port_value
                    # endif
                    
                    if (USE_SOCKET):
                      print("\naddr=",self.ipaddr)
                      print("port=",  self.port)
                    else:
                      print("port=",    self.serPath)
                      print("baud_rate=",self.baud_rate)
                    # end else

                    self.start_threads(True)
                  
                # end else more on the line
            # endif search for 'afe open'
            else:                 
                keyword = 'close'              #  *** afe close *** 
                kidx = len(keyword)
                if param1[0:kidx] == keyword:
                    cmd_handled = True
                    self.Cleanup()             # close threads
                # end if search for 'afe close'
            # end else command not handled 
            if (not cmd_handled):                                                      
                keyword = 'send'            #  *** afe send *** 
                kidx = len(keyword)
                if param1[0:kidx] == keyword:
                    cmd_handled = True
                    param2 = param1[kidx:]
                    param3 = param2.lstrip(' ')
                    param4 = param3.rstrip(' ')
                    qmsg = param4.strip()
                    #print("qmsg=", qmsg,"line=",line,
                    #      "param=",param,
                    #      "param1=",param1,
                    #      "param2=",param2)                   
                    try:
                       status, resp = self.afe_send(qmsg)
                    except Exception as eobj:
                          print("Exception in afe_send: 1,", eobj) # disambiguate
                          status = -4
                          err_f = True
                          g_response_f = False
                          return status;
                    else:
                       pass
                # end if search for 'afe send'
            # endif command not handled 

            if (not cmd_handled):                                                      
                keyword = 'settime'            #  *** afe settime *** 
                kidx = len(keyword)
                if param1[0:kidx] == keyword:
                    cmd_handled = True
                    param2 = param1[kidx:]
                    status = self.setTime(param2)
                # endif search for 'afe <cmd>'
            # endif command not handled  

            if (not cmd_handled):                                                      
                keyword = 'verifytime'            #  *** afe verifytime *** 
                kidx = len(keyword)
                if param1[0:kidx] == keyword:
                    cmd_handled = True
                    iter = 1                     # iterations
                    param2 = param1[kidx:]
                    if (len(param2) > 0):
                      param2a = param2.lstrip(' ')
                      param2b = param2a.rstrip(' ')
                      try:
                         iter = int(param2b)
                      except Exceptions as eobj:
                         print("Exception in process_line:",eobj,"- count expected") 
                         err_f = True 
                         status = -5
                      # end except  
                    # endif iterations entered
                    if (not err_f):                  
                      status = self.verifyTime(iter)
                    # endif
                # endif search for 'afe <cmd>'
            # endif command not handled 

            if (not cmd_handled):                                                      
                keyword = 'setrate'            #  *** afe setrate *** 
                kidx = len(keyword)
                if param1[0:kidx] == keyword:
                    cmd_handled = True
                    param2 = param1[kidx:]
                    status = self.setTelemRate(param2)
                # endif search for 'afe <cmd>'
            # endif command not handled  

            if (not cmd_handled):                                                      
                keyword = 'setimu'            #  *** afe setimu *** 
                kidx = len(keyword)
                if param1[0:kidx] == keyword:
                    cmd_handled = True
                    param2 = param1[kidx:]
                    status = self.setImu(param2)
                # endif search for 'afe <cmd>'
            # endif command not handled  

            if (not cmd_handled):                                                      
                keyword = 'setmax'            #  *** afe max *** 
                kidx = len(keyword)
                if param1[0:kidx] == keyword:
                    cmd_handled = True
                    param2 = param1[kidx:]
                    status = self.setMax(param2)
                # endif search for 'afe <cmd>'
            # endif command not handled  


            if (not cmd_handled):                                                      
                keyword = 'setmag'            #  *** afe settime *** 
                kidx = len(keyword)           # > afe setmag [200 150]
                if param1[0:kidx] == keyword:
                   cmd_handled = True     
                   param2 = param1[kidx:]               
                   status = self.setMag(param2)
                # endif search for 'afe setmag'
            # endif command not handled  

            if (not cmd_handled):                  # none of the above
                emsg= "Cannot parse: \'%s\'" % (param) + " note: options go at end of line"
                cmd_handled = True
                print(emsg)
                if self.lfp != None:
                   logging.error(emsg)
            # endif not command handled
        # endif afe
        #
        # The above are of the form: 'afe <cmd>'
        #
        if (not cmd_handled):
              keyword = 'scr '             #  *** script ***
              kidx = len(keyword)
              if param[0:kidx] == keyword:
                cmd_handled = True
                try:
                  self.do_script_cmd(line,kidx)
                except UserDefinedException as ude:
                   try:
                     errDict = ude.value
                   except Exception as eobj:
                     print("Other exception in self.do_script_cmd()", eobj)
                     err_f = True
                   else:
                     print("Exception:",errDict['err_str'])
                     err_f = True    

        if (not cmd_handled):  
                keyword1 = 'quit'              # *** quit  *** 
                kidx1 = len(keyword1)
                keyword2 = 'exit'
                kidx2 = len(keyword2)
                if param[0:kidx1] == keyword1 or param[0:kidx2]\
                     == keyword2:
                    cmd_handled = True
                    print('exit or quit detected')
                    self.Cleanup()
                    self.quit_f = True
                # endif match quit

        if (not cmd_handled):                
                keyword = 'log'                  #   *** log ***
                kidx = len(keyword)
                if param[0:kidx] == keyword:
                    cmd_handled = True
                    param1 = line[kidx:]
                    param1 = param1.lstrip(' ')
                    param1 = param1.rstrip(' ')
                    param1 = param1.rsplit(' ')  # remove extra params
                    param1 = param1[0]
                    lfilenm = param1
                    if (lfilenm.rfind('.') == -1):
                       lfilenm = lfilenm + '.log'
                    err_f = self.enable_logging(lfilenm)
                # endif match log

        if (not cmd_handled):                 
                keyword = 'delay'                # *** delay *** 
                kidx = len(keyword)
                if param[0:kidx] == keyword:
                    cmd_handled = True
                    param1 = param[kidx:]
                    param1 = param1.lstrip(' ')
                    param1 = param1.rstrip(' ')
                    param1 = param1.rsplit(' ')  # remove extra params
                    param1 = param1[0]
                    try:
                        delay_val = int(param1)
                    except Exception as eobj:
                        emsg= 'delay value, integer expected: '+ str(param1)
                        print(emsg)
                        if self.lfp != None:
                           logging.error(emsg)
                        status = -10
                        return status
                    time.sleep(delay_val)
                # endif match delay
        # endif command not handled
        
        if (not cmd_handled):  # none of the above
                emsg= "\nunrecognized command: \'%s\'" % (param)
                print(emsg)
                if self.lfp != None:
                   logging.error(emsg)
        
        if (status < 0):
          print ("Error detected processing line, code=",status)
        # endif error detected

        return status

    # end process_line
    #
    # -------------------------------------------------------------------------
    #
    def uart_listen(self,wait_value):
      global g_response_f
      status = 0           # return status
      err_f = False        # set on error
      tmo_f = False        # timeout is a flag, not an error
      lf_f = False
      listen_buf = ""      # the response
      debug_f = False
      
              
      if (not self.connected):
        status = -1
        return (status, lf_f, tmo_f, listen_buf)  # early return
      # endif not connected
      #
      # if there is an expected response
      #   then suppress printing at this low level
      #
      

      if (debug_f):
         print("suppress_f=", suppress_f)
      # endif debug_f
      
      if (wait_value != -1):          # init timeout on wait for data
        time_start = time.time()
      # endif 
      lf_f = False               # set true when data available
      total = 0
      listen_buf = ''
      while True:                     # the NOT QUITE FOREVER loop 
          suppress_f =  g_response_f  
          err_f = False  
          err_code, piece = self.getSerialData()         
          #
          if (err_code < 0):          # error detected
             print("error detected in getSerialData():",err_code)
             err_f = True
             status = -2
             break
          # endif
          if (err_code > 0):                    # data available
            #
            # on startup/restart there can be binary noise on the line
            # that gets mistaken for csp data
            #
            if (isinstance(piece,list)):
              print("dropping unexpected binary data:",piece)
            else:
              listen_buf = listen_buf + piece
              if (listen_buf.rfind('\n') >= 0):   # data terminator
                #char_dump(listen_buf)         # diagnosing protocol problem
                lf_f = True
                break
              # end if end of line
            # end else not noise
            # end else no exception on read data
          else:                           # else no data
             err_f = True                 # check for timeout
          # end else no data
          #
          # error checking 
          #
          if (err_f):
              if wait_value != -1:
                  time_now = time.time()
                  if time_now >= time_start + wait_value:
                      tmo_f = True
                      break                  # timeout break
              # end else not timeout
          # endif error
          #    
      # end while not quite forever
      #
      if (lf_f):
        #
        # the purpose of the lock is to protect the listen buffer
        # from being modified and read at the same time
        #
        self.lock.acquire()               # acquire lock on buffer
        self.listen_buf = listen_buf
        #
        if (not suppress_f):               # if interactive....
          #                                # don't double-print linefeeds
          if (len(listen_buf) > 1):
            #print(": ",  end=" ")        # added chars can add confusion
            #
            if (listen_buf[-1] == '\n'):   # terminator (protocol)
              try:
                print("ul=",listen_buf, end=" ") # response has lf
              except Exception as eobj:
                pass
              # end exception
              #char_dump(listen_buf)
              #sys.stdout.flush()             # make sure print is seen
              #                      
          # endif data in listen buffer
        # endif
    
        #
        if self.lfp != None:
            tidx = listen_buf.find("\r")   # logging adds crlf
            if (tidx < 0):
              tidx = len(listen_buf)
            # endif 
            logging.info(listen_buf[0:tidx])
        # endif logging
        #
        self.lock.release()  # buffer release
        #
      # endif linefeed
      #
      return (status, lf_f, tmo_f, listen_buf)
      #
    # end uart_listen
    #
    # -------------------------------------------------------------------------
    #
    def main_loop(self):

        self.interactive_f = True   # echo device replies to console
        self.lock.acquire()
        print('''\n Interactive afe debugger v1.0\v''')
        self.lock.release()

        if (self.success_f):                 # if init success
          if (self.startup_script != None):  # if there is a script to run
            self.do_script_cmd(self.startup_script,0,user_f=False)
          # endif script
        # endif startup success

        while 1:
            
            line = ""
            line = history_input(self.prompt)     # print prompt, get line

            if ((line == '\n') or (len(line)==0)):   # skip empty lines
                continue
            # endif empty line

            status = self.process_line(line)  # process line
            if (self.quit_f):  # if quit then quit
                print('exiting program')
                break
             
            time.sleep(0.1)    # reduce chance of collision w/processing messages
        # end while

        if ((self.bk_listen != None) or (self.quit_f)):  # shutdown thread
            self.Cleanup()
            self.bk_listen = None
            print("afe listener shut down")

    # end main_loop
    #
    # -------------------------------------------------------------------------
    #
# end class
#
# -------------------------------------------------------------------------
# -------------------------------------------------------------------------
#
class listenThread (threading.Thread):  # background thread
    def __init__(self,intrfc, period, listen_f):
        threading.Thread.__init__(self)
        self.env = intrfc      # the calling object's self
        self.running = False      # thread loop control
        self.period = period      # listen period 
        self.listen_f = listen_f  # disable listen (listen elsewhere) 
        self.response = ""        # for building multiline  
    # end __init__
    #
    # -------------------------------------------------------------------------
    #
    def run(self):                   # call .start() NOT .run()
        env = self.env               # the afecmds class
        disconnected_f = False
        self.running = True          # this thread is alive
        
        time.sleep(2)                # wait for threads to start
        #print("listen thread running")
        while (self.running):  # suppress listening to closed socket
            #print(".", end=" ",flush=True)  
            if env.isConnected():
              ping_ok_f = True 
              if (ping_ok_f):  
                disconnected_f = False  
                if (self.listen_f):    
                  #
                  # that this is in a thread allows for always listening
                  # if a response is expected from afe_send() then
                  #    suppress the lower level incoming data printing
                  #    because the will be passed to where appropriate for printing
                  # else
                  #   don't suppress and allow printing (for debugging)
                  #
                   
                  status,lf_f,tmo_f,lbuf = env.uart_listen(self.period)
                                                           
                  #print("lbuf=",lbuf,"status=",status)
                  if (status == -2):
                     disconnected_f = True          # usart read error
                  # endif disconnect error
                  else:
                     if (status >=0):
                       #
                       # the append allows multiline responses
                       # that can be read by read_accumulate()
                       #
                       self.response = self.response + lbuf
                       #print("(1)self.response=",self.response) 
                       
                     # end if building multiline response
                  # end else not disconnect error
                # endif listening on device
                else:
                  pass # time.sleep(0.1)     # avoiding a thread w/o OS calls
                #
              # end if check for still connected returns true
              #
              # ---------------------------------------
              #
              # This path is taken on connection error 
              #
              # ---------------------------------------
              #
              if (disconnected_f):
                 #
                 # The problem:
                 #    resetting the afe may cause garbage to sent
                 #    to the socket when the afe comes back
                 # The solution:
                 #   disconnect
                 #   attempt reconnect
                 #
                 if (disconnected_f):                 
                   wmsg = "closing intrfc, will reopen if test succeeds..." 
                   print(wmsg)
                   if env.lfp != None:
                      env.logging.info(wmsg)

                   env.myClose()                       # disconnect 
                          
                   while (self.running):                  # forever
                     time.sleep(5)                        # wait 
                     #                                    # attempt reconnect
                     success_f = False
 
                     success_f = env.myConnect(env.serPath,env.baud_rate)
   
                     if (success_f):
                        wmsg = "connection restored"
                        print(wmsg)
                        if env.lfp != None:
                           env.logging.info(wmsg)
                        break
                     else:
                          env.myClose()           # cleanup, try again
                     # end else close and retry
                   # end inner while forever
                 # end if disconnected
              # end if connect error 
        # end outer while
    # end run
    # 
    # -------------------------------------------------------------------------
    #
    def read(self):
      #
      # A common but always surprising issue is enabling character echo 
      #  on the afe side, done when debugging by minicom.
      # The effect is what appears to be the command showing up in the
      #  response. This is not the command, but the echo of the command.
      #
      #print("self.response=",self.response)
      return self.response
      
    # end read 
    #
    # -------------------------------------------------------------------------
    #
    def clear(self):
       self.pause()             # stop
       self.response = ""       # erase the whole
       self.listen_buf = ''     # erase the piece
       self.resume()            # start
    # end clear
    #
    # -------------------------------------------------------------------------
    #
    def pause(self):
       self.listen_f = False
    # end pause
    #
    # -------------------------------------------------------------------------
    #
    def resume(self):
       self.listen_f = True
    # end resume
    #
    # -------------------------------------------------------------------------
    #
    def shutdown(self):
        print('listener thread shutting down')
        self.running = False
        #self.kill()  # python2 artifact, for K(illable)Threads.py
    # end shutdown
    #
    # -------------------------------------------------------------------------
    #
  
# end class listenThread
#
#-------------------------------------------------------------------
#
# periodic thread, a useful tool for sending a python parseable command
#
class peThread(threading.Thread):

  def __init__(self,interface,    # self of the instantiator
                    command,      # the event to exec at interval
                    period,       # period is interval
                    max       = None,
                    verbose_f = False): 
      threading.Thread.__init__(self)   
      self. interface    = interface   
      self.command       = command     # 
      self.running       = False       # thread loop control
      self.period        = period      # period between sending events
      self.max           = max         # max number of events
      self.counter       = 0           # counts to max if max is set
      self.suspend_f     = False
      self.verbose_f     = verbose_f
  # end __init__
  #
  #-----------------------------------------------------------------
  #
  def run(self):                      # call .start() NOT .run()           
      interface     = self.interface
      self.running  = True   
      sleepTime     = 0.5   
      theCmd        = "self.interface.%s"%(self.command) 
      debug_f       = False

      if (self.verbose_f):
        print("task command=",theCmd)
      # endif verbose
  
      ii = 0
      while (self.running): 
        if (self.period > 0):
          time.sleep(sleepTime)
          ii = ii + sleepTime
          if (ii < self.period):
             continue
          else:
             ii = 0
          # endif not yet time to wake up
              
        if (not self.suspend_f):
          if (debug_f):
            try:
              print("cmd:",theCmd)
            except Exception as eobj:    # thread print collision
              pass
          # endif debug
          try:
            exec(theCmd)
          except Exception as eobj:
            print("Exception in peThread:", eobj)  # don't leave
            print( "Exec'd function:", theCmd)
            #
            # Don't die, there's nowhere to go
            #
          # end exception
                     
          if not (self.period > 0):
             print("peThread: bad param")
             self.shutdown() 
             break
          else:
             if (self.max != None):
               self.counter = self.counter + 1
               if (self.counter >= self.max):
                 self.shutdown("peThread: max count exceeded") 
                 break 
               # endif max count exceeded
             # endif max count set
          # end else not period 
        # end if not suspended
  #end run
  #
  #-----------------------------------------------------------------
  #
  def is_paused(self):
    return(self.suspend_f)
  # end is_paused
  #
  #-----------------------------------------------------------------
  #
  def pause(self):
    self.suspend_f = True
  # end pause
  #
  #-----------------------------------------------------------------
  #
  def resume(self):
    self.suspend_f = False
  # end resume
  #
  #-----------------------------------------------------------------
  #                                                                                           
  def shutdown(self,msg=None):   

      if (msg != None):
        print(msg) 
      # endif message
  
      self.running = False 
      #self.kill()  # python2 artifact, for K(illable)Threads.py 
  #end shutdown
  #
  #-----------------------------------------------------------------
  #
      
#end class peThread 
#
#-------------------------------------------------------------------
#
#
#-------------------------------------------------------------------
#
#              MAIN
#
#-------------------------------------------------------------------
#

if __name__ == '__main__':

    err_f = False

    signal.signal(signal.SIGINT, signal_handler) # for control c

    err_f, args = parse_command_line() # parse command line options
    if (err_f):
      print("exiting early")
      sys.exit()


    if (args.extra_help):
      print_help()
      print("exiting early")
      sys.exit()

    ipaddr  = None
    
    wait    = -1
    if (args.ipaddr != None):   # IP address, or path to serial device
      ipaddr = args.ipaddr
           
    port    = None
    if (args.port) != None:     
      port = args.port
    else:
      if (not USE_SOCKET):
        port = BAUD_RATE

    wait = args.wait 

    metadata = None
    if (args.metadata != None):
      metadata = args.metadata
    else:
      if (g_verbose):
        print("metadata path not provided, metadata collection disabled!")
      # endif

    script = None
    if (args.script != None):
      script = args.script

    g_verbose = args.verbose  # global

    afeObj = afecmds(ipaddr,port,wait,metadata,script)

    afeObj.main_loop()
#
#-------------------------------------------------------------------
#
#             END
#
#-------------------------------------------------------------------
#
