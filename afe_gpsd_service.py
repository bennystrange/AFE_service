#!/usr/bin/env python3
#
# afe_service.py
#
# MIT Haystack Observatory
# Ben Welchman 06-23-2025 -- 07-03-2025
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
#   request_reg_states
#   main_loop
#
# --------------------------


# TODO:
#
# - commenting and housekeeping
# - integrate tuner sock
# - remove unneccessary class functions (self.clear)
# - fix print functionality
# - add cleanups and failsafe for gpsd not connecting

# >> sudo gpsd --nowait --badtime --readonly --speed 460800 -D 3 /var/run/gpsd.sock /dev/ttyGNSS1

import os
import time
import socket
import threading
from datetime import datetime, timezone
import signal
import sys
import numpy as np
import csv

SOCKET_PATH = '/tmp/afe_service.sock'

global device
global rate
global new_run

global gpsd_in
global gpsd_out

device = '/dev/ttyGNSS1'
rate = 60

def send_nmea_command(cmd_str):

  device = "/dev/ttyGNSS1"
    
  if not cmd_str.endswith("\r\n"):
    cmd_str += "\r\n"

  hexcmd = cmd_str.encode("ascii").hex()
  message = f"&{device}={hexcmd}\n"

  gpsd_out.sendall(message.encode("ascii"))

  # gpsd sends "OK\n" or "ERROR\n" back
  reply = gpsd_out.recv(16).decode("ascii").strip()
  print(reply)

  if reply != "OK":
    raise RuntimeError(f"gpsd error on send: {reply}")

class Telemetry:

    def __init__(self):

      self.gps = []
      self.telem = []
      self.registers = []
      self.RTCtime = None

    def add_gps(self, data):

      self.gps.append(data)

    def add_telem(self, data):

      self.telem.append(data)

    def add_registers(self, data):

      self.registers.append(data)

    def request_telem(self):

      self.telem.clear()

      msg_draft = "$TELEM?*"
      msg = add_cksum(msg_draft)

      send_nmea_command(msg)

    def size(self):
        return len(self.telem)

    def print(self):

      self.request_telem()

      request_reg_states()

      return np.stack(self.telem, self.registers)

    def log(self):

        global new_run

        global path

        self.request_telem()

        if len(self.registers) == 0:
          request_reg_states()

        if self.RTCtime is None:
          self.RTCtime = int(datetime.now(timezone.utc).timestamp())
        
        t = datetime.fromtimestamp(self.RTCtime, tz=timezone.utc)
        timestamp = t.strftime("%Y-%m-%d_%H:%M:%S")

        if new_run is True:
          base = "/data/telemetry_log"
          folder_name = f"mep-telemetry-log_{timestamp}"
          path = os.path.join(base, folder_name)
          os.makedirs(path, exist_ok=True)
          new_run = False

        filename = f"telemetry_{timestamp}.csv"

        full_path = os.path.join(path, filename)

        with open(full_path, "w", newline="", encoding="utf-8") as telem_csv:
          writer = csv.writer(telem_csv)
          for i in range(len(self.telem)):
            try:
              line = self.telem[i].split('*')[0]
              line = line.split(',')
              writer.writerow(line)
            except IndexError as e:
              print("Index Error on: ", line)

#          writer.writerow(<tuner>) # ADD TUNER TELEM HERE

          for row in range(7):

            if row == 0:
              line = ["MAINREG"]

            elif row in (1, 2):
              idx = str(row)
              line = ["TX" + idx + "REG"]

            elif row in (3, 4, 5, 6):
              idx = str(row - 2)
              line = ["RX" + idx + "REG"]

            for column in range(10):
              state = self.registers[row][column]
              line.append(state)

            writer.writerow(line)
            line = []

        print(self.registers)
        print(np.stack(self.telem))
        print("Telemetry logged at: ", filename) # /data/metadata

global_telemetry = Telemetry()

def handle_commands(conn):

  global rate

  raw = conn.recv(1024)
  if not raw:
    conn.close()
    return

  command = raw.decode('utf-8', errors='ignore').split()

  block, channel, addr, bit = map(int, command)

  if block in (0, 1, 2):          # Instruction is to write a register
    write_max(block, channel, addr, bit)
    global_telemetry.log()
    conn.sendall(b"AFE Controls Updated\n")

  elif block == 3:

    array = global_telemetry.print()
    msg = array.tobytes()
    print(msg)                                #    NEED TO FIX
    conn.sendall(b"Placeholder\n")

  elif block == 4:

    global_telemetry.log()
    conn.sendall(b"Telemetry logged\n")

  elif block == 5:
    rate = channel
    threading.Timer(rate-2, tick).start()
    str_rate = str(channel)
    print(rate)
    conn.sendall(b"Telemetry log period updated")

  conn.close()

def start_command_server():

  try:
    os.remove(SOCKET_PATH)
  except OSError:
    pass

  server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  server.bind(SOCKET_PATH)
  server.listen(1)

  while True:
    conn, _ = server.accept()
    threading.Thread(target=handle_commands, args=(conn,), daemon=True).start()

def gpsd_monitor():

  while True:
    buffer = b""
    try:
      chunk = gpsd_in.recv(1024)
      print("Chunk: ", chunk)
      if not chunk:
        break
      buffer += chunk
      while b'\n' in buffer:
        line, buffer = buffer.split(b'\n', 1)
        line = line.strip().decode("ascii", errors="ignore")

        print("Line Received: ", line)

        if line.startswith('$NEWGPS'):
          global_telemetry.gps.clear()

        elif line.startswith('$NEWTEL'):
          global_telemetry.telem.clear()

        elif line.startswith('$NEWREG'):
          global_telemetry.registers.clear()

        elif line.startswith('$G'):
          global_telemetry.add_gps(line)

          if line.startswith('$GNRMC'):
            error, RTCtime = NMEA_to_RTC(line)
            if not error:
              global_telemetry.RTCtime = RTCtime

        elif line.startswith('$PMIT') and '$PMITSR' not in line:
          global_telemetry.add_telem(line)

        elif line.startswith('$PMITSR'):
          row = []
          for i in range(14, 33, 2):
            row.append(i)
          global_telemetry.add_registers(row)

        else:
          continue

    except:
      time.sleep(0.005)

def ctrlc(signal, frame):

        sys.exit(0)

def NMEA_to_RTC(nmea_str):
  global g_rtc_init
  global g_time_NMEA_f
  global g_rtcObj
  global g_rtc_save

  rtcTime = None
  debug_f = False     # print useful info
  verify_f = True    # print time read vs time set
  err_f = False

  if (debug_f):
    print("NMEA time:", nmea_str)
  # endif debug

  aa = nmea_str.split(",")
  try:
    hh = int(aa[1][0:2])
    mm = int(aa[1][2:4])
    ss = int(aa[1][4:6])
    DD = int(aa[9][0:2])
    MM = int(aa[9][2:4])
    YY = int(aa[9][4:6]) + 2000
  except Exception as eobj:
    err_f = True             # no time values means no satellites
    g_time_NMEA_f = False
  else:
    dtobj = dt.datetime(YY, MM, DD, hh, mm, ss)
    rtcTime = dtobj._mktime()
   
    if (verify_f and g_rtc_init):    
      nowTime = time.time()
      if (rtcTime != nowTime):
        print("NMEA rtcTime:",rtcTime,"RTC rtcTime:",nowTime)
    # endif verify and something to verify agains

    g_rtcObj.datetime = time.struct_time((dtobj.year,    # tm_year
                                          dtobj.month,   # tm_mon
                                          dtobj.day,     # tm_mday
                                          dtobj.hour,    # tm_hour
                                          dtobj.minute,  # tm_min
                                          dtobj.second,  # tm_sec
                                          0, # tm_wday
                                         -1, # yday
                                         -1)) # tm_isdst
    #
    g_time_NMEA_f  = True      # time has been updated 
    g_rtc_save     = rtcTime   # saved time
    g_rtc_init     = True      # rtc has been initialized/reset
  # end else valid time
  
  return err_f, rtcTime

def reduce(iterable, initializer=None):
    it = iter(iterable)
    if initializer is None:
        value = next(it)
    else:
        value = initializer
    for element in it:
        value = value ^ element
    return value

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
       calculated_checksum = reduce((ord(s) for s in nmeadata), 0)
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

def write_max(block, channel, addr, bit):

  msg_draft = "$PMIT"

  if block == 0:
    msg_draft += "MAX"

  elif block == 1:
    msg_draft += "XT"
    msg_draft += str(channel)

  elif block == 2:
    msg_draft += "XR"
    msg_draft += str(channel)

  else:
    return 0

  msg_draft += "," + str(addr) + "," + str(bit) + "*"
  msg = add_cksum(msg_draft)

  send_nmea_command(msg)

  print("Writing to MAX: " + msg)

  return

def request_reg_states():

  global_telemetry.registers.clear()

  for query in ("MA?*", "XT1?*", "XT2?*", "XR1?*", "XR2?*", "XR3?*", "XR4?*"): 

    msg_draft = "$PMIT" + query
    msg = add_cksum(msg_draft)
    send_nmea_command(msg)
    time.sleep(1)  # TEMPORARY, ONLY FOR DEBUGGING

  return

def tick():

  global rate

  print("Periodic log:")
  global_telemetry.log()
  threading.Timer(rate-1, tick).start() #rate offset will need to vary as the script is updated, likely rate -1

def init_gpsd():

  global gpsd_in
  global gpsd_out

  gpsd_in = socket.create_connection(("127.0.0.1", 2947))
  gpsd_in.sendall(b'?WATCH={"enable":true,"raw":1};\n')  

  gpsd_out = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  gpsd_out.connect("/var/run/gpsd.sock")

def main():

  global rate

  signal.signal(signal.SIGINT, ctrlc) # for control c

  print("Initial log:")
  global_telemetry.log()

  threading.Timer(rate, tick).start()

  #while True:

if __name__ == '__main__':

  new_run = True

  init_gpsd()

  threading.Thread(target=start_command_server, daemon=True).start()
  threading.Thread(target=gpsd_monitor, daemon=True).start()

  main()