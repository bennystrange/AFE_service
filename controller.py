#
#  MIT Haystack Observatory
#  rps on or about 8/21/2024
#  Ben Welchman 08-01-2025
#

"""
 documents:
   mag: RM3100-RM2100-Magneto-Inductive-Magnetometer-User-Manual-V13.0.pdf

   overlapping but not identical, the 2nd below contains more detail
   imu: lsm6dso.pdf
        dm00517282-lsm6dso-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf

   MAX7317: https://www.analog.com/media/en/technical-documentation/data-sheets/max7317.pdf
        

 functions:
   dictInList        -         a specific dictionary in a list of dictionaries
   myljust           -         left justification for printing to console (debug)
   list_columns      -         pretty printing to console (debug)
   reduce            -         fancy way to apply function to each byte in packet
   xor               -         xor as a function
   eval_packet       -         calculate nmea checksum, versus received checksum
   add_cksum         -         calculate nmea checksum, add to packet without checksum
   rem_time_nmea     -         remove time from nmea message
   gen_mag_pkt       -         read time packet, regen as time plus mag packet
   gen_acc_pkt       -         read time packet, regen as time plus acc packet
   gen_gyr_pkt       -         read time packet, regen as time plus gyr packet
   gen_hk_pkt        -         read time packet, regen housekeeping packet
   inact_sec2code    -         sleep/inactive is a function of ODR, convert to code
   inactive_spb      -         sleep/inactive seconds per bit, depends on ODR
   get_voltage       -         adc raw to volts
   get_mag_tp        -         get, optionally print magnetometer temperature
   get_sw_tp         -         get, optionally print switcher temperature  
   get_imu_tp        -         get, optionally print imu temperature (from fifo)
   get_all_tps       -         call get_imu_tp, get_sw_tp, get_mag_tp
   isCW_OCXO_LOCKED  -         read of OXCO status bit
   init_uart         -         init uart
   send_telem        -         output nmea synthetic messages
   set_NMEA_acq      -         set flag to update time by NMEA
   get_NMEA_acq      -         get flag to update time by NMEA
   setTimeFromTs     -         set rtc from timestamp
   NMEA_to_RTC       -         decode NMEA time and apply as UTC
   forward_nmea      -         forward nmea
   cleanup_spi       -         deinit
   init_spi          -         init spi bus (chip select)
   imu_cmd_check     -         test for not reserved commands
   write_spi         -         write, readback
   maxStr2SpiList    -         utility for writeMAX
   writeMax          -         spi bus tailored for the MAX chip
   maxList2SpiOut    -         from list to spi command
   maxLStr2SpiOut    -         from string to spi command
   pr_mag_id         -         read, compare w/expected result
   pr_imu_id         -         read, compare w/eexpected result
   set_mag_params    -         encapsulates globals
   get_mag_params    -         encapsulates globals
   set_imu_params    -         encapsulates globals
   get_imu_params    -         encapsulates globals
   mag_eval          -         convert reading to measurement
   read_mag          -         one read
   init_mag          -         init to continuous read, do one read
   twos_comp         -         decode negative numbers
   is_imu_rdy        -         ready ready bits
   imu_temp_conv     -         bytes to degrees C
   imu_mode_check     -        power/performance mode settings
   init_imu          -         init imu
   read_imu          -         read acc, gyro, temperature
   loop_imu          -         loop for a count of data
   calc_tare         -         calculate offset for no motion
   zero_crossings    -         counts threshold crossings
   do_acc            -         tare acc, emit on zero crossings
   do_gyro           -         tare gyro, emit on zero crossings
   is_tilt           -         imu tilt?
   is_active         -         imu active? not idle (idle can lead to low power)
   odr_to_label      -         user feedback for debugging
   read_odr          -         read imu odr registers (Output Data Rate)
   reset_imu         -         reset imu
   init_all          -         initialize all
   odr2fifo_trate    -         set the imu fifo trate variable's bits
   odr2fifo_odr      -         set the imu fifo's odr control register 
   reset_fifo        -         reset imu fifo
   init_fifo         -         init imu fifo
   fifo_status       -         return imu fifo status
   decode_fifo_tag   -         decode imu fifo tag
   read_fifo_tag     -         read imu fifo tag
   read_fifo_data    -         read imu fifo data
   process_fifo      -         process imu fifo data
   backup_char       -         input console editing operation
   set_human         -         set and get for human interactivity
   get_human         -         set and get for human interactivity
   PMITTE_to_ts      -         extract ts from NMEA time message
   params2string     -         extract parameters for feedback NMEA response
   handle_time_cmd   -         vector for NMEA $PMITTx commands
   PMITR_to_rate     -         wrapper on common sequence, R = status rates
   handle_rate_cmd   -         vector for NMEA $PMITRx commands
   PMITMGS_to_vals   -         extract parameters from $PMIT MAG cmd
   handle_mag_cmd    -         vector for NMEA $PMITMGS command
   PMITIMU_to_vals   -         extract parameters from $PMIT IMU cmd
   handle_imu_cmd    -         vector for NMEA $PMITIMx command
   updateShadowMax   -         utility to update MAX shadow bits
   init_max          -         set the MAX defaults
   maxList2str       -         utility for NMEA max status
   handle_max_cmd    -         vector for NMEA commanding the MAX chips
   send_NMEA_ok      -         NMEA response if NMEA command completes ok
   send_NMEA_err     -         NMEA response if NMEA command completes not ok
   do_console        -         input command processing
   run_mode          -         executive loop and function tests
   my_main           -         code starts here, more or less
"""
import sys
import math
import io
import busio
from digitalio import DigitalInOut
import board
import microcontroller as mc
import bitbangio                       # allows any 3 pins to become a spi bus
import analogio
import time
import rtc                             # allows setting time.time() start time
import adafruit_datetime as dt
 
import supervisor                      # access to millisecond timing
supervisor.runtime.autoreload = False  # set False to prevent CIRCUITPY autoreload
import gc

#
# ----------------------------------
#     GLOBALS
# ----------------------------------
#

telemetry_due = False

USE_MINICOM = False     # set True for uart delays for slower host
g_human_f = False       # set True for echo and prompt, needs "quit" to exit console
                        # if using afecmds.py then > afe send "quit"

IAM_GNSS    = False      # set True for GNSS board, False for AFE board
if (IAM_GNSS == True):
  IAM_AFE = False
else:
  IAM_AFE = True
# end else

USE_IMU     = True       # early board spi bus had problems
USE_MAG     = True       # early board spi bus had problems
USE_FIFO    = True       # FIFO is faster than direct's wait until data ready
EXT_GPS     = True       # external GPS antenna, GNSS only, AFE uses MAX for this
BYPASS_LNA = False       # switchable LNA, GNSS only 

g_rtcObj = None          # realtime time object, init prior to use
g_rtcObj = rtc.RTC()     # if fails then exceptions, nothing time-dependent works 
g_rtc_init = False       # set True on handling time message
g_time_NMEA_f = False    # set true on NMEA has seen satellites
g_rtc_save = 0           # saved time used as threshold for periodic updates
#
# default startup time
#
g_nmea_time_f = True      # use NMEA to set a better time
g_rtcObj.datetime = time.struct_time((1970, 1, 1, 0, 0, 0, 0, -1, -1))

                          # values set to fastest w/o failing
                          # test range: 9600, 38400, 57600, 115200, 230400, 
                          #             460800,  921600, 1843200, 3686400
UART0_BAUD = 460800      # out to world (GNSS1) 921600  
UART1_BAUD = 1350000      # in from ublox 

g_uart_delay = 0.2  # slow down for host device 

g_mag_x = None  # magnetometer values for sharing
g_mag_y = None
g_mag_z = None

g_acc_x = None
g_acc_y = None
g_acc_z = None

g_gyr_x = None
g_gyr_y = None
g_gyr_z = None

#
# see handle_time_cmd()
#
TSRC_NOTSET = 0  # time source constants/enumerated types
TSRC_GNSS   = 1
TSRC_EXT    = 2
 
TEPOCH_NOTSET = 0 # time epoch constants/enumerated types
TEPOCH_PPS    = 1
TEPOCH_NMEA   = 2
TEPOCH_IMM    = 3

g_time_source = TSRC_GNSS     # time setting defaults
g_time_epoch  = TEPOCH_NMEA

#
# update rate max is 1
#
TELEM_UPD_DEF = 1 # default update rates
MAG_UPD_DEF   = 1
IMU_UPD_DEF   = 1


g_telem_updr  = TELEM_UPD_DEF # default update rates
g_mag_tupdr   = MAG_UPD_DEF   # "t" for telemetry note: there is also g_mag_iupdr
g_imu_updr    = IMU_UPD_DEF
g_telem_time  = None  # initial time for scheduling
g_mag_time    = None
g_imu_time    = None

#
# note: the mag params below are a 
#       direct copy-and-paste from
#       afecmds.py
#
# note: might want to consider 
#       whether to have a common import
#       for h/w configuration
#
MAG_CCR_MIN       = 50     # per user manual RM3100 & RM2100
MAG_CCR_MAX       = 400
MAG_CCR_DEF       = 200
MAG_UPDR_MIN      = 146
MAG_UPDR_MAX      = 159
MAG_UPDR_DEF      = 150

g_mag_ccr   = MAG_CCR_DEF     
g_mag_iupdr = MAG_UPDR_DEF  # "i" for internal note: there is also g_mag_tupdr


g_ocxo_lock = None   # ocxo locked
g_sw_tp     = None   # switch temperature
g_mag_tp    = None   # magnetometer temperature
g_imu_tp    = 0.0    # imu temperature, do not leave None when imu disabled
g_active    = 0      # imu active flag
g_tilt      = 0      # imu tilt flag

#
# threshold zero crossings
# value is a tradeoff on the quantity of false positives
# and false positive detection and filtering
#
ACC_MAX = 0.016   # 0.0036 leads to false positives
GYR_MAX = 0.20    # 0.15 leads to false positives

g_active_thresh = None # movement threshold for inactive

g_accOfs = [0,0,0]  # calibration data
g_gyrOfs = [0,0,0]  # calibration data
g_cal_acc_f = False # initialized flag
g_cal_gyr_f = False # initialized flag

#
# note: every register declared here must be added to g_imu_cmdList 
#       the purpose is to prevent accidental writing to "reserved" 
#       registers that could adversely alter firmware 
#
FIFO_CTRL1 = 0x7
FIFO_CTRL2 = 0x8 
FIFO_CTRL3 = 0x09
FIFO_CTRL4 = 0x0A
FIFO_STATUS1 = 0x3A
FIFO_STATUS2 = 0x3B
FIFO_DATA_OUT_TAG = 0x78
FIFO_DATA_OUT_X_L = 0x79

INT1_CTRL        = 0x0d    # interrupt out (is this used?)
CTRL1_XL         = 0x10    # sets acc ODR (output data rate)
CTRL2_G          = 0x11    # sets gyro ODR 
CTRL3_C          = 0x12    # BDU and SPI enabe
CTRL4_C          = 0x13    # various enables
CTRL5_C          = 0x14    # xl_ulp_en
CTRL6_C          = 0x15    # enable low power
CTRL7_G          = 0x16
STATUS_REG       = 0x1E    # data ready

#                     # temperatue
OUT_TEMP_L    = 0x20  # use first address, auto-increment
OUT_TEMP_H    = 0x21
#                     # gyro X,Y,Z values
OUTX_L_G      = 0x22  # use first address, auto-increment
OUTX_H_G      = 0x23
OUTY_L_G      = 0x24
OUTY_H_G      = 0x25
OUTZ_L_G      = 0x26
OUTZ_H_G      = 0x27
#                     # acc X,Y,Z values
OUTX_L_A      = 0x28  # use first address, auto-increment
OUTX_H_A      = 0x29
OUTY_L_A      = 0x2A
OUTY_H_A      = 0x2B
OUTZ_L_A      = 0x2C
OUTZ_H_A      = 0x2D

#                                # tilt, significant movement, tap
FUNC_CFG_ACCESS          = 0x01  # embedded features are "locked"
EMB_FUNC_EN_A            = 0x04                   
EMB_FUNC_INT1            = 0x0A  # tilt and signficant motion
PAGE_RW                  = 0x17  # emb dependency on non emb register
EMB_FUNC_STATUS_MAINPAGE = 0x35
WAKE_UP_THS              = 0x5b
WAKE_UP_DUR              = 0x5c
EMB_FUNC_INIT_A          = 0x66

#                                # activity/inactivity
WAKE_UP_SRC = 0x1B    # (R)
TAP_CFG0    = 0x56    # (R/W)
TAP_CFG2    = 0x58    # (R/W)

#                                 # ODR settings for registers: CTRL1_XL and CTRL2_G
ODR_OFF         = 0x0
ACC_1_6_HZ_ULP  = 0xB0  # _ULP is Ultra Low Power, ACC only
GYR_6_5_HZ_LP   = 0x0B
ODR_12_5_HZ_LP  = 0x10  # _LP can be active in power down
ODR_26_HZ_LP    = 0x20
ODR_52_HZ_NP    = 0x30  # _NP is Normal Power
ODR_104_HZ_NP   = 0x40
ODR_208_HZ_HP   = 0x50  # _HP is High Performance
ODR_416_HZ_HP   = 0x60
ODR_833_HZ_HP   = 0x70
ODR_1_66_KHZ_HP = 0x80
ODR_3_33_KHZ_HP = 0x90
ODR_6_66_KHZ_HP = 0xa0

NO_CHANGE    = 1    # configurable select for setting imu response to noactivity
ACC_ONLY     = 2
ACC_AND_GYRO = 3
INACT_EN     = ACC_ONLY

# 
# fifo notes:
#   note 1. fifo decompression algorithm isn't in the spec
#           
#   note 2. temperature is 2 bytes, but all 6 bytes 
#           do need to be read from fifo
#
ODR_T_1_6Hz    = 0x10
ODR_T_12_5Hz   = 0x20
ODR_T_52Hz     = 0x30
FIFO_CONT_MODE = 0x6
#
# default setting for FIFO_CTRL4
# temperature rate, no timestamp, continuous
#
g_fifo_mode = FIFO_CONT_MODE | ODR_T_12_5Hz 

uart0           = None   # write outide world as GNSS1
uart1           = None   # read ublox' second port
mag_temp_pin    = None   # A/D on this pin
switch_temp_pin = None   # A/D on this pin
imu_int1        = None   # imu interrupt pin, not used
#
# -----------------------------------------------------
#
# -----------------------------------------------------
#   FUNCTIONS OUT OF PLACE FOR A REASON
# -----------------------------------------------------
#
# see SLEEP_DUR in the imu spec
#
# spb = sleep seconds per bit
#
# NN = bits to calculate
#  0: min or spb seconds
#  15: max
#
# this function is placed here because it would be
# a syntax failing call ahead if placed after
# the dictionaries that are being initialized
#
# this is a staging function, i.e. do this in order
# to determine what is better, and make the better
# a wrapping function
#
# no blank lines to enable copy-and-paste into
# circuitpy for testing
#
def inactive_spb(odr_hz,NN):  
  # bits are 0..15 but range is 1..16
  if (NN < 0):           # boundary limit low
     NN = 0
  elif (NN > 15):        # boundary limit high
    NN = 15
  ssec =  ((NN+1) * 512) / odr_hz   # per spec
  return ssec
# end inactive_spb
#
# -----------------------------------------------------
#
# ntg_max = not greater than
#
# no blank lines to enable copy-and-paste into
# circuitpy for testing
#
def inact_sec2code(odr_hz,vmin,vmax,ngt_sec = 10):
  smin = inactive_spb(odr_hz,0)
  smax = inactive_spb(odr_hz,16)
  #
  if (smax < ngt_sec):       #  force upper bound
    NN = 15
  elif (ngt_sec <= smin):     # force lower bounds
    NN = 1
  else:
    NN = int( ((odr_hz/512) * ngt_sec))     # reverse of inactive_spb
    s_calc = inactive_spb(odr_hz,NN)        # low ODR tends to high sec jumps per bit
    if ((s_calc > ngt_sec) and (NN > 2)):
      NN = NN -1
  return NN 
# end inact_sec2code
#
# -----------------------------------------------------
#

# -------------------------
#  acc and gyr ODR dicts
# -------------------------

ODR_OFFdct = {"name": "ODR_OFF",
               "cmd":  ODR_OFF,
               "hz":   0,
               "smin": 0,       # sleep per bit (don't care)
               "smax": 0,       # inactivity threshold max (don't care)
               "s2s":  0,       # seconds until inactive
               "fifo_gyr": 0,   # fifo odr control bits
               "fifo_acc": 0}   # fifo odr control bits

ACC_1_6_HZdct = {"name":  "ACC_1_6_HZ_ULP",
                  "cmd":   ACC_1_6_HZ_ULP,
                  "hz":    1.6,
                  "smin": inactive_spb(1.6,0),    # sleep per bit
                  "smax": inactive_spb(1.6,16),   # inactivity threshold max
                  "s2s":  5,                      # seconds until inactive
                  "fifo_gyr": None,               # invalid
                  "fifo_acc": 0xB0}               # fifo odr control bits

GYR_6_5_HZdct = {"name":  "GYR_6_5_HZ_LP",
                  "cmd":   GYR_6_5_HZ_LP,
                  "hz":    1.6,
                  "smin": inactive_spb(6.5,0),    # sleep per bit
                  "smax": inactive_spb(6.5,16),   # inactivity threshold max
                  "s2s":  5,                      # seconds until inactive
                  "fifo_gyr": 0x0B,               # fifo odr control bits
                  "fifo_acc": None}               # invalid

ODR_12_5_HZdct = {"name": "ODR_12_5_HZ_LP",
                  "cmd":   ODR_12_5_HZ_LP,
                  "hz":    12.5,
                  "smin": inactive_spb(12.5,0),    # sleep per bit
                  "smax": inactive_spb(12.5,16),   # inactivity threshold max
                  "s2s":  5,                      # seconds until inactive
                  "fifo_gyr": 0x10,                # fifo odr control bits
                  "fifo_acc": 0x01}                # fifo odr control bits

ODR_26_HZdct = {"name": "ODR_26_HZ_LP",
                 "cmd":  ODR_26_HZ_LP,
                 "hz":    26,
                 "smin": inactive_spb(26,0),    # sleep per bit
                 "smax": inactive_spb(26,16),   # inactivity threshold max
                 "s2s":  5,                    # seconds until inactive
                 "fifo_gyr": 0x20,              # fifo odr control bits
                 "fifo_acc": 0x02}              # fifo odr control bits

ODR_52_HZdct = {"name": "ODR_52_HZ_NP",
                 "cmd":  ODR_52_HZ_NP,
                 "hz":    52,
                 "smin": inactive_spb(52,0),    # sleep per bit
                 "smax": inactive_spb(52,16),   # inactivity threshold max
                 "s2s":  5,                    # seconds until inactive
                 "fifo_gyr": 0x30,              # fifo odr control bits
                 "fifo_acc": 0x03}              # fifo odr control bits                    

ODR_104_HZdct = {"name": "ODR_104_HZ_NP",
                 "cmd":  ODR_104_HZ_NP,
                 "hz":    104,
                 "smin": inactive_spb(104,0),    
                 "smax": inactive_spb(104,16),
                 "s2s":  5,                    # seconds until inactive
                 "fifo_gyr": 0x40,              # fifo odr control bits
                 "fifo_acc": 0x04}              # fifo odr control bits   

ODR_208_HZdct = {"name": "ODR_208_HZ_HP",
                 "cmd":   ODR_208_HZ_HP,
                 "hz":    208,
                 "smin": inactive_spb(208,0),    
                 "smax": inactive_spb(208,16),
                 "s2s":  5,                    # seconds until inactive
                 "fifo_gyr": 0x50,              # fifo odr control bits
                 "fifo_acc": 0x05}              # fifo odr control bits   

ODR_416_HZdct = {"name": "ODR_416_HZ_HP",
                 "cmd":   ODR_416_HZ_HP,
                 "hz":    416,
                 "smin": inactive_spb(416,0),    
                 "smax": inactive_spb(416,16),
                 "s2s":  5,                    # note: max achievable is 19 sec
                 "fifo_gyr": 0x60,              # fifo odr control bits
                 "fifo_acc": 0x06}              # fifo odr control bits                 

ODR_833_HZdct = {"name": "ODR_833_HZ_HP",
                 "cmd":   ODR_833_HZ_HP,
                 "hz":    833,
                 "smin": inactive_spb(833,0),    
                 "smax": inactive_spb(833,16),
                 "s2s":  5,                    # note: max achievable is 9 sec
                 "fifo_gyr": 0x70,              # fifo odr control bits
                 "fifo_acc": 0x07}              # fifo odr control bits

ODR_1660_HZdct = {"name": "ODR_1_66_KHZ_HP",
                  "cmd":  ODR_1_66_KHZ_HP,
                  "hz":    1660,
                  "smin": inactive_spb(1660,0),    
                  "smax": inactive_spb(1660,16),
                  "s2s":  5,                    # note: max achievable is 4 sec
                  "fifo_gyr": 0x80,              # fifo odr control bits
                  "fifo_acc": 0x08}              # fifo odr control bits

ODR_3330_HZdct = {"name": "ODR_3_33_KHZ_HP",
                  "cmd":  ODR_3_33_KHZ_HP,
                  "hz":    3330,
                  "smin": inactive_spb(3330,0),    
                  "smax": inactive_spb(3330,16),
                  "s2s":  5,                    # note: max achievable is 2 sec
                  "fifo_gyr": 0x90,              # fifo odr control bits
                  "fifo_acc": 0x09}              # fifo odr control bits

ODR_6660_HZdct = {"name": "ODR_6_66_KHZ_HP",
                  "cmd":  ODR_6_66_KHZ_HP,
                  "hz":    6660,
                  "smin": inactive_spb(6660,0),    
                  "smax": inactive_spb(6660,16),
                  "s2s":  5,                    # note: max achievable is 1 sec
                  "fifo_gyr": 0xA0,              # fifo odr control bits
                  "fifo_acc": 0x0A}              # fifo odr control bits

odrDictList = [ODR_OFFdct,    ACC_1_6_HZdct, GYR_6_5_HZdct,  ODR_12_5_HZdct, 
               ODR_26_HZdct,  ODR_52_HZdct,  ODR_104_HZdct,  ODR_208_HZdct,  
               ODR_416_HZdct, ODR_833_HZdct, ODR_1660_HZdct, ODR_3330_HZdct, 
               ODR_6660_HZdct]


#
# Envokes telemetry_due every 60s to signal the periodic telemetry dump
#
telemetry_timer = supervisor.ticks_ms()

# ----------------------
# params for init_imu
# ----------------------
#
# note: do not move the first two behind their definitions above
#
IMU_ACC_DEF       = ODR_416_HZdct     # dictionary defaults for init_imu
IMU_GYR_DEF       = ODR_416_HZdct
IMU_ACC_HIPERF_DEF = False             # boolean default
IMU_GYR_LOPWR_DEF  = False
IMU_ACC_LOPWR_DEF  = False

g_imu_acc       = IMU_ACC_DEF   # dictionary defaults for global state
g_imu_gyr       = IMU_GYR_DEF
g_imu_acc_hipf  = IMU_ACC_HIPERF_DEF # boolean
g_imu_acc_LOPWR = IMU_ACC_LOPWR_DEF
g_imu_gyr_lp    = IMU_GYR_LOPWR_DEF

# ---------------------------------------------------------
#  SPI bus objects or chip selects or shared control pins
# ---------------------------------------------------------
cs_mag          = None   # mag spi chip select, GNSS and AFE
cs_imu          = None   # imu spi chip select, GNSS and AFE
g_spi_ctl       = None   # spi device, GNSS and AFE's cs_max_misc
g_mag_f         = None   # same
g_imu_f         = None   # same
cs_max_misc     = None   # AFE only, spi bus ctl is shared w/g_spi_ctl 
cs_max_tx       = None   # AFE only
cs_max_rx       = None   # AFE only
g_spi_max_rxtx     = None   # AFE only, used w/cs_max_tx, cs_max_rx 
lna_en          = None   # GNSS only
g_gps_ant       = None   # GNSS only

init_spi_f      = False # set True spi has been setup

# -----------------------
#  shadow ram for MAX
# -----------------------
#
"""
misc:
 P0 - d. TX Trigger source external (TRIG_TX_SRC_SEL = 1, no actual effect unless enabled)
 P1 - c. RX Trigger source external (TRIG_RX_SRC_SEL = 1, no actual effect unless enabled)
 P2 - b. External TX trigger not enabled (EXT_TX_TRIG_ENABLE = 1, complementary enable)
 P3 - a. External RX trigger not enabled (EXT_RX_TRIG_ENABLE = 1, complementary enable)
 P4 - not used, set 0
 P5 - e. External bias disabled (EXT_BIAS_ENABLE = 0, ensure HDR10 not installed)
 P6 - test LED, not used, set 0
 P7 - f. PPS Source select internal GNSS (PPS_SOURCE_SEL = 1)
 P8 - g. REF source select internal OCXO (REF_SOURCE_SEL = 1)
 P9 - h. GNSS Antenna select internal (GNSS_ANT_SEL = 1 ; need to check that this sees satellites and locks)

misc list = [1,1,1,0,0,0,1,1,1]

tx:
 P0 - 0 not used
 P1 - b. Transmitter not blanked (TX_BLANK_SEL = 1, this is inverted from what I wanted but I think correct)
 P2 - a. RF path through the installed filters (FILTER_BYPASS_SEL = 0)
 P3 .. P8 - not used
 P9 - test LED
 

 TX list = [0,1,0,0,0,0,0,0,0]

rx:
 P0 - d. Channel bias enable (CHAN_BIAS_EN = 0, make sure HDR5 is not installed)
 P1 - e. Internal Trigger not asserted (INT_RF_TRIG_SEL = 0)
 P2 - c. RF path through the installed filters (FILTER_BYPASS_SEL = 1)
 P3 - b. Amplifier enabled (AMP_BYPASS_SEL = 1, amp enabled / double check with network analyzer)
 P4  .. P8 - a. Minimum attenuation (ATTEN_C1 to ATTEN_C16 = 0)
 P9 - test LED

list = [0,0,0,1,0,0,0,0,0,0]

"""
#
# note: the intent of x_DEF is to set default lists but if the _DEF is
#       used in multiple places for different purposes, and the variable 
#       the _DEF sets is changed dynamically, then the set variable
#       feeds back to the _DEF itself and the _DEF will be changed everyplace 
#       the _DEF is used - so, .copy()
#
#
#
# FOR VLA EXPERIMENT 2025 EXT ANTENNA DEFAULT CHANGED
#
#                # 0 1 2 3 4 5 6 7 8 9 
#MAX_MISC_DEF   = [1,1,1,1,0,0,0,1,1,1]
MAX_MISC_DEF    = [1,1,1,1,0,0,0,1,1,0]
g_shdw_max_misc = MAX_MISC_DEF.copy()



#                # 0 1 2 3 4 5 6 7 8 9 
MAX_TX_DEF     = [0,1,1,0,0,0,0,0,0,0]
g_shdw_max_tx1  = MAX_TX_DEF.copy()
g_shdw_max_tx2  = MAX_TX_DEF.copy()

# FOR VLA EXPERIMENT 2025 RF INPUT DEFAULT CHANGED
#
#                # 0 1 2 3 4 5 6 7 8 9 
#MAX_RX_DEF     = [0,0,1,1,0,0,0,0,0,0]  
MAX_RX_DEF      = [0,1,1,1,0,0,0,0,0,0]  
g_shdw_max_rx1  = MAX_RX_DEF.copy()
g_shdw_max_rx2  = MAX_RX_DEF.copy()
g_shdw_max_rx3  = MAX_RX_DEF.copy()
g_shdw_max_rx4  = MAX_RX_DEF.copy()

#
# accessing IMU "reserved" registers is claimed to damage the chip
# there are many reserved registers, so many, it makes sense
# to list the much fewer safe registers
#
g_imu_cmdList = [INT1_CTRL,     CTRL1_XL,        CTRL2_G,       CTRL3_C,
                 CTRL4_C,       CTRL5_C,         CTRL6_C,       CTRL7_G,
                 STATUS_REG,    OUT_TEMP_L,      OUT_TEMP_H,    OUTX_L_G, 
                 OUTX_H_G,      OUTY_L_G,        OUTY_H_G,      OUTZ_L_G,   
                 OUTZ_H_G,      OUTX_L_A,        OUTX_H_A,      OUTY_L_A,   
                 OUTY_H_A,      OUTZ_L_A,        OUTZ_H_A,      FUNC_CFG_ACCESS,
                 EMB_FUNC_EN_A, EMB_FUNC_EN_A,   EMB_FUNC_INT1, PAGE_RW,  
       EMB_FUNC_STATUS_MAINPAGE, EMB_FUNC_INIT_A, WAKE_UP_SRC,  TAP_CFG0,      
                 TAP_CFG2,       WAKE_UP_THS,    WAKE_UP_DUR,
                 FIFO_CTRL1,     FIFO_CTRL2,     FIFO_CTRL3,    FIFO_CTRL4,
                 FIFO_STATUS1,   FIFO_STATUS2,   FIFO_DATA_OUT_TAG, FIFO_DATA_OUT_X_L]

#
# ----------------------------------
#     Functions
# ----------------------------------
#
#
# -----------------------------------------------------
#
def dictInList(dictList,theKey,matchTag):
  retDict = None

  ii = 0
  while(ii < len(dictList)):

    theDict = dictList[ii]

    try:
      theTag = theDict[theKey]
    except Exception as eobj:
       break
    else:
       if (theTag == matchTag):
         retDict = theDict
         break
       # endif
    # end else
     
    ii = ii + 1

  # end while 

  return retDict

# end dictInList
#
# -----------------------------------------------------
#
def myljust(foo,igap):   # left justification for pretty printing
  sgap = str(igap)
  sgap = "-" + sgap + 's'
  toFormat = "%" + sgap
  #print("toFormat=", toFormat)
  return toFormat % foo
# end myljust
#
# -----------------------------------------------------
#
# https://stackoverflow.com/questions/1524126/how-to-print-a-list-more-nicely
#
def list_columns(obj, cols=4, columnwise=True, gap=4):
    """
    Print the given list in evenly-spaced columns.

    Parameters
    ----------
    obj : list
        The list to be printed.
    cols : int
        The number of columns in which the list should be printed.
    columnwise : bool, default=True
        If True, the items in the list will be printed column-wise.
        If False the items in the list will be printed row-wise.
    gap : int
        The number of spaces that should separate the longest column
        item/s from the next column. This is the effective spacing
        between columns based on the maximum len() of the list items.
    """

    sobj = [str(item) for item in obj]
    if cols > len(sobj): cols = len(sobj)
    max_len = max([len(item) for item in sobj])
    if columnwise: cols = int(math.ceil(float(len(sobj)) / float(cols)))
    plist = [sobj[i: i+cols] for i in range(0, len(sobj), cols)]
    if columnwise:
        if not len(plist[-1]) == cols:
            plist[-1].extend(['']*(len(sobj) - len(plist[-1])))
        plist = zip(*plist)
    #printer = '\n'.join([''.join([c.ljust(max_len + gap) for c in p])
    printer = '\n'.join([''.join([myljust(c,max_len + gap)  for c in p])
        for p in plist])
    print(printer)
# end list_columns
#
# -----------------------------------------------------------------
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
# rp2040 ADC
#
def get_voltage(raw):
    return (raw * 3.3) / 65536
# end get_voltage
#
# ----------------------------------
#
# NMEA packet format checking
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
         if (debug_f):   # potential unwanted side effect of read
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
# calculate NMEA checksum, 
# if checksum in input then 
#    compare against checksum 
#  else add checksum to packet
#
def add_cksum(pkt_in,debug_f=False):
  err_f = False
  packet_out = ""
  
  if (debug_f):
    print("pre_checksum=", pkt_in)
  # endif debug

  err_code, dck, cck = eval_packet(pkt_in,gen_cksm_f=True,debug_f=debug_f)
  if (err_code != 0):
    err_f = True        # doesn't matter why unless this happens a lot
    if (debug_f):
      print("add_cksum(): eval_packet (): err_code=", err_code)
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
# -----------------------------------------------------------------
#
# remove ",193002.00" part so timestamp can be replaced
#
# ",193002.00,13,12,2024,00,00,-35.56,11.09,-12.49*69"
# 
# this is from an early version of the code and is now obsolete
# MAG and other status emits were tied to reception of time commands
# and the seconds field was removed and replaced and the data fields
# were replaced with AFE data
#
def rem_time_nmea(in_str):
  idx1 = in_str.index(',')
  idx2 = in_str[idx1+1:].index(',')
  out_str = in_str[idx2+idx1+1:]
  return out_str
# end rem_time_nmea
#
# -----------------------------------------------------------------
#
def gen_mag_pkt(pkt_in,prefix,debug_f=False):

  global g_mag_x
  global g_mag_y
  global g_mag_z
  
  err_f = False
  magdata = ""
  
  synth_f = False      # used for testing emit of preset values

  if (synth_f):
    print("*** synthetic magnetometer values ***")
    mag_x = -37.1867 
    mag_y = -13.4933 
    mag_z = -4.89333
  else:
    mag_x = g_mag_x
    mag_y = g_mag_y
    mag_z = g_mag_z
  # endif

  nmea_str = ""
  if (len(pkt_in) != 0):
    nmeadata, ascii_checksum = pkt_in.split("*",1)    # remove checksum
    nmea_str = rem_time_nmea(nmeadata[6:])           # remove command and timestamp 
  # endif

  pre_ck = prefix + ",%d"%(time.time()) + nmea_str # rebuild 
  #                                                 # add new info  
  pre_ck_mag = pre_ck + ",%3.2f,%3.2f,%3.2f"%(mag_x,mag_y,mag_z) + "*"

  err_f, magdata =  add_cksum(pre_ck_mag,debug_f=False)  # add checksum

  if (debug_f and (not err_f)):
    print("mag=", magdata,supervisor.ticks_ms())
  # endif

  return err_f, magdata

# end gen_mag_pkt
#
# -----------------------------------------------------------------
#
def gen_acc_pkt(pkt_in,prefix,debug_f=False):
  global g_acc_x
  global g_acc_y
  global g_acc_z
  
  acc_x = g_acc_x
  acc_y = g_acc_y
  acc_z = g_acc_z 

  err_f = False
  acc_str = ""
 
  nmea_str = ""
  if (len(pkt_in) != 0):
    nmeadata, ascii_checksum = pkt_in.split("*",1)    # remove checksum
    nmea_str = rem_time_nmea(nmeadata[6:])           # remove command and timestamp 
  # endif
 
  pre_ck = prefix + ",%d"%(time.time()) + nmea_str # rebuild
  #                                                 # add data  
  pre_ck_acc = pre_ck + ",%3.2f,%3.2f,%3.2f"%(acc_x,acc_y,acc_z) + "*"

  err_f, acc_str =  add_cksum(pre_ck_acc,debug_f=False)  # add checksum

  if (debug_f and (not err_f)):
    print("acc=", acc_str,supervisor.ticks_ms())

  return err_f, acc_str

# end gen_acc_pkt
#
# -----------------------------------------------------------------
#
def gen_gyr_pkt(pkt_in,prefix,debug_f=False):
  global g_gyr_x
  global g_gyr_y
  global g_gyr_z
  
  gyr_x = g_gyr_x
  gyr_y = g_gyr_y
  gyr_z = g_gyr_z

  err_f = False
  gyro_str = ""

  nmea_str = ""
  if (len(pkt_in) != 0):
    nmeadata, ascii_checksum = pkt_in.split("*",1)    # remove checksum
    nmea_str = rem_time_nmea(nmeadata[6:])           # remove command and timestamp 
  # endif

  pre_ck = prefix + ",%d"%(time.time()) + nmea_str # rebuild
  #                                                 # add data  
  pre_ck_gyr = pre_ck + ",%3.2f,%3.2f,%3.2f"%(gyr_x,gyr_y,gyr_z) + "*"

  err_f, gyro_str =  add_cksum(pre_ck_gyr,debug_f=False)  # add checksum

  if (debug_f and (not err_f)):
    print("gyr=", gyro_str,supervisor.ticks_ms())
  # endif debug

  return err_f, gyro_str

# end gen_gyr_pkt
#
# -----------------------------------------------------------------
#
# note: Q: why do sprintf this way?
#       A: learning new things
#
# note: Q: why so many globals?
#       A: less code can be better code
#          if the less code is also simpler code
#
#
def gen_hk_pkt(pkt_in,prefix,debug_f=False):

  global g_mag_f       # MAG is ok
  global g_imu_f       # IMU is ok
  global g_spi_ctl     # spi bus is ok
  global g_sw_tp       # switcher temperature
  global g_mag_tp      # magnetometer temperature
  global g_imu_tp      # IMU temperature
  global g_active      # IMU is not sleeping
  global g_tilt        # IMU is tilted
  global g_time_source # coded time source
  global g_time_epoch  # coded time epoch


  err_f = False

  if (debug_f):
    
    pass # print("switch,mag,imu,active,tilt=", g_sw_tp,g_mag_tp,g_imu_tp,
         #                                    g_active, g_tilt)
  # endif

  ocxo_val = 0
  ocxo_f = isCW_OCXO_LOCKED()
  if (ocxo_f):
    ocxo_val = 1

  spi_str = "0"
  if (g_spi_ctl != None):
    spi_str = "1"
  # endif

  mag_str = "0"
  if (g_mag_f):
    mag_str = "1"
  # endif

  imu_str = "0"
  if (g_imu_f):
    imu_str = "1"
  # endif  

  g_mag_tp, g_imu_tp, g_sw_tp = get_all_tps(debug_f)

  io_obj_sw_tp  = io.StringIO()  # io object declaration ('c' sprintf)
  io_obj_mag_tp = io.StringIO()
  io_obj_imu_tp = io.StringIO()

  print("%3.2f"%(g_sw_tp),  file= io_obj_sw_tp, end="")  # io object set
  print("%3.2f"%(g_mag_tp), file= io_obj_mag_tp, end="")
  print("%3.2f"%(g_imu_tp), file= io_obj_imu_tp, end="")
 

  sw_tp_str = io_obj_sw_tp.getvalue()           # io object get
  mag_tp_str = io_obj_mag_tp.getvalue()
  imu_tp_str = io_obj_imu_tp.getvalue()

  nmea_str = ""
  if (len(pkt_in) != 0):
    nmeadata, ascii_checksum = pkt_in.split("*",1)    # remove checksum
    nmea_str = rem_time_nmea(nmeadata[6:])           # remove command and timestamp 
  # endif
  #  
  #  switch temperature, magnetometer temperature, is_active, is tilt
  #
  pre_ck = prefix + ",%d"%(time.time()) + nmea_str # rebuild
                                       
  pre_ck_hk = pre_ck + ",%d,%s,%s,%s,%s,%s,%s,%d,%d,%d,%d"%(
                                          ocxo_val,
                                          spi_str,
                                          mag_str,
                                          imu_str,
                                          sw_tp_str,
                                          mag_tp_str,
                                          imu_tp_str,
                                          g_active,g_tilt,
                                          g_time_source,g_time_epoch) + "*"
  err_f, hkdata =  add_cksum(pre_ck_hk,debug_f=False)  # add checksum

  if (debug_f and (not err_f)):
    print("hk=", hkdata,supervisor.ticks_ms())
  # endif

  return err_f, hkdata

# end gen_hk_pkt
#
# ----------------------------------
#
def get_mag_tp(init_f=False, print_f=False):

  global mag_temp_pin  # ADC
  global g_mag_tp      # for sharing w/NMEA

  if (init_f):
    #
    # note: not yet fully reentrant, soft reboot
    #
    if (mag_temp_pin == None):
      mag_temp_pin = analogio.AnalogIn(board.A1)
    # endif 
  # endif

  degC= (get_voltage(mag_temp_pin.value) - .509)/.00645

  g_mag_tp = degC  # write to global

  if (print_f):
    print("MAG degC=",degC)
  # endif print

  return degC

# end get_mag_tp
#
# ----------------------------------
#
def get_sw_tp(init_f=False, print_f=False):

  global switch_temp_pin # ADC
  global g_sw_tp         # for sharing w/NMEA

  if (init_f):
    #
    # note: not yet fully reentrant, soft reboot
    #
    if (switch_temp_pin == None):
      switch_temp_pin = analogio.AnalogIn(board.A0)
  # endif

  raw = switch_temp_pin.value

  Vm = get_voltage(raw)

  degC = 25 + (Vm - 0.250)/0.0095

  g_sw_tp = degC   # write to global

  if (print_f):
    print("Switcher degC=",degC)
  # endif print

  return degC

# end get_sw_tp
#
# ----------------------------------
#
# note: only call after the fifo has been initialized
#
def get_imu_tp(print_f=False):
  global g_imu_tp
  global USE_IMU

  if (USE_IMU==False):
    g_imu_tp = 0.0
    return 0.0
  # endif 
  
  accList, gyrList, timList = process_fifo(count=1,
                                           acc_rq_f=False,
                                           gyr_rq_f=False,
                                           tim_rq_f=True)
  degC = timList[0]
  g_imu_tp = degC

  if (print_f):
     print("IMU degC=",degC)
  # endif print

  return degC

# end get_imu_tp
#
# ----------------------------------
#
def get_all_tps(debug_f):

  #
  # note: functions write to globals
  #
  mag_temp = get_mag_tp(print_f=debug_f)   # print magnetometr temperature

  sw_temp = get_sw_tp(print_f=debug_f)    # print switcher temperature

  imu_temp = get_imu_tp(print_f=debug_f)   # print imu temperature

  return mag_temp, imu_temp, sw_temp
# end get_all_tps
#
#
# ----------------------------------
#
def isCW_OCXO_LOCKED():
  global g_ocxo_lock

  locked_f = False

  if (g_ocxo_lock == None):             # if need to init
    g_ocxo_lock = DigitalInOut(mc.pin.GPIO21)
    g_ocxo_lock.switch_to_input()
  # endif need to init
  #
  # there's a code path to de-init and reinit
  # but there's no need for that yet
  #
  if (g_ocxo_lock.value == 1):
     locked_f = True
  # endif

  return locked_f

# end isCW_OCXO_LOCKED
#
# ----------------------------------
#
def init_uart(baud0,baud1):
  global uart0  # write outside, read MINICOM for commands
  global uart1  # read ublox

  if (uart0 == None):
    uart0 = busio.UART(mc.pin.GPIO0,       
                       mc.pin.GPIO1, 
                       baudrate=baud0,
                       timeout=0.1)        # in/out at edge connector
  # endif first time

  uart0.reset_input_buffer()  # noise remediation

  if (uart1 == None):
    uart1 = busio.UART(mc.pin.GPIO4, 
                       mc.pin.GPIO5, 
                       baudrate=baud1,
                       timeout=0.1)         # in from ublox
  # endif first time
 
  uart1.reset_input_buffer()  # noise remediation
 
# end init_uart
#
# ----------------------------------
#
def set_NMEA_acq(en_f):
  global g_nmea_time_f

  g_nmea_time_f = en_f

# end set_NMEA_acq
#
# ----------------------------------
#
def get_NMEA_acq():
  global g_nmea_time_f

  return g_nmea_time_f

# end get_NMEA_acq
#
# ----------------------------------
#
def setTimeFromTs(ts):
  global g_rtcObj
  global g_rtc_init

  err_f = False

  try:
    dtobj = dt.datetime.fromtimestamp(ts)
  except Exception as obj:
    err_f = True
    print("Exception: setTimeFromTs()",eobj)

  else:
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
    g_rtc_init = True       # rtc has been re-initialized
    #
  # end else ok

  return err_f

# end setTimeFromTs
#
# ----------------------------------
#
"""
         |-> seconds                                   |-> day,month,year
         v                                             v
$GNRMC,193409.00,A,4237.38614,N,07129.34227,W,0.001,,270325,,,A,V*07

"""
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

# end NMEA_to_RTC

#
# ----------------------------------
#
# note: rate (not so) constant globals are set through
#
def send_telem(debug_f=False):
  global uart0                  # where telemetry will be seen
  global USE_MINICOM            # assists slow minicom
  global USE_IMU                # enable (early versions of hardware)
  global USE_MAG                # enable (early versions of hardware)
  global g_telem_updr           # HK,  0 .. 60, 0 is off
  global g_mag_tupdr            # MAG, 0 .. 60, 0 is off
  global g_imu_updr             # IMU, 0 .. 60, 0 is off
  global g_telem_time           # base time to next telem tx
  global g_mag_time             # base time to next mag tx
  global g_imu_time             # base time to next imu tx
  global g_uart_delay

  err_f = False                # keep going on error flag
  any_err_f = False            # report any error flag

  pkt_in = ""              # output is not based on any received input
  nowTime = time.time()    # now time for periodic telemetry

  if (USE_MAG and (g_mag_tupdr > 0)):
    if ((g_mag_time == None) or ((nowTime - g_mag_time) >= g_mag_tupdr)):
      g_mag_time = nowTime  # reset base time
      prefix = "$PMITMAG"              
      err_f, mag_nmea_str = gen_mag_pkt(pkt_in,prefix,debug_f) # magnetometer   
      if (not err_f):
        uart0.write(mag_nmea_str + "\r\n")
        if (USE_MINICOM):  
          time.sleep(g_uart_delay)  # then delay 
        # endif minicom 
      else:
        any_err_f = True
      # end else not error
    # endif time to send
  # endif USE_MAG

  if (g_telem_updr > 0):   
    if ((g_telem_time == None) or ((nowTime - g_telem_time) >= g_telem_updr)): 
      g_telem_time = nowTime  # reset base time   
      prefix = "$PMITHK"
      err_f, hk_nmea_str = gen_hk_pkt(pkt_in,prefix,debug_f) # housekeeping 
      if (not err_f):
        uart0.write(hk_nmea_str + "\r\n")
        if (USE_MINICOM):  
          time.sleep(g_uart_delay)  # then delay 
        # endif minicom 
      else:
        any_err_f = True
      # end else not error
    # endif time to send
  # endif enabled
      
  if (USE_IMU and (g_imu_updr > 0)): 
    if ((g_imu_time == None) or ((nowTime - g_imu_time) >= g_imu_updr)):
      g_imu_time = nowTime
      prefix = "$PMITACC"      
      err_f, acc_nmea_str = gen_acc_pkt(pkt_in,prefix,debug_f) # accelerometer
      if (not err_f):      
        uart0.write(acc_nmea_str + "\r\n")
        if (USE_MINICOM):  
          time.sleep(g_uart_delay)  # then delay 
        # endif minicom 
      # endif not error 
      #       
      prefix = "$PMITGYR"  
      err_f, gyr_nmea_str = gen_gyr_pkt(pkt_in,prefix,debug_f) # gyro
      if (not err_f):                                       
        uart0.write(gyr_nmea_str + "\r\n")
        if (USE_MINICOM):  
          time.sleep(g_uart_delay)  # then delay 
        # endif minicom
      else:
        any_err_f = True
      # end else not error   
    # endif time to send
  # endif USE_IMU

  #prefix = "$TEMP"
  #uart0.write(prefix + str(get_sw_tp()) + "\r\n")

  return any_err_f

# end send_telem
#
# -------------------------
# uart crib
# -------------------------
# >> import busio
# >> import microcontroller as mc
# >> uart1 = busio.UART(mc.pin.GPIO4, mc.pin.GPIO5, baudrate=9600)
# >> while(True):
# >>   data = uart1.read(64)
# >>   if data is not None:
# >>      data_string = ''.join([chr(b) for b in data]) # convert bytearray to string
# >>      print(data_string, end="")
#
# Q: why not use uart.readline()?
# A: sometimes ublox binary sneaks in
#

def forward_nmea(wait_f       = False, # if True then wait for first char
                 tmoSec       = 8,     # (was 5) tmo can be from many causes
                 maxMsgCnt    = 20,     # 20 reliably returns $GNZDA if present at all
                 debug_f=False):
  global uart0         # write outside, accept commands from MINICOM
  global uart1         # read ublox
  global g_uart_delay  # for minicom to not drop chars 
  global USE_MINICOM   # signals MINICOM in use
  global g_rtc_save   # for periodic update of RTC
  delay_val = g_uart_delay
  err_f = False
  baseSec = time.time()           # anti-lockup seconds timer
  baseMs = supervisor.ticks_ms()  # millisecond periodicity timer
  msg_cnt = 0                   # increments to maxMsgCnt
  err_cnt = 0                   # increments on badly formed packet
  
  bin_cnt = 0                   # increments on binary found
  ck_cnt  = 0                   # increments on bad checksum
  tmo_f = False                 # timeout flag
  nmea_en_f = False             # nmea acq enabled flag 
      
  while (True and (not tmo_f)):   # outer loop for count or tmo, NOT exit on error  
    err_f = False                 # reset errors     
    ii = 0                        # state phrase char counter
    start_f = False               # begin sentence acquisition
    cksum_f = False               # begin checksum state
    end_f = False                 # end sentence state
    nmea_string = ""              # grammar sentence

    while (True and (not tmo_f)): # inner loop for nmea grammar / state machine    
      err_f = False               # reset errors  
      if (uart1.in_waiting == 0):
        return False,0,0
      else:
         raw_data = uart1.read(1)  
      # end else read data
      if (debug_f):
        pass # print("type=",type(raw_data),"len=",len(raw_data),"raw_data=",raw_data)
      # endif debug
      
      elemStr = raw_data.decode()  # noise in stream check
      theChar = elemStr[0]
      #
      # serious protocol diagnostic
      #
      #print(theChar)                   # prints every ascii character
      #
      if (start_f == False):            # if not started
        if (theChar == "$"):            #   then if start character
          nmea_string = theChar         #     then save character as first char
          start_f = True                #          signal starat
          baseSec = time.time()         #          restart tmo timer 
        else:                           #    else not started
          continue                      #      back to top
        # end else
      else:                             # else is started
        #
        if (theChar == "$"):            #   if '$' found, then false start 
          nmea_string = theChar         #      save char as first char
          baseSec = time.time()         #      re-restart tmo timer
          continue                      #      back to top
        else:
          #   
          nmea_string = nmea_string + theChar       
          if (not cksum_f):             # test for checksum phrase
            if (theChar == '*'):        # checksum phrase start
              cksum_f = True            # prevent reentry here
              ii = 0                    # start counting
            # endif not checksum next
          else:                         # expect 2 bytes checksum
            ii = ii + 1                 # followed by crlf
            if (ii >= 4):               # all chars received
               ii = 0
               end_f = True             # done 
               baseSec = time.time()    # reset timeout timer  
               break                    # done with inner loop
             # endif done with sentence
          # end else inside checksum phrase
        # end else keep going
      # end else started

      nowSec = time.time()             # check for timout
      if (nowSec > baseSec + tmoSec):  # timeout
        if (start_f and not end_f):    # guard against accidentally tossing complete packet
           tmo_f = True
        # detect tmo in middle of message
      
        break  # 

      # endif anti-lockup test

    # end while inner loop
    if (start_f and end_f):   # process a sentence
      start_f = False
      end_f   = False
      cksum_f = False
      
      err_code, dck, cck = eval_packet(nmea_string)     
      if (err_code != 0):
        err_f = True
        if (err_code == -3):     # binary seen at checksum
          bin_cnt = bin_cnt + 1
        elif (err_code == -4):
          ck_cnt = ck_cnt +1     # bad checksum (dropped data)
                  
      # endif
      #
      # ublox generates some bad checksums
      #
      if (err_f):
        err_cnt = err_cnt + 1     # count faults       
        err_f = True              # tpss the packet
      # endif error

      if (not err_f):        
        #                         # ---------------
        uart0.write(nmea_string)  # forward NMEA --
        #                         # ---------------
        #
        msg_cnt = msg_cnt + 1     # count messages in
        
        if (USE_MINICOM):
          time.sleep(delay_val)      # add delay for minicom 
        # endif USE_MINICOM
        #                       #012345 
        if (nmea_string[0:6] == "$GNRMC"):        # time message
          nmea_en_f = get_NMEA_acq()              # get time acq enable
          nowTime = time.time()        
          #
          # if ocxo is locked and nmea acquisition enabled and
          #   (first setting or 60 seconds since last setting)
          #  
          if (nmea_en_f and ((g_rtc_save==0) or ((nowTime - g_rtc_save) >= 60)) ):   
            err_f, ts = NMEA_to_RTC(nmea_string) #   set time
            # endif error
          # endif enabled and locked
        # endif gnzda       
      # endif full sentence
    # endif not error

    nowSec = time.time()            # check for timeout
    if (nowSec > baseSec + tmoSec):
      if (msg_cnt == 0):
         if (not tmo_f):
           tmo_f = True
         # fine tuning tmo reporting
      # endif timeout

      break  # timeout

    # endif anti-lockup test
    if (msg_cnt == maxMsgCnt):     # check for max messages
       break
    # endif

  # end while outer loop

  if (USE_MINICOM):
    uart0.write("\r\n")            # user feedback
    time.sleep(delay_val)          # slow down for the far end
  # endif minicom

  if (debug_f):
    if (msg_cnt > 0) or (err_cnt > 0) or (bin_cnt > 0):               # if data then
      print("mc=", msg_cnt,"ec=",err_cnt,"bc=",bin_cnt,"ck=",ck_cnt)  # statistics
    else:    #                                                        # else no data
      print("forward_nmea(): no data, tmo_f=%d, delay_val=%2.2f"%(tmo_f,delay_val))
    # end else no data
    time.sleep(delay_val) 
  # endif debug   
                                # -----------------------------
  return err_f,msg_cnt,err_cnt  # note: multiple return paths
  #                             # -----------------------------
# end forward_nmea
#
# ----------------------------------
#
def cleanup_spi():
  global g_spi_ctl

  if g_spi_ctl != None:  # if bus is in use then
    g_spi_ctl.deinit()   #   allow retry 
    g_spi_ctl = None     #   flag bus no longer in use
  # endif

# endif cleanup_spi
#
# ----------------------------------
#
# gnss versus afe
# note 1: spi bus has common hardware w/IMU and Mag but different pins
# note 2: different cs, different sck, mosi, miso
# note 3: gnss has sd afe does not
# note 4: afe has 7 MAX bus expanders, 1 is direct, 1 is 2 daisy chained,
#         and 1 is 4 daisy chained
# note 5: the direct MAX chip select can be faked by enabling LNA_BYPASS
#
def init_spi():
  global cs_mag
  global cs_imu
  global cs_max_misc
  global g_spi_ctl
  global init_spi_f
 
  debug_f = False

  if (debug_f):
    print("init_spi: init_spi_f=",init_spi_f)
  # endif 

 
  if (cs_mag == None):                    # for handling soft reset
    cs_mag = DigitalInOut(mc.pin.GPIO14)  # mag (both AFE and GNSS)
  # endif first time 

  if (cs_imu == None): 
    cs_imu = DigitalInOut(mc.pin.GPIO13)  # imu (both AFE and GNSS)
  # endif first time 
         
  cs_mag.switch_to_output(value=True)
  cs_imu.switch_to_output(value=True)

  cs_mag.value = 1
  cs_imu.value = 1

  if (g_spi_ctl == None):
    g_spi_ctl = busio.SPI(mc.pin.GPIO10, # clock
                          mc.pin.GPIO11, # mosi
                          mc.pin.GPIO12) # miso
  # endif first time 

  init_spi_f = True

  if (debug_f):
    print("init_spi: init_spi_f=",init_spi_f)
  # endif

# end init_spi
#
# ----------------------------------
#
def imu_cmd_check(datum):

  global g_imu_cmdList

  err_f = True

  if (datum in g_imu_cmdList):
     err_f = False
  # endif

  return err_f

# end imu_cmd_check
#
# ----------------------------------
# 
# note 1: same bus, different chip select
#
# note 2: chip select must be de-asserted at 
#         completion of handshake even if
#         next command is to the same device
#
# note 3: not checking lock results because the
#         featherwing is not multitasking
#
def write_spi(which,wr_ctl,wr_in):
  global cs_mag
  global cs_imu
  global g_spi_ctl
  global init_spi_f

  err_f = False    # return tuple 1
  rd_bytes = []    # return tuple 2

  debug_f = False

  if ((wr_ctl == "w") or (wr_ctl == "r")):
    pass
  else:
    err_f = True
    return err_f,rd_bytes  # return tuple 1,2

  if (not init_spi_f):  # belt and suspenders
    init_spi()  
  # endif

  cs_mag.value = 1             # init for access
  cs_imu.value = 1

  w_mask = 0x7f
  r_mask = 0x80
  if (wr_ctl == "w"):
    wr_in[0] = wr_in[0] & w_mask
    if (which == "IMU"):                   # if possibility of bad command
      err_f = imu_cmd_check(wr_in[0])   # then do bad command 
      if (err_f):
        print("Error: reserved IMU command:",hex(wr_in[0]))
        return err_f,rd_bytes  # return tuple 1,2
      # endif error 
    # endif bad command check needed 

  elif (wr_ctl == "r"):
    wr_in[0] = wr_in[0] | r_mask
  # end elif
  wr_bytes = bytes(wr_in)      # int to bytes

  if (which == "MAG"):
     cs_mag.value = 0          # cs asserted low
  elif (which == "IMU"):
     cs_imu.value = 0          # cs asserted low
  else:
    err_f = True
    print("unknown device selection:", which)
  # end else

  ok_f = g_spi_ctl.try_lock()           # lock
  if (not ok_f):
    err_f = True
    print("error: try_lock()")  # programming error
  # endif
  
  if (wr_ctl == "w"): 
   if (debug_f):
      print("which=",which)
      print("wr_bytes=",[hex(x) for x in wr_bytes])
   g_spi_ctl.write(wr_bytes)
  elif (wr_ctl == "r"): 
    if (debug_f):
      print("which=",which)
      print("wr_in=",[hex(x) for x in wr_in])
    # endif debug
     
    rd_bytes = bytearray(len(wr_in))
    g_spi_ctl.write_readinto(wr_bytes,rd_bytes)
  else:
    err_f = True
    print("unknown wr_ctl:", wr_ctl)

  g_spi_ctl.unlock()           # unlock
  
  cs_mag.value = 1           # reinit for next access
  cs_imu.value = 1

  rd_data = [x for x in rd_bytes]  # if something, then convert to int

  if (len(rd_data) > 0):           # readback, toss first element
    rd_bytes = rd_data[1:]
    if (debug_f):
      hexList = [hex(x) for x in rd_bytes]
      print("rd=", hexList)
    # endif debug
  # endif readback

  return err_f, rd_bytes           # empty list or bytes as ints

# end write_spi
#
# ------------------------------------------
#
"""
 Per the MAX spec, the writes are repeated so they can be clocked into 
 each daisy chained chip. The last chip gets the first bits and
 the first chip gets the last bits.

 The address 0x20 or 32 decimal is the don't care address

""" 
#
def maxStr2SpiList(str_in,         # comma separated integers
                 addr_ofs=0,       # address offset for shorter strings
                 daisy=1,          # 1, 2, or 4
                 dai_sel=1,        # selects chip in the chain
                 copy_f = False,   # same data for all chips, for init
                 debug_f=False):   # enables instrumentation
  err_f = False
  iList = []
  byteList = []
  theDummy = [32,0]  # NOP address, note: printing to console 0x20 is a ' ' space

  scanList = str_in.split(",")   # split string into list by commas
  max_cnt = len(scanList)        # count the commas
  if ((max_cnt + addr_ofs) > 10):
    print("maxStr2SpiList(): max_cnt (%d) + addr_ofs (%d) > 10"%(max_cnt,addr_ofs))
    err_f = True
  if (not err_f):
    if ((daisy == 1) or (daisy == 2) or (daisy == 4)):
      pass   # this is the good case
    else:
      print("maxStr2SpiList(): daisy must be 0 or 2 or 4:",daisy)
      err_f = True
    # end else
  # endif not error
  if (not err_f):
    #
    # verify dependencies of input parameters
    #
    if ( ((daisy == 1) and (dai_sel == 1)) or ((daisy == 2) and ((dai_sel >= 1) and (dai_sel <= 2))) or ((daisy == 4) and ((dai_sel >= 1) and (dai_sel <= 4))) ):
      if (debug_f):
        print("daisy=%d, dai_sel=%d, addr_ofs=%d, max_cnt=%d"%(daisy,
                                                               dai_sel,
                                                               addr_ofs,
                                                               max_cnt))
      # endif debug
    else:
      print("maxStr2SpiList(): dai_sel must be between 1 and %d"%(daisy))
      err_f = True
    # end else
  # endif not error
  # ---------------
  # still here?
  # ---------------
  if (not err_f):                    # write to each bit in one MAX, not daisy chained
      ii = 0
      while (ii < max_cnt):  # for each bit in the list
        #
        if(scanList[ii] == '0'):
           addr = ii + addr_ofs
           ival = 0
        elif (scanList[ii] == '1'):
           addr = ii + addr_ofs
           ival = 1
        else:          # assume 'x' for don't care
          ii = ii + 1  # next address
          continue     # continue
        # end else
        #
        thePair = [addr,ival]
        if (daisy == 1):             # not daisy chained
          if (debug_f and (ii==0)):
            print("maxStr2SpiList() path daisy=1")
          # endif 
          wr_bytes = bytes(thePair)
          iList = iList +  [thePair]
          byteList = byteList + [ wr_bytes ]
        else:
          if (daisy == 2):                    # Tx is 2
            if (debug_f and (ii==0)):
              print("maxStr2SpiList() path daisy=2")
            # endif 
            if (copy_f):                      # if copied to all in chain (i.e. init)
               theQuad = thePair + thePair
            else:                             # else individualized
               if (dai_sel == 1):
                 theQuad = theDummy + thePair
               else:
                 theQuad = thePair + theDummy
               # end else
            # end else daisy individualized
            wr_bytes = bytes(theQuad)
            iList = iList +  [theQuad]
            byteList = byteList + [ wr_bytes ]
          else:                                # Rx is 4
            if (debug_f and (ii==0)):
             print("maxStr2SpiList() path daisy=4")
            # endif 
            if (copy_f):                       # if copied to all in chain (i.e. init)
                theOctet = thePair + thePair + thePair + thePair
            else:                              # else individualized
               if (dai_sel == 1):
                 theOctet = theDummy + theDummy + theDummy + thePair
               elif (dai_sel == 2):
                 theOctet = theDummy + theDummy + thePair + theDummy 
               elif (dai_sel == 3):
                  theOctet = theDummy + thePair + theDummy + theDummy
               else:
                  theOctet = thePair + theDummy + theDummy + theDummy
               # end else dai_sel == 4
            # end else daisy individualized 
            wr_bytes = bytes(theOctet)
            iList = iList +  [theOctet]
            byteList = byteList + [ wr_bytes ]
          # end else daisy == 4           
        #
        ii = ii + 1
        #
      # end while
    # endif increment address
  
  # end else not error
  #
  if (debug_f):
    print("iList=",iList,"len=",len(iList))
    print("byteList=",byteList,"len=",len(byteList))
  # endif debug
  #
  return err_f,byteList
  #
# end maxStr2SpiList
#
# ------------------------------------------
#
def writeMax(theCtl,theCS,theByteList):
  debug_f = True

  if (debug_f):
    pass

  ii = 0
  while (ii < len(theByteList)):
    theCS.value = 1
    theCtl.try_lock()
    theCS.value = 0
    theCtl.write(theByteList[ii])
    theCS.value = 1
    theCtl.unlock()
    ii = ii + 1
    #
  # end while
  #
# end writeMax
#
# ------------------------------------------
#
def  maxStr2SpiOut(str_in,           # comma separated integers
                   addr_ofs=0,       # address offset for shorter strings
                   daisy=1,          # 1, 2, or 4
                   dai_sel=1,        # selects chip in the chain
                   copy_f = False,   # same data for all chips, for init
                   debug_f=False):

  global g_spi_ctl        # singular MAX, shared w/IMU, MAG
  global cs_max_misc      # chip select for same
  #
  global g_spi_max_rxtx   # daisy chained MAX, shared w/MAX rx tx
  global cs_max_tx        # chip select for above
  global cs_max_rx        # chip select for above
  
  theSPI = None
  theCS = None
  err_f = False
  err_f, theByteList = maxStr2SpiList(str_in,        
                                   addr_ofs= addr_ofs,       
                                   daisy   = daisy,          
                                   dai_sel = dai_sel,        
                                   copy_f  = copy_f,   
                                   debug_f=debug_f)
  if (not err_f):
    if (daisy==1):
      theSPI = g_spi_ctl
      theCS = cs_max_misc
    elif (daisy==2):
       theSPI = g_spi_max_rxtx
       theCS = cs_max_tx
    elif (daisy==4):
       theSPI = g_spi_max_rxtx
       theCS = cs_max_rx
    # end daisy check
    try:

      writeMax(theSPI,theCS,theByteList)

    except Exception as eobj:        # most likely cause is bad daisy selection
      err_f = True
      print("maxStr2SpiOut: Exception:",eobj)
    # end except
  # endif not error

  return err_f

# end maxStr2SpiOut
#
# ------------------------------------------
#
def maxList2SpiOut(list_in,          # comma separated integers
               addr_ofs=0,       # address offset for shorter strings
               daisy=1,          # 1, 2, or 4
               dai_sel=1,        # selects chip in the chain
               copy_f = False,   # same data for all chips, for init
               debug_f=False):

  global g_spi_ctl        # singular MAX, shared w/IMU, MAG
  global cs_max_misc      # chip select for same
  #
  global g_spi_max_rxtx   # daisy chained MAX, shared w/MAX rx tx
  global cs_max_tx        # chip select for above
  global cs_max_rx        # chip select for above

  theSPI = None
  theCS = None
  err_f = False

  err_f, theStr = maxList2str(list_in)
  if (not err_f):
    err_f = maxStr2SpiOut(theStr,        
                          addr_ofs= addr_ofs,       
                          daisy  = daisy,          
                          dai_sel= dai_sel,        
                          copy_f = copy_f,   
                          debug_f=debug_f)
  # endif not error

  return err_f

# end maxList2SpiOut
#
# ------------------------------------------
#
# note 1. spi read is signaled by setting the 
#         ms bit of the register high
# note 2. there is no board select, first byte
#         is the register select
#
# note 3. the readback readback echoes the size of the write
#         past the register write
#         to read id, write the id register plus a pad byte
#         the readback will put the id in the pad byte space
#
def pr_mag_id():

  err_f = False

  err_f,rd_data = write_spi("MAG","r",[0xB6, 0x00])

  if (not err_f):
    expected = hex(0x22)  # from the spec
    detected = hex(rd_data[0])
    print("pr_mag_id:",detected,"expected:",expected)
    if (detected != expected):
      err_f = True
      print("*** pr_mag_id(): detected != expected ***")
    # endif error check
  else:
    print("error: pr_mag_id() -> write_spi()")
  # end else

  return err_f

# end pr_mag_id
#
# ----------------------------------
#
def pr_imu_id():

  err_f = False

  err_f,rd_data = write_spi("IMU","r",[0x8F, 0x00])
  if (not err_f):
    expected = hex(0x6c)  # from the spec
    detected = hex(rd_data[0])
    print("pr_imu_id:",detected,"expected:",expected)
    if (detected != expected):
      err_f = True
      print("*** pr_imu_id(): detected != expected ***")
    # endif error check
  else:
    print("error: pr_imu_id() -> write_spi()")
  # end else

  return err_f

# end pr_imu_id
#
# ----------------------------------
#
def set_mag_params(new_ccr, new_updr):
  global g_mag_ccr
  global g_mag_iupdr   # i for internal, similar name different use

  g_mag_ccr  = new_ccr
  g_mag_iupdr = new_updr

# end set_mag_params
#
# ----------------------------------
#
def get_mag_params():
  global g_mag_ccr
  global g_mag_iupdr

  staging_ccr  = g_mag_ccr  # staging simplifies debugging
  staging_updr = g_mag_iupdr

  return staging_ccr, staging_updr

# end get_mag_params
#
# ----------------------------------
#
def set_imu_params(accDict, gyrDict, acc_hiper_f, acc_ulp_f, gyr_lp_f):
  global g_imu_acc       
  global g_imu_gyr       
  global g_imu_acc_hipf 
  global g_imu_acc_ulp
  global g_imu_gyr_lp

  g_imu_acc       = accDict
  g_imu_gyr       = gyrDict
  g_imu_acc_hipf = acc_hiper_f
  g_imu_acc_ulp    = acc_ulp_f
  g_imu_gyr_lp     = gyr_lp_f

# end set_imu_params
#
# ----------------------------------
#
def get_imu_params():
  global g_imu_acc       
  global g_imu_gyr       
  global g_imu_acc_hipf 
  global g_imu_acc_ulp
  global g_imu_gyr_lp
  
  debug_f = False 

  x1 = g_imu_acc
  x2 = g_imu_gyr
  x3 = g_imu_acc_hipf
  x4 = g_imu_acc_ulp
  x5 = g_imu_gyr_lp

  if (debug_f):
    print("get_imu_params()=",x1,x2,x3,x4,x5)

  return x1,x2,x3,x4,x5

# end get_imu_params

#
# ----------------------------------
#
def mag_eval(valList): #returns measurement values in microTesla (uT)

        #inputs to transfer
        
        x2 = valList[0]
        x1 = valList[1] 
        x0 = valList[2]
        y2 = valList[3]
        y1 = valList[4]
        y0 = valList[5]
        z2 = valList[6]
        z1 = valList[7]
        z0 = valList[8]

        x = (256**2)*x2 + (256)*x1 + x0
        y = (256**2)*y2 + (256)*y1 + y0
        z = (256**2)*z2 + (256)*z1 + z0

        #handle 2's complement/signed encoding
        x = x if x<2**23 else x-2**24
        y = y if y<2**23 else y-2**24
        z = z if z<2**23 else z-2**24

        return (y/75, x/75, -z/75)

# end mag_eval
#
# ----------------------------------
#
def read_mag(print_f=False):
  global g_mag_x
  global g_mag_y
  global g_mag_z
  global USE_MAG

  err_f = False

  values = [0,0,0] # 3 mag values after processing

  if (not USE_MAG):   # early exit
    return err_f, values
  # endif 

  #
  # structure is artifact of spi processing
  # 9 values back overwrite the 0's
  #
  err_f,rdbk = write_spi("MAG","r",
                   [0xA4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
  if (err_f):
    print("read_mag: can't read data") 
  else:
    values = mag_eval(rdbk) # 9 in, 3 ouy
    #time.sleep(0.1)        # delay was in code early on but removed w/o problems
  # end else

  g_mag_x = values[0]
  g_mag_y = values[1]
  g_mag_z = values[2]

  if (print_f):
    print("MAG values=", g_mag_x, g_mag_y, g_mag_z)
  # endif printing

  return err_f, values

# end read_mag
#
# ----------------------------------
#
def init_mag(ccr=MAG_CCR_DEF,updr=MAG_UPDR_DEF):

  err_f = False
  debug_f = True

  values = [] # 9 mag values expected

  #
  # x, y, z
  # cycle count reg
  #
  # per spec, tradeoff is between acquisition time
  # and resolution, typical steps from min to max
  # are 50, 100, 200, 400
  #
  # spec default/example shows 200
  #
  lb = ccr & 0xff
  hb = ccr & 0xff00
  hb = hb >> 8                    # correctness of combined logics in single statement?
  write_spi("MAG","w",[ 0x4,hb,lb,  
                            hb,lb, 
                            hb,lb ]) 
 
  write_spi("MAG","w",[0x0B,updr])   # 0x96 = 150d update rate
  write_spi("MAG","w",[0x01,0x71])    # continuous mode

  time.sleep(0.1)                     # wait for settling

  err_f, valList = write_spi("MAG","r",[0xB4,0x00])    # status
  if (err_f):
    print("init_mag: can't read status")
  else:
    if ((valList[0] & 0x80) == 0):
      err_f = True
      print("init_mag: data not ready") 
    else:
      err_f,values = read_mag(True)
 
  return err_f, values

# end init_mag
#
# ----------------------------------
#
def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is
# end twos_comp
#
# ----------------------------------
#
def is_imu_rdy(delay=0.1):
  err_f = False
  debug_f = True

  xlda_f = False
  gda_f = False
  tda_f = False

  time.sleep(delay)
  err_f,imu_status=write_spi("IMU","r",[STATUS_REG,0x00]) 
  if (err_f):
    err_f = True
    print("is_imu_rdy(): can't read status")
  else:
    if ((imu_status[0] & 0x1) > 0):
      xlda_f = True
    # endif
    if ((imu_status[0] & 0x2) > 0):
      gda_f = True
    # endif
    if ((imu_status[0] & 0x4) > 0):
      tda_f = True
    # endif
  # end else
  if (not err_f):
    if (not xlda_f):
      err_f = True    
      print("is_imu_rdy(): XLDA not set") 
    # endif 
    if (not gda_f):
      err_f = True
      print("is_imu_rdy(): GDA not set") 
    # endif
    if (not tda_f):
      err_f = True
      print("is_imu_rdy(): TDA not set") 
    # end elif
  # end if not error

  return err_f, xlda_f, gda_f, tda_f

# end is_imu_rdy
#
# -----------------------------------------------
#
def imu_temp_conv(tempHL):
  #
  # value is a +/- offset from 25 deg C
  #
  temp_value = tempHL[0] + (tempHL[1] * 256)
  temp_2s_comp = twos_comp(temp_value,16)
  temp_scaled = (temp_2s_comp/256) + 25
 
  return temp_scaled

# end imu_temp_conv
#
# -----------------------------------------------
#
# averaging tare entries may be applied 
#
#
def read_imu(xlda_f=None,     # read acc (xl)
             gda_f=None,      # ready gyro
             tda_f=None,      # read temperature
             cal_f=False,     # apply calibration
             dataList=[],
             debug_f=False):

  global g_accOfs
  global g_cal_acc_f
  global g_gyrOfs
  global g_cal_gyr_f
  global ACC_MAX
  global GYR_MAX

  err_f = False        # immediate error
  any_err_f = False    # accumulated errors
  accList = []         # 3 acc value expected
  gyrList = []         # 3 gyro values expected
  tempList = []
  raw_f = False        # alternative view
  instru_f = False

  fifo_f = False       # if true, data is in the fifo
  if (len(dataList) != 0):
    fifo_f = True
  # endif fifo data test

  if (instru_f):
     print("entered read_imu():", supervisor.ticks_ms())
  # endif debug_f

  if (fifo_f):
    pass          # if fifo tthen no need to check for ready here
  else:
    err_f, xlda_f, gda_f, tda_f = is_imu_rdy(0.1) # testing minimum is 0.1
    if (debug_f):
       print("is_imu_rdy() done:", supervisor.ticks_ms())
    # endif debug_f
  # end else not fifo
               
  any_err_f = err_f
    
  if (xlda_f):  # if direct spi, uses auto-increment read all values

    if (fifo_f):
      err_f = False
      acc_HL = dataList
    else:

      err_f,acc_HL = write_spi("IMU","r",[OUTX_L_A,0x00,0x00,0x00,0x00,0x00,0x00])

    if (err_f):
      print("read_imu(): write_spi(): OUTX_L_A does not respond")
    else:
      accX = acc_HL[0] + (acc_HL[1] * 256)     
      accY = acc_HL[2] + (acc_HL[3] * 256) 
      accZ = acc_HL[4] + (acc_HL[5] * 256)

      accX_2sC = twos_comp(accX,16)
      accY_2sC = twos_comp(accY,16)
      accZ_2sC = twos_comp(accZ,16)
      #
      # 1 g = 0x4009 = 16393
      # 
      accX_g = float(accX_2sC)/16393.0
      accY_g = float(accY_2sC)/16393.0
      accZ_g = float(accZ_2sC)/16393.0

      if (cal_f and g_cal_acc_f):
        
        calVal = abs(g_accOfs[0])   # compare absolute values
        absVal = abs(accX_g)
        minus_f = False             # save original sign
        if (accX_g < 0):
          minus_f = True
        # endif
        if (debug_f):
          print("X:",calVal,absVal)  
        # endif debug  
        if (absVal > ACC_MAX):
          accX_g = abs(absVal - calVal)     # apply correction 
          if (minus_f and (accX_g > 0.0)):  # test sign
            accX_g =  -accX_g               # restore sign
          # endif   
        # endif

        calVal = abs(g_accOfs[1])     # compare absolute values
        absVal = abs(accY_g)
        minus_f = False               # save original sign
        if (accY_g < 0):
          minus_f = True
        # endif
        if (debug_f):
          print("Y:",calVal,absVal)
        # endif debug
        if (absVal > ACC_MAX):
          accY_g = abs(absVal - calVal)        # apply correction n            
          if (minus_f and (accY_g > 0.0)):     # test sign
            accY_g =  -accY_g                  # restore sign
          # endif
        # endif
        
        calVal = abs(g_accOfs[2])     # compare absolute values
        calVal = calVal - 1.0         # remove 1g gravity from cal
        absVal = abs(accZ_g)
        absVal = absVal -1.0            # remove 1g gravity from abs 
        minus_f = False               # save original sign
        if (accZ_g < 0):
          minus_f = True
        # endif
        if (debug_f):
          print("Z:",calVal,absVal)
        # endif debug
        if (absVal > ACC_MAX):
          accZ_g = abs(absVal - calVal)    # apply correction, restore gravity             
          if (minus_f and (accZ_g > 0.0)): # test sign
            accZ_g =  -accZ_g              # restore sign
          # endif        
          accZ_g = accZ_g + 1.0           # restore gravity
        # endif 
        
      accList = [accX_g, accY_g, accZ_g]
    # end else                                        
  # endif 
  
  if (gda_f):

    if (fifo_f):
      err_f = False
      gyr_HL = dataList
    else:
      err_f,gyr_HL = write_spi("IMU","r",[OUTX_L_G,0x00,0x00,0x00,0x00,0x00,0x00])
    # end else not fifo read

    if (err_f):
      print("read_imu(): write_spi(): OUTX_L_G does not respond")
    else:
      gyrX = gyr_HL[0] + (gyr_HL[1] * 256)     
      gyrY = gyr_HL[2] + (gyr_HL[3] * 256) 
      gyrZ = gyr_HL[4] + (gyr_HL[5] * 256)  

      gyrX_2sC = twos_comp(gyrX,16)
      gyrY_2sC = twos_comp(gyrY,16)
      gyrZ_2sC = twos_comp(gyrZ,16)
      #
      # 100 dps = 0x2CA4 = 11428
      # 1 dps = 114.28
      # 
      gyrX_dps = float(gyrX_2sC)*100/11428
      gyrY_dps = float(gyrY_2sC)*100/11428
      gyrZ_dps = float(gyrZ_2sC)*100/11428
      
      if (raw_f):
         gyrList = [gyrX, gyrY, gyrZ]
         if (debug_f):
           hexList = [hex(x) for x in gyrList]
           print("gyrList=", hexList)
      else:
        # apply calibration

        if (cal_f and g_cal_gyr_f):
              
          calVal = abs(g_gyrOfs[0])   # compare absolute values
          absVal = abs(gyrX_dps)
          minus_f = False             # save original sign
          if (gyrX_dps < 0):
            minus_f = True
          # endif
          if (debug_f):
            print("X:",calVal,absVal)  
          # endif debug   
          if (absVal > GYR_MAX):
            gyrX_dps = abs(absVal - calVal)   # apply correction          
            if (minus_f and (gyrX_dps > 0.0)):                     # test sign
              gyrX_dps =  -gyrX_dps           # restore sign
            # endif
          # endif test to apply tare
        
          calVal = abs(g_gyrOfs[1])     # compare absolute values
          absVal = abs(gyrY_dps)
          minus_f = False               # save original sign
          if (gyrY_dps < 0):
            minus_f = True
          # endif
          if (debug_f):
            print("Y:",calVal,absVal)
          # endif debug
          if (absVal > GYR_MAX):
            gyrY_dps = abs(absVal - calVal)      # apply correction n    
            if (minus_f and (gyrY_dps > 0.0)):   # test sign
              gyrY_dps =  -gyrY_dps              # restore sign
            # endif
  
          calVal = abs(g_gyrOfs[2])     # compare absolute values
          absVal = abs(gyrZ_dps)
          minus_f = False               # save original sign
          if (gyrZ_dps < 0):
            minus_f = True
          # endif
          if (debug_f):
            print("Z:",calVal,absVal)
          # endif debug
          if (absVal > GYR_MAX):
            gyrZ_dps = abs(absVal - calVal)      # apply correction, restore gravity
            if (minus_f and (gyrZ_dps > 0.0)):   # test sign
              gyrZ_dps =  -gyrZ_dps              # restore sign
            # endif
          # endif test to apply tare

         # endif cal applied

        gyrList = [gyrX_dps, gyrY_dps, gyrZ_dps]

      # end else gyro
  # endif  

  if (tda_f):
    if (fifo_f):
       pass
    else:
      err_f, tempHL = write_spi("IMU","r",[OUT_TEMP_L,0x00,0x00])  
      if (err_f):
        print("read_imu(): write_spi(): OUT_TEMP_L does not respond")
      else:
        tempList = [imu_temp_conv(tempHL)]     
      # end else
    # end else not fifo
  # endif 

  if (debug_f):
    print("accList=",accList,
          "gyrList=", gyrList,
          "tempList=", tempList) 
  # endif debug

  if (err_f):                # accumulate errors
    any_err_f = err_f
  # endif

  return any_err_f, accList, gyrList, tempList

# end read_imu
#
# ----------------------------------
#
"""

CTRL6_C XL_ULP_EN = 0x10

CTRL5_C XL_ULP_EN = 0x80

acc ulp = XL_ULP_EN = 1, XL_HM_MODE = 0
          rates above 208Hz: not available

acc lp  = XL_ULP_EN = 0, XL_HM_MODE = 1
           rates at or below 52Hz: low power
           rates above 52Hz: normal mode or high performance

acc hp =  XL_ULP_EN = 0, XL_HM_MODE = 0
           all rates: high performance

"""
def imu_mode_check(accDict,gyrDict,hiperf_acc_f,ulp_acc_f,lp_gyr_f):
  global CTRL6_C
  global CTRL5_C
  global CTRL7_G

  err_f = False

  #
  # attempt to access odr that does not exist for device
  #
  if ((accDict["fifo_acc"] == None) or (gyrDict["fifo_gyr"] == None)):
    theItem = "gyro"
    if (accDict["fifo_acc"] == None):
      theItem = "acc"
    # end if acc
    print("Error: odr selected doesn't match available for:",theItem)
    err_f = True
    return err_f
  # endif 

  if (hiperf_acc_f and ulp_acc_f):
    print("Error: Can't select both high performance and ultra low power for acc")
    err_f = True
    return err_f
  # endif 

  if (hiperf_acc_f):
    write_spi("IMU","w",[CTRL6_C,0x00])
  else:
    write_spi("IMU","w",[CTRL6_C,0x10])       # disable acc high performance mode 
  # end else not highp

  if (ulp_acc_f):
     write_spi("IMU","w",[CTRL5_C,0x80]) 
     #
     #
     #
     theHz = accDict["hz"]
     if (theHz > 208):
        print("Error: Can't select both ulp and > 208Hz idr for acc:",theHz)
        err_f = True
        return err_f
     # endif
  else:
     write_spi("IMU","w",[CTRL5_C,0x00])
  # end else not ultra low

  if (lp_gyr_f):
    write_spi("IMU","w",[CTRL7_G,0x80]) 
  else:
    write_spi("IMU","w",[CTRL7_G,0x00])      # enable gyro high performance mode 
  # end else not low power

  set_imu_params(accDict,gyrDict,hiperf_acc_f,ulp_acc_f,lp_gyr_f)

  #             # when there is an actual error to return then
  return err_f  # check that this flag is checked when called

# end imu_mode_check
#
# ----------------------------------
#
# Q: why default 416?
# A: this value matches the example in the spec
#
def init_imu(accDict=ODR_416_HZdct,
             gyrDict=ODR_416_HZdct,
             fifo_f = False,
             hiperf_acc_f = IMU_ACC_HIPERF_DEF,
             ulp_acc_f = IMU_ACC_LOPWR_DEF,
             lp_gyr_f = IMU_GYR_LOPWR_DEF,
             debug_f=False):

  global g_active_thresh
  global NO_CHANGE    # configurable select for setting imu response to noactivity
  global ACC_ONLY     
  global ACC_AND_GYRO 
  global INACT_EN 
  global USE_IMU
  global imu_int1
  global IMU_ACC_DEF
  global IMU_GYR_DEF
  global INT1_CTRL     # these are the same spelling as the imu spec
  global CTRL1_XL
  global CTRL2_G
  global CTRL3_C
  global WAKE_UP_THS
  global WAKE_UP_DUR
  global FUNC_CFG_ACCESS
  global EMB_FUNC_EN_A
  global PAGE_RW
  global EMB_FUNC_INIT_A
  global EMB_FUNC_INT1
  global FUNC_CFG_ACCESS
  global TAP_CFG0
  global TAP_CFG2

  err_f = False      # immediate error

  any_err_f = False
  accList = []       # 3 acc value expected
  gyrList = []       # 3 gyro values expected
  tempList = []

  if (USE_IMU==False):   # early exit
    return any_err_f, accList, gyrList, tempList
  # endif 

  xlda_f = False
  gda_f = False
  tda_f = False
  #
  # note: make interrupt pin safe (make an input, not an output)
  # note: code not fully recursive, circuitpy complains
  #       about reusing pin w/o deinit() call, soft reboot
  #
  if (imu_int1 == None):
    imu_int1 = DigitalInOut(mc.pin.GPIO16)
    imu_int1.switch_to_input()
  # end if

  #
  # BDU = block data update
  #  if polling reads at low rate, prevents mix of low byte 
  # from one sample sequentially with high byte from next sample
  #
  write_spi("IMU","w",[CTRL3_C, 0x44]) # enable BDU (don't change SPI)

  # ---------------------------------------
  # acc and gyro settings
  #
  # Q: why disable high performance?
  # A: disable of high performace enables 
  #    access to low power modes
  #
  write_spi("IMU","w",[INT1_CTRL,3])        # gyro and acc on int1 (are interrupts used?)

  #                                         # power/perf consistency check
  err_f = imu_mode_check(accDict,gyrDict,hiperf_acc_f,ulp_acc_f,lp_gyr_f)   

  if (err_f):
    print("init_imu: using odr defaults:", IMU_ACC_DEF["name"], IMU_GYR_DEF["name"])
    # ---------------------
    #  use defaults on error
    # ---------------------
    lp_gyr_f   = False
    hiperf_acc_f = False
    imu_mode_check(accDict,gyrDict,False,False,False)
    write_spi("IMU","w",[CTRL1_XL,IMU_ACC_DEF['cmd']])
    write_spi("IMU","w",[CTRL2_G, IMU_GYR_DEF['cmd']])    # gyro ODR 
    any_err_f = True
    gyrDict = IMU_GYR_DEF
    accDict = IMU_ACC_DEF
  else:       
    write_spi("IMU","w",[CTRL1_XL,accDict['cmd']])  # passed in odr
    write_spi("IMU","w",[CTRL2_G,gyrDict['cmd']])    # passed in odr   
  # endif

  #
  # acc rate determines wakeup
  #
  if (accDict["hz"] == 0):  # if off then
    g_active_thresh = 0     #  no activity threshold seconds
  else:
    active_val = inact_sec2code(accDict['hz'],      # odr_hz
                                0,15,               # min/max
                                accDict["s2s"])     # seconds to sleep
    g_active_thresh = inactive_spb(accDict['hz'], active_val) # save as seconds global
    write_spi("IMU","w",[WAKE_UP_THS,1])                      # threshold to wake up 0x1 minimum
    write_spi("IMU","w",[WAKE_UP_DUR, 0x10 | active_val])     # sec to inactive is a 
    #                                                         # function of odr and bits set
    #                                                         # in WAKE_UP_DUR
  # end else not OFF

  write_spi("IMU","w",[CTRL4_C,0x80])       # dataready masked until filter settles
  #
  # tilt and significant motion
  #
  write_spi("IMU","w",[FUNC_CFG_ACCESS,0x80]) # enable emb register access
  write_spi("IMU","w",[EMB_FUNC_EN_A,0x30])   # enable tilt, sigif motion
  write_spi("IMU","w",[PAGE_RW,0x80])         # EMB_FUNC_LIR interrupt requests latched
  write_spi("IMU","w",[EMB_FUNC_INIT_A,0x30]) # initialize functions
  write_spi("IMU","w",[EMB_FUNC_INT1,0x30])   # route to INT1
  write_spi("IMU","w",[FUNC_CFG_ACCESS,0x00]) # disable emb register access
  #
  # activity/inactivity
  #
  write_spi("IMU","w",[TAP_CFG0,0x61])     # enable, send to pins, latched, clear on read
  if (INACT_EN == NO_CHANGE):
    write_spi("IMU","w",[TAP_CFG2,0x80])    # no change on inactive
  elif (INACT_EN == ACC_ONLY):              
     write_spi("IMU","w",[TAP_CFG2,0xA0])   # acc to 12.5 hz on inactive
  elif (INACT_EN == ACC_AND_GYRO):          
     write_spi("IMU","w",[TAP_CFG2,0xC0])   # acc to 12.5 hz, gyro to sleep on inactive
  else:
    write_spi("IMU","w",[TAP_CFG2,0x80])    # no change on inactive
                                          
  #
  # -------------------------------------
  #
  err_f, xlda_f, gda_f, tda_f = is_imu_rdy()

  any_err_f = err_f  # preserve past errors
  #
  # read acc. gyro, temp once just becauase
  # if there's going to be a problem, raise the error early
  #

  err_f, accList, gyrList, tempList = read_imu(xlda_f,gda_f,tda_f,debug_f=debug_f)

  if (err_f):
    any_err_f = True  # accumulate errors
  # endif error check
  # ------------------
  #  INIT FIFO
  # ------------------
  """ 
    temperature data goes into the fifo at a max 52Hz ODR
    but when the gyro is in power down mode, and the acc is 
    less than 52 Hz, then the temp acq rate follows
    the acc ODR.

    what this means is, temperature data may not show up in the fifo
    at low rates, in any given fifo batch read, as this particular
    case (ODR < 52Hz) hasn't been tested
  """
  if (fifo_f):
    err_f  = init_fifo(accDict,gyrDict,hiperf_acc_f,lp_gyr_f)  
    if (err_f):         # will print to console on error
      any_err_f = True  # accumulate errors
    # endif error check
  # endif initializing fifo 
   
  return any_err_f, accList, gyrList, tempList

# end init_imu
#
# ----------------------------------
#
# only first of 3 flags used
# 
def loop_imu(cnt,                    # count of elements to process
             xlda_in,gda_in,tda_in,  # flags signaling data type to process
             use_cal=False,          # if true then apply calibration
             dataList_in=[],         # list of lists of fifo input data
             debug_f=False):

  err_f = False      # return error flag
  any_err_f = False  # collects errors while looping
  ii = 0             # iterator
  retList = []       # return list of processed data

  fifo_f = False     # if true, then data from fifo
  if (len(dataList_in) > 0):
    fifo_f = True
  # endif
   
  while(True):    # collect

    if (fifo_f):
      if (debug_f):
        pass # print("len(dataList_in),ii=",len(dataList_in),ii)  # hunting off by 1
      # endif debug
      dataList_out = dataList_in[ii]  # iterate thru array
    else:
      dataList_out=[]  # direct read
    # end else

    err_f, accList, gyrList, tempList = read_imu(xlda_f= xlda_in,
                                                gda_f= gda_in,
                                                tda_f= tda_in,
                                                cal_f= use_cal,
                                                dataList= dataList_out,
                                                debug_f=debug_f)
    if (cnt < 0):  # forever
      continue     
    else:          # count up
      if (err_f):
        any_err_f = True  # accumulate errors
      # endif
      #
      # -----------
      # save data
      # -----------
      #
      if (xlda_in): 
        retList = retList + [accList]  # list of triplets 
      elif (gda_in):
        retList = retList + [gyrList]  # list of triplets
      elif (tda_in):
        retList = retList + tempList  # simple append
      # end if
      # ------------
      # count, exit
      # ------------
      if (ii < (cnt-1)):
        ii = ii + 1
      else:
         break           # exit loop
      # end else
    # eb=nd else no error
  # end while forever
  #
  #                             
  if ((debug_f) and (cnt > 0)):     # debug print at the end
    ii = 0
    while(ii < cnt):   
      print("retList[%d]="%(ii),retList[ii])
      ii = ii + 1
    # end while
  # endif debug
 
  return any_err_f, retList

# end loop_imu
#
# ----------------------------------
#
# the alternative to averaging is to mask out the noisiest lsbits
# but the effort of determination how many bits are noisy
# might end up being the same level of computation
# as averaging
#
def calc_tare(cnt=128,
              xlda_f=False,gda_f=False,
              dataList_in=[],
              debug_f=False):
  global g_accOfs
  global g_gyrOfs
  global g_cal_acc_f
  global g_cal_gyr_f

  err_f = False
  any_err_f = False
  retList = [[],[]]

  if ((not xlda_f) and (not gda_f)):
    print("tare, error: no selection")
    err_f = True
    return err_f, retList
  # endif
  # -------------
  # still here?
  # -------------
  if (xlda_f):

    err_f,accList = loop_imu(cnt,                # count
                               True,False,False,   # which
                               False,              # don't apply corrections
                               dataList_in,        # works for fifo or direct
                               debug_f)            # if debug, then dump the data
    if (err_f):
      print("tare: error detected in loop_imu")
      any_err_f = True  # accumulate errors
    # endif

    xSum = 0
    ySum = 0
    zSum = 0

    ii = 0
    while (ii < cnt):
      xSum = xSum + accList[ii][0]
      ySum = ySum + accList[ii][1]
      zSum = zSum + accList[ii][2]
      ii = ii + 1
    # end while

    xOfs = xSum/cnt
    yOfs = ySum/cnt
    zOfs = zSum/cnt

    if (zOfs > 2.0):
      err_f = True
      print("calibration data error, expected Zg < 2:", zOfs)
      g_cal_acc_f = False
    else: 

      g_accOfs = [xOfs, yOfs, zOfs ]  # 1g is force of gravity
 
      g_cal_acc_f = True  # flags values present

    # end else ok
  # endif xlda

  if (gda_f):
    
    err_f,gyrList = loop_imu(cnt,                # count
                             False,True,False,   # which
                             False,              # don't apply corrections
                             dataList_in,        # works for fifo or direct
                             debug_f)            # if debug, then dump the data
    if (err_f):
      any_err_f = True  # accumulate errors
    # endif

    xSum = 0
    ySum = 0
    zSum = 0

    ii = 0
    while (ii < cnt):
      xSum = xSum + gyrList[ii][0]
      ySum = ySum + gyrList[ii][1]
      zSum = zSum + gyrList[ii][2]
      ii = ii + 1
    # end while

    xOfs = xSum/cnt
    yOfs = ySum/cnt
    zOfs = zSum/cnt

    g_gyrOfs = [xOfs, yOfs, zOfs ]  # 1g is force of gravity
 
    g_cal_gyr_f = True  # flags values present

   
  # endif gda

  retList = [g_accOfs, g_gyrOfs]
  
  if (debug_f):
    print("tareList=", retList)

  return any_err_f, retList

# end calc_tare
#
# ----------------------------------
#
def zero_crossings(cnt=64,           # sample size (tradeoff size and time for preciseness)
                   xlda_f=False,     # acc
                   gda_f=False,      # gyro
                   auto_zero=True,   # remove noise near 0
                   dataList_in=[],   # used w/fifo read
                   debug_f=False):   # debug

  global ACC_MAX
  global GYR_MAX

  detects = []        # return list of indices
  err_f = False
  first_f = True
  auto_zero = True    # False for testing

  #
  # note, this was done in two passes (xlda,gda) for testing
  # it can be done in one pass
  #
                 
  if (xlda_f):
    err_f,retList = loop_imu(cnt,     # sample size
                             True,    # xlda
                             False,   # gda
                             False,   # temperature
                             use_cal=True,
                             dataList_in=dataList_in,
                             debug_f=debug_f) 
    if (err_f):
      print("zero_crossings (): Error in loop_imu()")
    else:
      ii = 0
      while(ii < cnt):  
        thresh_f = False 
        elem = retList[ii]

        accX_g = elem[0] # unmarshall parameters
        accY_g = elem[1]
        accZ_g = elem[2]

        accX_abs = abs(accX_g)  # absolute for threshold comparison
        accY_abs = abs(accY_g)
        accZ_abs = abs(accZ_g)  # expect 1.0
        #
        # if any crossing
        #
        if ((accX_abs > ACC_MAX) or (accY_abs > ACC_MAX) or (abs(accZ_abs - 1.0) > ACC_MAX)):       
          #
          # false postive filtering
          #
          if (accX_abs <= ACC_MAX):
            if (auto_zero):
              accX_g = 0.0  
            # end if
          else:
             if ((ii+1 < cnt) and (retList[ii+1][0] > ACC_MAX)): 
               detects = detects + [ii+1]
               thresh_f = True
          # end else
          #
          # test only if previous test passed
          #
          if (not thresh_f):
            if (accY_abs <= ACC_MAX):
              if (auto_zero):
                accY_g = 0.0  
              # end if
            else:
              if ((ii+1 < cnt) and (retList[ii+1][1] > ACC_MAX)): 
                detects = detects + [ii+1]
                thresh_f = True
            # end else
          # endif previously not detected
          #
          # test only if previous test passed
          #
          if (not thresh_f): 
            if (abs(accZ_abs - 1.0) <= ACC_MAX):
              if (auto_zero):
                accZ_g = 1.0  
              # end if
            else:
              if ((ii+1 < cnt) and (abs(retList[ii+1][2] - 1.0) > ACC_MAX)): 
                detects = detects + [ii+1]
                thresh_f = True
            # end else          
          # endif previously not detected

          if (thresh_f and debug_f):
            print("acc [%d][%d] thresh[%2.4f]:"%(len(detects),
                                                 ii, 
                                                 ACC_MAX),
                                                 [accX_g, accY_g, accZ_g])
          # endif above threshold
      
        # endif above threshold

        ii = ii + 1   # counter, next sample

      # end while
    # end else
  # endif xlda

  if (gda_f):
    err_f,retList = loop_imu(cnt,
                             False,True,False,
                             use_cal=True,
                             dataList_in=dataList_in,
                             debug_f=False) 
    if (err_f):
      print("zero_crossings (): Error in loop_imu()")
    else:
      ii = 0
      while(ii < cnt):  
        thresh_f = False 
        elem = retList[ii]

        gyrX_dps = elem[0] # unmarshall parameters
        gyrY_dps = elem[1]
        gyrZ_dps = elem[2]

        gyrX_abs = abs(gyrX_dps)  # absolute for threshold comparison
        gyrY_abs = abs(gyrY_dps)
        gyrZ_abs = abs(gyrZ_dps)
        #
        # if any crossing
        #
        if ((gyrX_abs > GYR_MAX) or (gyrY_abs > GYR_MAX) or (gyrZ_abs > GYR_MAX)):
          #
          # false postive filtering
          #
          if (gyrX_abs <= GYR_MAX):
            if (auto_zero):
              gyrX_dps = 0.0  
            # end if
          else:
             if ((ii+1 < cnt) and (retList[ii+1][0] > GYR_MAX)): 
               detects = detects + [ii+1]
               thresh_f = True
          # end else
          #
          # test only if previous test passed
          #
          if (not thresh_f):
            if (gyrY_abs <= GYR_MAX):
              if (auto_zero):
                gyrY_dps = 0.0  
              # end if
            else:
              if ((ii+1 < cnt) and (retList[ii+1][1] > GYR_MAX)): 
                detects = detects + [ii+1]
                thresh_f = True
            # end else
          # endif previously not detected
          #
          # test only if previous test passed
          #
          if (not thresh_f): 
            if (gyrZ_abs <= GYR_MAX):
              if (auto_zero):
                gyrZ_dps = 0.0  
              # end if
            else:
              if ((ii+1 < cnt) and (retList[ii+1][2] > GYR_MAX)): 
                detects = detects + [ii+1]
                thresh_f = True
            # end else          
          # endif previously not detected

          if (thresh_f and debug_f):
            print("gyro [%d][%d] thresh[%2.4f]:"%(len(detects),
                                                  ii, 
                                                  GYR_MAX),
                                                 [gyrX_dps, gyrY_dps, gyrZ_dps])
          # endif above threshold

        # endif threshold test

        ii = ii + 1
      # end while
    # end else

  # endif gda

  return err_f, detects, retList

# end zero_crossings
#
# ----------------------------------
#
def do_acc(tare_f=False,    # generate tare offset (unit must be physically stable)
           tare_cnt=128,    # number of samples for tare (tradeoff time vs preciseness)
           data_cnt=16,     # number of samples for detecting zero crossings 
           loop_cnt=1,      # how many times in loop
           instru_f=False,
           debug_f=False): # time instrumentation for tradeoff calculation

    global USE_IMU
    global g_acc_x
    global g_acc_y
    global g_acc_z
    global USE_FIFO
    #
    # the problem with direct reads versus the added complication of
    # setting up the fifo, is time waiting for the data ready 
    # the direct read data ready wait time is about 100 ms
    # with the fifo, the fill time is proportional to the ODR
    #
    err_f = False
    detects = []   # zero crossings (to keep quiet when no motion)
    retList = []
    accList=[]     # fifo related return values
    gyrList=[]
    timList=[]

    if (USE_IMU==False):              # early exit
       return err_f, detects, retList
    # endif 

    auto_zero = not tare_f  # turn off autozero for diagnostics on tare

    if (tare_f):
      print("entering calc_tare() for acc, tare_cnt=",tare_cnt)

      if (instru_f):
        t1 = supervisor.ticks_ms()
      # endif instru
      if (USE_FIFO):
        accList, gyrList, timList = process_fifo(count=tare_cnt,
                                                 gyr_rq_f=False,tim_rq_f=False) 
      # endif fifo 

      err_f,tareList = calc_tare(cnt=tare_cnt,
                                 xlda_f=True,gda_f=False,
                                 dataList_in=accList,debug_f=False) 

      if (instru_f):
        t2 = supervisor.ticks_ms()
        print("acc tare elapsed: %d ms"%(t2-t1))
      # endif instru

      print("tareList=", tareList)
    # endif do tare
    
    if (err_f):
      print("error generating calibration data for acc")
    # endif err

    if (instru_f):
      print("acc: %d loops of %d samples"%(loop_cnt,data_cnt))
    # endif

    ii = 0
    while(ii<loop_cnt):

      if (instru_f):
        t1 = supervisor.ticks_ms()
      # endif instru

      if (USE_FIFO):
        accList, gyrList, timList = process_fifo(count=data_cnt,
                                                 gyr_rq_f=False,tim_rq_f=False)
      # endif fifo
      
      err_f, detects, retList = zero_crossings(cnt=data_cnt,
                                               xlda_f=True,
                                               auto_zero=auto_zero,
                                               dataList_in=accList,
                                               debug_f = debug_f)  
      if (instru_f):
        t2 = supervisor.ticks_ms()
        print("acc acq elapsed time: %d ms"%(t2-t1))
      # endif instru

      if (not err_f):
        if ((len(detects) > 0) and debug_f):
          print("acc tc=%d"%(len(detects)))
        # endif
      else:
         print("errors detected in zero_crossings()")
      # end else

      ii = ii + 1

    # end while

    if (not err_f):
      if (len(detects) > 0):
        didx = detects[-1]          # pull detection index from evidence of motion
        g_acc_x = retList[didx][0]  # save for NMEA
        g_acc_y = retList[didx][1]
        g_acc_z = retList[didx][2]
 
      else:
        elem = retList[-1]    # pull data of evidence of non motion
        g_acc_x = elem[0]     # save for NMEA
        g_acc_y = elem[1]
        g_acc_z = elem[2]
      # end else

    return err_f, detects, retList
# end do_acc
#
# ----------------------------------
#
def do_gyro(tare_f=False,    # generate tare offset (unit must be physically stable)
            tare_cnt=64,     # number of samples for tare (tradeoff time vs preciseness)
            data_cnt=16,     # number of samples for detecting zero crossings
            loop_cnt=1,      # how many times in loop
            instru_f=False,
            debug_f=False): # time instrumentation for tradeoff calculation

    global USE_IMU
    global g_gyr_x
    global g_gyr_y
    global g_gyr_z
    global USE_FIFO

    err_f = False
    detects = []
    retList = []
    accList=[]     # fifo related return values
    gyrList=[]
    timList=[]

    if (USE_IMU==False):       # early exit
      return err_f, detects, retList
    # endif 

    auto_zero = not tare_f  # turn off autozero for diagnostics on tare

    if (tare_f):                                 # if tare flag then
      print("entering calc_tare() for gyro...")  #   provide user feedback

      if (instru_f):                             #   if instrumentation flag then
        t1 = supervisor.ticks_ms()               #     set start time
      # endif instru

      if (USE_FIFO):
        accList, gyrList, timList = process_fifo(count=tare_cnt,
                                                 acc_rq_f=False,tim_rq_f=False)
      # endif fifo

      err_f,tareList = calc_tare(cnt=tare_cnt,
                                 xlda_f=False,gda_f=True,
                                 dataList_in=gyrList,debug_f=False) 

      if (instru_f):                             #   if instrumentation flag then
        t2 = supervisor.ticks_ms()               #     set end time
        print("gyro tare elapsed: %d ms"%(t2-t1))            #     print duration
      # endif instru
  
      print("tareList=", tareList,"GYR_MAX=",GYR_MAX)  #   provide user feedback
    # endif do tare
      
    if (err_f):
      print("error generating calibration data for gyro")
    # endif err

    if (instru_f):                               #   provide user feedback
      print("gyro: %d loops of %d samples"%(loop_cnt,data_cnt))
    # endif

    ii = 0
    while(ii < loop_cnt):
      if (instru_f):
        t1 = supervisor.ticks_ms()
      # endif instru

      if (USE_FIFO):
        accList, gyrList, timList = process_fifo(count=data_cnt,
                                                 acc_rq_f=False,tim_rq_f=False)
      # endif fifo

      err_f, detects, retList = zero_crossings(cnt=data_cnt,gda_f=True,
                                               auto_zero=auto_zero,dataList_in=gyrList)  
      if (not err_f):
        if ((len(detects )> 0) and debug_f):
          print("gyro tc=%d"%(len(detects)))
        # endif
      else:
        print("errors detected in zero_crossings()")
      # end else

      if (instru_f):
        t2 = supervisor.ticks_ms()
        print("gyr acq elapsed time: %d ms"%(t2-t1))  # print seconds
      # endif instru

      ii = ii + 1

    # end while


    if (not err_f):
      if (len(detects) > 0):
        didx = detects[-1]          # detection index of evidence of motion
        g_gyr_x = retList[didx][0]  # save for NMEA
        g_gyr_y = retList[didx][1]
        g_gyr_z = retList[didx][2]
 
      else:
        elem = retList[-1]    # evidence of non motion
        g_gyr_x = elem[0]     # save for NMEA
        g_gyr_y = elem[1]
        g_gyr_z = elem[2]
      # end else
 
    # endif

    return err_f, detects, retList

# end do_gyro
#
# -----------------------------------------------------
#
# note: detector coded to detect tilt in status 
#       but not extreme motions in status
#
def is_tilt(cnt=1,            # cnt = -1 -> forever
            debug_f=False):   # debug enable delay before tilt

  global g_tilt        # for sharing with NMEA
  global USE_IMU

  err_f = False
  forever_f = False    # clear loop forever flag
  toggle = None        # filters print of status to changes only
  tilt_mask = 0x10
  sgnif_mask = 0x20    # significant motion (not tested, yet)
  tilt_val = 0
  tilt_delay = 10

  if (USE_IMU==False): # early exit
    return err_f, tilt_val
  # endif 

  if (cnt == -1):      # set loop forever flag
    forever_f = True
  # endif
  
  ii = 0               # exit condition loop counter
  jj = 0               # clear test loop counter
  kk = 0
  while(True):         # loop possibly forever
    jj = jj + 1
    err_f,rd_data = write_spi("IMU","r",[EMB_FUNC_STATUS_MAINPAGE, 0x00])
    if (not err_f):
      if ((rd_data[0] & tilt_mask) == 0):  # if cleared
        #                    # ----------
        tilt_val = 0         # tilt cleared
        #                    # ----------
        if (toggle == None):               # if first time in
          toggle = 0               
        elif ((toggle & tilt_mask) == tilt_mask):  # transition detected
          toggle = 0                                # clear toggle for next xition
          if (debug_f):
            print("tilt bit cleared:",jj,time.time())
            print("waiting 10 seconds before reading tilt status again")
            kk = tilt_delay
            while (kk > 0):
              print("tilt_delay =",kk)
              time.sleep(1)
              kk = kk - 1
            # end while           
          # endif debug
        # endif last tilt set
      else:
        #                    # ----------
        tilt_val = 1         # tilt set
        #                    # ----------
        if (toggle == None):               # if first time in
          toggle = 1

        elif ((toggle & tilt_mask) == 0x0):  # transition detected
          toggle = 1                         # set toggle for next xition
          if (debug_f):
            print("tilt bit set",jj,time.time())
          # endif debug          
        # end else last tilt cleared
      # end else tilted
    else:
       err_f = True
       print("Error in write_spi() in is_tilt()")
    # end else

    if (not forever_f):       # if not forever then 
      ii = ii + 1             #   increment counter
      if (ii >= cnt):         #   if counter exceeds maximum then
         break                #     exit loop
      # endif count termination

    # endif not looping forever

  # end while looping forever

  
  if (tilt_val >= 1):
    tilt_val = 1

  g_tilt = tilt_val

  return err_f, tilt_val

# end is_tilt
#
# -----------------------------------------------------
#
def is_active(cnt = 1,         # cnt = -1 -> forever
              debug_f=False):

  global g_active_thresh  # inactive s before sleep is asserted (calculated based on ODR)
  global g_active         # for sharing with NMEA
  global USE_IMU

  err_f       = False
  forever_f   = False   # clear loop forever flag
  change_mask = 0x40    # change in sleep/active detected (seems to be always true)
  wu_mask     = 0x08    # wake up detected (never programmed)
  sleep_mask  = 0x10    # if set, then sleep state
  active_val  = 0       # inversion of sleep_f and 0,1 not False, True
  wake_up_reg = None    # for debug
  first_f     = True    # first time in loop, part of preventing talky debug messaging
  sleep_f     = None    # boolean, set to None for return value if SPI fails 
  

  if (USE_IMU==False): # early exit
    return err_f, active_val
  # endif 
   
  if (cnt == -1):      # set loop forever flag
    forever_f = True
  # endif
 
  base_time = time.time()
  ii = 0               # exit condition loop counter
  jj = 0               # clear test loop counter
  kk = 0
  while(True):         # loop possibly forever
    jj = jj + 1

    sleep_f = None     # init for error return
    err_f,rd_data = write_spi("IMU","r",[WAKE_UP_SRC, 0x00])

    now_time = time.time()

    if (not err_f):   
      if ((rd_data[0] & sleep_mask) == sleep_mask):
        sleep_f = True 
        if (debug_f):
          deltatime = now_time - base_time
          if ((deltatime >= 5) or first_f):
            print("WAKE_UP_SRC=0x%x,ts=%d,deltat=%d,tts=%d,jj=%d"%(rd_data[0], # status
                                                     now_time,                    # timestapm
                                                     deltatime,                   # delta time
                                                     int(g_active_thresh+0.5),    # time to sleep
                                                     jj))
            # 
            # the next line was a test to see if the ODR changed
            # when the IMU went to sleep
            # he spec says the ODR will change when enabled
            #
            # -> the ODR register setting does not change on sleep
            #                                      
            # read_odr(True)     # does acc odr change?
            #
          
            base_time = now_time

          # endif periodic or first time in

          if (first_f):
            first_f = False
          # endif first entry in loop
              
        # endif debug 
      else: # sleep bit not set
        sleep_f = False
        if (debug_f):                               # provide user feedback
          deltatime = now_time - base_time
          if ((deltatime >= 5) or first_f):
            print("(WAKE_UP_SRC=0x%x,ts=%d,deltat=%d,tts=%d,jj=%d"%(rd_data[0], # status
                                                     now_time,                    # timestapm
                                                     deltatime,                   # delta time
                                                     int(g_active_thresh+0.5),    # time to sleep
                                                     jj))
            base_time = now_time
            first_f = False
          # endif periodic or first time in

          if (first_f):
            first_f = False
          # endif first entry in loop
        # endif debug
          
      # endif else not sleeping

      wake_up_reg = rd_data[0] & (change_mask  | sleep_mask | wu_mask) # save for future use

    else:
       print("Error in write_spi() in is_active()")
    # end else

    if (not forever_f):       # if not forever then 
      ii = ii + 1             #   increment counter
      if (ii >= cnt):         #   if counter exceeds maximum then
         break                #     exit loop
      # endif count termination

    # endif not looping forever

  # end while looping forever

  
  active_val = 1             # init as active
  if (sleep_f == None):      # if error on SPI bus
    active_val = -1          #   then set value undefined 
  else:                      # else
    if (sleep_f):            #    if sleeping
      active_val = 0         #       then not active
    # endif
  # end else decode sleep as true/false

  g_active = active_val

  return err_f, active_val

# end is_active
#
#
# -----------------------------------------------------
#
def odr_to_label(odr_cmd):

  global odrDictList

  theString = ""  # init 

  dictCnt = len(odrDictList)
  ii = 0
  while (ii < dictCnt):

    elem = odrDictList[ii]

    if (elem["cmd"] == odr_cmd):   # match found
       theString = elem["name"]    # get name
       break
    # endif

    ii = ii + 1
  # end while

  return theString

# end odr_to_label
#
# -----------------------------------------------------
#
# if the IMU transitions to low power odr, will the 
# changed odr be reflected in the odr register?
#
def read_odr(debug_f):
  err_f = False
  any_err_f = False
  result = ()

  #
  # odr bits are msnibble 
  #

  err_f,acc_odr = write_spi("IMU","r",[CTRL1_XL, 0x00])
  if (err_f):
    any_err_f = True

  err_f,gyro_odr = write_spi("IMU","r",[CTRL2_G, 0x00])
  if (err_f):
    any_err_f = True

  result = (acc_odr[0], gyro_odr[0])

  if (debug_f):
    label = odr_to_label(result[0])
    print("odr=",hex(result[0]),hex(result[1]),label)
  # endif debug

  return any_err_f,result

# end read_odr
#
# ----------------------------------------------------
#
def reset_imu():
  err_f = False

  write_spi("IMU","w",[CTRL3_C,1]) # reset set
  
  err_f,rdbkList=write_spi("IMU","r",[CTRL3_C,0]) # reset get

  if (err_f or (rdbkList[0] & 1) != 0):
    err_f = True
    print("error: imu doesn't clear reset")
  # endif reset test

  return err_f

# end reset_imu
#
# -----------------------------------------------------
#
def init_all(accDict=ODR_416_HZdct,
             gyrDict=ODR_416_HZdct,
             fifo_f=False,
             debug_f=False):
  global UART0_BAUD       
  global UART1_BAUD   
  global IAM_GNSS
  global IAM_AFE
  global lna_en       # gnss shared pin with afe cs_max_misc
  global BYPASS_LNA
  global EXT_GPS
  global g_gps_ant
  global cs_max_misc  # afe shared pin with gnss lna_en
  global cs_max_tx
  global cs_max_rx
  global g_spi_max_rxtx   # spi bus shared w/cs_max_tx and cs_max_rx

  err_f = False
  imu_err_f = False
  mag_err_f = False
  any_err_f = False

  print("\r\ninit_all() ...")

  print("initializing SPI bus")
  init_spi()                   # gnss and afe shared

  if (IAM_AFE):
    print("\n*** IAM_AFE ***\n")
    if (cs_max_misc != None):
      print("WARNING: init_all() soft reboot skips pin re-initialization")
    else:
      cs_max_misc =  DigitalInOut(mc.pin.GPIO8)
      cs_max_misc.switch_to_output(value=True)
      cs_max_misc.value = 1
      cs_max_tx = DigitalInOut(mc.pin.GPIO24)
      cs_max_tx.switch_to_output(value=True)
      cs_max_tx.value = 1
      cs_max_rx = DigitalInOut(mc.pin.GPIO9)
      cs_max_rx.switch_to_output(value=True)
      cs_max_rx.value = 1
      g_spi_max_rxtx = bitbangio.SPI(mc.pin.GPIO18, MOSI=mc.pin.GPIO19, MISO=mc.pin.GPIO20)

      init_max()
  else:
    print("\n*** IAM_GNSS ***\n")
    #
    # ---------------------------------------------------
    # power on default, gps is external, lna is enabled
    # ---------------------------------------------------
    #
    if (g_gps_ant != None):   # guard skips clause on soft reboot
      print("WARNING: init_all() soft reboot skips pin re-initialization")
    else:
      g_gps_ant = DigitalInOut(mc.pin.GPIO6) 
      if (EXT_GPS == True):
        gps_sel_str="external"
        g_gps_ant.switch_to_output(value=True)  # default external
      else:
        gps_sel_str="internal"
        g_gps_ant.switch_to_output(value=False) # low switches to internal
      # endif
      print("selecting GPS antenna:", gps_sel_str)
 
      if (lna_en == None):    # define once and only once w/o deinit
        lna_en = DigitalInOut(mc.pin.GPIO8)
      # endif once and only once
      if (BYPASS_LNA == True):    
        lna_en.switch_to_output(value=True)  # BYPASS enabled
      else:
        lna_en.switch_to_output(value=False) # BYPASS off
      # end else BYPASS disabled
    # end if first time in 
  # end else GNSS

  locked = isCW_OCXO_LOCKED()
  print("isCW_OCXO_LOCKED:",locked)

  print("initializing uarts, uart0(usb)=%d, uart1(ublox)=%d"%(UART0_BAUD,UART1_BAUD))
  init_uart(UART0_BAUD,UART1_BAUD)

  any_err_f = pr_mag_id()         # print constant to verify comms
  if (any_err_f):
    err_f = True
    mag_err_f = True
  else:
    print("initializing magnetometer")
    any_err_f,values = init_mag()   # defaults for ccr and update rate
    if (any_err_f):
      err_f = True
      mag_err_f = True
    # endif
    
    if (not mag_err_f):
      read_mag(True) 
    # endif 
  # end else magnetometer ok

  print("reading temperatures")
 
  get_mag_tp(True,True)  # print magnetometer temperature (not on spi bus)
  get_sw_tp(True,True)   # print voltage switcher temperature

  print("init_imu(), fifo_f=",fifo_f)

  any_err_f = pr_imu_id()   # print constant to verify comms
  if (any_err_f):
    err_f = True
    imu_err_f = True
  else:  
    res = init_imu(accDict=accDict,      # init acc and gyro using defaults
                 gyrDict=gyrDict,
                 fifo_f=fifo_f,
                 hiperf_acc_f = IMU_ACC_HIPERF_DEF,
                 lp_gyr_f = IMU_GYR_LOPWR_DEF,
                 debug_f=debug_f) 

    print("imu fifo sampled out=(any_err_f, accList, gyrList, tempList)",res)
    if (any_err_f):
      err_f = True
      imu_err_f = True
    else:
      any_err_f, xlda_f, gda_f, tda_f = is_imu_rdy() 
      print("is_imu_rdy(): (err_f, xlda_f, gda_f, tda_f):", any_err_f, 
                                                            xlda_f, 
                                                            gda_f, 
                                                            tda_f)
      if (any_err_f):
        err_f = True
        imu_err_f = True
      # endif imu error check
      
    # end else no imu error, read imu data

    print("init_imu() complete")
    
  # end else init_imu

  print("init_all() complete\n")

  return err_f, mag_err_f, imu_err_f

# end init_all
#
# ----------------------------------------------------
#
# this is an attempt to keep the temperature fifo fill period
# within 1/200 of the acc and gyro rates as the temperature
# period settable range ihas only 3 possible settings (besides off)
#
# the goal is to sample the temperature within 200 reads while sampling
# gyro and acc values
#
# note: measurement shows that resetting/emptyng the fifo 
#       causes the temperature value to enter the fifo sooner
#       than at the periodic rate
#
# note: filling at less than 52Hz hasn't been tested
#
def odr2fifo_trate(theODR):
  global g_fifo_mode
  global ODR_T_1_6Hz 
  global ODR_T_12_5Hz
  global ODR_T_52Hz
  global FIFO_CONT_MODE

  if (theODR < 200):
    USE_ODR_TR = ODR_T_1_6Hz
  elif (theODR < 2000):
    USE_ODR_TR = ODR_T_12_5Hz
  else:
    USE_ODR_TR = ODR_T_52Hz
  # end else

  g_fifo_mode = FIFO_CONT_MODE | USE_ODR_TR  # write to global

  return g_fifo_mode  # value to be written to FIFO_CTRL4
  
# end odr2fifo_trate
#
# ----------------------------------------------------
#
# the fifo's odr is a different register 
# than the acc and gyro's odr
#
def odr2fifo_odr(accDict=None,gyrDict=None):
  err_f = False
  acc_ctrl = 0
  gyr_ctrl = 0
  odr_ctrl = 0
  debug_f = False

  if (accDict == None) and (gyrDict == None):
    err_f = True
    print("error: odr2fifo_odr(): missing parameter")
  # endif error check
  
  if (not err_f):
    if (accDict != None):
      try:
        acc_ctrl = accDict["fifo_acc"]
      except Exception as eobj:
        err_f = True 
        print("error: odr2fifo_odr(): bad dict parameter: 'fifo_acc'")
      # end try/except
  # end if not error

  if (not err_f):
    if (gyrDict != None):
      try:
        gyr_ctrl = gyrDict["fifo_gyr"]
      except Exception as eobj:
        err_f = True 
        print("error: odr2fifo_odr(): bad dict parameter: 'fifo_gyr'")
      # end try/except
  # end if not error

  if (not err_f):
    odr_ctrl = acc_ctrl | gyr_ctrl
    write_spi("IMU","w",[FIFO_CTRL3, odr_ctrl])
    if (debug_f):
     print("fifo odr_ctrl=", odr_ctrl)
    # endif debug
  # endif error check

  return err_f

# end odr2fifo_odr
#
# ----------------------------------------------------
#
def reset_fifo():
  global g_fifo_mode  # continuous and timing rate control bits

  write_spi("IMU","w",[FIFO_CTRL4,0])

  write_spi("IMU","w",[FIFO_CTRL4,g_fifo_mode])

# end reset_fifo
#
# ----------------------------------------------------
#
def init_fifo(accDict,gyrDict,hiperf_acc_f=False,ulp_acc_f=False,lp_gyr_f=False):
  global g_fifo_mode  # continuous and timing rate control bits

  err_f = False
  #
  err_f = reset_imu()  # prints to console if reset fails
  if (err_f):
    return err_f
  # endif early exit

  write_spi("IMU","w",[CTRL3_C,0x44])   # BDU and interrupts
    
  imu_mode_check(accDict,gyrDict,hiperf_acc_f,ulp_acc_f,lp_gyr_f)
    
  write_spi("IMU","w",[CTRL1_XL, accDict['cmd']]) # needs to match fifo setting
  write_spi("IMU","w",[CTRL2_G, gyrDict['cmd']])     
  #
  write_spi("IMU","w",[FIFO_CTRL1,16])  # fifo watermark
  #
  # fifo don't stop, don't compress, watermark msb's 0
  #
  write_spi("IMU","w",[FIFO_CTRL2,0])
  #
  # tested using 416Hz acc and gyro, match default rate
  #
  odr2fifo_odr(accDict,gyrDict)    # FIFO_CTRL3 has odr controls
  #
  # yes temperature, no compression w/timestamp, continuous
  #
  fifo_mode = odr2fifo_trate(accDict['hz'])       # writes to global g_fifo_mode

  write_spi("IMU","w",[FIFO_CTRL4,fifo_mode])

  return err_f

# end init_fifo
#
# ----------------------------------------------------------
#
# note: there is a watermark flag and it is set but
#       imu processing can't delay other processing
#       intent: return on any data present
#
def fifo_status():
  debug_f = False
  err_f = False
  rdy_f = False
  tag_cnt = 0
  #
  err1_f,res1=write_spi("IMU","r",[FIFO_STATUS1,0])  # data count
  #
  err2_f,res2=write_spi("IMU","r",[FIFO_STATUS2,0]) # count plus overflow
  if (err1_f or err2_f):
    print("error in fifo_status(): write_spi()")
    err_f = True
  else:
    #
    tag1 = res1[0]
    reg2 = res2[0]
    full = reg2 & 0xe0
    tag2 = (reg2 & 3) << 8   # see the spec, count is 10 bits
    #
    tag_cnt = tag2 | tag1
    #
    if (full or (tag_cnt > 0) ):
      rdy_f = True
      if (debug_f):
        print("tag_cnt=", tag_cnt)
      # endif 
    # endif full or data present
  # end else not error

  return err_f,rdy_f,tag_cnt

# end fifo_status
#
# ----------------------------------------------------------
#
def decode_fifo_tag(theTag):
  acc_f = False
  gyr_f = False
  temp_f = False
  
  if (theTag == 0x1):
    gyr_f = True
  elif (theTag == 0x2):
    acc_f = True
  elif (theTag == 0x03):
    temp_f = True
  else:
    print("unhandled tag=",theTag)
  #
  # note: not running compression mod
  # note: timestamp is dependent on compression mode
  #
  return acc_f,gyr_f,temp_f
# end decode_fifo_tag
#
# ----------------------------------------------------------
#
def read_fifo_tag():
  err_f = False    # error
  retTag = None    # return tag 
  acc_f = False    # tag decoded
  gyr_f = False
  temp_f = False
  ts_f = False
  err_f,res3=write_spi("IMU","r",[FIFO_DATA_OUT_TAG,0])
  if (err_f):
    print("error in read_fifo_tag(): write_spi()")
  else:
     tag_reg = res3[0]
     theTag = tag_reg >> 3
     acc_f,gyr_f,temp_f = decode_fifo_tag(theTag)
     if (acc_f or gyr_f or temp_f or ts_f):
       retTag = theTag     
     # endif any
  # endif not error
  return err_f,retTag,acc_f,gyr_f,temp_f

  # endif not error
# end read_fifo_tag
#
# ----------------------------------------------------------
# 
def read_fifo_data(theTag):
  err_f = False
  readList = []
  #
  err_f,readList= write_spi("IMU","r",[FIFO_DATA_OUT_X_L,0,0,0,0,0,0])
  if (err_f):
     print("error in read_fifo_data(): write_spi()")
  #
  return err_f, readList
# end read_fifo_data
#
# ----------------------------------------------------------
#
# note: gyro and acc values are returned raw
#       (to be converted into floats elsewhere)
#       temperature is converted to deg C here
#
#
def process_fifo(count=1,            # count of minimum searches       
                 acc_rq_f=True,      # requesting acc data
                 gyr_rq_f=True,      # requesting gyro data
                 tim_rq_f=True):     # requesting time data

  err_f = False
  print_f = False       # debugging
  print_temp_f = False  # temperature specific debugging
  jj = 0                # outer loop counter
  instru_f = False      # timing instrumentation
  any_gyr_f = False     # flag part of determining when all data is updated
  any_acc_f = False
  any_temp_f = False
  all_f = False         # all data updated
  accList=[]
  gyrList=[]
  timList=[]

  if (instru_f):
     print("start ts=",supervisor.ticks_ms())
  # endif instrumenting

  reset_fifo()         # empty fifo

  while (True):        # outer loop
    if (instru_f):
      pass # print("enter fifo_status ts=",supervisor.ticks_ms())
    # endif instru

    err_f,rdy_f,tag_cnt = fifo_status()  # how much data?

    if (instru_f):
      pass # print("exit fifo_status ts=",supervisor.ticks_ms())
    # endif

    if (not err_f):
      if (rdy_f):
        ii = 0
        while (ii < tag_cnt):     # inner loop
          err_f,theTag,acc_f,gyr_f,temp_f = read_fifo_tag()
          if (not err_f):
            err_f,dataList=read_fifo_data(theTag)  
          if (not err_f):
            if (theTag != None):  # if handleable
              if (acc_f):
                any_acc_f = True
                accList  = accList + [dataList]
                if (print_f):
                  print("acc=",dataList)
              elif(gyr_f):
                any_gyr_f = True
                gyrList  = gyrList + [dataList]

                if (print_f):
                  print("gyr=",dataList)
              elif(temp_f):
                any_temp_f = True
                temp = imu_temp_conv(dataList)  # conversion done in place
                timList = timList+[temp]
                if (print_f or print_temp_f):
                  print("temp=", temperature)                 
                # endif print temperature
                if (instru_f):
                  print("temp ts=",supervisor.ticks_ms())
                # endif timestamping
              #
            # endif any useful data
          # endif not error
          #
          # note: data in the fifo may show up at different rates
          #       the known issue is temperature, temperature may be
          #       requested at different periods from acc and gyr 
          #       otherwise these flags allow returning on the count
          #       of specific tagged-type data captured
          #
          if (tim_rq_f==False):
             any_temp_f = True
          # endif not searching for time data
          if (acc_rq_f==False):
             any_acc_f = True
          # endif not searching for acc data
          if (gyr_rq_f==False):
             any_gyr_f = True
          # endif not searching for gyro data
          
          if (any_gyr_f and any_acc_f and any_temp_f):  # check completeness
             any_gyr_f = False
             any_acc_f = False
             any_temp_f = False
             all_f = True            # check in outer loop
             break                   # break inner loop
          # endif exit early test

          ii = ii + 1  # read until no more data
          
        # end inner while
    # end if not error
    #  
    if (all_f):            # if all measurements taken
      all_f = False        # reset all flag
      if (count != -1):    # if not forever                    
         jj = jj + 1       #   count 
         if (jj == count): #   if test count is limit
           break           #      break outer loop
         # endif done
       # endif counting
    # endif test on all

  # end outer while

  if (instru_f):
     print("end ts=",supervisor.ticks_ms())
  # endif instrumenting

  return accList, gyrList, timList

# end process_fifo 
#
# -----------------------------------------------------
#
def backup_char(theUart,prompt_in,theStr):
  theLen = len(theStr)                  # get length
  if (theLen > 0):                       # if anything on line then
    bsStr = theStr[0:theLen-1]          #   backup in line
    theStr = bsStr                      #   reconstruct input
    theStr = theStr + " "              #   add overwriting space
    out_str = "\r" + prompt_in + theStr #   reconstruct line
    theUart.write(out_str)                 #   reprint line overwrite last char
    bsStr = theStr[0:theLen-1]          #   backup once again
    theStr = bsStr                      #   reconstruct input 
    out_str = "\r" + prompt_in + theStr #   reconstruct shorter line
    theUart.write(out_str)                 #   reprint shorter line
  # endif
  theLen = len(theStr)
  return theLen, theStr
# end backup_char
#
# -----------------------------------------------------
#
def set_human(interact_f):
  global g_human_f
  debug_f = True
  #
  # -----------------------------------------------------
  # this command is used to discriminate between a human 
  # and a machine, will want to disable if a machine
  #
  # the effect will be, if human then echo input
  #
  # -----------------------------------------------------
  #
  g_human_f = interact_f

  if (debug_f):
    print("interact_f=", interact_f)  # debugging feedback
  # endif debug

# end set_human
#
# -----------------------------------------------------
#
def get_human():
  global g_human_f

  return g_human_f

# end get_human
#
# -----------------------------------------------------
#
def PMITTE_to_ts(theStr):
   theTs = -1
   err_code = 0

   #
   # sample string: "$PMITTEP,1738761458*69"
   #

   theList = theStr.split(",")       # ts is index 1
   try:
     theItem = theList[1].split('*') # ts is index 0, should never happen
   except Exception as eobj:
      err_code = -1
      print("Error: PMITTE_to_ts(), badly formed NMEA")
   else:
     try: 
       theTs = int(theItem[0])       # string to int
     except Exception as eobj:
        err_code = -2
        print("Error: PMITTE_to_ts(), expected an integer")
     else:
       if (theTs < 0):              #    if no errors then 
         err_code = -3
         print("Error: PMITTE_to_ts(), negative time")
       # endif time check
     # end else time is an iteger
   # end else time extracted ok

   return err_code, theTs

# end PMITTE_to_ts
#
# -----------------------------------------------------
#
def params2string(theCmd):

  cmdNoCk = theCmd.split('*')          # remove '*' to end
  paramList = cmdNoCk[0].split(",")    # enlist remove commas
  paramStr = ','.join(paramList[1:])   # put back commas w/o command

  return paramStr

# end params2string
#
# -----------------------------------------------------
#
def handle_time_cmd(theCmd):
  global TSRC_NOTSET       # time source and epoch constants
  global TSRC_GNSS
  global TSRC_EXT
  global TEPOCH_NOTSET
  global TEPOCH_PPS
  global TEPOCH_NMEA
  global TEPOCH_IMM

  global g_time_source   # time source and epoch setting
  global g_time_epoch

  debug_f = True
  err_f = False
  err_code = 0
  set_time_f = False  # guard on external source without epoch
  cmd_code = "XXX"    # if triggered, then internal error
  extra = ""          # return info for ok response
  
  # -----------------------------------
  # 012345678
  # $PMITTP? - query time parameters
  # $PMITTSG - set time source gnss
  # $PMITTSE - set time source external
  # $PMITTEP - set time epoch pps
  # $PMITTEN - set time epoch nmea
  # $PMITTEI - set time epoch immediate
  # -----------------------------------
  if (debug_f):
    print("theCmd=", theCmd)
  # endif

  #
  # 1. verify checksum
  # 2, do the command
  #
  err_f,pktout = add_cksum(theCmd,debug_f=False)  # if debug_f then print error info
  if (err_f):                                     # any error here is ultimately a checksum error
    err_code = -1
  else:
    if ((theCmd[0:8] == "$PMITTSG") or ((theCmd[0:8] == "$PMITTEN"))):
       cmd_code = "TEN"
       if (theCmd[0:8] == "$PMITTSG"):
         cmd_code = "TSG"
       # endif 
       if (debug_f):
         print("time cmd detected: internal")
       # endif 
       g_time_source = TSRC_GNSS
       g_time_epoch  = TEPOCH_NMEA
       set_NMEA_acq(True) 
    elif ((theCmd[0:8] == "$PMITTSE") or (theCmd[0:8] == "$PMITTEP") or (theCmd[0:8] == "$PMITTEI")): 
       set_time_f = False              # guard on setting time
       if (theCmd[0:8] == "$PMITTEP"):
         cmd_code = "TEP"
         set_time_f = True
         if (debug_f):
           print("time cmd detected: external pps, not fully handled") 
         # endif 
         g_time_source = TSRC_EXT
         g_time_epoch  = TEPOCH_PPS
       elif (theCmd[0:8] == "$PMITTEI"):
         cmd_code = "TEI"
         set_time_f = True
         if (debug_f):
           print("time cmd detected: external immediate") 
         # endif
         g_time_source = TSRC_EXT
         g_time_epoch  = TEPOCH_IMM
       else:                         # else external source but no epoch
         cmd_code = "TSE"
         g_time_source = TSRC_EXT
         g_time_epoch  = TEPOCH_NOTSET
         #
         # timestamp may be in NMEA data but
         # don't use to set time because no epoch yet
         #
       # end else source only
       #
       if (set_time_f):                       # if intent is to set time then
         extra = params2string(theCmd)        # echo parameters in response
         err_code, ts = PMITTE_to_ts(theCmd)  #    extract ts from message
         if (err_code != 0):
           err_code = err_code -10 # differentiate checksum error from others
           err_f = True
         else:                              # ------------------- 
           err_f = setTimeFromTs(ts)        #   set time from ts
           if (err_f):                      # -------------------
              err_code = -20
           # end if error
           if (err_code == 0):
             if (debug_f):
               print("time set:",ts)
             # endif debug
           # endif no error
         # else no error, but more checking to do
       
         if (err_code == 0):        # if time set by command was ok then
           set_NMEA_acq(False)      # don't continue to update from NMEA
         # endif not error in previous operations
       # endif time set flag set
    # end elif handled
    #                     012345678
    elif (theCmd[0:8] == "$PMITTP?"): 
      cmd_code = "TP?"
      err_code = 0
      extra = "%d,%d"%(g_time_source, 
                       g_time_epoch)                       
    else:
      print("unhandled PMITT case:",theCmd)
      cmd_code = theCmd[5:8]
      err_code = -2
    # end else unhandled PMIT type command
  # end else not a checksum error

  return err_code,cmd_code,extra

# end handle_time_cmd
#
# -----------------------------------------------------
#
def PMITR_to_rate(theCmd):
  err_f = False
  err_code = 0
  retval = None
  dd = None
  #
  # typical input: "$PMITRI,2*05"
  #
 
  bb = theCmd.split('*') # remove '*' to end

  cc = bb[0].split(",")  # isolate first comma

  try:
    dd = int(cc[1])        # the rate is here
  except Exception as eobj:
    err_f = True
    err_code = -1
    print("Error: PMITR_to_rate(), expected an integer")
  # end try/except

  if (not err_f):
    if ((dd > -1) and (dd < 61)):
       retval = dd
    else:
       err_f = True
       err_code = -2
       print("Error: PMITR_to_rate(), out of range:",dd,"limits: 0,60")
    # end else error
  # endif ok value

  return err_code, retval

# end PMITR_to_rate
#
# -----------------------------------------------------
#
def handle_rate_cmd(theCmd):
  """
   PMITRT  - telemetry rate
   PMITRM  - telemetry rate magnetometer
   PMITRI  - telemetry rate imu
   PMITRA  - telemetry rate set all
  """
  global g_telem_updr           # the periodic rate
  global g_mag_tupdr            # t for telemetry
  global g_imu_updr
  global g_telem_time           # base time to next telem tx
  global g_mag_time             # base time to next mag tx
  global g_imu_time             # base time to next imu tx

  err_f = False
  debug_f = True                # debug_f provides path feedback
  cmd_code = "RX"
  extra = ""

  err_f,pktout = add_cksum(theCmd)  # returns error if bad checksum
  if (not err_f):
    if (theCmd[6:8] == "T,"):
      cmd_code = "RT"
      err_code, trate = PMITR_to_rate(theCmd)
      if (err_code == 0):
        if (debug_f):
          print("rate cmd detected: telemetry=",trate)
        # endif debug
        g_telem_updr = trate
        g_telem_time  = None    # restart the clock
      # endif ok
    elif (theCmd[6:8] == "M,"):
      cmd_code = "RM"
      err_code, trate = PMITR_to_rate(theCmd) 
      if (err_code == 0):
        if (debug_f):
          print("rate cmd detected: mag=",trate)
        # endif debug
        g_mag_iupdr = trate
        g_mag_time  = None    # restart the clock
      # endif ok
    elif (theCmd[6:8] == "I,"):
      cmd_code = "RI"
      err_code, trate = PMITR_to_rate(theCmd) 
      if (err_code == 0):
        if (debug_f):
          print("rate cmd detected: imu=",trate)
        # endif debug
        g_imu_updr = trate
        g_imu_time  = None    # restart the clock
      # endif ok
    elif (theCmd[6:8] == "A,"):   # A for all
      cmd_code = "RA"
      extra = params2string(theCmd)       # echo parameters in response
      err_code, trate = PMITR_to_rate(theCmd) 
      if (err_code == 0):
        if (debug_f):
          print("rate cmd detected: all=",trate)
        # endif debug
        g_telem_updr  = trate      # set/reset all three  
        g_mag_iupdr   = trate  
        g_imu_updr    = trate
        g_telem_time  = None
        g_mag_time    = None 
        g_imu_time    = None
      # endif ok
    elif (theCmd[6:8] == "?*"):
      cmd_code = "R?"
      err_code = 0
      extra = "%d,%d,%d"%(g_telem_updr, 
                           g_mag_tupdr, 
                           g_imu_updr)
      # 
    else:                # else not a handled rate command
      cmd_code = theCmd[5:7]
      err_f = True
      err_code = -10
      if (debug_f):
        print("unhandled PMITR case:",theCmd)
      # endif debug
    # end else bad rate command
  # endif no error

  return err_code,cmd_code,extra

# end handle_rate_cmd
#
# -----------------------------------------------------
#
def PMITMGS_to_vals(theCmd):
  global MAG_CCR_MIN   
  global MAG_CCR_MAX
  global MAG_UPDR_MIN
  global MAG_UPDR_MAX
  

  err_f = False
  ccr_ret = None
  updr_ret = None
  err_code = 0
  #
  # note: no cmd_code because the command so simple
  #       the code is known before calling this function
  #
 
  bb = theCmd.split('*') # remove '*' to end

  paramList = bb[0].split(",")  # isolate first comma

  if (len(paramList) < 3):
    err_f = True
    err_code = -1
    print("Error: PMITMGS_to_vals(): too few parameters")

  else:

    ccr_str = paramList[1]

    updr_str = paramList[2]

    try:
      ccr_int = int(ccr_str)        
    except Exception as eobj:
      err_f = True
      err_code = -2
      print("Error: PMITMGS_to_vals(), ccr, expected an integer")
    # end try/except
  # end else

  if (not err_f):
    try:
      updr_int = int(updr_str)        
    except Exception as eobj:
      err_f = True
      err_code = -3
      print("Error: PMITMGS_to_vals(), updr, expected an integer")
    # end try/except
  # end not error

  if (not err_f):
    if ((ccr_int >= MAG_CCR_MIN) and (ccr_int <= MAG_CCR_MAX)):  
       ccr_ret = ccr_int
    else:
       err_f = True
       err_code = -5
       print("Error: PMITMGS_to_vals(), out of range:", ccr_int,
              "limits: %d,%d"%(MAG_CCR_MIN, MAG_CCR_MAX))
    # end else error
  # endif not error

  if (not err_f):
    if ((updr_int >= MAG_UPDR_MIN) and (updr_int <= MAG_UPDR_MAX)):
       updr_ret = updr_int
    else:
       err_f = True
       err_code = -6
       print("Error: PMITMGS_to_vals(), out of range:",updr_int,
             "limits: %d,%d"%(MAG_UPDR_MIN, MAG_UPDR_MAX))
    # end else error
  # endif not error

  return err_code, ccr_ret, updr_ret

# end PMITMGS_to_vals
#
# -----------------------------------------------------
#
def handle_mag_cmd(theCmd,vector):
  """
  $PMITMGS,200,150*5F - cycle count, update rate 
  $PMITMG?            - query cycle count, update rate
  """
  err_f = False
  err_code = 0
  debug_f = True               # debug_f provides path feedback
  queryStr = ""
  tlc = "MG" + vector          # tlc = 3 letter code

  err_f,pktout = add_cksum(theCmd)  # returns error if bad checksum
  if (err_f):
    err_code = -10 
    return err_code,tlc,queryStr
  # endif
  # -------------
  # still here?
  # -------------
  if (vector == "S"):                                     # if set parameters then 
    queryStr = params2string(theCmd)                      # echo parameters in response
    err_code, ccr_new, updr_new = PMITMGS_to_vals(theCmd) #   extract parameters
    if (err_code == 0):
      err_f, values = init_mag(ccr_new,updr_new)
      # 
      # the only error possible here would be data never ready
      # either the mag hardware failed or the parameters cause failure
      #
      if (not err_f):
        set_mag_params(ccr_new,updr_new)             # write params to globals
      else:
        err_code = -20
        if (debug_f):
          print("Error: handle_mag_cmd(): init_mag(%d,%d)"%(ccr_new,updr_new)) 
        # endif debug
      # end else error
    # endif not error
  elif (vector == "?"):
    ccr_int, updr_int = get_mag_params()
    queryStr = str(ccr_int) + "," + str(updr_int)
  else:
     err_code = -30  # 
  # end else error

  return err_code,tlc,queryStr

# end handle_mag_cmd
#
# -----------------------------------------------------
#
def PMITIMU_to_vals(theCmd): 

  err_f = False
  accDict      = None
  gyrDict      = None
  ahiperf_bl = None
  aulp_bl    = None
  glp_bl     = None
  err_code = 0
  #
  # note: no cmd_code because the command so simple
  #       the code is known before calling this function
  #
 
  bb = theCmd.split('*') # remove '*' to end

  paramList = bb[0].split(",")  # isolate first comma

  if (len(paramList) < 5):
    err_f = True
    err_code = -1
    print("Error: PMITIMU_to_vals(): too few parameters") 

  else:

    acc_str = paramList[1]

    gyr_str = paramList[2]

    ahiperf_str = paramList[3]

    aulp_str = paramList[4]

    glp_str = paramList[5]

    #
    # find string in dictionary list "name"
    # found, then return dictionary
    #

    accDict = dictInList(odrDictList,"name",acc_str) 
    if (accDict==None):
      err_f = True
      err_code = -2
      print("Error: PMITIMU_to_vals(): accDict not found for label:", acc_str) 
    # endif
  # end else not error
  
  if (not err_f):
    gyrDict = dictInList(odrDictList,"name",gyr_str) 
    if (gyrDict==None):
      err_f = True
      err_code = -3
      print("Error: PMITIMU_to_vals(): gyrDict not found for label:", acc_str) 
  # endif not error

  if (not err_f):
    if (ahiperf_str == "0"):
      ahiperf_bl = False
    elif (ahiperf_str == "1"):
       ahiperf_bl = True
    else:
      err_f = True
      err_code = -5
      print("Error: PMITIMU_to_vals(): boolean not found for label:", ahiperf_str)
    # end else error
  # endif not error

  if (not err_f):
    if (aulp_str == "0"):
      aulp_bl = False
    elif (aulp_str == "1"):
       aulp_bl = True
    else:
      err_f = True
      err_code = -6
      print("Error: PMITIMU_to_vals(): boolean not found for label:", ahiperf_str)
    # end else error
  # endif not error

  if (not err_f):
    if (glp_str == "0"):
      glp_bl = False
    elif (glp_str == "1"):
       glp_bl = True
    else:
      err_f = True
      err_code = -7
      print("Error: PMITIMU_to_vals(): boolean not found for label:", glp_str)
    # end else error
  # endif not error

  return err_code, accDict, gyrDict, ahiperf_bl, aulp_bl, glp_bl

# end PMITIMU_to_vals
#
# -----------------------------------------------------
#
def handle_imu_cmd(theCmd,vector):
  """
     $PMITIMU,ODR_416_HZ_HP,ODR_416_HZ_HP,0,0,0*4D

     $PMITSR,0,IM?,ODR_416_HZ_HP,ODR_416_HZ_HP,0,0,0*16
  """
  err_f = False
  err_code = 0
  debug_f = True               # debug_f provides path feedback
  queryStr1 = ""               # build-a-string intermediate 
  queryStr2 = ""               # build-a-string final answer
  tlc = "IM" + vector          # tlc = 3 letter return code

  err_f,pktout = add_cksum(theCmd)  # returns error if bad checksum
  if (err_f):
    err_code = -10 
    return err_code,tlc,queryStr
  # endif
  # -------------
  # still here?
  # -------------
  if (vector == "U"):                                    # if set parameters then 
    queryStr2 = params2string(theCmd)                    # echo parameters in response
    packed_params = PMITIMU_to_vals(theCmd) #   extract parameters
    err_code, accDict, gyrDict, acc_hipf_f, acc_ulp_f, gyr_lp_f = packed_params
    if (debug_f):
      print("handle_imu_cmd:",err_code, accDict, gyrDict, 
                              acc_hipf_f, gyr_lp_f)
    if (err_code == 0):                                     # if no error, then 
      res = init_imu(accDict, gyrDict,              #   re-init the imu
                     acc_hipf_f, acc_ulp_f, gyr_lp_f)   
      err_f, accList, gyrList, tempList = res               #   unpack return
      # 
      # the only error possible here would be data never ready
      # either the mag hardware failed or the parameters cause failure
      #
      if (not err_f):                                    # if no error then 
        any_err_f, xlda_f, gda_f, tda_f = is_imu_rdy()   #   check data available
        print("is_imu_rdy(): (err_f, xlda_f, gda_f, tda_f):", any_err_f, 
                                                              xlda_f, 
                                                              gda_f, 
                                                              tda_f)
        set_imu_params(accDict, gyrDict,                 # write params to globals
                       acc_hipf_f, acc_ulp_f, gyr_lp_f)       
      else:
        err_code = -30
        print("Error: handle_imu_cmd(): init_imu(%s,%s,%s,%s,%s)"%(accDict["name"],
                                                                   gyrDict["name"],
                                                                   str(acc_hipf_f),
                                                                   str(acc_ulp_f),
                                                                   str(gyr_lp_f)))
      # end else error
    # endif not error
  elif (vector == "?"):

    accDict, gyrDict, accperf_b, acculp_b, gyrpwr_b =  get_imu_params()

    accperf_str = "0"                   # acc high performance
    if (accperf_b):
      accperf_str = "1"
    # endif convert bool to string

    acculp_str = "0"                   # acc ultra low power
    if (acculp_b):
      acculp_str = "1"
    # endif convert bool to string

    gyrpwr_str = "0"                   # gyro low power
    if (gyrpwr_b):
      gyrpwr_str = "1"
    # endif convert bool to string

    #                                                  # build response
    queryStr1 = accDict["name"] + "," + gyrDict["name"] + "," 
    queryStr2 = queryStr1 + accperf_str + "," + acculp_str + "," + gyrpwr_str

  else:
     err_code = -30  # 
  # end else error

  return err_code,tlc,queryStr2

# end handle_imu_cmd
#
# ----------------------------------
#
def maxList2str(maxList,maxLen=10):

  err_f = False  # an error not likely on self-created list
                 # as is the default settings but this is
                 # meant for bit-setting requests
  maxStr = ""
  ii = 0
  theLen = len(maxList)
  if (theLen > maxLen):
    err_f = True
    print("Error: maxList2str() list too long, len=%d, nax=%d"%(theLen, maxLen))
  # end if

  while ((ii < theLen) and (not err_f)):

    theItem = maxList[ii]
    #
    # filter what is allowed
    #
    if ((theItem == 1) or (theItem == '1')):
      strItem = '1'
    elif ((theItem == 0) or (theItem == '0')):
      strItem = '0'
    elif (theItem.upper() == 'X'):
      strItem = 'x'
    else:
      err_f = True
      print("Error: maxList2str() invalid item:",theItem)
    # end else error
     
    if (not err_f):
      maxStr = maxStr + strItem  # build the return string
      
      if ((ii + 1) < theLen):         # if more then
        maxStr = maxStr + ","  #   add a comma
      # endif more to to
    # endif not error
    #
    ii = ii + 1

  # end while
  
  return err_f,maxStr

# end maxList2str
#
# ----------------------------------
#
def updateShadowMax(offset,srcList,dstList):

  ii = 0
  while (ii < len(srcList)):

     dstList[offset +ii] = int(srcList[ii])

     ii = ii + 1

  # end while

# end updateShadowMax
#
# ----------------------------------
#
def init_max(debug_f = False):
  global g_shdw_max_misc   # not daisy chained
  global g_shdw_max_tx1    # daisy chained 2 dev
  global g_shdw_max_tx2    # tx2 tested for daisy-chain but not used
  global g_shdw_max_rx1    # daisy chained 4 dev
  global g_shdw_max_rx2    # rx2 tested for daisy, but not used
  #                        # rx3 and rx4 not used because of copy flag

  global g_spi_ctl         # the not daisy-chained MAX      
  global cs_max_misc       # the chip select for the aforementioned device

  global g_spi_max_rxtx    # the daisy-chained MAX 
  global cs_max_tx         # the tx chip select for the aforementioned device
  global cs_max_rx         # the rx chip select for the aforementioned device
  
  err_f   = False
  any_err_f = False        # keep going on error
  enable_wr_f = True       # disable for testing daisy chain
  #
  # ---------------------------------
  # misc MAX is not daisy chained 
  # ---------------------------------
  #
  # an error not likely on self-created list
  #
  err_f, miscStr = maxList2str(g_shdw_max_misc)
  if (err_f):
    any_err_f = True
  err_f, byteList = maxStr2SpiList(miscStr,addr_ofs=0,debug_f=debug_f)
  if (not err_f):
    if (enable_wr_f):
      writeMax(g_spi_ctl,cs_max_misc,byteList)
    else:
      print("Warning writeMax() disabled")
  else:
    print("init_max error: 1")                  # uniquify the error
    any_err_f = True # keep going on error
  # end else
  # ------------------
  # tx daisy chained 2
  #-------------------
  err_f, tx1Str = maxList2str(g_shdw_max_tx1)
  if (err_f):
    any_err_f = True  # keep going
  # endif 
  err_f, byteList = maxStr2SpiList(tx1Str, addr_ofs =0,daisy=2,dai_sel=1,copy_f=True,debug_f=debug_f)
  if (not err_f):
    if (enable_wr_f):
      writeMax(g_spi_max_rxtx, cs_max_tx,byteList)
      #
      # note: intended for use with copy_f set True
      #       only 1 list needs to be applied
      #       the 2nd list is for testing
    else:
      print("Warning writeMax() disabled")
  else:
    print("init_max error: 2")                 # uniquify the error
    any_err_f = err_f # keep going on error
  # end else error
  # ------------------
  # rx daisy chained 4
  # ------------------
  err_f, rx1Str = maxList2str(g_shdw_max_rx1)
  if (err_f):
    any_err_f = True  # keep going
  # endif 

  err_f, byteList = maxStr2SpiList(rx1Str, addr_ofs =0,daisy=4,dai_sel=1,copy_f=True,debug_f=debug_f)
  
  if (not err_f ):
    if (enable_wr_f):
      writeMax(g_spi_max_rxtx,cs_max_rx,byteList)
      #
      # note: intended for use with copy_f set True
      #       only 1 list needs to be applied
      #       the 2nd list is for testing
    else:
      print("Warning writeMax() disabled")
  else:
    print("init_max error: 3")                 # uniquify the error
    any_err_f = err_f # keep going on error

  return any_err_f

# end init_max
#
# ----------------------------------
#
def handle_max_cmd(theCmd,vector):
  """
                     |-----------------------> index to start bits
                     |
    PMITM -> PMITMAX,0,0,1,0,1,0,1,0,1,0,1*  -> set 10 bits, starting at index = 0

                     9,1*                    -> set 10th bit to 1

                     0,x,x,x,x,x,x,x,x,x,1*  -> set 10 bits, starting at index = 0
                       |
                       |----------------------> 'x' or 'X' is don't care
                                                set 10th bit to 1
          -> PMITMA?


    PMITX -> PMITXT
                  -> PMITXn             -> 'n' ::= 1|2
                          -> PMITXn?
          -> PMITXR
                  -> PMIRXn             -> 'n' ::= 1|2|3|4
                          -> PMIRXn?

  """
  global g_spi_ctl     # spi control for singular MAX
  global cs_max_misc   # cs for same

  global g_spi_max_rxtx # spi control for daisy-chained MAXs
  global cs_max_rx      # cs for tx chain
  global cs_max_tx      # cs for rx chain

  global g_spi_max_rxtx
  global g_shdw_max_misc
  global g_shdw_max_tx1
  global g_shdw_max_tx2
  global g_shdw_max_rx1
  global g_shdw_max_rx2
  global g_shdw_max_rx3
  global g_shdw_max_rx4

  err_code = 0
  debug_f = True             # debug_f provides path feedback
  queryStr = ""               # build-a-string intermediate 
  tlc = ""                    # tlc = 3 letter return code
                               
  err_f,pktout = add_cksum(theCmd)  # returns error if bad checksum
  if (err_f):
    err_code = -10 
    return err_code,tlc,queryStr
  # endif
  # -------------
  # still here?
  # -------------
  if (vector == "MAX,"):    # 4 chars sent, starting w/ "M" or "X"
    tlc = "MAX" 
    if (debug_f):
      print("MAX") 
    # endif debug
    # ------------------------------
    # acquire and verify parameters
    # ------------------------------
    bb = theCmd.split('*')        # remove '*' to end
    paramList = bb[0].split(",")  # isolate first comma
    if (debug_f):
      print("paramList=", paramList)
    # endif

    #                          #     0      1      2 ...      
    if (len(paramList) < 3):   # <comand>,<index>,<bit>[,<bit> ...]
      err_code = -1
      err_f = True
      print("too few parameters")
    # endif
    if (not err_f):
      try:
        addr_ofs = int(paramList[1])
      except Exception as eobj:
        err_code = -2
        err_f = True
        print("addr_ofs not an integer:",paramList[1])
      else:
        if ((addr_ofs < 0)  or (addr_ofs > 9)):
          err_code = -3
          err_f = True
          print("addr_ofs is out of range 0..9:",index)
        # endif 
      # end else
    # end if not error
    # -------------------------------------------------
    # convert params to form suitable for MAX and apply
    # -------------------------------------------------
    # 
    if (not err_f):
       err_f,bitStr = maxList2str(paramList[2:])  # skip offset, convert to string
    # endif not error
    if (not err_f) :                                         # convert to byte list
      err_f, byteList = maxStr2SpiList(bitStr,
                                       addr_ofs=addr_ofs,
                                       debug_f=debug_f)     
    # endif not error
    if (not err_f):
      updateShadowMax(addr_ofs,paramList[2:],g_shdw_max_misc)    # update shadow
      writeMax(g_spi_ctl,cs_max_misc,byteList)                   # write to MAX
    # endif not error
    #   
    # ------------------
    # clause complete
    # ------------------
    #
    queryStr = ','.join(paramList[1:])   # echo command params in response
    #                 
  elif (vector == "MA?*"):   # misc query
    if (debug_f):
      print("MA?") 
    # endif debug

    tlc = "MA?"
    err_f, queryStr = maxList2str(g_shdw_max_misc)  # err not likely
    #   
    # ------------------
    # clause complete
    # ------------------
    # 
  elif ((vector == "XT1?") or (vector == "XT2?")):              # TX query
    if (debug_f):
      print("XTn?") 
    # endif debug
    if (vector == "XT1?"):
      tlc = "XT1"
      err_f,queryStr = maxList2str(g_shdw_max_tx1) # err not likely
    else:
      tlc = "XT2"
      err_f, queryStr = maxList2str(g_shdw_max_tx2)
    # end else
    #
  elif ((vector == "XT1,") or (vector == "XT2,")):           # TX set
    if (debug_f):
      print("XTn") 
    # endif debug
    if (vector == "XT1,"):   # XT1
       tlc = "XT1"
       txKey = 1
    else:                    # XT2 
       tlc = "XT2"
       txKey = 2
    # end else
    # ------------------------------
    # acquire and verify parameters
    # ------------------------------
    bb = theCmd.split('*') # remove '*' to end

    paramList = bb[0].split(",")  # isolate first comma
    if (debug_f):
      print("paramList=", paramList)
    # endif

    #                          #     0      1      2 ...      
    if (len(paramList) < 3):   # <comand>,<index>,<bit>[,<bit> ...]
      err_code = -1
      err_f = True
      print("too few parameters")
    # endif
    if (not err_f):
      try:
        addr_ofs = int(paramList[1])
      except Exception as eobj:
        err_code = -2
        err_f = True
        print("addr_ofs not an integer:",paramList[1])
      else:
        if ((addr_ofs < 0)  or (addr_ofs > 9)):
          err_code = -3
          err_f = True
          print("addr_ofs is out of range 0..9:",index)
        # endif 
      # end else
    # end if not error
    # -------------------------------------------------
    # convert params to form suitable for MAX and apply
    # -------------------------------------------------
    # 
    if (not err_f):
       err_f,bitStr = maxList2str(paramList[2:])  # skip offset, convert to string
    # endif not error
    if (not err_f) :
      err_f, byteList = maxStr2SpiList(bitStr,addr_ofs=addr_ofs, # convert to byte list
                                       daisy=2,dai_sel=txKey,debug_f= False) 
    # endif not error
    if (not err_f):
      theMaxList = g_shdw_max_tx2
      if (txKey==1):
        theMaxList = g_shdw_max_tx1
      # endif key test
        
      updateShadowMax(addr_ofs,paramList[2:],theMaxList)    # update shadow
      writeMax(g_spi_max_rxtx,cs_max_tx,byteList)               # write to MAX
    # endif not error
    #   
    # ------------------
    # clause complete
    # ------------------
    #
    queryStr = ','.join(paramList[1:])   # echo command params in response
    #
  elif ((vector=="XR1?")or(vector=="XR2?")or(vector=="XR3?")or(vector=="XR4?")):
     print("XRn?") 
     if (vector == "XR1?"):                                  # RX query
       tlc = "XR1"
       err_f, queryStr = maxList2str(g_shdw_max_rx1) 
     elif (vector == "XR2?"):
       tlc = "XR2"
       rr_f, queryStr = maxList2str(g_shdw_max_rx2)
     elif (vector == "XR3?"):
       tlc = "XR3"
       rr_f, queryStr = maxList2str(g_shdw_max_rx3)
     else:
       tlc = "XR4"
       rr_f, queryStr = maxList2str(g_shdw_max_rx4)
     # end else
     #
  elif ((vector=="XR1,")or(vector=="XR2,")or(vector=="XR3,")or(vector=="XR4,")): 
    if (debug_f):
       print("XRn") 
    # endif debug                                        # RX set
    if (vector == "XR1,"):                # XR1
       tlc = "RX1"
       rxKey = 1
       theMaxList = g_shdw_max_rx1
    elif (vector == "XR2,"):              # XR2 
       tlc = "XR2"
       rxKey = 2
       theMaxList = g_shdw_max_rx2
    elif (vector == "XR3,"):              # XR3
       tlc = "XR3"
       rxKey = 3
       theMaxList = g_shdw_max_rx3
    else:                                 # XR4
       tlc = "XR4"
       rxKey = 4
       theMaxList = g_shdw_max_rx4
    # end else
    # 
    # ------------------------------
    # acquire and verify parameters
    # ------------------------------
    bb = theCmd.split('*') # remove '*' to end

    paramList = bb[0].split(",")  # isolate first comma
    if (debug_f):
      print("paramList=", paramList)
    # endif

    #                          #     0      1      2 ...      
    if (len(paramList) < 3):   # <comand>,<index>,<bit>[,<bit> ...]
      err_code = -1
      err_f = True
      print("too few parameters")
    # endif
    if (not err_f):
      try:
        addr_ofs = int(paramList[1])
      except Exception as eobj:
        err_code = -2
        err_f = True
        print("addr_ofs not an integer:",paramList[1])
      else:
        if ((addr_ofs < 0)  or (addr_ofs > 9)):
          err_code = -3
          err_f = True
          print("addr_ofs is out of range 0..9:",index)
        # endif 
      # end else
    # end if not error
    # -------------------------------------------------
    # convert params to form suitable for MAX and apply
    # -------------------------------------------------
    # 
    if (not err_f):
       err_f,bitStr = maxList2str(paramList[2:])  # skip offset, convert to string
    # endif not error
    if (not err_f) :
      err_f, byteList = maxStr2SpiList(bitStr,addr_ofs=addr_ofs, # convert to byte list
                                       daisy=4,
                                       dai_sel=rxKey) 
    # endif not error
    if (not err_f):
      updateShadowMax(addr_ofs,paramList[2:],theMaxList)    # update shadow
      writeMax(g_spi_max_rxtx,cs_max_rx,byteList)                   # write to MAX
    # endif not error
    #   
    # ------------------
    # clause complete
    # ------------------ 
    #
    queryStr = ','.join(paramList[1:])   # echo command params in response
    #        
  else:
     print("MAX undecoded:",vector) 
     tlc = vector
     err_code = -30 
  # end else

  return err_code,tlc,queryStr

# end handle_max_cmd
#
# ----------------------------------
#
# note: remember these NMEA format rules:
#         1. initial "$" 
#         2. the comma after the command and before the first parameter
#         3. the terminating "*" after the last parameter
#
def send_NMEA_ok(cmd_code,extra="0"):
  global uart0
  global g_uart_delay

  debug_f = False

  err_f = False    # the error is extremely unlikely
  nmea_resp = ""
  print("CMD CODE: ", cmd_code)
 
  #
  # the tail end of the response is either 0
  # or a string of query response parameters
  #
  # Q: why the tailing '0' if not a query?
  # A: its easier to do this than to remove an uneeded comma
  #    or always remember to pass in the comma
  #    of course, the adding the comma could be done in
  #    this routine, oh well, does it really matter?
  #
  pre_ck_ok = "$PMITSR," + "0,%s,%s*"%(cmd_code,extra)

  if (debug_f):
    print("send_NMEA_ok(): pre_ck_ok=", pre_ck_ok)
  # endif 

  err_f, nmea_resp =  add_cksum(pre_ck_ok,debug_f=debug_f) 

  if (debug_f):
    print("nmea_resp=", nmea_resp)
  # endif 

  uart0.write(nmea_resp + "\r\n")

# end send_NMEA_ok()
#
# ----------------------------------
#
# note: remember these NMEA format rules:
#         1. initial "$" 
#         2. the comma after the command and before the first parameter
#         3. the terminating "*" after the last parameter
#
def send_NMEA_err(cmd_code,err_code,wait_f=False):
  global uart0
  global g_uart_delay

  debug_f = False

  if (wait_f):
    time.sleep(g_uart_delay)
  # endif 

  err_f = False    # the error is extremely unlikely
  nmea_resp = ""
  
  pre_ck_err = "$PMITSR," + "-1,%s,%d*"%(cmd_code,err_code)

  if (debug_f):
    print("send_NMEA_err(): pre_ck_err=", pre_ck_err)
  # endif 

  err_f, nmea_resp =  add_cksum(pre_ck_err,debug_f=debug_f)  

  if (debug_f):
    print("nmea_resp=", nmea_resp)
  # endif 

  uart0.write(nmea_resp + "\r\n")

# end send_NMEA_err
#
# -----------------------------------------------------
#

def send_telemetry_log():
  
  switch_temp = get_sw_tp()
  print(switch_temp)
  uart0.write("$START")
  uart0.write(switch_temp)
  uart0.write()
  uart0.write()
  uart0.write("$STOP\r\n")

def do_nmea_command(nmea_string):

  if True:
      print("READ NMEA As: ", nmea_string)
      uart0.write(nmea_string + "\r\n")
      pass


def do_console(count,              # outer loop count
               tilda_f = False,    # switch from low overhead poll to full control
               instru_f=False,     # debug timing
               debug_f=False):     # debug characters

  global USE_MINICOM
  global uart0               # rp uart at board edge
  global g_uart_delay        # uart delay for slow device 
  echo_f            = True   # echo mode
  lineStr = ""
  eol_f = False

  promptStr = " gnss> "
  crlf = "\r\n"
  crlfPrompt = crlf + promptStr

  forever_f = False
  if (count == -1):
    forever_f = True
  # endif loop forever test
  #                                 # ---------------------------------
  interact_f = get_human()          #  if human then interact
  if (interact_f):                  #    if interactive then
    tilda_f   = True                #      use tilda to get attention
  # endif                           # ---------------------------------

  bq_f = False  # back quote flag

  #uart0.reset_input_buffer()  # belts and suspenders

  ii = 0                      # character counter
  esc_ctr = 0                 # escape counter
  esc_f = False               # escape character flag
  escStr = ""                 # escape string
  # -------------------------------------
  # count checked at bottom no data loop
  # -------------------------------------
  while(True):          # until count==0 or "quit" 
    inChar = ""         # byte converted to character 
    valid_f = False     # valid character flag
    eof_f = False       # end of line flag
    bs_f = False        # backspace character flag
    err_f = False
    #
    # read wait/tmo is set in serial port initialzation
    #
    raw_data = uart0.read(1)  # polled read
    if (raw_data == None):                # if timeout then 
      if (not forever_f):                 #   check for forever
        count = count -1
        if (count <= 0):     # --------------
          break              # exit the loop
        # endif              # --------------
      # endif counting down
      #     
      continue                           #   go to top, try again
      #
    else:                                # else
      if (debug_f):
        print("[%d]char:"%(ii),raw_data) #   debug 
      # endif 
      ii = ii + 1                        #   count character
      inOrd = ord(raw_data)
      #
      # ------------------------------------------------------
      #  grab attention clause
      #    if tilda '~' or back-single-quote "`" then 
      #      a human, provide a prompt and echo input
      #    elif back-single-quote prompt then 
      #      an interactive program run by a human
      #      don't provide a prompt, don't echo 
      # ------------------------------------------------------
      #
      if (tilda_f and ((inOrd == 126) or (inOrd == 96)) ):  # 126 = '~' 96 = '`'
         if (debug_f):
           uart0.write("you have my attention\n")
         # endif 
         forever_f = True               # stay in loop until quit
         interact_f = False             # provide user interaction or not
         if (inOrd == 126):
           interact_f = True
           uart0.write(crlfPrompt)
         elif (inOrd == 96):           # if backquote then interacting w/afecmds.py
           bq_f = True                 # and want to quiet background activities
         # endif
         set_human(interact_f)
         continue                       # back to top
         #  
      elif ((inOrd == 10) or (inOrd == 13)):   # crlf indicates end of line
          eol_f = True
      elif ((inOrd >= 32) and (inOrd <= 127)): # valid printing character
          valid_f = True
      elif ((inOrd  == 0x08) and interact_f):   # if backspace then
          bs_f = True
          if (ii > 0):
            ii = ii - 1                        #   decrement char counter
          # endif
      elif ((inOrd == 0x1b) and interact_f):
        esc_f = True           
      else:                            # else not expected character
         continue                      #   pretend this never happened
      # end else non printing

      if (esc_f):
        esc_ctr = esc_ctr + 1
        if (esc_ctr == 1):
           pass
        else:
          escStr = escStr + chr(inOrd)
          # ------------------------------ 
          # escape processing
          #      0    1    2    3
          # b'\x1b',b'[',b'3',b'~'
          #
          # ------------------------------
          if (esc_ctr == 4):
            if (debug_f):
              print("escStr=", escStr)
            # endif 
            #                           # reset the escape state machine
            if (escStr == "[3~"):     # delete on keyboard
              ii,lineStr = backup_char(uart0,promptStr,lineStr)  
            # endif delete detected 
            #                           # reset escape state machine
            esc_ctr = 0                 # reset escape counter
            esc_f = False               # reset escape character flag
            escStr = ""                 # reset escape string
          # endif entire escape string captured
        # end else ignore first escape char

        continue  # keep going

      # endif escape
            
      if (valid_f):
        inChar = chr(inOrd)
        if (interact_f):
          uart0.write(inChar)   # user feedback
      # endif

      if (bs_f and interact_f):           # 
         ii,lineStr = backup_char(uart0,promptStr,lineStr)  
      elif (not eol_f):
        lineStr = lineStr + inChar                              
      else:                    # else end of line

        if (interact_f):
          uart0.write(crlf)      # print crlf
        # endif

        eol_f = False          # reset end of line flag
        ii = 0                 # reset character counter
        # --------------------------
        # line processing goes here
        # --------------------------
        if (debug_f):
          print("lineStr='%s'"%(lineStr))
        # endif
        lineStr = lineStr.strip()
        if ((lineStr == "quit") or (lineStr == "exit")):     # ---------------------
           print("quit detected")                            #   exit this function  
           break                                             # ---------------------
        # endif exit this function
        elif ((len(lineStr)==0) or (lineStr[0] == '#')):  # if comment
           pass
        elif (lineStr[0:5] == "eval("):                     # if backdoor
          print("rcvd:",lineStr)
          if ( (lineStr[5] != '"') and (lineStr[5] != "'")):
             err_f = True
          else:
            if ( (lineStr[-2] != '"') and (lineStr[-2] != "'")):
              err_f = True
            # endif error check
          # endif error check
          if (err_f):
            # -----------------------------------------------
            # as a .mpy, this causes and unhandled exception
            # in code.py
            # -----------------------------------------------
            emsg="badly formed eval"
            print(emsg)
            uart0.write(emsg + crlf)
          else:
            try:
              resp = eval(lineStr)
            except Exception as eobj:
              out_str = "Exception:" + repr(eobj) + "\r\n"
              uart0.write(out_str)
            else:
               if (resp==str):
                  if (len(resp) == 0):
                    resp = "''"         # easier to recognize an empty string this way
                  # endif 
               else:
                  resp = str(resp)
               # end else not a string
               out_str = resp + "\r\n"  
               # ------------------------------------------
               # how quickly can host turnaround to receive?
               # --------------------------------------------
               #  
               if (USE_MINICOM or bq_f):                    
                 time.sleep(0.8 + g_uart_delay)  # value picked trial-and-error
               #
               uart0.write(out_str)
               print("sent:", out_str)
            # end else not an exception
          # end else not an eval quotes error
          #                                  # ----------------------
          #                                  # NMEA command vectors
          #                                  # -----------------------
        elif (lineStr[0:6] == "$PMITT"):     
          err_code,cmd_code,extra = handle_time_cmd(lineStr)  # set time         
          if (err_code != 0):
            send_NMEA_err(cmd_code,err_code)       # send error message
          else:
            send_NMEA_ok(cmd_code,extra)           # send success message
          # end else success

        elif (lineStr[0:6] == "$PMITR"):     
          err_code, cmd_code, extra = handle_rate_cmd(lineStr)   # set telemetry rate 
          if (err_code != 0):
            send_NMEA_err(cmd_code,err_code) # send error message
          else:
            send_NMEA_ok(cmd_code,extra)  # send success message
          # end else success
        #                     #0123456
        elif (lineStr[0:7] == "$PMITMG"):     # set or query magnetometer parameters 
          #
          # expected 2nd param is "S" or "?"
          #
          err_code,cmd_code,qresp = handle_mag_cmd(lineStr,lineStr[7])
          if (err_code == 0):
            send_NMEA_ok(cmd_code,qresp)
          else:
            send_NMEA_err(cmd_code,err_code)    
          # end else handle error
        #                     #0123456
        elif (lineStr[0:7] == "$PMITIM"):     # set or query imu parameters 
          #
          # expected 2nd param is "U" or "?"
          #
          err_code,cmd_code,qresp = handle_imu_cmd(lineStr,lineStr[7])
          if (err_code == 0):
            send_NMEA_ok(cmd_code,qresp)
          else:
            send_NMEA_err(cmd_code,err_code)    
          # end else handle error
          #                  #012345
        elif ((lineStr[0:6] == "$PMITM") or (lineStr[0:6] == "$PMITX")): # 'M' or 'X" for MAX, XTn, XXn
          #
          # expected 2nd param is "A" or "T" or "R" followed by n=1..2|4 followed by optional '?'
          #
          err_code,cmd_code,qresp = handle_max_cmd(lineStr,lineStr[5:9])  # 'AX[?]', 'Tn[?]' 'Rn[?]'
          if (err_code == 0):
            send_NMEA_ok(cmd_code,qresp)
          else:
            send_NMEA_err(cmd_code,err_code)    
          # end else handle error
        else:
          print("undecoded rx:",lineStr)  # for the human, say unhandled
          badCode = lineStr.split(",")[0] 
          if (badCode[0:5] =="$PMIT"):    # for the machine, provide an error code
            tlc = badCode[5:8]
            send_NMEA_err(tlc,-99)  
          # endif   
          #
          # ignore all others
          #
        # end else unhandled
        #
        # single threaded, this program will switch context to accept
        # console commands while forwarding NMEA, and accept interactive
        # commands to change the internal state of this routine
        # for example, turning off the console prompt
        #
        interact_f = get_human()
        if (interact_f):
          forever_f = True
          tilda_f = True
          uart0.write(crlfPrompt)
        # endif interact
        lineStr = ""
      # end else end of line 
    # end else character to process

  # end while forever

  if (interact_f and debug_f):
    print("echo mode complete")
  # endif
# end do_console
#
# -----------------------------------------------------
#

def run_mode(mode,           # the vector
             count,          # outer loop count
             instru_f=False, # enable instrumentation
             extra_f =False, # extra flag
             debug_f=False): # used to print once
  global telemetry_due
  global telemetry_timer
  global USE_FIFO
  global USE_MINICOM
  ret1 = None
  ret2 = None
  ret3 = None
  cnt_iter = 0

  nmea_string = ""
                      # -------------------------------------------
  minimal_f = True    # if true, then print top of loop timestamp
                      # if false, then be very quiet in this loop
                      #
                      # note: at one time in the past, False
                      #       caused a heisenbug that caused
                      #       do_console() to take a longer time
                      #       to timeout if no data on GNSS1
                      # -------------------------------------------

                     # --------------------------------
  mode_f = False     # prints mode for mode recursions   
                     # --------------------------------

  if (instru_f or minimal_f):
    USE_MINICOM = False
    start_time=supervisor.ticks_ms()
  # endif instrumented
                            # 0 = gyro, acc, to fifo
  if (mode == 0):

    print("reading imu fifo data:")
    accList, gyrList, timList = process_fifo(count=128,tim_rq_f=False)  
    if (debug_f):
      print("accList=", accList)  
      print("gyrList=", gyrList)  
      print("timList=", timList)  
    # endif  

    #
    # fifo read returns list of 3 element lists
    #

    #
    # intermediate steps testing
    #
    #read_imu(xlda_f=True, gda_f=False, tda_f=False, dataList= accList[0],debug_f=debug_f)
    #read_imu(xlda_f=False, gda_f=True, tda_f=False, dataList= gyrList[0],debug_f=debug_f)


    #loop_imu(64,xlda_in=True,gda_in=False,tda_in=False,  
    #         use_cal=False,dataList_in=accList,debug_f=debug_f)

    #loop_imu(64,xlda_in=False,gda_in=True,tda_in=False,  
    #         use_cal=False,dataList_in=gyrList,debug_f=debug_f)

    calc_tare(cnt=128,xlda_f=True,gda_f=False,dataList_in=accList,debug_f=debug_f) 

    calc_tare(cnt=64,xlda_f=False,gda_f=True,dataList_in=gyrList,debug_f=debug_f) 

    ii = 0
    while (ii < count):
      
      accList, gyrList, timList = process_fifo(count=16,tim_rq_f=False)

      err_f, detects, retList = zero_crossings(cnt=16,xlda_f=True,
                                               auto_zero=False,dataList_in=accList) 
      print("acc: err_f,detects=",err_f,detects) 
   
      err_f, detects, retList = zero_crossings(cnt=16,gda_f=True,
                                               auto_zero=False,dataList_in=gyrList)
      print("gyro: err_f,detects=",err_f,detects)  
      
      ii = ii + 11
      
    # end while
  # end run 0 

  elif (mode == 1):          # magnetometer

    read_mag(print_f=debug_f)

  elif (mode == 2):          # temperatures, loops for count

     if (debug_f):
        print("reading temperatures")
     # endif debug

     ii = 0

     while (ii < count):

       get_all_tps(debug_f)
       ii = ii + 1

     # end while
 
     print("\nprogram done. ^C ^D to restart")

     while(True):    # prevent falling off then end of the program
        pass
     # end while
   
  elif (mode == 3):          # acc tare, for testing of tare sample size

    err_f, detects, retList = do_acc(tare_f=True,
                                     tare_cnt=count,  # using default datacnt
                                     loop_cnt=1,
                                     instru_f=True)
    if ((not err_f) and (debug_f)):
      print("retList=")
      list_columns(retList,2)

  elif (mode == 4):          # gyro tare, , for testing of tare sample size

    err_f, detects, retList = do_gyro(tare_f=True,
                                      tare_cnt=count,   # using default datacnt
                                      loop_cnt=1,
                                      instru_f=True)
    if ((not err_f) and (debug_f)):
      print("retList=")
      list_columns(retList,2)

  elif (mode == 5):         # acc tare, then acc data for loop count
      
     do_acc(tare_f=True,tare_cnt=128,data_cnt=16,loop_cnt=1,instru_f=True)

     err_f, detects, retList = do_acc(data_cnt=16,loop_cnt=count,instru_f=debug_f)

     ret1 = err_f    # marshal return values
     ret2 = detects
     ret3 = retList

  elif (mode == 6):          # gyro tare, then gyro data for loop count

      do_gyro(tare_f=True,tare_cnt=64,data_cnt=16,loop_cnt=1,instru_f=True)

      do_gyro(data_cnt=16,loop_cnt=count,instru_f=debug_f)

  elif (mode == 7):          # tare, then acc and gyro

    print("acc and gyro tare followed by listen forever ...")

    do_acc(tare_f=True,tare_cnt=128,data_cnt=16,loop_cnt=1,instru_f=True)

    do_gyro(tare_f=True,tare_cnt=64,data_cnt=16,loop_cnt=1,instru_f=True)

    print("acc and gyro tare complete, entering listen loop...")
    #
    # note: detects are indices into the return list 
    #       indicating above tare threshold
    #
    # note: this is tested on a flimsy folding table in my office
    #       I have no idea if zero crossings are really noise
    #       or the IMU is very, very sensitive
    #
    while(True):

      err_f, detects, retList = do_acc(data_cnt=16,loop_cnt=1,debug_f=False)
      if (len(detects) > 0):
        print("[%d] acc="%(time.time()), detects)

      err_f, detects, retList = do_gyro(data_cnt=16,loop_cnt=1,debug_f=False)
      if (len(detects) > 0):
        print("[%d] gyro="%(time.time()), detects)

    # end loop
  elif (mode == 8):      # tilt check
     
      if (debug_f):
        print("entering tilt check...")
      # endif 
      err_f, tilt_val = is_tilt(cnt=count,debug_f=debug_f)

      if (debug_f):
        if (not err_f):
          print("tilt_val=", tilt_val)
        # endif
      # endif

      ret1 = err_f

  elif (mode == 9):     # activity check

      if (debug_f):
        print("entering activity check...")
      # endif

      err_f, active_val= is_active(cnt=count,debug_f=debug_f)

      if (debug_f):
        if (not err_f):
          print("active_val =", active_val)
        # endif
      # endif

      ret1 = err_f

  elif (mode == 10):        # nmea forwarding 
    ii = 0                  # iterator
    while (True):           # forever

      msg_cnt = 0
      tmoSec = 8     # was 8
      maxMsgCnt = 8  # was 8

      if (ii == 0):
        if (instru_f):
          pass # print("forward_nmea: tmo %d sec or %d msgs"%(tmoSec,maxMsgCnt))
        # endif instru
      # endif first time in

      if (instru_f):
        enter_time = supervisor.ticks_ms()
      # endif
      #                      # ---------------------------------------
      debug_f = mode_f       # recursion warning, called from mode=12
      #                      # ---------------------------------------
      #
      err_f,msg_cnt,err_cnt = forward_nmea(tmoSec=tmoSec,           
                                maxMsgCnt= maxMsgCnt,  
                                debug_f=debug_f)           # user feedback

      if (instru_f):
        if ((not err_f) and (msg_cnt > 0)):
          print("fwd delta_t=", supervisor.ticks_ms() - enter_time)
        else:
          if (debug_f):
            print(".",end="")  # no flush for circuitpython
            time.sleep(0.1)    # sleep works as flush
          # endif 
        # endif data present
      # endif instrumented
        
      ii = ii  + 1
      if (count == -1):  # forever 
         pass           
      else:              # else not forever    
        if (ii >= count):
          break
      # end else

    # end while

  # end elif

  elif (mode == 11):       # test input commanding

     do_console(count,tilda_f=True,debug_f=debug_f)

  # end elif
  #                         # ---------------
  elif (mode == 12):        # executive loop
    #                       # ----------------
    #
    # init acc and gyro for synthetic NMEA
    #
    run_mode(3,128,debug_f=False)    #  acc tare, debug = dump values
    run_mode(4,64,debug_f=False)     #  gyro tare, debug = dump values
             
    ii = 0                  # iterator
    while (True):           # forever

      if (instru_f or minimal_f):
         print("top of loop:",supervisor.ticks_ms())  # instrumentation of timing
      # endif instru

      # ----------------
      # forward nmea  --
      # ----------------
      run_mode(10,1,instru_f=instru_f,debug_f=mode_f)  # there's a reason for flag set 

      # ------------------
      # console check  --
      # ------------------
      do_console(5,tilda_f=True)    # check for user

      # ----------------
      # forward nmea  --
      # ----------------
      run_mode(10,1,instru_f=instru_f,debug_f=mode_f)  # there's a reason

      # ----------------
      # magnetometer  --
      # ----------------
      run_mode(1,1,debug_f=False)   # magnetometer
      if (instru_f):
         print("mag time:",supervisor.ticks_ms())  # instrumentation of timing
      # endif instru

      # ----------------------
      # temperatures  --
      # ----------------------
      get_all_tps(debug_f=False)   # temperatures
      if (instru_f):
         print("temp time:",supervisor.ticks_ms())  # instrumentation of timing
      # endif instru

      # ----------------
      # forward nmea  --
      # ----------------
      run_mode(10,1,instru_f=instru_f,debug_f=mode_f)   # there's a reason

      # ---------------
      # tilt check  --
      # ---------------
      run_mode(8,1,debug_f=False)   # is_tilt, do once
      if (instru_f):
         print("tilt time:",supervisor.ticks_ms())  # instrumentation of timing
      # endif instru

      # ----------------
      # forward nmea  --
      # ----------------
      run_mode(10,1,instru_f=instru_f,debug_f=mode_f)  # there's a reason

      # --------------------
      # imu active check  --
      # ---------------------
      run_mode(9,1,debug_f=False)   # is_active, do once  
      if (instru_f):
         print("active time:",supervisor.ticks_ms())  # instrumentation of timing
      # endif 

      # ----------------
      # forward nmea  --
      # ----------------
      run_mode(10,1,instru_f=instru_f,debug_f=mode_f)  # there's a reason
      
      # ------------
      #  imu acc  --
      # ------------
      do_acc(data_cnt=1,debug_f=debug_f)    # read acc, defaults, no tare 
      if (instru_f):
         print("acc time:",supervisor.ticks_ms())  # instrumentation of timing
      # endif instru

      # ----------------
      # forward nmea  --
      # ----------------
      run_mode(10,1,instru_f=instru_f,debug_f=mode_f)  # there's a reason

      # -------------
      #  imu gyro  --
      # -------------
      do_gyro(data_cnt=1,debug_f=debug_f)    # read gyro, defaults, no tare     
      if (instru_f):
        print("gyro time:",supervisor.ticks_ms())  # instrumentation of timing
      # endif

      # ----------------
      # forward nmea  --
      # ----------------
      run_mode(10,1,instru_f=instru_f,debug_f=mode_f)  # there's a reason
      
      # -----------------
      # send telemetry --
      # -----------------     
      if (instru_f):
        print("telem start ts=",supervisor.ticks_ms())
      # endif instru                     
      
      err_f= send_telem(debug_f=debug_f)  
      
      if (instru_f):
        print("telem end ts=",supervisor.ticks_ms())
      # endif instru
   
      # ----------------
      # forward nmea  --
      # ----------------
      run_mode(10,1,instru_f=instru_f,debug_f=mode_f)  # there's a reason

      ii = ii  + 1
      if (count == -1):  # forever 
         pass           
      else:              # else not forever    
        if (ii >= count):
          break
      # end else

    # end while

    ret1 = False
    ret2 = msg_cnt


  elif (mode == 13):                 # Operating mode for afe_service.py

    run_mode(3,128,debug_f=debug_f)    #  acc tare, debug = dump values
    run_mode(4,64,debug_f=debug_f)     #  gyro tare, debug = dump values

    run_mode(1,1,debug_f=False)
    run_mode(8,1,debug_f=False)
    run_mode(9,1,debug_f=False)

    do_acc(data_cnt=1,debug_f=debug_f)
    do_gyro(data_cnt=1,debug_f=debug_f)

    _,gps_start_draft = add_cksum("$PGPS*") 
    gps_start = gps_start_draft + "\r\n"

    _,gps_end_draft = add_cksum("$PGPN*") 
    gps_end = gps_end_draft + "\r\n"

    while True:

      ublox = uart1.read()
      if ublox:
        start_time = supervisor.ticks_ms()
        uart0.write(gps_start)
        uart0.write(ublox)
        uart0.write(gps_end)
        uart1.reset_input_buffer()

      if supervisor.ticks_ms() - start_time < 600: # Ensures instructions wont still be processing while
                                                   # new nmea is ready to keep TOFF constant
        line = uart0.readline()
        if line:
          try:
            lineStr = line.decode()
            print(lineStr)
          except UnicodeError:
            continue

          #   ----------------------
          #   NMEA command vectors
          #   -----------------------

          if (lineStr[0:7] == "$TELEM?"):

            run_mode(1,1,debug_f=False)
            run_mode(8,1,debug_f=False)
            run_mode(9,1,debug_f=False)
            do_acc(data_cnt=1,debug_f=False)
            do_gyro(data_cnt=1,debug_f=False)

            msg_draft = "$PTEL*"
            _,msg = add_cksum(msg_draft) 
            uart0.write(msg + "\r\n")

            send_telem(debug_f=debug_f)
            print("REQUESTED TELEM DUMP")

          if (lineStr[0:5] == "$MAX?"):

            regs = [
              g_shdw_max_misc,
              g_shdw_max_tx1,
              g_shdw_max_tx2,
              g_shdw_max_rx1,
              g_shdw_max_rx2,
              g_shdw_max_rx3,
              g_shdw_max_rx4,]

            bitArray = []

            for reg in regs:
              _, str = maxList2str(reg)
              bitArray.append(str.replace(",", ""))
            regStr = ",".join(item for item in bitArray)

            print("MAX REQUESTED")

            msg_draft = "$PMAX," + regStr + "*"
            _,msg = add_cksum(msg_draft) 
            uart0.write(msg + "\r\n")
            print("WRITING REG: ", msg)


          elif (lineStr[0:6] == "$PMITT"):     
            err_code,cmd_code,extra = handle_time_cmd(lineStr)  # set time         
            if (err_code != 0):
              send_NMEA_err(cmd_code,err_code)       # send error message
            else:
              send_NMEA_ok(cmd_code,extra)           # send success message
          #  # end else success
          elif (lineStr[0:6] == "$PMITR"):     
            err_code, cmd_code, extra = handle_rate_cmd(lineStr)   # set telemetry rate 
            if (err_code != 0):
              send_NMEA_err(cmd_code,err_code) # send error message
            else:
              send_NMEA_ok(cmd_code,extra)  # send success message
            # end else success
          #                     #0123456
          elif (lineStr[0:7] == "$PMITMG"):     # set or query magnetometer parameters 
            #
            # expected 2nd param is "S" or "?"
            #
            err_code,cmd_code,qresp = handle_mag_cmd(lineStr,lineStr[7])
            if (err_code == 0):
              send_NMEA_ok(cmd_code,qresp)
            else:
              send_NMEA_err(cmd_code,err_code)    
            # end else handle error
          #                     #0123456
          elif (lineStr[0:7] == "$PMITIM"):     # set or query imu parameters 
            #
            # expected 2nd param is "U" or "?"
            #
            err_code,cmd_code,qresp = handle_imu_cmd(lineStr,lineStr[7])
            if (err_code == 0):
              send_NMEA_ok(cmd_code,qresp)
            else:
              send_NMEA_err(cmd_code,err_code)    
            # end else handle error
            #                  #012345
          elif ((lineStr[0:6] == "$PMITM") or (lineStr[0:6] == "$PMITX")): # 'M' or 'X" for MAX, XTn, XXn
            #
            # expected 2nd param is "A" or "T" or "R" followed by n=1..2|4 followed by optional '?'
            #
            err_code,cmd_code,qresp = handle_max_cmd(lineStr,lineStr[5:9])  # 'AX[?]', 'Tn[?]' 'Rn[?]'
            if (err_code == 0):
              send_NMEA_ok(cmd_code,qresp)
            else:
              send_NMEA_err(cmd_code,err_code)    
            # end else handle error
          else:
            continue

  if (instru_f or mode_f):
    print("mode=",mode,"dt=",supervisor.ticks_ms()-start_time)
  # endif instrumented

  return ret1, ret2, ret3

# end run_mode
#
# ----------------------------------
#     main
# ----------------------------------

def my_main():
  global USE_FIFO
  global USE_IMU
  global USE_MAG
  global g_imu_f
  global g_mag_f


  # ------------------------
  # user warnings
  # ------------------------

  if (USE_IMU==False):
    print("\n*** Default: IMU DISABLED ***\n")
    g_imu_f = False
  # endif

  if (USE_MAG==False):
    print("\n*** Default: MAGNETOMETER DISABLED ***\n")
    g_mag_f = False
  # endif

  # ------------------------
  # initialization
  # ------------------------

  err_f, mag_err_f, imu_err_f = init_all(fifo_f=USE_FIFO)

  if (err_f):
    if (mag_err_f):
      USE_MAG = False
      print("*** Disabling Magnetometer ***\n")
      g_mag_f = False
    # endif

    if (imu_err_f):
      USE_IMU = False
      print("*** Disabling IMU ***\n")
      g_imu_f = False
    # endif

    #
    # if both IMU and MAG failed, then spi bus likely failed
    #
    if ( (not g_mag_f) and (not g_imu_f)):
       cleanup_spi()
    # endif

  # endif error check

  # ------------------------
  # one-off tests go below
  # ------------------------

  pass

  # -------------------------------
  # mode test  
  # -------------------------------

  """
    mode:
      0 = fifo test
      1 = magnetometer - loop count
      2 = temperatures - loop count
      3 = acc tare -  once, tare count, data count constant
      4 = gyro tare - once, tare count, data count constant
      5 = tare then acc - loop count, data count constant
      6 = tare then gyro - loop count, data count constant
      7 = tare, acc and gyro - loop count, data count constant
      8 = tilt check - loop count
      9 = activity check - loop count
      10 = forward nmea - once, count = type to synthesize
      11 = exec loop = loop count (-1 = forever)
  """

  # run_mode(0,4,debug_f=False)    # fifo testing

  # run_mode(1,1,debug_f=True)     # magnetometer once, debug

  # run_mode(2,10,debug_f=True)    # temperatures

  # run_mode(3,128,debug_f=False)  #  acc tare, debug = dump values

  # run_mode(4,64,debug_f=False)   #  gyro tare, debug = dump values

  # err_f, detects, retList = run_mode(5,2,debug_f=True) #  acc tare then loop

  # err_f, detects, retList = run_mode(6,2,debug_f=True)   #  gyro tare then loop

  # run_mode(7,1,debug_f=False)       # gyro and acc forever

  # run_mode(8,1,debug_f=False)     # tilt

  # run_mode(9,1,debug_f=False)     # active

  # run_mode(10,-1,instru_f=True,debug_f=False)    # forward nmea

  # run_mode(11,10,debug_f=False,instru_f=True)    # forward commands thru GNSS1

  # run_mode(12,-1,instru_f=False,debug_f=False)   # executive loop

  run_mode(13,-1,instru_f=False,debug_f=False)     # VLA experiment 2025

# end my_main


if __name__ == '__main__':


  my_main()


#
# ----------------------------------
#      end of file
# ----------------------------------
#