# AFE Service

This repository contains instructions for configuring and running the AFE service on the MEP REV 1.00

![MEP PNG](MEP.png)

## Dependencies

### RP2040

- RP2040 must be flashed with [Circuit Python 9.2.8](https://circuitpython.org/board/raspberry_pi_pico/) via the RP2040 USB port on the AFE
- RP2040 must contain all the files in [CIRCUITPY](https://github.com/bennystrange/AFE_service/tree/main/CIRCUITPY)

### ublox ZED-F9O-00B

- ublox must be configured according to `/opt/mep-examples/scripts/ubxconfig.py` [here](https://github.com/spectrumx/mep-examples/blob/main/scripts/ubxconfig.py) via the GNSS USB port on the AFE
- **CFG-UART1-BAUDRATE must be 1350000**

### Nvidia Jetson

- `/opt/ansible/files/gpsd` must include:
   
  ```
    # Devices gpsd should collect to at boot time.
    # They need to be read/writeable, either by user gpsd or the group dialout.
    DEVICES="/dev/ttyGNSS1"

    # Other options you want to pass to gpsd
    GPSD_OPTIONS="-n -r -s 460800 -D 3 -F /var/run/gpsd.sock"

    # Automatically hot add/remove USB GPS devices via gpsdctl
    USBAUTO="false"
  ```
- `/opt/ansible/gpsd.yml` must include the following tasks **after** "Deploy gpsd default configuration" and **before** "Enable gpsd.service":

  ```
   - name: Add mepuser to gpsd group
     user:
       name: "mepuser"
       groups: "gpsd"
       append: true
        
   - name: Create systemd directory for gpsd.socket
     file:
       path: /etc/systemd/system/gpsd.socket.d
       state: directory
       mode: '0755'

   - name: Make gpsd.socket world-writable
     copy:
       dest: /etc/systemd/system/gpsd.socket.d/override.conf
       mode: '0644'
       content: |
         [Socket]
         SocketMode=0666
  
   - name: Install afe.py into /usr/local/bin
     copy:
       src: /opt/mep-examples/scripts/afe.py
       dest: /usr/local/bin/afe
       mode: '0755'
       owner: root
       group: root
     ```
  - gpsd must be initialized before afe_service.py
     
## Usage

From anywhere, run `afe`

```
Tools:
>> afe  [-h]  [-p]  [-l]  [-r <logging rate>]
  -h, --help         Show this help message and exit
  -p, --print        Print the current telemetry and register states
  -l, --log          Log the current telemetry and register states
  -r, --rate         Select period of telemetry logging (in seconds)
  
Shortcuts:
>> afe  [-a <0/1>]  [-i <channel> <0,1>]
  -h, --help         Show this help message and exit
  -a, --antenna      GNSS Antenna Select <0/1> (Internal/External)
  -i, --inputrf      RF Input select <channel> (1,2,3,4) <0/1/> (Internal/External)

Manual Reguster Programming:
>> afe  <block>  <addr>  <value>
>> afe  -rx2  9  1
  <block>
     -m, --main           Main register (trigs, ebias, pps sel, ref sel, antenna sel)
     -tx1, tx2            TX registers (blank sel, filter bypass)
     -rx1, rx2, rx2, rx4  RX registers (chan bias, rf trig, filter byp, amp byp, atten)

  <addr>
     Register address: 0-9

  <value>
     Register value to set: 0,1
```
