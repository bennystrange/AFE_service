#
# mit haystack observatory
# rps 2/6/2025
#
# done in the attempt to save memory space for execution
#
# $ python3 -m mpy_cross ~/eclipse/gps/staging/gnss_imu_mag.py
#
import sys
import traceback
import time
import controller as con
print("\n***code.py***")
while(True):
  try:
    con.my_main()
  except Exception as eobj:
    print("Exception: con.my_main():",eobj)
    traceback.print_exception(eobj) 
    print("Soft reset")
    time.sleep(10)      # allow time for reading console
  # end try/except
# end while
# end code.py
