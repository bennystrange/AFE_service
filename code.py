#
# MIT Haystack Observatory
# bww 06-23-2025
#
#
import sys
import traceback
import time
import controller as afe
print("\n***code.py***")
while(True):
  try:
    afe.my_main()
  except Exception as eobj:
    print("Exception: ctl.my_main():",eobj)
    traceback.print_exception(eobj) 
    print("Soft reset")
    time.sleep(10)      # allow time for reading console
  # end try/except
# end while
# end code.py
