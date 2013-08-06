#!/usr/bin/python

########################################################################
# Description: AutoTune.py                                             #
#                                                                      #
#                                                                      #
#                                                                      #
#                                                                      #
#                                                                      #
#                                                                      #
# Author(s): Troy Jacobson                                             #
# License: GNU GPL Version 2.0 or (at your option) any later version.  #
#                                                                      #
# Major Changes:                                                       #
# 2013-July   Troy Jacobson                                            #
#             Initial version                                          #
########################################################################
# Copyright (C) 2013  Troy Jacoboson                                   #
#                     <troy AT thismuch DOT net>                       #
#                                                                      #
# This program is free software; you can redistribute it and/or        #
# modify it under the terms of the GNU General Public License          #
# as published by the Free Software Foundation; either version 2       #
# of the License, or (at your option) any later version.               #
#                                                                      #
# This program is distributed in the hope that it will be useful,      #
# but WITHOUT ANY WARRANTY; without even the implied warranty of       #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        #
# GNU General Public License for more details.                         #
#                                                                      #
# You should have received a copy of the GNU General Public License    #
# along with this program; if not, write to the Free Software          #
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA        #
# 02110-1301, USA.                                                     #
#                                                                      #
# THE AUTHORS OF THIS PROGRAM ACCEPT ABSOLUTELY NO LIABILITY FOR       #
# ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE   #
# TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of      #
# harming persons must have provisions for completely removing power   #
# from all motors, etc, before persons enter any danger area.  All     #
# machinery must be designed to comply with local and national safety  #
# codes, and the authors of this software can not, and do not, take    #
# any responsibility for such compliance.                              #
########################################################################

import argparse
import time
import hal

parser = argparse.ArgumentParser(description = 'HAL component to control temperatures.')
parser.add_argument('-n', '--name', help='HAL name for this component', required = True)
args = parser.parse_args()


h = hal.component(args.name)
h.newparam("min_temp", hal.HAL_FLOAT, hal.HAL_RW)
h.newparam("max_temp", hal.HAL_FLOAT, hal.HAL_RW)
h.newparam("pid_active_range", hal.HAL_FLOAT, hal.HAL_RW)
h.newparam("fast_pwm", hal.HAL_FLOAT, hal.HAL_RW)
h.newparam("preheat_delay", hal.HAL_U32, hal.HAL_RW)
h.newparam("preheat_target", hal.HAL_FLOAT, hal.HAL_RW)

h.newpin("temp_set", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("temp_meas", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("pid_return", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("enable", hal.HAL_BIT, hal.HAL_IN)

h.newpin("pid_command", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("pid_enable", hal.HAL_BIT, hal.HAL_OUT)
h.newpin("pwm", hal.HAL_FLOAT, hal.HAL_OUT)

h.newpin("active", hal.HAL_BIT, hal.HAL_OUT)
h.newpin("stabalizing", hal.HAL_BIT, hal.HAL_OUT)
h.newpin("ready", hal.HAL_BIT, hal.HAL_OUT)

h.pid_enable = 0
h.pid_command = 0
h.pwm = 0


tempActive = 0
pidActive = 0
directPwmActive = 0
lastPidCmd = 0

prevTempSet = 0
reachedSetTemp = 0
stabalizeTime = 0

h.ready()

try:
  while(1):
    sTemp = h.temp_set
    if(sTemp != prevTempSet):
      h.active = 0
      h.stabalizing = 0
      h.ready = 0
      reachedSetTemp = 0
      stabaslizeTime = 0
      prevTempSet = sTemp

    if(h.enable and sTemp > 0):
      tempActive = 1
      h.active = 1
      cTemp = h.temp_meas
      if(h.ready != 1):
        target = h.preheat_target
        tm = time.time()
        if(cTemp > sTemp):
          reachedSetTemp = 1
        if(cTemp >= sTemp - target and cTemp <= sTemp + target):
          if(reachedSetTemp == 1 and stabalizeTime == 0):
            stabalizeTime = tm
            h.stabalizing = 1
          if(stabalizeTime > 0 and tm - stabalizeTime >= h.preheat_delay):
            h.ready = 1
        else:
          stabalizeTime = 0
          h.stabalizing = 0
        
      if(cTemp < sTemp - h.pid_active_range):
        if(pidActive == 1):
          h.pid_enable = 0
          pidActive = 0
        if(directPwmActive != 1):
          h.pwm = h.fast_pwm 
          directPwmActive = 1
      elif (cTemp > sTemp + h.pid_active_range):
        if(pidActive == 1):
          h.pid_enable = 0
          pidActive = 0
        if(directPwmActive != -1):
          h.pwm = 0
          directPwmActive = -1
      else:
        directPwmActive = 0
        if(pidActive == 0):
          h.pid_enable = 1
          pidActive = 1
          h.pid_command = sTemp
          lastPidCmd = sTemp
        if(sTemp != lastPidCmd):
          h.pid_command = sTemp
          lastPidCmd = sTemp
        if(h.pid_return < 0):
          h.pwm = 0
        else:
          h.pwm = h.pid_return
      time.sleep(.1)
    else: # temp_set == 0
      if(tempActive == 1):
        h.pid_enable = 0
        h.pwm = 0
        pidActive = 0
        directPwmActive = 0
        tempActive = 0
        h.active = 0
        h.stabalizing = 0
      time.sleep(1)

except KeyboardInterrupt:
    h['pwm'] = 0
    raise SystemExit

