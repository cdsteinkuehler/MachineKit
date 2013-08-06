#!/usr/bin/python

########################################################################
# Description: AutoTune.py                                             #
#                                                                      #
#  This is a routine to calculate PID parameters for extruders and     #
#  heated beds.  It is a python version of the autotune route in the   #
#  Marlin firmware.                                                    #
#                                                                      #
#  See Autotune.hal for further instructions.                          #
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

import sys
import time
import hal

class HalOutPin:
  def __init__(self, halcomp, type, name, initVal):
    self.halcomp = halcomp
    halcomp.newpin(name, type, hal.HAL_OUT)
    halcomp[name] = initVal
    self.name = name
    self.lastVal = initVal

  def set(self, value):
    if(value <> self.lastVal):
      self.halcomp[self.name] = value
      self.lastVal = value    
  def get(self):
     return self.lastVal

class HalInPin:
  def __init__(self, halcomp, type, name):
    self.halcomp = halcomp
    halcomp.newpin(name, type, hal.HAL_IN)
    self.name = name

  def get(self):
    return self.halcomp[self.name]

class State:
  def __init__(self, halcomp):
    self.active = 0
    self.stableCycle = 2
    self.max_high = .95
    self.min_delta = .1
    self.max_delta = .5

    self.forextruder = HalInPin(halcomp, hal.HAL_BIT, "forextruder")
    self.high = HalOutPin(halcomp, hal.HAL_FLOAT, "high", 0)
    self.low = HalOutPin(halcomp, hal.HAL_FLOAT, "low", 0)
    self.cycle = HalOutPin(halcomp, hal.HAL_U32, "cycle", 0)
    self.pwm = HalOutPin(halcomp, hal.HAL_FLOAT, "pwm", 0)
    self.heating = HalOutPin(halcomp, hal.HAL_BIT, "heating", 0)
    self.tcooling = HalOutPin(halcomp, hal.HAL_FLOAT, "tcooling", 0)
    self.theating = HalOutPin(halcomp, hal.HAL_FLOAT, "theating", 0)
    self.min = HalOutPin(halcomp, hal.HAL_FLOAT, "min", 1000)
    self.max = HalOutPin(halcomp, hal.HAL_FLOAT, "max", 0)
    self.ku = HalOutPin(h, hal.HAL_FLOAT, "ku", 0)
    self.tu = HalOutPin(h, hal.HAL_FLOAT, "tu", 0)
    self.p = HalOutPin(h, hal.HAL_FLOAT, "p", 0)
    self.i = HalOutPin(h, hal.HAL_FLOAT, "i", 0)
    self.d = HalOutPin(h, hal.HAL_FLOAT, "d", 0)
    self.change = HalOutPin(h, hal.HAL_FLOAT, "change", 0)
    self.errori = HalOutPin(h, hal.HAL_FLOAT, "errori", 0)
    self.pidClassic = HalInPin(halcomp, hal.HAL_BIT, "pidClassic")
    self.pidSomeOS = HalInPin(halcomp, hal.HAL_BIT, "pidSomeOS")
    self.pidNoOS = HalInPin(halcomp, hal.HAL_BIT, "pidNoOS")

  def isActive(self):
    return self.active
  def setActive(self, active):
    if(active == 1):
      self.activate()
    else:
      self.deactivate()
    self.active = active
  
  def deactivate(self):
    self.heating.set(0)
    self.pwm.set(0)

  def activate(self):
    self.cycle.set(0)
    self.stableCycle = 2
    if(self.forextruder.get() == 1):
      self.preheat = .5
      self.setLow(0)
      self.setHigh(.5)
    else:
      self.preheat = 1
      self.setLow(.1)
      self.setHigh(.6)
    self.tcooling.set(0)
    self.theating.set(0)
    self.min.set(1000)
    self.max.set(0)

  def setLow(self, low):
    self.low.set(low)

  def setHigh(self, high):
    self.high.set(high)

  def slowResponse(self):
    if(self.heating.get() == 1):
      self.high.set(self.high.get() + .05)
      self.low.set(self.low.get() + .05)
      self.pwm.set(self.high.get())
    else:
      self.low.set(self.low.get() - .05)
      self.high.set(self.high.get() - .05)
      self.pwm.set(self.low.get())
    if(self.cycle.get() > self.stableCycle):
      self.stableCycle = self.cycle.get()
 
  def heat(self):
    self.heating.set(1)
    if(self.cycle.get() > 0):
      self.pwm.set(self.high.get())
    else:
      self.pwm.set(self.preheat) 
  def cool(self):
    self.heating.set(0)
    if(self.cycle.get() > 0):
      self.pwm.set(self.low.get())
    else:
      self.pwm.set(0)
  def isHeating(self):
    return self.heating.get()
 
  def updateMinMax(self, current):
    if(current < self.min.get()):
      self.min.set(current)
    if(current > self.max.get()):
      self.max.set(current)
  def setMax(self, max):
    self.max.set(max)
  def setMin(self, min):
    self.min.set(min)
 
  def setTimeHeating(self, theating):
    self.theating.set(theating) 
  def setTimeCooling(self, tcooling):
    self.tcooling.set(tcooling)
  
  def adjustForNextCycle(self):
    h = self.high.get()
    l = self.low.get()
    theating = self.theating.get()
    tcooling = self.tcooling.get()
    change = (h*(theating-tcooling)) / (tcooling+theating)
    if(change > .05):
      change = .05
    if(change < -.05):
      change = -.05

    self.change.set(change)
    if(h >= self.max_high or (change > 0 and (h-l+change) > self.max_delta)):
      l = l + change
      if(h-l < self.min_delta):
        h = l + self.min_delta
    else:
      h = h + change
      if(h-l < self.min_delta):
        l = h - self.min_delta

    if(l < 0):
      l = 0
    if(h < self.min_delta):
      h = self.min_delta
    if(h > self.max_high):
      h = self.max_high
    if(l > self.max_high - self.min_delta):
      l = self.max_high - self.min_delta
    self.high.set(h)
    self.low.set(l)

  def getCycle(self):
    return self.cycle.get()

  def nextCycle(self):
    self.cycle.set(self.cycle.get()+1)

  def calculatePIDParameters(self):
    tempDif = self.max.get() - self.min.get()
    pwmDif = self.high.get() - self.low.get()

    self.ku.set( (4*pwmDif) / (3.14159*tempDif/2) )
    self.tu.set(self.tcooling.get() + self.theating.get())
          
  def generatePID(self):
    if(self.cycle.get() > self.stableCycle):
      i = 0
      if(self.pidClassic.get()):
        self.p.set(0.6 * self.ku.get())
        i = (2*self.p.get() / self.tu.get())
        self.i.set(i)
        self.d.set(self.p.get() * self.tu.get()/8)
      if(self.pidSomeOS.get()):
        self.p.set(0.33 * self.ku.get())
        i = (self.p.get() / self.tu.get())
        self.i.set(i)
        self.d.set(self.p.get() * self.tu.get()/3)
      if(self.pidNoOS.get()):
        self.p.set(0.2 * self.ku.get())
        i = (2*self.p.get() / self.tu.get())
        self.i.set(i)
        self.d.set(self.p.get() * self.tu.get()/3)
      if(i > 0):
        self.errori.set(1)
        #self.errori.set(0.25 / i)
    else:
      self.p.set(0)
      self.i.set(0)
      self.d.set(0)
      self.errori.set(0)

h = hal.component("autotune")
enable = HalInPin(h, hal.HAL_BIT, "enable")
temp = HalInPin(h, hal.HAL_FLOAT, "temp")
preheat = HalInPin(h, hal.HAL_FLOAT, "preheat")
current = HalInPin(h, hal.HAL_FLOAT, "current")
steptime = HalOutPin(h, hal.HAL_FLOAT, "steptime", 0)


state = State(h)
h.ready()


timeHyst = 5
tempHyst = .15
try:
  tm = time.time()
  t1 = tm
  t2 = tm
  cm = tm

  while 1:
    curTemp = current.get()
    state.generatePID()

    if(enable.get()):
      tm = time.time();
      setTemp = temp.get()
      if(state.isActive() == 0):
        state.setActive(1)
        state.heat()
        cm = tm
      
      state.updateMinMax(curTemp)

      if(state.getCycle() > 0 and tm - cm > 45):
        state.slowResponse()
        cm = tm
      steptime.set(tm-cm)

      if(state.isHeating() == 1 and curTemp > setTemp + tempHyst):
        if(tm - t2 > timeHyst):
          state.cool()
          t1 = tm
          cm = tm
          state.setMax(curTemp)
          state.setTimeHeating(t1 - t2)

      if(state.isHeating() == 0 and curTemp < setTemp - tempHyst):
        if(tm - t1 > timeHyst):
          t2 = tm
          cm = tm
          state.setTimeCooling(t2 - t1)

    
          if(state.getCycle() > 1):
            state.calculatePIDParameters()
          if(state.getCycle() > 0):
            state.adjustForNextCycle()
          state.setMin(curTemp)
          state.heat()
          state.nextCycle()
     
      time.sleep(.25)

    else: # Not enabled
      if(state.isActive() == 1):
        state.setActive(0)
      time.sleep(1)          

except KeyboardInterrupt:
    state.setActive(0)
    raise SystemExit

