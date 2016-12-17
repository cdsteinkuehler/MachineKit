#!/usr/bin/tclsh

#-----------------------------------------------------------------------
# Copyright: 2014
# Author:    Dewey Garrett <dgarrett@panix.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#-----------------------------------------------------------------------

# pass 1: set environmental variable as flag
#         then restart this script in background
#         exit
# pass 2: detect environmental variable and continuously monitor
#         the xhc-hb04.connected pin

set ::progname [file tail $::argv0]
set ::startup_delay_ms 5000
set ::periodic_delay_ms 2000
set ::popup_hold_ms 0 ;# use 0 for forever
set ::debug 0

proc bool_from_string {v} {
  # hal getp returns TRUE,FALSE
  switch [string tolower $v] {
    true  {set v 1}
    false {set v 0}
  }
  return $v
} ;# bool_from_string

proc dputs {msg} {
  if $::debug {
    puts stderr $::progname:debug:$msg
  }
} ;# dputs

proc start_monitor {} {
  package require Hal
  package require Tk

  wm withdraw .
  set ::popupw  [toplevel .p]
  wm withdraw $::popupw
  wm title $::popupw $::progname
  wm protocol $::popupw WM_DELETE_WINDOW hide
  pack [label ${::popupw}.t -text [now] -font bold]
  pack [label ${::popupw}.l -text "startup" -font bold]
  pack [button ${::popupw}.b -text "OK" -command dismiss] -anchor e

  if [catch {set value [hal getp xhc-hb04.connected]} msg ] {
    # this can occur if loadusr xhc-hb04 fails, 
    puts "$::progname: xhc-hb04 comp not loaded" 
    puts "$::progname: <$msg>"
    puts "$::progname: exiting"
    return
  }
  set ::connected false
  if [catch { set ::connected [hal getp xhc-hb04.connected]
            } msg] {
    puts "$::progname: connected? <$msg>"
  }
  set ::connected [bool_from_string $::connected]

  set ::require_pendant false
  if [catch { set ::require_pendant [hal getp xhc-hb04.require_pendant]
            } msg] {
    puts "$::progname: require_pendant? <$msg>"
    puts "$::progname: pendant connection not required, Continuing"
  }
  set ::require_pendant [bool_from_string $::require_pendant]
  dputs "startup:connected=$::connected required=$::require_pendant"

  if {!$::require_pendant && !$::connected} {
    popup "\nrequire_pendant==0\nPendant not connected at startup"
  }

  after $::periodic_delay_ms check
} ;# start_monitor

proc now {} {
  return [clock format [clock seconds]]
} ;# now

proc popup {msg} {
  dputs $msg
  $::popupw.l configure -text [now]
  $::popupw.l configure -text $msg
  center $::popupw

  if {$::popup_hold_ms != 0} {
    after $::popup_hold_ms [list wm iconify $::popupw]
  }
} ;# popup

proc center {w} {
    update idletasks
    set x [expr [winfo screenwidth $w]/2 \
              - [winfo reqwidth $w]/2  - [winfo vrootx [winfo parent $w]]]
    set y [expr [winfo screenheight $w]/2 \
              - [winfo reqheight $w]/2  - [winfo vrooty [winfo parent $w]]]
    wm geom $w +$x+$y
    wm deiconify $w
} ;# center

proc hide {} {
  wm withdraw $::popupw
} ;# hide

proc dismiss {} {
  wm withdraw $::popupw
} ;# dismiss

proc check {} {
  if [catch { set ::connected_new [hal getp xhc-hb04.connected]
            } msg] {
    puts "$::progname <$msg>"
  }
  set ::connected_new [bool_from_string $::connected_new]
  dputs "check:connected_new=$::connected_new required=$::require_pendant"

  set allow_recheck 1
  if {$::connected_new != $::connected} {
    if $::connected_new {
      set msg "\nConnected to pendant"
    } else {
      set msg "\nLost connection to pendant"
    }
    if $::require_pendant {
      if {$::connected_new} {
        set msg "$msg\nUnexpected"
      } else {
        # xhc-hb04 will timeout and exit so no more checks
        set msg "$msg\nReconnect not supported for require_pendant==1"
        set allow_recheck 0
      }
    }
    dputs "$::progname: $msg"
    popup $msg
  }

  set ::connected $::connected_new
  if $allow_recheck {after $::periodic_delay_ms check}
} ;# check

# begin -----------------------------------------------------
if ![info exists ::env(monitor_xhc-hb04)] {
  set ::env(monitor_xhc-hb04) $::argv0
  exec ./monitor_xhc-hb04.tcl & ;# restart this with env(monitor_xhc-hb04) set
  exit 0
} else {
  after $::startup_delay_ms
  start_monitor
}
