#!/usr/bin/python
# GT Mission Controller Module
# Giancarlo Troni
import sys, time, socket, signal
import threading
from collections import namedtuple
import collections
import math

########################################################################
# GENERAL PURPOSE STUFF
""" --------------------------------------------------------------------  
Version
"""
version = '0.0.7'

""" -------------------------------------------------------------------- 
Define CONSTANTS
"""
# SOCKET
RXPORT = 50300        # MC(rx) internal port
TXPORT = 50301        # ROV(rx) controller external port
# DOF
DOF_X=0
DOF_Y=1
DOF_Z=2
DOF_HDG=3
DOF_PITCH=4
DOF_ROLL=5
# GENERAL
OFF=0
ON=1
# Mission limits
GOAL_MIN_DIST_POS=0.3 #meters
GOAL_MIN_DIST_ANG=3.0 #degrees

""" -------------------------------------------------------------------- 
Define STRUCTURES
"""
struct_pos = namedtuple('pos','px py pz hdg pitch roll')
state=struct_pos(0.0,0.0,0.0,0.0,0.0,0.0)

""" -------------------------------------------------------------------- 
Define MATH functions
"""
def fdist_pos(x,y): return (sum((u-v)**2 for u,v in zip(x,y)))**.5
def fdist_ang(x,y):
  if False:
    dwrap=360.0 #use "THIS" when wraping is required 
  else:
    dwrap=float('inf')   # (without wrapping)
    
  if isinstance(x, collections.Iterable):
    #LAZY SOLUTION! FIX!
    dd=[abs(u-v) for u,v in zip(x,y)]   # Calculate diference
    dd=[u if abs(u) <= 180.0 else abs(u%dwrap-360.0) for u in dd] # Move to +/-180 (wrap 360)
    return (sum(u**2 for u in dd))**.5  # Calculate norm of the vector
  else:
    dd=abs(x-y)%dwrap
    return dd if abs(dd) <= 180.0 else abs(dd-360.0)
  
def rtod(x): return math.degrees(x)
def dtor(x): return math.radians(x)

""" --------------------------------------------------------------------  
Add CTRL+C Handler
"""
def signal_handler(signal, frame):
  print 'You pressed Ctrl+C!'
  print 'Proceding with a clean EXIT:'
  # VEHICLE CONF OUT
  tx(mc_make_WMT(0))
  tx(mc_make_WCT(DOF_X,     OFF))
  tx(mc_make_WCT(DOF_Y,     OFF))
  tx(mc_make_WCT(DOF_Z,     OFF))
  tx(mc_make_WCT(DOF_HDG,   OFF))
  tx(mc_make_WCT(DOF_PITCH, OFF))
  tx(mc_make_WCT(DOF_ROLL , OFF))
  print 'Bye!'
  sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

""" --------------------------------------------------------------------  
Add Timeout Function
"""
class TimeoutException(Exception): 
  pass 
 
def timeout(default_timeout_time,default_value):
  def timeout_function(f):      
    def f2(*args,**kwargs):
      # Grab timeout_time variable from f
      timeout_time = kwargs.get('timeout_time', kwargs.get('to', default_timeout_time))
      
      def timeout_handler(signum, frame):
        raise TimeoutException()

      old_handler = signal.signal(signal.SIGALRM, timeout_handler) 
      signal.alarm(timeout_time) # triger alarm in timeout_time seconds
      try: 
        retval = f(*args)
      except TimeoutException:
        return default_value
      finally:
        signal.signal(signal.SIGALRM, old_handler) 
      signal.alarm(0)
      return retval
    return f2
  return timeout_function

""" --------------------------------------------------------------------  
configure the socket
"""
#bufferSize = 1024   # verify
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 0))
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

""" -------------------------------------------------------------------- 
Send UDP broadcast packets function
"""
def tx(data):
  print data
  s.sendto(data, ('<broadcast>', TXPORT))


########################################################################
# READ VEHICLE STATUS THREAD
def handleUDPInput():
  sock = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
  sock.bind(('', RXPORT))
  while True:
    data, addr = sock.recvfrom(512)
    if  (data[0:3]=="DVZ"):
      print "Message DVZ Received: " + data
      # Parse DVZ message
      try:
        stype, sdate, stime, sid, sid2, sid3, spx, spy,\
        spz, shdg, spitch, sroll, sdepth, sremainder\
        = data.split()
        px,py,pz,hdg,pitch,roll,depth=(float(x) for x in (spx,spy,spz,shdg,spitch,sroll,sdepth))
        print px,py,pz
      except ValueError:
        print "ERROR pyMC: Fail parsing DVZ data"
        
    elif(data[0:3]=="AUV"):
      #~ print "Message AUV Received: " + data
      # Parse AUV message
      try:
        # Parse string
        stype, sdate, stime, svehicle, spx, spy, spz, shdg, spitch, sroll,\
        sdepth, slx1, slx2, saltitude, su, sv, sw, sr ,sp ,sh\
        = data.split()
        # Convert to float
        px,py,pz,hdg,pitch,roll,depth=(float(x) for x in (spx,spy,spz,shdg,spitch,sroll,sdepth))
        # Update global variables
        global state
        state=struct_pos(px,py,pz,rtod(hdg),rtod(pitch),rtod(roll))
        #print state
      except ValueError:
        print "ERROR pyMC: Fail parsing AUV data"
    else:
      print "Unknown message received: " + data


########################################################################
# DEFINE JASON TALK MESSAGES
""" -------------------------------------------------------------------- 
Makes the Write Control Type string (enables control loop)
WCT [DOF integer] [type] [cr]
""" 
def mc_make_WCT(dof,num,ref=0): 
  return "WCT " + str(dof) + ' ' + str(num) + ' ' + str(ref) + '\r'
  
""" -------------------------------------------------------------------- 
Makes the Write Move Type
WMT [type] [cr]
""" 
def mc_make_WMT(type): 
  return "WMT " + str(type)  + '\r'
  
"""  --------------------------------------------------------------------
Makes the Write Reference Absolute string (sets reference)
WRA [DOF integer] [reference absolute] [cr]
"""   
def mc_make_WRA(dof,ref_pos):
  return "WRA " + str(dof) + ' ' + str(ref_pos) + '\r'

"""  --------------------------------------------------------------------
Makes the Write Reference Velocity string (sets max auto velocity)
WRV [DOF integer] [reference velocity in m/s] [cr]
"""
def mc_make_WRV(dof,ref_vel):
  return "WRV " + str(dof) + ' ' + str(ref_vel) + '\r'

""" --------------------------------------------------------------------
Makes the Write Goal Absolute Single degree of freedom string (Goal 1 DOF))
WGA [DOF integer] [dof goal, double] [cr]
"""
def mc_make_WGA(dof,goal_dof_pos):
  return "WGA " + str(dof) + ' ' + str(goal_dof_pos) + '\r'

""" --------------------------------------------------------------------
Makes the Write Goal string
WGO [x goal, double][y goal, double][z goal, double][hdg goal, double][pitch goal, double][roll goal, double] [cr]
"""
def mc_make_WGO(goal_pos):
  return "WGO " + ' '.join(str(x) for x in goal_pos)  + '\r'


########################################################################
# HIGH LEVEL FUNCTIONS
""" --------------------------------------------------------------------
GOTO_DOF
GOTO_DOF [x goal, double][y goal, double][z goal, double][hdg goal, double][pitch goal, double][roll goal, double] [cr]
"""
@timeout(60,0)
def goto_dof(dof,goal_dof_pos,**kwargs):
  # Send DOF GOAL Command
  tx(mc_make_WGA(dof,goal_dof_pos))
  # VERIFY FINAL DOF POS
  if(dof<DOF_HDG):
    min_dist = GOAL_MIN_DIST_POS
  else:
    min_dist = GOAL_MIN_DIST_ANG
  dist = float('inf')
  while (dist >= min_dist):
    dist=abs(state[dof]-goal_dof_pos)
    #use dist=fdist_ang(state[dof],goal_dof_pos) when wraping is required
    if(dof<DOF_HDG):
      print "goto_dof: Dist Pos (" + str(dof) + ") = " + str(dist)
    else:
      print "goto_dof: Dist Ang (" + str(dof) + ") = " + str(dist)
    time.sleep(1)
  # When success (without timeout)
  return 1

""" --------------------------------------------------------------------
GOTO_POINT
GOTO_POINT [x goal, double][y goal, double][z goal, double][hdg goal, double][pitch goal, double][roll goal, double] [cr]
"""
@timeout(60,0)
def goto_point(goal_pos,**kwargs):
  tx(mc_make_WGO(goal_pos))
  # VERIFY FINAL DOF POS
  min_dist_pos = GOAL_MIN_DIST_POS # meters
  min_dist_ang = GOAL_MIN_DIST_ANG # degrees
  dpos = dang = float('inf')
  while (dpos >= min_dist_pos) or (dang >= min_dist_ang):
    dpos=fdist_pos(goal_pos[0:3],state[0:3])
    dang=fdist_ang(goal_pos[3],state[3]) # ONLY HDG!
    print "goto_point: Dist Pos= " + str(dpos) + " - Dist Ang= " + str(dang)
    time.sleep(1)
  # When success (without timeout)
  return 1

""" --------------------------------------------------------------------
MC_CONF_REF_VEL
MC_CONF_REF_VEL [x vel, double][y vel, double][z vel, double][hdg vel, double][pitch vel, double][roll vel, double] [cr]
"""
def mc_conf_ref_vel(ref_vel):
  for ix in range(len(ref_vel)):
    tx(mc_make_WRV(ix,ref_vel[ix]))

""" --------------------------------------------------------------------
MC_CONF_CONTROL
MC_CONF_CONTROL [x on/off, int][y on/off, int][z on/off, int][hdg on/off, int][pitch on/off, int][roll on/off, int] [cr]
"""
def mc_conf_control(control):
  for ix in range(len(control)):
    tx(mc_make_WCT(ix, control[ix]))

""" --------------------------------------------------------------------
MC_CONF_MODE
MC_CONF_MODE [mode, int: 0=joystick 1=reference 2=sinusoidal]
"""
def mc_conf_mode(mode):
  tx(mc_make_WMT(mode))

""" --------------------------------------------------------------------
MC_DELAY

"""
def mc_delay(time_delay_sec,*args):
  # Grab verbose variable
  lverbose=["verbose","v"]
  verbose = any( str(k).lower() in lverbose for k in args )
  # Print main message
  print "Time delay " + repr(time_delay_sec) + " sec"
  if verbose:
    for i in range(int(time_delay_sec),0,-1):
      sys.stdout.write("\rRemaining time: %d sec  " % i)
      sys.stdout.flush()
      time.sleep(1)
    sys.stdout.write("\r                        \r\n") # clean up
  else:
    time.sleep(time_delay_sec)

""" --------------------------------------------------------------------
MC_WHERE

"""
def mc_state():
  print state
  
########################################################################
# Launching Threads
recv_thread=threading.Thread(target=handleUDPInput)
recv_thread.setDaemon(True)
recv_thread.start()


########################################################################
# THE MAIN (only for testing)
""" --------------------------------------------------------------------
MAIN
"""
if __name__ == "__main__":
    import sys
    # Testing functions
    mc_conf_ref_vel((0.1,0.1,0.1,10,10,10))
    tx(mc_make_WMT(1)) # Configure Reference Mode
    goto_dof(DOF_HDG,4.0,to=10)
    #goto_point((1,2,3,4,5,6),to=10)
    time.sleep(1)
    print "state: " + repr(state)

