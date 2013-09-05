#!/usr/bin/python
# GT Simple Mission Controller Program
from pyMC import *

print "GT Simple Mission Controller Program!";
print " Survey v1";
  
#######################################################################
# MISSION

# Define Survey loop
def survey_loop():
   mc_conf_ref_vel((0.2, 0.2, 0.2, 10, 10, 10))
   goto_point([-0.6, 3.5,  -1.6, -90,  0,  0],   to=90)
   goto_point([-1.65, 3.5, -1.6, -90,  0,  0],   to=90)
   goto_point([-1.65, 3.5, -1.6,   0,  0,  0],   to=90)
   goto_point([-1.65, 4.0, -1.2,   0,  0,  0],   to=90)
   goto_point([-1.65, 4.5, -1.2,   0,  0,  0],   to=90)
   goto_point([-1.65, 4.5, -1.2,  45,  0,  0],   to=90)
   goto_point([-1.35, 4.5, -1.2,  90,  0,  0],   to=90)
   goto_point([-1.35, 4.5, -1.2, 180,  0,  0],   to=90)
   goto_point([-1.35, 4.0, -1.2, 180,  0,  0],   to=90)

# Configure control loop modes 
mc_conf_control([ON,ON,ON,ON,OFF,OFF])
    
# Configure reference velocity
mc_conf_ref_vel((0.1,0.1,0.1,10,10,10))

# Start mission...

# Center Tank
tx(mc_make_WMT(1)) # Configure Reference Mode
goto_point([-0.60, 3.50, -1.2, 0, 0, 0], to=90)
mc_delay(1)

# Start survey (8 loops)
for ix in range(0,8):
  print "-----------------------"
  print "Starting Loop: " + str(ix+1)
  survey_loop()
  
#Stop
goto_point([-0.60, 3.50, -1.2, 0, 0, 0],to=90)
mc_delay(30)


# END
tx(mc_make_WMT(0))
mc_conf_control([ON,ON,ON,ON,OFF,OFF]) # Configure control loop modes 

