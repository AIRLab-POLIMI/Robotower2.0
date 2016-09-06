#   This reduce the gameState msg size in order to ease arduino communications. 
#

#-----------------------------
# Player Status
#----------------------

bool isPlayerLost
bool isPlayerTooClose

#-----------------------------
# Robot interaction
#----------------------

int8 LEDColorindex	    # Assumes the arduino has a map for the color.
int32 blinkInterval     # the LED blink rate
bool beep               # makes the robot beep
