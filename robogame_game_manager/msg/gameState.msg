#	Message containing the complete game state.


#-----------------------------
# Header
#----------------------
std_msgs/Header header

#-----------------------------
# Player resources
#----------------------

int8 numBullets		#number of shots (button presses) the player still has.

#-----------------------------
# Player Status
#----------------------

bool isPlayerLost
bool isPlayerTooClose

#-----------------------------
# Resource manager
#----------------------
# recharger time
bool isRecharger
int64 startedAt
int64 duration

#-----------------------------
# Wiimote state
#----------------------
bool shootButton		#wiimote trigger like button (on the back of the remote)
bool rechargeButton		#wiimote 'A' button.

#-----------------------------
# Robot interaction
#----------------------

int8 LEDColorindex	        # Assumes the arduino has a map for the color.
int32 blinkInterval         # the LED blink rate
bool beep                   # makes the robot beep
