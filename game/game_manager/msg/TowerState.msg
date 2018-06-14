std_msgs/Header header

uint8 TYPE_TOWER_HAS_FALLEN=1
uint8 TYPE_TOWER_OFFLINE=2
uint8 TYPE_TOWER_ONLINE=3
uint8 TYPE_TOWER_CAPTURED=4

uint8 id
uint8 status
bool[4] leds
uint8 bt_presses
float32 press_accuracy