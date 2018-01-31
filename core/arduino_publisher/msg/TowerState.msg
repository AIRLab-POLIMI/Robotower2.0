std_msgs/Header header
int8 id                   # tower id
bool button               # is tower button being pressed?
int8 status               # 1: captured, -1: fallen, 0: iddle?
bool[4] leds              # tower leds array
int64 press_counter