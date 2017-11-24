std_msgs/Header header    # time stamp
int64 pipe_id             # communication pipe id (tower id)
bool is_tower_enable      # is game on?
bool is_button_pressed    # is tower button being pressed?
bool is_captured          # has the tower been captured?
bool is_tower_down        # has tower fallen?
bool[4] leds              # tower leds array
int64 press_counter       # press counter
