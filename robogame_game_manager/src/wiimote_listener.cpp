/* A callback function executed each time a wiimote message arrives */
int[] dealWiimoteState(const wiimote::State& msg){
        /*   msg.button[5] - Shoot button (back trigger button)
		     msg.button[4] - Recharge button (A button) */
		int buttonState[2] = {msg.buttons[5], msg.buttons[4]};
		return buttonState;
}
