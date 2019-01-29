int64 id            # id to identify the action
int64 intention_id  # id to identify intention this action is trying to accomplish

int16 action_code   # code representing action type
string action_name  # name of the action

planning2/Target[] targets    # targets to be reached by the action
int8[] tower_indexes # indexes of towers to take down, multiple ones in case of deception
