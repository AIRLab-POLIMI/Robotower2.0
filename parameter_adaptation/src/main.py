#!/usr/bin/env python
import rospy
import sqlite3
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import String
from trueskill_manager import TrueskillManager

class TrueskillManagerNode(object):

    def __init__(self, name, surname, trueskill_db_path, microgame_db_path, parameter_set_publisher):
        self.trueskill_db_path = trueskill_db_path
        self.t_manager = TrueskillManager(name, surname, self.trueskill_db_path)
        self.microgame_db_path = microgame_db_path
        self.param_pub = parameter_set_publisher
        self.init_table()
        self.reset_callback(True)

    def init_table(self):
        conn = sqlite3.connect(self.microgame_db_path)
        cur = conn.cursor()

        cur.execute('''CREATE TABLE IF NOT EXISTS microgame_history(
            id integer PRIMARY KEY,
            player_id integer, 
            parameter_id integer, 
            outcome integer, 
            duration real)''')

        cur.execute('''CREATE TABLE IF NOT EXISTS trueskill_history(
            id integer PRIMARY KEY,
            idx integer,
            player_id integer, 
            player_mu real,
            player_sigma real,
            set0_mu real,
            set0_sigma real,
            set1_mu real,
            set1_sigma real,
            set2_mu real,
            set2_sigma real
        ) ''')
        conn.commit()
        conn.close()
    
    def init_opponent(self):
        self.current_parameters_id = self.t_manager.choose_opponent()
        self.publish_parameter_choice()
    
    def outcome_callback(self, msg):
        now = rospy.get_rostime()
        duration = now - self.start_time

        player_id = self.t_manager.get_player_id()
        parameter_id = self.current_parameters_id
        self.persist_microgame(player_id, parameter_id, msg.data, duration)
        self.persist_trueskill_snapshot()
        
        self.t_manager.handle_game_outcome(msg.data)
        self.current_parameters_id = self.t_manager.choose_opponent()

        self.start_time = rospy.get_rostime()
        self.publish_parameter_choice()

    def change_player_callback(self, msg):
        s = msg.data
        name, surname = s.split()
        # self.t_manager = TrueskillManager(name, surname, self.trueskill_db_path)
        self.t_manager.set_player_tuple(name, surname)
        player_id = self.t_manager.get_player_id()
        self.t_manager.set_opponent_tuples(player_id)
        self.reset_callback(True)
        
    def publish_parameter_choice(self):
        self.param_pub.publish(self.current_parameters_id)
        
    def reset_callback(self, msg):
        rospy.loginfo("Resetting trueskill manager")
        self.start_time = rospy.get_rostime()
        self.init_opponent()

    def persist_trueskill_snapshot(self):
        conn = sqlite3.connect(self.microgame_db_path)
        cur = conn.cursor()

        parameters_ratings = [s.rating for s in self.t_manager.parameter_sets]
        player_rating = self.t_manager.player.rating

        values = (self.t_manager.get_player_id(), self.t_manager.player.games_played,
            player_rating.mu, player_rating.sigma, 
            parameters_ratings[0].mu, parameters_ratings[0].sigma,
            parameters_ratings[1].mu, parameters_ratings[1].sigma,
            parameters_ratings[2].mu, parameters_ratings[2].sigma,
            )
        cur.execute('''INSERT INTO trueskill_history(player_id, idx,
            player_mu, player_sigma,
            set0_mu, set0_sigma,
            set1_mu, set1_sigma,
            set2_mu, set2_sigma) VALUES(?,?,?,?,?,?,?,?,?,?)''', values)
        conn.commit()
        conn.close() 


    def persist_microgame(self, player_id, parameter_id, outcome, duration):
        conn = sqlite3.connect(self.microgame_db_path)
        cur = conn.cursor()

        values = (player_id, parameter_id, outcome, duration.to_sec(),)
        cur.execute('INSERT INTO microgame_history(player_id, parameter_id, outcome, duration) VALUES(?,?,?,?)', values)
        conn.commit()
        conn.close() 


if __name__ == '__main__':
    rospy.init_node('parameter_manager', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    db_path = rospy.get_param('/trueskill/trueskill_database_path')
    microgame_db_path = rospy.get_param('/trueskill/microgame_db_path')
    microgame_outcome_topic = rospy.get_param('/trueskill/microgame_outcome_topic')
    parameter_set_topic = rospy.get_param('/trueskill/parameter_set_topic')
    reset_topic = rospy.get_param('/trueskill/reset_topic')
    change_player_topic = rospy.get_param('/trueskill/change_player_topic')

    try:
        name = rospy.get_param('/trueskill/player_name')
        surname = rospy.get_param('/trueskill/player_surname')
    except KeyError as e:
        rospy.logerr('MISSING PLAYER INFORMATION')

    pub = rospy.Publisher(parameter_set_topic, Int8, queue_size=10)
    t_manager_node = TrueskillManagerNode(name, surname, db_path, microgame_db_path, pub)
    
    rospy.Subscriber(microgame_outcome_topic, Int8, t_manager_node.outcome_callback)
    rospy.Subscriber(reset_topic, Bool, t_manager_node.reset_callback)
    rospy.Subscriber(change_player_topic, String, t_manager_node.change_player_callback)


    while not rospy.is_shutdown():
        rate.sleep()
