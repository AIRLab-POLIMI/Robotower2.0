#!/usr/bin/env python
import rospy
import sqlite3
from sqlite3 import Error
import trueskill
import numpy as np
from std_msgs.msg import Int8

class DBManager(object):
    sql_insert_player_statement = ''' INSERT INTO players(name,surname,mu,sigma)
              VALUES(?,?,?,?) '''

    sql_select_user_statement = "SELECT * FROM players WHERE name=? and surname=?"
    sql_update_user_statement = "UPDATE players SET mu=?, sigma=? WHERE name=? and surname=?"

    sql_create_table_statement = """CREATE TABLE IF NOT EXISTS players (
                                        id integer PRIMARY KEY,
                                        name text NOT NULL,
                                        surname text NOT NULL,
                                        mu real,
                                        sigma real
                                    );"""
    
    sql_delete_player_statement = "DELETE FROM players WHERE name=? and surname=?"
    sql_drop_table_statement = "DROP TABLE players"

    sql_create_robot_table_statement = """CREATE TABLE IF NOT EXISTS robot (
                                            id integer PRIMARY KEY,
                                            id_player integer NOT NULL,
                                            level integer NOT NULL,
                                            mu real,
                                            sigma real
                                        );"""

    sql_select_all_robotic_players = "SELECT * FROM robot WHERE id_player=?"
    sql_select_robotic_player = "SELECT * FROM robot WHERE id=?"
    sql_insert_robot_statement = ''' INSERT INTO robot(id_player,level,mu,sigma) VALUES(?,?,?,?) '''
    sql_update_robot_statement = "UPDATE robot SET mu=?, sigma=? WHERE id=?"

    def connect(self, database):
        """ create a database connection to a SQLite database, default in memory """
        try:
            self.conn = sqlite3.connect(database)
        except Error as e:
            self.conn = None
            print(e)

    def create_table(self):
        try:
            cur = self.conn.cursor()
            cur.execute(self.sql_create_table_statement)
            self.create_robot_table()
        except Error as e:
            print(e)

    def create_robot_table(self):
        try:
            cur = self.conn.cursor()
            cur.execute(self.sql_create_robot_table_statement)
        except Error as e:
            print(e)

    def start_levels(self, player_id):
        for l in range(3):
            self.insert_robotic_player(player_id, l, 25.0 + l*3, 8.3)
        return

    def insert_player(self, name, surname, mu=25.0, sigma=8.3):
        player = (name, surname, mu, sigma)
        cur = self.conn.cursor()
        cur.execute(self.sql_insert_player_statement, player)
        self.start_levels(cur.lastrowid)
        return cur.lastrowid

    def get_player(self, name, surname):
        cur = self.conn.cursor()
        cur.execute(self.sql_select_user_statement, (name, surname,))
    
        rows = cur.fetchall()
    
        if(len(rows) > 0):
            return rows[0]
        else:
            # Create new record
            rospy.loginfo("Player not found, inserting new record") 
            self.insert_player(name, surname)
            return self.get_player(name, surname)

    def update_player(self, name, surname, mu, sigma):
        cur = self.conn.cursor()
        try:
            cur.execute(self.sql_update_user_statement, (mu, sigma, name, surname,))
        except Error as e:
            print(e)

    def delete_player(self, name, surname):
        cur = self.conn.cursor()
        cur.execute(self.sql_delete_player_statement, (name, surname,))

    def delete_table(self):
        cur = self.conn.cursor()
        cur.execute(self.sql_drop_table_statement) 

    def get_all_robotic_players(self, id_player):
        cur = self.conn.cursor()
        cur.execute(self.sql_select_all_robotic_players, (id_player,))
    
        rows = cur.fetchall()
        return rows

    def get_robotic_player(self, id):
        cur = self.conn.cursor()
        cur.execute(self.sql_select_robotic_player, (id,))
    
        rows = cur.fetchall()
        return rows

    def insert_robotic_player(self, id_player, level_name, mu, sigma):
        cur = self.conn.cursor()
        cur.execute(self.sql_insert_robot_statement, (id_player, level_name, mu, sigma))

    def update_robotic_player(self, id, mu, sigma):
        cur = self.conn.cursor()
        cur.execute(self.sql_update_robot_statement, (mu, sigma, id,))

    def persist_changes(self):
        self.conn.commit()

    def close_connection(self):
        self.persist_changes()
        self.conn.close()
    
    def reconnect(self, database):
        # self.conn.close()
        try:
            self.conn = sqlite3.connect(database)
        except Error as e:
            self.conn = None
            print(e)


class TrueSkillManager(object):

    def __init__(self, name, surname, database=":memory"):
        self.changes = 0 # number of random explorations
        self.current_player_name = name
        self.current_player_surname = surname
        self.difficulty_pub = rospy.Publisher("/difficulty_level", Int8, queue_size=1)
        self.game_outcome_sub = rospy.Subscriber("/microgame_outcome", Int8, self.game_outcome_callback)

        self.database = database

        self.db = DBManager()
        self.db.connect(database)
        self.db.create_table()
        self.set_current_player_record(self.db.get_player(name, surname))
        self.choose_opponent()
        self.db.close_connection()


    def set_current_robotic_player(self, id):
        self.current_robotic_player_record = self.db.get_robotic_player(id)[0]
        self.current_robotic_player_rating = trueskill.Rating(self.current_robotic_player_record[3], self.current_robotic_player_record[4])
        print("Robot database row {}".format(self.current_robotic_player_record))
    
    def set_current_player_record(self, player):
        self.current_player_record = player
        self.current_player_rating = trueskill.Rating(player[3], player[4])
        print("Player database record {}".format(player))

    def game_outcome_callback(self, message):
        self.db.reconnect(self.database)
        msg = Int8()
        if(message.data == 0):
            new_player_rating, new_robotic_player_rating = trueskill.rate_1vs1(self.current_player_rating, self.current_robotic_player_rating, drawn=True)
        elif(message.data == 1):
            new_player_rating, new_robotic_player_rating = trueskill.rate_1vs1(self.current_player_rating, self.current_robotic_player_rating)
        else:
            new_robotic_player_rating, new_player_rating = trueskill.rate_1vs1(self.current_robotic_player_rating, self.current_player_rating)

        self.current_player_rating = new_player_rating
        self.current_robotic_player_rating = new_robotic_player_rating
        self.update_database()
        self.db.persist_changes()
        msg.data = self.choose_opponent()
        self.difficulty_pub.publish(msg)
        self.db.close_connection()
   
    def update_database(self):
        self.db.update_player(self.current_player_record[1], self.current_player_record[2], self.current_player_rating.mu, self.current_player_rating.sigma)
        self.db.update_robotic_player(self.current_robotic_player_record[0], self.current_robotic_player_rating.mu, self.current_robotic_player_rating.sigma)

    def choose_opponent(self):
        robotic_players_rows = self.db.get_all_robotic_players(self.current_player_record[0])
        robotic_players = []
        for robotic_players_row in robotic_players_rows:
            robotic_players.append(trueskill.Rating(robotic_players_row[3], robotic_players_row[4]))
       
        draw_probabilities = []

        for robotic_player in robotic_players:
            draw_probabilities.append(trueskill.quality_1vs1(self.current_player_rating, robotic_player))


        if np.random.uniform() < (0.2/(self.changes+1)):
            best_index = np.random.choice(range(3))
            self.changes += 1
        else:
            choice_made = False
            for i, rp in enumerate(robotic_players):
                sum_sigma = 0
                for j, rp2 in enumerate(robotic_players):
                    if j != i:
                        sum_sigma += rp2.sigma       
                
                if robotic_players[i].sigma >= sum_sigma:
                    choice_made = True
                    best_index = i
                    break
            # Seems this is overriding the choice made during the previous loops
            if not choice_made:
                best_index = np.argmax(draw_probabilities)
            
        best_robotic_player_id = robotic_players_rows[best_index][2] # Get level id
        self.set_current_robotic_player(robotic_players_rows[best_index][0])
        return best_robotic_player_id

if __name__ == '__main__':
    rospy.init_node('trueskill', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    
    database = rospy.get_param("/database")
    name = rospy.get_param("/name")
    surname = rospy.get_param("/surname")

    rospy.sleep(5.0)

    # name = "Stefano"
    # surname = "Boriero"
    # database = "/home/airlab/catkin_ws/src/phd_robogame/control/bayesian_skill_learning/db/robotower.db"
    t_manager = TrueSkillManager(name, surname, database)
    # db = DBManager()
    # db.connect("/home/stefano/sqlite/db/pythonsqlite.db")
    # db.create_table()
    # db.insert_robotic_player(25.0, 8.3)
    # print(db.get_all_robotic_players())
    # print(db.get_player("Stefano", "Boriero"))
    # db.update_player("Mario", "Rossi", 20, 6.2)
    # db.delete_table()
    # db.delete_player("Mario", "Rossi")

    # db.persist_changes()
    while not rospy.is_shutdown():
        rate.sleep()
    # db.close_connection()
