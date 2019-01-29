import sqlite3
from sqlite3 import Error
import json

class Dumper(object):

    def __init__(self):
        self.db = "/home/airlab/catkin_ws/src/phd_robogame/control/bayesian_skill_learning/db/robotower.db"
        self.conn = None

    def connect(self):
        """ create a database connection to a SQLite database, default in memory """
        try:
            self.conn = sqlite3.connect(self.db)
            return self.conn.cursor()
        except Error as e:
            return    
            print(e)
    
    def get_levels(self, id_player):
        cur = self.connect()
        cur.execute("SELECT level, mu, sigma FROM robot WHERE id_player={}".format(id_player))
        return cur.fetchall()

    def dump(self, table):
        cur = self.connect()
        cur.execute("SELECT * FROM {}".format(table))

        all_entries = []

        for row in cur.fetchall():
            dic = {"Player": {"Surname": row[2], "mu": row[3], "sigma": row[4]}}
            all_levels = self.get_levels(row[0])
            for r in all_levels:
                dic["level_{}".format(r[0])] = {"mu": r[1], "sigma": r[2]}
            all_entries.append(dic)

        # if table == "players":
        #     all_rows = [{"Id": r[0], "Surname": r[2], "mu": r[3], "Sigma": r[4]} for r in cur.fetchall()]
        # elif table == "robot":
        #     all_rows = [{"Player ID": r[1], "Level ID": r[2], "mu": r[3], "Sigma": r[4]} for r in cur.fetchall()]
        return all_entries

if __name__ == "__main__":
    dumper = Dumper()
    print "*** PLAYER ***"
    print json.dumps(dumper.dump("players"), indent=4)
    # print "*** ROBOT ***"
    # print json.dumps(dumper.dump("robot"), indent=4)




    