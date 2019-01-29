import sqlite3


class DatabaseManager(object):
    
    def __init__(self, db_path, baseline=False):
        if baseline:
            self.database_path = db_path + '_baseline.db'
        else:
            self.database_path = db_path + '.db'
        self.create_database()
        self.baseline=baseline


    def create_database(self):
        conn = sqlite3.connect(self.database_path)
        cur = conn.cursor()

        cur.execute('''CREATE TABLE IF NOT EXISTS players (
            id integer PRIMARY KEY,
            name text NOT NULL,
            surname text NOT NULL,
            mu real,
            sigma real,
            games_played integer,
            times_rank_adjusted integer
            )''')
        
        cur.execute('''CREATE TABLE IF NOT EXISTS robots (
            id integer PRIMARY KEY,
            player_id integer NOT NULL,
            level_id integer NOT NULL,
            mu real,
            sigma real
            )''')
        
        conn.commit()
        conn.close()
        

    def insert_player(self, name, surname, mu=25.0, sigma=8.3, games_played=0, times_rank_adjusted=0):
        conn = sqlite3.connect(self.database_path)
        cur = conn.cursor()

        # Insert the player with the given name
        values = (name, surname, mu, sigma, games_played, times_rank_adjusted)
        cur.execute('INSERT INTO players(name,surname,mu,sigma,games_played,times_rank_adjusted) VALUES(?,?,?,?,?,?)', values)
        player_id = cur.lastrowid

        for i in range(3):
            # For each player, create a local ranking
            level_id = i
            values_robot = (player_id, level_id, 25.0, 8.3)
            try:
                cur.execute('INSERT INTO robots(player_id,level_id,mu,sigma) VALUES(?,?,?,?)', values_robot)
            except e:
                print(e)

        conn.commit()
        conn.close()
        return player_id

    def remove_player(self, name, surname):
        conn = sqlite3.connect(self.database_path)
        cur = conn.cursor()

        # Retrieve the player_id with the given name
        values = (name, surname,)
        cur.execute('SELECT id FROM players WHERE name=? AND surname=?', values)
        player_id = cur.fetchone()[0]

        cur.execute('DELETE FROM robots WHERE player_id=?', (player_id,))
        cur.execute('DELETE FROM players WHERE id=?', (player_id,))

        conn.commit()
        conn.close()
        return player_id
    
    def remove_opponents(self, player_id):
        conn = sqlite3.connect(self.database_path)
        cur = conn.cursor()

        cur.execute('DELETE FROM robots WHERE player_id=?', (player_id,))

        conn.commit()
        conn.close()
        return

    def get_player_record(self, name, surname):
        conn = sqlite3.connect(self.database_path)
        cur = conn.cursor()

        values = (name, surname,)
        cur.execute('SELECT * FROM players WHERE name=? AND surname=?', values)

        player_tuple = cur.fetchone()
        
        if player_tuple is None:
            # We have not encountered this player before
            player_id = self.insert_player(name, surname)
            cur.execute('SELECT * FROM players WHERE id=?', (player_id,))
            player_tuple = cur.fetchone()
        
        conn.commit()
        conn.close()
        return player_tuple

    def get_opponents(self, player_id):
        conn = sqlite3.connect(self.database_path)
        cur = conn.cursor()

        cur.execute('SELECT * FROM robots WHERE player_id=?', (player_id,))
        opponents = cur.fetchall()

        conn.close()
        return opponents

    def update_player_tuple(self, player_id, mu, sigma, games_played, times_rank_adjusted):
        conn = sqlite3.connect(self.database_path)
        cur = conn.cursor()

        cur.execute('UPDATE players SET mu=?, sigma=?, games_played=?, times_rank_adjusted=? WHERE id=?', (mu, sigma, games_played, times_rank_adjusted, player_id,))

        conn.commit()
        conn.close()
        return

    def update_robot_tuple(self, player_id, level_id, mu, sigma):
        conn = sqlite3.connect(self.database_path)
        cur = conn.cursor()

        cur.execute('UPDATE robots SET mu=?, sigma=? WHERE player_id=? and level_id=?', (mu, sigma, player_id, level_id,))

        conn.commit()
        conn.close()
        return

