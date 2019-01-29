import sys
import sqlite3

database_trueskill = '/home/airlab/catkin_ws/src/phd_robogame/parameter_adaptation/data/trueskill.db'
database_microgame = '/home/airlab/catkin_ws/src/phd_robogame/parameter_adaptation/data/microgame.db'

def get_player_id(name, surname):
    conn = sqlite3.connect(database_trueskill)
    cur = conn.cursor()

    cur.execute('SELECT id FROM players WHERE name=? AND surname=?', (name, surname,))
    query_result = cur.fetchone()
    if query_result is None:
        return None
    else:
        return query_result[0]

def remove_from_trueskill(p_id):
    conn = sqlite3.connect(database_trueskill)
    cur = conn.cursor()

    cur.execute('DELETE FROM players WHERE id=?', (p_id,))
    cur.execute('DELETE FROM robots WHERE player_id=?', (p_id,))

    conn.commit()
    conn.close()


def remove_microgame_sequence(p_id):
    conn = sqlite3.connect(database_microgame)
    cur = conn.cursor()

    cur.execute('DELETE FROM microgame_history WHERE player_id=?', (p_id,))
    cur.execute('DELETE FROM trueskill_history WHERE player_id=?', (p_id,))

    conn.commit()
    conn.close()


def remove(name, surname):
    p_id = get_player_id(name, surname)
    if p_id is None:
        print "Unable to find any record for {} {}".format(name, surname)
        return
    remove_from_trueskill(p_id)
    remove_microgame_sequence(p_id)

if __name__ == '__main__':
    args = sys.argv[1:]
    if len(args) == 2:
        name = args[0]
        surname = args[1]
        remove(name, surname)
    else:
        print "Incorrect number of arguments: insert name and surname"