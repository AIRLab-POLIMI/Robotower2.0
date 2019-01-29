import sqlite3
import sys

def get_player_id(name, surname):
    conn = sqlite3.connect('/home/airlab/catkin_ws/src/phd_robogame/parameter_adaptation/data/trueskill.db')
    cur = conn.cursor()

    cur.execute('SELECT id FROM players WHERE name=? AND surname=?', (name, surname,))
    p_id = cur.fetchone()[0]
    return p_id


def dump_microgame_sequence(name, surname):
    conn = sqlite3.connect('/home/airlab/catkin_ws/src/phd_robogame/parameter_adaptation/data/microgame.db')
    cur = conn.cursor()

    p_id = get_player_id(name, surname)
    cur.execute('SELECT * FROM microgame_history WHERE player_id=?', (p_id,))
    sequence = cur.fetchall()

    conn.close()
    return sequence

def print_player_sequence(seq):
    print 'Player {}'.format(seq[0][1])
    for game in seq:
        line = '\tParameter {}\n\tOutcome {}\n\tDuration {}\n'.format(game[2], game[3], game[4])
        print line

def dump_trueskill_sequence(name, surname):
    conn = sqlite3.connect('/home/airlab/catkin_ws/src/phd_robogame/parameter_adaptation/data/microgame.db')
    cur = conn.cursor()

    p_id = get_player_id(name, surname)
    cur.execute('SELECT * FROM trueskill_history WHERE player_id=? ORDER BY idx', (p_id,))

    sequence = cur.fetchall()

    conn.close()
    return sequence

def print_trueskill_sequence(seq):
    print 'Player {}'.format(seq[0][2])
    for game in seq:
        line = '\tPlayer: mu {}\tsigma{}\n\ts0: mu {}\tsigma {}\n\ts1: mu {}\tsigma {}\n\ts2: mu {}\tsigma {}'.format(
            game[3], game[4], game[5], game[6], game[7], game[8], game[9], game[10])
        print line
        print '################'
        
def main(name, surname):
    seq = dump_trueskill_sequence(name, surname)
    print_trueskill_sequence(seq)
    seq = dump_microgame_sequence(name, surname)
    print_player_sequence(seq)

if __name__ == '__main__':
    args = sys.argv[1:]

    if len(args) == 2:
        name = args[0]
        surname = args[1]
        main(name,surname)
    else:
        print "Incorrect number of arguments: specify name and surname"
