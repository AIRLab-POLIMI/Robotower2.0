import numpy as np
import math
import trueskill
from trueskill import Rating
from trueskill import rate_1vs1
from trueskill import quality_1vs1
from database_utils import DatabaseManager


######################################################################
# TUPLES FORMAT:
# player = (id, name, surname, mu, sigma, games_played, rank_updates)
# opponent = (id, player_id, level_id, mu, sigma)
######################################################################

class TrueskillManager(object):
 
    def __init__(self, name, surname, db_path, baseline=False, verbose=False):
        self.db = DatabaseManager(db_path, baseline=baseline)
        self.set_player_tuple(name, surname)
        self.set_changes = 0
        self.set_opponent_tuples(self.player.id)
        self.baseline = baseline
        self.verbose = verbose

    def set_player_tuple(self, name, surname):
        self.opponent_index = None
        player_tuple = self.db.get_player_record(name, surname)
        self.player = PlayerRatingWrapper(player_tuple)

    def set_opponent_tuples(self, player_id):
        opponent_tuples = self.db.get_opponents(player_id)
        self.parameter_sets = [OpponentRatingWrapper(t) for t in opponent_tuples]
        self.parameter_sets.sort(key=self.sort_fun)

    def sort_fun(self, element):
        return element.level_id

    def choose_opponent(self):
        # Return the opponent with the best drawing probability
        # The drawing prob is given by quality_1vs1(r1,r2)
        games_played = self.player.games_played
        if not self.baseline:
            if self.player.rank_updates == 0:
                self.update_opponent_ranking()

        draw_probabilities = [quality_1vs1(self.player.rating, opp.rating) for opp in self.parameter_sets]
        sigmas = [opp.rating.sigma for opp in self.parameter_sets]
        if self.verbose:
            rospy.loginfo('Draw probabilities: {}'.format(draw_probabilities))
            rospy.loginfo('Sigmas: {}'.format(sigmas))

        new_opponent_index = np.argmax(draw_probabilities)
        
        if self.baseline or self.player.games_played < 4:
            # Take greedy decision
            self.opponent_index = new_opponent_index
        else:
            try:
                if new_opponent_index != self.opponent_index:
                    delta_sigma = abs(sigmas[new_opponent_index] - sigmas[self.opponent_index])
                    delta_treshold = 0.5
                    # Verifiy how much is better the new one
                    delta_p = draw_probabilities[new_opponent_index] - draw_probabilities[self.opponent_index]
                    p_treshold = 0.1
                    if delta_p > p_treshold:
                        delta_sigma = abs(sigmas[new_opponent_index] - sigmas[self.opponent_index])
                        if delta_sigma > delta_treshold:
                            self.opponent_index = new_opponent_index

                    # delta_sigma = sigmas[new_opponent_index] - sigmas[self.opponent_index]
                    # delta_treshold = 0.2 * self.set_changes
                    # delta_treshold = max(delta_treshold, 0.7)
                    if self.verbose:
                        rospy.loginfo('Setting a treshold of {}'.format(delta_treshold))
                        rospy.loginfo('Delta sigma: {}'.format(delta_sigma))
                    # if delta_sigma > delta_treshold:
                    #     if self.verbose:
                    #         rospy.loginfo('Changing set..')
                    #     self.set_changes += 1
                    #     self.opponent_index = new_opponent_index
            except Exception as e:
                # First time, we have to inizialize the variable
                self.opponent_index = new_opponent_index
        
        best_level = self.parameter_sets[self.opponent_index].level_id
        return best_level


    def win_probability(self, player, opponent):
        delta_mu = player.mu - opponent.mu
        sum_sigma = player.sigma ** 2 + opponent.sigma ** 2
        ts = trueskill.global_env()
        BETA = ts.beta
        denom = math.sqrt(2 * (BETA * BETA) + sum_sigma)
        return ts.cdf(delta_mu / denom)

    def handle_game_outcome(self, outcome):
        # Updates Trueskill rankings

        self.player.games_played = self.player.games_played + 1
        if(self.player.games_played < 2):
            # Do not update first game, the player still has to learn the game
            return

        opp = self.parameter_sets[self.opponent_index]

        p_rating, opp_rating = self.update_ratings(self.player.rating, opp.rating, outcome)
        
        self.parameter_sets[self.opponent_index].rating = opp_rating
        self.player.rating = p_rating
        
        if not self.baseline:
            games_played = self.player.games_played
            if games_played % 5**self.player.rank_updates == 0:
                self.update_opponent_ranking()
        self.update_database()
        return

    def update_ratings(self, r_1, r_2, outcome):
        r1_old = r_1
        r2_old = r_2
        
        if outcome == 1:
            r1_new, r2_new = rate_1vs1(r1_old, r2_old)
        elif outcome == -1:
            r2_new, r1_new = rate_1vs1(r2_old, r1_old)
        else:
            r1_new, r2_new = rate_1vs1(r1_old, r2_old, drawn=True)

        return r1_new, r2_new

    def update_opponent_ranking(self):
        # Fake games between opponents: we already know what the ordering should be
        if self.verbose:
            rospy.loginfo('Updating rankings...')
        couples = []
        couples.append((0, 1)) # Easy vs Medium
        couples.append((1, 2)) # Medium vs Hard
        couples.append((0, 2)) # Easy vs Hard

        ratings = [opp.rating for opp in self.parameter_sets]

        for c in couples:
            winner = c[1]
            loser = c[0]
            r0, r1 = rate_1vs1(ratings[winner], ratings[loser])
            
            ratings[winner] = r0
            ratings[loser] = r1
        
            self.parameter_sets[winner].rating = r0
            self.parameter_sets[loser].rating = r1

        self.player.rank_updates = self.player.rank_updates + 1
        self.update_database()
        return

    def update_database(self):
        # Updates the database with the current values of ratings
        self.db.update_player_tuple(self.player.id, self.player.rating.mu, self.player.rating.sigma, self.player.games_played, self.player.rank_updates)

        for opp in self.parameter_sets:
            self.db.update_robot_tuple(self.player.id, opp.level_id, opp.rating.mu, opp.rating.sigma)
        return

    def get_player_id(self):
        return self.player.id

class PlayerRatingWrapper(object):
    ID_IDX = 0
    NAME_IDX = 1
    SURNAME_IDX = 2
    MU_IDX = 3
    SIGMA_IDX = 4
    GAMES_PLAYED_IDX = 5
    RANK_UPDATES_IDX = 6

    def __init__(self, values):
        self.id = values[self.ID_IDX]
        self.name = values[self.NAME_IDX]
        self.surname = values[self.SURNAME_IDX]
        self.rating = Rating(values[self.MU_IDX], values[self.SIGMA_IDX])
        self.games_played = values[self.GAMES_PLAYED_IDX]
        self.rank_updates = values[self.RANK_UPDATES_IDX]


class OpponentRatingWrapper(object):
    ID_IDX = 0
    PLAYER_ID_IDX = 1
    LEVEL_ID_IDX = 2
    MU_IDX = 3
    SIGMA_IDX = 4

    def __init__(self, values):
        self.id = values[self.ID_IDX]
        self.player_id = values[self.PLAYER_ID_IDX]
        self.level_id = values[self.LEVEL_ID_IDX]
        self.rating = Rating(values[self.MU_IDX], values[self.SIGMA_IDX])


