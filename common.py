
TICK_LEN = 0.5  # length of one time tick, in seconds
ENV_TEMP = 23.0  # temperature of the environment, cf. newton's law of cooling
WINDOW_SIZE = 1  # number of ticks to use for estimation of current thermal egress

HEAT_CONDUCT_METAL = 0.4
HEAT_CONDUCT_AIR = 0.4 / 200

def clamp(value, lower, upper):
    return max(lower, min(upper, value))

