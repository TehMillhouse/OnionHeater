
# I measured these values at home, in my room, on my printer. YMMV

TICK_LEN = 0.833
# TICK_LEN = 0.5  # length of one time tick, in seconds
ENV_TEMP = 21.4
WINDOW_SIZE = 2  # number of ticks to use for estimation of current thermal egress

HEAT_CONDUCT_METAL = 0.05  # dunno lol, how do I measure this?
HEAT_CONDUCT_AIR = 0.0043445  # degree per tick for every degree of temperature differential
# HEAT_CONDUCT_AIR doesn't seem to behave right...

# HEAT_CONDUCT_AIR = 0.4 / 200
HEATER_POWER = 2.0166  # degrees celsius / s
NOISE_AMP = 0.2


def clamp(value, lower, upper):
    return max(lower, min(upper, value))

