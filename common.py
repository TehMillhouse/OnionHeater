
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

def plot(trace, pwm_output):
    import matplotlib.pyplot as plt
    # trace = list of tuples, one for each shell
    shells = len(trace[0])
    time = [ i * TICK_LEN for i in range(len(trace))]
    for i in range(shells):
        if i == shells-2:
            continue
        shell = [blip[i] for blip in trace]
        plt.plot(time, shell, label='c. shell ' + str(i), linestyle='--')
    plt.plot(time, [blip[-2] for blip in trace], label='Sensor temp')
    plt.bar(time, [y * 100 for y in pwm_output], color="#aaaaaa20", width=0.25, label='Heater power')
    plt.legend(loc='upper left')
    plt.show()
