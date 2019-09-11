import matplotlib.pyplot as plt

TICK_LEN = 0.5  # length of one time tick, in seconds
ENV_TEMP = 23.0  # temperature of the environment, cf. newton's law of cooling
WINDOW_SIZE = 8  # number of ticks to use for estimation of current thermal egress

HEAT_CONDUCT_METAL = 0.4
HEAT_CONDUCT_AIR = 0.4 / 200

class Sim(object):
    def __init__(self, target, shells, power, smoothing_passes):
        self.controller = OnionController(shells, power, smoothing_passes)
        self.controller_decisions = [0.0]
        # heater is at first (innermost) shell, sensor at second-to-last shell, outermost shell is outside
        self.temp_shells = [ENV_TEMP] * shells
        self.data = []
        for i in range(shells):
            self.data.append(list([ENV_TEMP]))
        self.controller.set_target(target)

    def dissipate_temps(self):
        # heat dissipation within metal
        for i in range(len(self.temp_shells)-1):
            if self.temp_shells[i] > self.temp_shells[i+1]:
                hi = i
                lo = i+1
            else:
                hi = i+1
                lo = i
            temp_diff = self.temp_shells[hi] - self.temp_shells[lo]
            # different heat exchange factors for metal/metal and metal/air
            if i < len(self.temp_shells)-2:
                self.temp_shells[hi] -= HEAT_CONDUCT_METAL * temp_diff
                self.temp_shells[lo] += HEAT_CONDUCT_METAL * temp_diff
            else:
                # heat exchange between metal and cooling air
                self.temp_shells[hi] -= HEAT_CONDUCT_AIR * temp_diff
                self.temp_shells[lo] += HEAT_CONDUCT_AIR * temp_diff
        # the outermost shell is always at ENV_TEMP
        self.temp_shells[-1] = ENV_TEMP

    def simulate_tick(self):
        cont = self.controller
        cont.record_sample(self.sensor_temp())
        self.temp_shells[0] += cont.get_decision() * cont.power
        self.controller_decisions.append(cont.get_decision())
        self.dissipate_temps()
        self.dissipate_temps()
        for i in range(len(self.temp_shells)):
            self.data[i].append(self.temp_shells[i])

    def simulate_ticks(self, n):
        for _ in range(n):
            self.simulate_tick()

    def plot(self):
        time = [ i * TICK_LEN for i in range(len(self.data[0]))]
        for i, shell in enumerate(self.data):
            plt.plot(time, shell, label='shell ' + str(i))
        # plt.plot(time, self.heater_data, label='Heater')
        plt.bar(time, [y * 100 for y in self.controller_decisions], color="#aaaaaa20", width=0.25, label='Heater power')
        plt.legend(loc='upper left')
        plt.show()

    def sensor_temp(self):
        return self.temp_shells[-2]

class History(object):
    # 0 is present, -1 is one tick ago, etc.
    def __init__(self, _list):
        # _list is expected to already be the right length
        # input: idx 0 is newest, 1 is one tick ago, etc.
        self._oldest_idx = len(_list) - 1
        self._buf = list(_list)

    def record_new(self, item):
        self._buf[self._oldest_idx] = item
        self._oldest_idx = (self._oldest_idx-1) % len(self._buf)

    def __getitem__(self, idx):
        # semantics of this: 0 is newest, -1 is one tick ago, -1 is two ticks ago, etc.
        idx *= -1
        # want newest: 0, second-newest: 1, etc.
        return self._buf[(self._oldest_idx+1+idx) % len(self._buf)]

    def __setitem__(self, idx, data):
        assert(idx == 0)  # we can't change the past
        self._buf[(self._oldest_idx+1) % len(self._buf)] = data

    def __repr__(self):
        return self._buf.__str__()

class HistItem(object):
    def __init__(self, time, sensor_temp, heater_output, shells, cooling_factor=1.0):
        self.time           = time
        self.sensor_temp    = sensor_temp
        self.heater_output  = heater_output
        self.shells         = shells
        self.cooling_factor = cooling_factor

def clamp(value, lower, upper):
    return max(lower, min(upper, value))

class OnionController(object):
    # internally models hotend as made of shells of metal surrounded by air
    # heater is in innermost shell (0), sensor in outermost metal shell
    # heat gradients are smoothed, w/ different conductivity b/ween air & metal

    def __init__(self, shells, power, smoothing_passes=2, history_length=None, initial_temp=ENV_TEMP):
        self.power = power
        # heater is at first (innermost) shell, sensor at second-to-last shell, outermost shell is outside
        self.shells = [initial_temp] * shells
        self.smoothing_passes = smoothing_passes
        if history_length is None:
            history_length = 2*shells
        self.history = History([ HistItem(i, initial_temp, 0, list(self.shells)) for i in range(history_length) ])
        self.target = initial_temp
        self.time = history_length

    def set_target(self, target):
        self.target = target

    def extrapolate(self, source_time, target_time):
        # take source_time history item as ground truth, ignore rest of history since
        hist_idx = source_time - self.time
        ground_truth = self.history[hist_idx]
        heater_output = ground_truth.heater_output
        # ground the ground truth in... truth? Anyway, scale the model so it confirms to reality
        scale = ground_truth.sensor_temp / ground_truth.shells[-2]
        shells = [scale * temp for temp in ground_truth.shells]
        prediction = []
        for tick in range(target_time - source_time):
            shells[0] += heater_output * self.power
            # todo cooling factor?
            for _ in range(self.smoothing_passes):
                self.dissipate_temps(shells, ground_truth.cooling_factor)
            heater_avg_temp = sum(shells[:-1]) / (len(shells)-1)
            heater_output = clamp( (self.target - heater_avg_temp) / self.power, 0.0, 1.0)
            prediction.append(HistItem(source_time+tick+1, None, heater_output, list(shells)))
        return prediction

    def record_sample(self, temp, cooling_factor=1.0):
        self.time += 1
        record = HistItem(self.time, temp, 0.0, list(self.shells), cooling_factor)
        self.history.record_new(record)

    # precondition: record_sample has been run this tick
    def get_decision(self):
        action = self.extrapolate(self.time, self.time+1)[0]
        self.history[0].shells = action.shells
        self.shells = action.shells
        self.history[0].heater_output = action.heater_output
        return action.heater_output

    def dissipate_temps(self, shells, env_cooling_factor=1.0):
        # heat dissipation within metal
        for i in range(len(shells)-1):
            if shells[i] > shells[i+1]:
                hi = i
                lo = i+1
            else:
                hi = i+1
                lo = i
            temp_diff = shells[hi] - shells[lo]
            # different heat exchange factors for metal/metal and metal/air
            if i < len(shells)-2:
                shells[hi] -= HEAT_CONDUCT_METAL * temp_diff
                shells[lo] += HEAT_CONDUCT_METAL * temp_diff
            else:
                # heat exchange between metal and cooling air
                shells[hi] -= env_cooling_factor * HEAT_CONDUCT_AIR * temp_diff
                shells[lo] += env_cooling_factor * HEAT_CONDUCT_AIR * temp_diff
        # the outermost shell is always at ENV_TEMP
        shells[-1] = ENV_TEMP


# Unsolved questions:
# How do we autotune this monster?
# Suppose heater wattage is known
# * First: measure env_temp
# * passive Convective cooling:
#       set heater to 30%, wait till it reaches equilibrium. Now, input = convective losses
# * heater power estimation (bootstrapping?)
#       First, one long burn, find peak, measure angle of climb (heater power)
#       After burn: measure falloff angle (air dissipation) (<-- does this need to be temperature delta dependent?)
# * number of shells / metal heat conductivity:
#    heat with a square wave, so the temp graph becomes staircase
#    predict whole graph, scale it so the peaks are aligned
#    choose number of shells for best fit
# How do we calculate thermal egress?
# How do I prevent drift?
