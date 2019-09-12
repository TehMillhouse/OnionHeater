from common import *

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

class OnionController(object):
    # internally models hotend as made of shells of metal surrounded by air
    # heater is in innermost shell (0), sensor in outermost metal shell
    # heat gradients are smoothed, w/ different conductivity b/w air & metal

    def __init__(self, shells, power, dissipation_passes=2, history_length=None, initial_temp=ENV_TEMP):
        self.power = power
        # heater is at first (innermost) shell, sensor at second-to-last shell, outermost shell is outside
        self.shells = [initial_temp] * shells
        self.dissipation_passes = dissipation_passes
        if history_length is None:
            history_length = 2*shells
        self.history = History([ HistItem(-history_length + i + 1, initial_temp, 0, list(self.shells)) for i in reversed(range(history_length)) ])
        self.target = initial_temp
        self.time = 0

    def set_target(self, target):
        self.target = target

    def extrapolate(self, source_time, target_time, cooling_factor):
        # take source_time history item as ground truth, ignore rest of history since
        hist_idx = source_time - self.time
        ground_truth = self.history[hist_idx]
        heater_output = ground_truth.heater_output
        # ground the ground truth in... truth? Anyway, scale the model so it confirms to reality
        error = ground_truth.sensor_temp - ground_truth.shells[-2]
        if abs(error) != 0:
            print(f"Significant divergence of {error} degrees from model at time {source_time}")
        shells = list(ground_truth.shells)
        shells[-2] = ground_truth.sensor_temp
        prediction = []
        # todo: cooling factor
        egress = self.estimate_egress(hist_idx)  # extrapolate based on constant egress
        egress_delta_t = shells[-2] - shells[-1]  # amount of convective cooling is ~linear in temp difference
        for tick in range(target_time - source_time):
            shells[0] += heater_output * self.power

            egress_factor = ((shells[-2] - shells[-1]) / egress_delta_t) if egress_delta_t != 0 else 1
            shells[-2] -= egress * egress_factor
            for _ in range(self.dissipation_passes):
                self.dissipate_temps(shells, ground_truth.cooling_factor)
            heater_avg_temp = sum(shells[:-1]) / (len(shells)-1)
            # How much should we heat? Since we have a full thermal model, we could just heat just enough
            # to bringt the average temperature to setpoint. However! Since we're constantly losing heat
            # through convection, there will always be a temperature gradient in the hotend. Since we
            # want *the sensor* to be at setpoint, that won't be enough.
            degrees_needed = (self.target + egress * egress_factor - heater_avg_temp) * len(self.shells)
            heater_output = clamp( degrees_needed / self.power, 0.0, 1.0)
            prediction.append(HistItem(source_time+tick+1, None, heater_output, list(shells)))
        return prediction

    def estimate_egress(self, hist_idx):
        # how much energy are we losing at a given time through convection, filament, etc.?
        # This is useful to know because we can assume that we will lose about as much in the future
        # It also lets us rapidly correct for sudden gusts of wind, etc.
        egress = 0.0
        # we attribute to thermal egress whatever energy loss *has been measured* that is *not* accounted for
        # in our model of thermal dissipation
        for i in range(WINDOW_SIZE):
            h1 = self.history[hist_idx - i - 1]
            h2 = self.history[hist_idx - i]
            measured_loss = h1.sensor_temp - h2.sensor_temp
            predicted_loss = h1.shells[-2] - h2.shells[-2]
            print(f"m: {measured_loss}     p: {predicted_loss} --> egress {measured_loss - predicted_loss}")
            if measured_loss - predicted_loss > 0:
                egress += measured_loss - predicted_loss
        return egress / WINDOW_SIZE

    def record_sample(self, temp, cooling_factor=1.0):
        action = self.extrapolate(self.time, self.time+1, cooling_factor)[0]
        self.time += 1
        # record = HistItem(self.time, temp, None, list(self.shells), cooling_factor)
        action.sensor_temp = temp
        self.history.record_new(action)

        self.shells = action.shells
        return action.heater_output

    def get_decision(self):
        assert not self.history[0].heater_output is None
        return self.history[0].heater_output

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
            #else:
            #    # heat exchange between metal and cooling air
            #    shells[hi] -= env_cooling_factor * HEAT_CONDUCT_AIR * temp_diff
            #    shells[lo] += env_cooling_factor * HEAT_CONDUCT_AIR * temp_diff
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
