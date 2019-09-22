import model
from common import *

class HeatController(object):
    # internally models hotend as made of shells of metal surrounded by air
    # heater is in innermost shell (0), sensor in outermost metal shell
    # heat gradients are smoothed, w/ different conductivity b/w air & metal

    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.config = config

        # heater power in degrees per second
        self.heater_output = config.getfloat('model_heater_power', 1.68, minval=0)
        # number of cells in thermal simulation
        metal_shells = config.getint('model_thermal_mass', 6, minval=2)
        # minimum number of simulation passes per second
        passes_per_sec = config.getfloat('model_accuracy', 1)
        # heat dissipation rate within metal
        thermal_conductivity = config.getfloat('model_thermal_conductivity', 0.15, minval=0, maxval=1)
        # heat - air dissipation rate
        base_cooling = config.getfloat('model_air_cooling', 0.00943445, minval=0, maxval=1)
        # additional heat dissipation effected by fan at 100%
        fan_cooling = config.getfloat('model_fan_cooling', base_cooling, minval=0, maxval=(1-base_cooling))
        initial_temp = 21.5

        self.model = model.Model(self.heater_output, metal_shells, passes_per_sec, initial_temp, thermal_conductivity, base_cooling, fan_cooling)
        self.current_heater_pwm = 0.0
        # we keep a tally of how long on avg a control tick lasts
        self.last_read_times = [-4, -3, -2, -1]

    def stable_state_gradient(self):
        # TODO: start by initializing the model to 200 degrees, and run it a handfull of seconds
        # always putting back in what is lost through convection
        return 27

    def temperature_update(self, read_time, temp, target_temp):
        self.model.advance_model(read_time, self.current_heater_pwm)
        self.model.adjust_to_measurement(temp)

        # TODO: Is this really needed?
        self.last_read_times.append(read_time)
        self.last_read_times = self.last_read_times[1:]
        tick_lens = [self.last_read_times[i] - self.last_read_times[i-1] for i in range(1, len(self.last_read_times))]
        tick_len = sum(tick_lens) / len(tick_lens)

        # now calculate how much heat we still need to dump into the hotend
        # TODO also add expected thermal egress
        model_avg_temp = sum(self.model.shells[:-1]) / (len(self.model.shells)-1)
        degrees_needed = (target_temp - model_avg_temp \
                # since we're constantly losing heat, there is always an internal gradient in the hotend.
                # if we don't compensate for this, we'll have a steady state error
                + self.stable_state_gradient() / 2  \
                ) * (len(self.model.shells)-1)
        # if the expected tick length is long, we need less heat
        self.current_heater_pwm = clamp(degrees_needed / (self.heater_output * tick_len), 0.0, self.heater_max_power)
        self.heater.set_pwm(read_time, self.current_heater_pwm)

    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return abs(smoothed_temp - target_temp) < 15


class ControlAutoTune(object):
    def __init__(self, heater, target):
        pass
