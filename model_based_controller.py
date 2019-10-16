import model

def clamp(value, lower, upper):
    return max(lower, min(upper, value))

class ModelBasedController(object):
    # internally models hotend as made of cells of metal surrounded by air
    # heater is in innermost shell (0), sensor in outermost metal shell
    # heat gradients are smoothed, w/ different conductivity b/w air & metal

    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.config = config

        # heater power in degrees per second
        self.heater_output = config.getfloat('model_heater_power', minval=0)
        # number of cells in thermal simulation
        metal_cells = config.getint('model_metal_cells', 6, minval=2)
        # minimum number of simulation passes per second
        passes_per_sec = config.getint('model_passes_per_sec', 3)
        # heat dissipation rate within metal
        thermal_conductivity = config.getfloat('model_thermal_conductivity', minval=0, maxval=1)
        # heat - air dissipation rate
        base_cooling = config.getfloat('model_base_cooling', minval=0, maxval=1)
        # additional heat dissipation effected by fan at 100%
        fan_cooling = config.getfloat('model_fan_cooling', base_cooling, minval=0, maxval=(1-base_cooling))
        # TODO get rid of initial_temp and env_temp
        initial_temp = config.getfloat('model_initial_temp', 21.4, minval=0)
        self.internal_gradient = config.getfloat('model_internal_gradient', 0)

        self.model = model.Model(self.heater_output, initial_temp, thermal_conductivity, base_cooling, fan_cooling, initial_temp, metal_cells, passes_per_sec)
        self.current_heater_pwm = 0.0
        # we keep a tally of how long on avg a control tick lasts
        self.last_read_times = [-4, -3, -2, -1]

    def stable_state_gradient(self):
        # TODO: start by initializing the model to 200 degrees, and run it a handfull of seconds
        # always putting back in what is lost through convection
        return self.internal_gradient

    def temperature_update(self, read_time, temp, target_temp):
        self.model.advance_model(read_time - self.last_read_times[-1], self.current_heater_pwm)
        self.model.adjust_to_measurement(temp)

        # TODO: Is this really needed?
        self.last_read_times.append(read_time)
        self.last_read_times = self.last_read_times[1:]
        tick_lens = [self.last_read_times[i] - self.last_read_times[i-1] for i in range(1, len(self.last_read_times))]
        tick_len = sum(tick_lens) / len(tick_lens)

        # now calculate how much heat we still need to dump into the hotend
        # TODO also add expected thermal egress
        model_avg_temp = sum(self.model.cells[:-1]) / (len(self.model.cells)-1)
        degrees_needed = (target_temp - model_avg_temp \
                # since we're constantly losing heat, there is always an internal gradient in the hotend.
                # if we don't compensate for this, we'll have a steady state error
                + self.stable_state_gradient() / 2  \
                ) * (len(self.model.cells)-1)
        # if the expected tick length is long, we need less heat
        self.current_heater_pwm = clamp(degrees_needed / (self.heater_output * tick_len), 0.0, self.heater_max_power)
        self.heater.set_pwm(read_time, self.current_heater_pwm)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return abs(smoothed_temp - target_temp) < 15


