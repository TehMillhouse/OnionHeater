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
        self.heater_output = config.getfloat('model_heater_power', minval=0.0)
        # number of cells in thermal simulation
        metal_cells = config.getint('model_metal_cells', 6, minval=2)
        # heat dissipation rate within metal
        thermal_conductivity = config.getfloat('model_thermal_conductivity', minval=0.0, maxval=1.0)
        # heat - air dissipation rate
        base_cooling = config.getfloat('model_base_cooling', minval=0.0, maxval=1.0)
        # TODO get rid of initial_temp and env_temp
        initial_temp = config.getfloat('model_initial_temp', 21.4, minval=0.0)
        env_temp = config.getfloat('model_env_temp', 21.4, minval=0.0)
        self.internal_gradient = config.getfloat('model_steadystate_offset_base', 0.0)

        self.model = model.Model(self.heater_output, initial_temp, thermal_conductivity, base_cooling, metal_cells)
        self.current_heater_pwm = 0.0
        # we keep a tally of how long on avg a control tick lasts
        self.last_read_times = [-4, -3, -2, -1]

    def stable_state_offset(self, target_temp):
        # internal gradients require us to target a higher temperature, which in turn
        # increases the internal gradient! Since the internal gradients in an autotuned
        # model can get quite big, we need to solve this problem

        # Due to the nonlinearity of hotends, we can't know in advance how high this will end up being,
        # but we need some sort of prior / estimation
        est_gradient = self.model.egress_p_sec * (len(self.model.cells)-1) / 2.0
        # est_gradient = (self.model.avg_energy() - self.model.cells[-2]) / (self.model.cells[-2] - self.model.env_temp) * (target_temp - self.model.env_temp)

        # we have
        #     orig_trg = new_trg - (new_trg - env_temp) * gradient
        # <=> new_trg = orig_trg - (env_temp * gradient) / (1 - gradient)
        # Solving for new_trg and subtracting the old target temp yields
        return (target_temp - self.model.env_temp * est_gradient) / (1 - est_gradient) - target_temp


    def temperature_update(self, read_time, temp, target_temp):
        self.model.advance_model(read_time - self.last_read_times[-1], self.current_heater_pwm, temp)

        # TODO: Is this really needed?
        self.last_read_times.append(read_time)
        self.last_read_times = self.last_read_times[1:]
        tick_lens = [self.last_read_times[i] - self.last_read_times[i-1] for i in range(1, len(self.last_read_times))]
        tick_len = sum(tick_lens) / len(tick_lens)

        # now calculate how much heat we still need to dump into the hotend
        degrees_needed = (target_temp - self.model.avg_energy() \
                # since we're constantly losing heat, there is always an internal gradient in the hotend.
                # if we don't compensate for this, we'll have a steady state error
                + self.stable_state_offset(target_temp) \
                + self.model.egress_p_sec  \
                ) * (len(self.model.cells)-1)
        # if the expected tick length is long, we need less heat
        self.current_heater_pwm = clamp(degrees_needed / (self.heater_output * tick_len), 0.0, self.heater_max_power)
        self.heater.set_pwm(read_time, self.current_heater_pwm)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return abs(smoothed_temp - target_temp) > 7


