# Calibration of model-based controller settings

import math, logging
import model


TARGET_IS_HIGHER = -1
TARGET_IS_LOWER = 1
EPSILON = 0.0005  # how closely to tune model parameters

# generator for binary search! initialize with bounds, and give feedback via
# send(TARGET_IS_LOWER/HIGHER)! Only the lower bound needs to be correct
def bin_search_float(lower, upper):
    # exponential growth first to find upper bound
    feedback = yield upper
    while feedback is TARGET_IS_HIGHER:
        lower = upper
        upper *= 2.0
        feedback = yield upper
    while abs(upper - lower) > EPSILON:
        current = (lower + upper) / 2.0
        feedback = yield current
        assert not feedback is None
        if feedback is TARGET_IS_HIGHER:
            lower = current
            continue
        if feedback is TARGET_IS_LOWER:
            upper = current
            continue


DELTA_T = 0.5  # how many seconds back / forward to seek for computing momentary values

class ShellCalibrate:
    cmd_MODEL_CALIBRATE_help = "Run calibration for model-based controller"
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'MODEL_CALIBRATE', self.cmd_MODEL_CALIBRATE,
            desc=self.cmd_MODEL_CALIBRATE_help)

    def cmd_MODEL_CALIBRATE(self, params):
        heater_name = self.gcode.get_str('HEATER', params)
        target = self.gcode.get_float('TARGET', params)
        write_file = self.gcode.get_int('WRITE_FILE', params, 1)
        pheater = self.printer.lookup_object('heater')
        try:
            heater = pheater.lookup_heater(heater_name)
        except self.printer.config_error as e:
            raise self.gcode.error(str(e))
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        calibrate = ControlAutoTune(heater, target)
        old_control = heater.set_control(calibrate)
        try:
            heater.set_temp(print_time, target)
        except heater.error as e:
            heater.set_control(old_control)
            raise self.gcode.error(str(e))
        self.gcode.bg_temp(heater)
        heater.set_control(old_control)
        if write_file:
            calibrate.write_file('/tmp/heattest.txt')
        try:  # TODO REMOVE
            config = calibrate.calc_params()

            logging.info("Autotune: model config: " + str(config))
            self.gcode.respond_info("Model parameters:\n"
                    + "\n".join(['model_' + setting + ": " + str(val) for setting, val in config.items()]))
            # Store results for SAVE_CONFIG
            configfile = self.printer.lookup_object('configfile')
            for key, val in config.items():
                setting = 'model_' + key
                configfile.set(heater_name, setting, str(val))
        except e:  # TODO REMOVE
            with open('/tmp/error','wb') as f:
                f.write(str(e))
            logging.error(e)
            raise e

class ControlAutoTune:
    # These are the variables we need to find
    # controller phases in order:
    phases = ['heatup', 'overshoot', 'cooldown', 'heatup_fan', 'overshoot_fan', 'cooldown_fan', 'done']

    def __init__(self, heater, target):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.calibrate_temp = target
        # Sample recording
        self.last_pwm = 0.
        self.pwm_samples = []
        self.timestamps = []
        self.raw_samples = []
        self.smoothed_samples = []
        self.phase_start = {}
        self.phase = 'heatup'
        self.fan_phase = False

    # Heater control
    def set_pwm(self, read_time, value):
        if value != self.last_pwm:
            self.pwm_samples.append(
                (read_time + self.heater.get_pwm_delay(), value))
            self.last_pwm = value
        self.heater.set_pwm(read_time, value)

    def temperature_update(self, read_time, temp, target_temp):
        if self.raw_samples:
            last_temp = self.raw_samples[-1]
        else:
            last_temp = temp
            self.env_temp = temp
        self.timestamps.append(read_time)
        self.raw_samples.append(temp)
        suffix = '_fan' if self.fan_phase else ''

        # control code for phases should be listed in reverse order to prevent skipping a phase
        if self.phase == 'cooldown_fan' and temp < target_temp:
            self.heater.alter_target(0.0)
            # accessing the gcode object through this ref and then calling an internal method feels very hacky :\
            self.heater.gcode._set_fan_speed(0.0)
            self.phase = 'done'
        elif self.phase == 'cooldown' and temp < target_temp:
            self.heater.alter_target(self.calibrate_temp)
            self.heater.gcode._set_fan_speed(1.0)
            self.phase = 'heatup_fan'
            self.fan_phase = True
        elif self.phase.startswith('overshoot') and temp < last_temp:
            self.phase = 'cooldown' + suffix
        elif self.phase.startswith('heatup') and temp >= target_temp:
            self.phase = 'overshoot' + suffix
            self.heater.alter_target(self._cooldown_target())

        if self.phase.startswith('heatup'):
            self.set_pwm(read_time, self.heater_max_power)
        else:
            self.set_pwm(read_time, 0.)

        # TODO: phases need to be determined AFTER smoothing
        if self.phase not in self.phase_start:
            self.phase_start[self.phase] = len(self.raw_samples)-1

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return not self.phase == 'done'

    # Offline analysis helpers
    def write_file(self, filename):
        pwm = ["pwm: %.3f %.3f" % (time, value)
               for time, value in self.pwm_samples]
        out = ["%.3f %.3f" % (self.timestamps[idx], self.raw_samples[idx]) for idx in range(len(self.timestamps))]
        phases = ["phase %s start: %d" % (p, t) for p, t in self.phase_start.items()]
        f = open(filename, "wb")
        f.write('\n'.join(pwm + out + phases))
        f.close()

    def from_file(self, filename='heattest.txt'):
        self.pwm_samples = []
        self.timestamps = []
        self.raw_samples = []
        self.phase_start = {}
        self.phase = 'done'
        with open(filename, "rb") as f:
            for line in f.readlines():
                line = line.decode('utf-8').strip()
                if line.startswith('pwm: '):
                    _, time, val = line.split(' ')
                    self.pwm_samples.append((float(time), float(val)))
                elif line.startswith('phase '):
                    _, phase, _, idx = line.split(' ')
                    self.phase_start[phase] = int(idx)
                else:
                    time, temp = line.split(' ')
                    self.timestamps.append(float(time))
                    self.raw_samples.append(float(temp))
        self.smoothed_samples = self._smooth(self.raw_samples)

    def _lerp(self, a, b, alpha):
        return a + alpha * (b - a)

    def _replicate_curve(self, model_config, start_idx, end_idx, fan_power=0):
        # Creates a model with the given config, and attempts to replicate the temperature
        # curve between (start_idx, end_idx) in smoothed_samples
        # pads the resulting list with start_idx many None values to make index calculation easier

        overwritten = model_config['initial_temp']
        model_config['initial_temp'] = self.smoothed_samples[start_idx]

        m = model.Model(**model_config)
        time = self.timestamps[start_idx]
        pwm_idx = 0
        model_temp_samples = ([None] * start_idx) + [self.smoothed_samples[start_idx]]
        # pwm_idx is now the index of the last pwm_sample before current time, i.e. the active decision
        for tick in range(start_idx+1, end_idx+1):
            # Find the heater output decision that's relevant to know
            while True:
                if pwm_idx+1 < len(self.pwm_samples) and self.pwm_samples[pwm_idx+1][0] < time:
                    pwm_idx += 1
                else:
                    break

            time = self.timestamps[tick]
            dt = self.timestamps[tick] - self.timestamps[tick-1]
            new_temp = m.advance_model(dt, self.pwm_samples[pwm_idx][1], fan_power)
            model_temp_samples.append(new_temp)
        model_config['initial_temp'] = overwritten
        return (m, model_temp_samples)

    def _deriv_at(self, idx):
        before_idx = idx
        after_idx = idx
        global DELTA_T
        while before_idx > 0 and self.timestamps[idx] - self.timestamps[before_idx] < DELTA_T:
            before_idx -= 1
        while after_idx < len(self.timestamps)-1 and self.timestamps[after_idx] - self.timestamps[idx] < DELTA_T:
            after_idx += 1
        # before_idx and after_idx are now indices at least DELTA_T away from idx time
        before = (self.timestamps[before_idx], self.smoothed_samples[before_idx])
        after = (self.timestamps[after_idx], self.smoothed_samples[after_idx])
        now = (self.timestamps[idx], self.smoothed_samples[idx])

        deriv_before = (now[1] - before[1]) / (now[0] - before[0])
        deriv_after = (after[1] - now[1]) / (after[0] - now[0])
        alpha = (now[0] - before[0]) / (after[0] - before[0])

        return self._lerp(deriv_before, deriv_after, alpha)

    def _find_temp(self, temp, phase='cooldown'):
        start_idx, end_idx = self._get_index_range(phase)
        best = start_idx
        best_error = 100
        for idx in range(start_idx, end_idx):
            error = abs(self.smoothed_samples[idx] - temp)
            if error < best_error:
                best = idx
                best_error = error
        return best

    def calc_params(self):
        self.env_temp = self.smoothed_samples[0]
        config = self._fit_model()
        # we need to derive the internal hotend gradients. These are emergent properties,
        # not model parameters

        # start by initializing the model to 200 degrees, and run it a handfull of seconds
        # always putting back in what is lost through convection
        tmp_cnf = config.copy()
        tmp_cnf['initial_temp'] = self.calibrate_temp
        tmp_model = model.Model(**tmp_cnf)
        initial_energy = sum(tmp_model.cells[:-1])
        # simulate for a total of 0.2s * 500 = 10s
        for tick in range(500):
            tmp_model.advance_model(dt=0.2, heater_pwm_until_now=0.0)
            loss = initial_energy - sum(tmp_model.cells[:-1])
            tmp_model.cells[0] += loss
        # The value we actually care about is how much the controller needs to offset its target
        # this value scales with temperature differential at the hotend
        config['steadystate_offset_base'] = (self.calibrate_temp - tmp_model.cells[-2]) / (self.calibrate_temp - self.env_temp)
        # ...and do the whole dance again, but with fans on
        for tick in range(500):
            tmp_model.advance_model(dt=0.2, heater_pwm_until_now=0.0, fan_power=1.0)
            loss = initial_energy - sum(tmp_model.cells[:-1])
            tmp_model.cells[0] += loss
        config['steadystate_offset_fans'] =  (self.calibrate_temp - tmp_model.cells[-2]) / (self.calibrate_temp - self.env_temp)
        return config

    def _cooldown_target(self):
        return int(self.env_temp + 15)

    def _cooling_curve(self, phase):
        temp_range = range(self.calibrate_temp, self._cooldown_target()-1,-1)
        measured_derivs = [self._deriv_at(self._find_temp(i, phase)) for i in temp_range]
        smoothed_derivs = self._smooth(measured_derivs)
        # we're messing with temperature *differentials* here
        temp_range = [t - self.env_temp for t in temp_range]

        # poor man's linear regression: pick 2 points, fit to those
        x1 = temp_range[10]
        y1 = smoothed_derivs[10]  # the first few will be skewed due to smoothing...
        x2 = temp_range[-20]
        y2 = smoothed_derivs[-20]  # as will the last few. The high end of the temperature scale
        # also sees less cooling since the data came from a moment when the hotend might not have
        # thermally equalized yet

        a = (y2-y1)/(x2-x1)
        b = y1 - (a*x1)
        # cooling is the function a*(t-env_temp)+b
        return a, b

    def _fit_model(self):
        # Order of calibration:
        #   1. heater strength (initial guess, fit to compensated)
        #   2. thermal mass (fit to compensated)
        #   3. base_cooling (fit to smoothed)
        #   4. heater strength (fit to smoothed)

        heat_start, heat_stop = self._get_index_range('heatup')
        cool_start, cool_end = self._get_index_range('cooldown')
        heat_fan_start, heat_fan_stop = self._get_index_range('heatup_fan')
        cool_fan_start, cool_fan_end = self._get_index_range('cooldown_fan')

        # we'll measure passive convective cooling during the cooldown phase
        # and compensate our input data for the energy lost.
        # the resulting temp data allows us to get a good guess at heater power
        a, b = self._cooling_curve('cooldown')
        compensated_temps = [self.smoothed_samples[0]]
        total_loss = 0
        for idx in range(heat_start+1, cool_end+1):
            t = self.smoothed_samples[idx]
            new_temp = max(compensated_temps[-1], t-total_loss)  # preclude occasional modeling errors
            compensated_temps.append(new_temp)
            dt = self.timestamps[idx+1] - self.timestamps[idx]
            loss = dt * (a*(t - self.env_temp) + b)
            total_loss += loss

        config = {
            'thermal_conductivity': 0.4,
            'initial_temp': self.smoothed_samples[heat_start],
            'env_temp': self.env_temp,
            'base_cooling': 0.0,
            'fan_cooling': 0.0
            }

        # We'll use binary search for every parameter. The following does the lifting for that,
        # given the initial bounds of the search, the name of the parameter to fit, and a signed error function
        def binsearch_param(bounds, param, error_fn, start, end, fan_power=0.0):
            binsrch = bin_search_float(*bounds)
            curval = next(binsrch)
            try:
                while True:
                    config[param] = curval
                    m, model_samples = self._replicate_curve(config, start, end, fan_power)
                    if param not in ['heater_power', 'thermal_conductivity', 'base_cooling']:
                        self._plot_candidate(model_samples[start:end], start, end-1)
                    error = error_fn(model_samples)
                    if error == 0:
                        break
                    curval = binsrch.send(TARGET_IS_HIGHER if error > 0 else TARGET_IS_LOWER)
            except StopIteration:
                pass
            return curval
        heater_power = binsearch_param((0,100), 'heater_power',
                lambda mdl: compensated_temps[cool_end-1] - mdl[cool_end-1], heat_start, cool_end)
        print('power done')

        # fitting for thermal conductivity
        fit_pivot = (heat_start + cool_start) // 2
        def thermal_mass_error(mdl):
            idx = self._find_temp(mdl[fit_pivot], 'heatup')
            if fit_pivot == idx:
                print("found ideal thermal mass")
            return fit_pivot - idx

        # alternative: sum of errors during heatup, with ramp-up and wind-down being weighted opposite
        # def thermal_mass_error(mdl):
        #     error = 0
        #     for i in range(fit_start, fit_pivot):
        #         error += mdl[i] - compensated_temps[i]
        #     for i in range(fit_pivot, fit_end):
        #         error += compensated_temps[i] - mdl[i]
        #     return error
        th_conduct = binsearch_param((0,1.0), 'thermal_conductivity', thermal_mass_error, heat_start, heat_stop)
        print('conduct done')

        def cooling_error(mdl):
            # We can't completely isolate cooling and heater_power, since our model of cooling
            # will be slightly off. We get around this by using our (very) educated guess
            # of heater power, and compensating for slight scaling misalignment here
            model_peak = max(*enumerate(mdl), key=lambda s: s[1] if not s[1] is None else 0)
            scale = self.smoothed_samples[cool_start] / model_peak[1]
            if scale > 1.3:
                # too off, try again
                scale = 1.0
            error = 0
            for i in range(cool_start, cool_end):
                error += mdl[i] * scale - self.smoothed_samples[i]
            return error
        cooling = binsearch_param((0,1.0), 'base_cooling', cooling_error, heat_start, cool_end)

        # we're almost done, do one more round of fitting for heater power to vertically align peaks
        binsearch_param((0,100), 'heater_power', lambda mdl: self.smoothed_samples[cool_start] - mdl[cool_start], heat_start, cool_start)
        print('old done')

        # TODO tune fan_cooling
        def fan_cooling_error(mdl):
            # We can't completely isolate cooling and heater_power, since our model of cooling
            # will be slightly off. We get around this by using our (very) educated guess
            # of heater power, and compensating for slight scaling misalignment here
            model_peak = max(*enumerate(mdl), key=lambda s: s[1] if not s[1] is None else 0)
            error = 0
            for i in range(cool_fan_start, cool_fan_end):
                error += mdl[i] - self.smoothed_samples[i]
            return error
        cooling = binsearch_param((0,1.0), 'fan_cooling', fan_cooling_error, heat_fan_start, cool_fan_end, fan_power=1.0)

        _, msamples = self._replicate_curve(config, heat_start, cool_end, fan_power=0.0)
        _, fansamples = self._replicate_curve(config, heat_fan_start, cool_fan_end, fan_power=1.0)
        self._plot_candidate(msamples[heat_start:cool_end] + fansamples[heat_fan_start:cool_fan_end], heat_start, cool_fan_end-1)

        return config

    def _plot_candidate(self, samples, _from, to):
        import matplotlib.pyplot as plt
        plt.plot(self.timestamps, self.raw_samples, label='measured [raw]')
        plt.plot(self.timestamps[_from:to+1], samples, label='model prediction')
        l = [i for i in range(int(self.pwm_samples[0][0]),int(self.pwm_samples[1][0]))]
        plt.bar(l, [200 for _ in l], color="#aaaaaa40", width=1.0, label='Heater turned on')
        plt.legend(loc='upper left')
        plt.show()

    def _smooth(self, samples):
        # The thermistor on my printer has a noise amplitude of about +- 0.4 degrees,
        # which is higher than some of the derivatives we want to measure, so we'll
        # need some good smoothing of our data sets
        from scipy import signal
        window_length = min(100, max(20, len(samples)//5))
        if window_length % 2 == 0:
            window_length += 1
        return signal.savgol_filter(samples, window_length, 3)

    def _get_index_range(self, phase):
        start_idx = self.phase_start[phase]
        end_idx = self.phase_start[ self.phases[self.phases.index(phase)+1] ]
        return start_idx, end_idx


def load_config(config):
    return ShellCalibrate(config)

def get(filename='heattest_200'):
    class FHeater:
        def get_max_power(self):
            return 1.0
    c = ControlAutoTune(FHeater(), 200)
    c.from_file(filename)
    return c
