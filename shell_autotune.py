# Calibration of model-based controller settings

import math
import model

import matplotlib.pyplot as plt

class Trace:
    def __init__(self):
        self.samples = []
        self.phases = {}

    def record_sample(self, time, temp):
        self.samples.append( (time, temp) )

    def ent_temp(self):
        return self.samples[0][1]

    def _find_sample(self, phase=None, target_temp=None, target_time=None):
        if phase is None:
            lower = 0
            upper = len(self.samples)-1
        else:
            #  TODO phases?
            pass
        if not target_temp is None:
            target = target_temp
            target_idx = 1

        while upper - lower >= 1:
            # prevent returning the wrong idx
            if self.samples[upper][target_idx] <= target:
                return upper
            middle = lower + (upper-lower)//2
            if not target_temp is None:
                setlower = target_temp > self.samples[middle][1]
            if setlower:
                if lower == middle:
                    # target is between two samples, return lower
                    return lower
                lower = middle
            else:
                upper = middle
        return lower

    def temp_at(self, time):
        # binsearch closest, then lerp
        lower_idx = self._find_sample(target_temp=time)
        (a_time, a_temp) = self.samples[lower_idx]
        if a_time == time or len(self.samples) == lower_idx + 1:
            return a_temp
        (b_time, b_temp) = self.samples[lower_idx + 1]
        alpha = (time - a_time) / (b_time - a_time)
        return a_temp + alpha * (b_temp - a_temp)


DELTA_T = 0.5  # how many seconds back / forward to seek for computing momentary values

# phase transition temperatures
COOLDOWN_TARGET_TMP = 40

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

class ControlAutoTune:
    # These are the variables we need to find
    env_temp = None
    base_cooling_per_deg = None
    fan_extra_cooling_per_deg = None
    shell_conductivity = None
    heater_power = None

    # controller phases in order:
    phases = ['heatup', 'overshoot', 'cooldown', 'heatup_fan', 'overshoot_fan', 'cooldown_fan']

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

        # control code for phases should be listed in reverse order to prevent skipping a phase
        if self.phase == 'heatup_fan':
            self.phase = 'done'
        if self.phase == 'cooldown' and temp < target_temp:
            self.heater.alter_target(0)
            self.phase = 'heatup_fan'

        if self.phase == 'overshoot' and temp < last_temp:
            self.phase = 'cooldown'

        if self.phase == 'heatup' and temp >= target_temp:
            self.phase = 'overshoot'
            self.heater.alter_target(COOLDOWN_TARGET_TMP)

        if self.phase in ['heatup', 'heatup_fan']:
            self.set_pwm(read_time, self.heater_max_power)
        else:
            self.set_pwm(read_time, 0.)

        if self.phase not in self.phase_start:
            self.phase_start[self.phase] = len(self.raw_samples)-1

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return not self.phase == 'done'

    # Offline analysis helpers
    def write_file(self, filename):
        pwm = ["pwm: %.3f %.3f" % (time, value)
               for time, value in self.pwm_samples]
        out = ["%.3f %.3f" % (self.timestamps[idx], self.raw_samples[idx]) for idx in range(len(self.timestamps))]
        phases = ["phase %s start: %.3f" % (p, t) for p, t in self.phase_start.items()]
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

    def _simulate_model(self, model_config, start_idx, end_idx, fan_power=0):
        # Creates a model with the given config, and attempts to replicate the temperature
        # curve between (start_idx, end_idx) in smoothed_samples

        m = model.Model(**model_config)
        time = self.timestamps[start_idx]
        pwm_idx = 0
        model_temp_samples = [(time, self.smoothed_samples[start_idx])]
        # pwm_idx is now the index of the last pwm_sample before current time, i.e. the active decision
        for tick in range(start_idx+1, end_idx+1):
            # Find the heater output decision that's relevant to know
            while True:
                if pwm_idx+1 < len(self.pwm_samples) and self.pwm_samples[pwm_idx+1][0] < time:
                    pwm_idx += 1
                else:
                    break

            time = self.timestamps[tick]
            ðt = self.timestamps[tick] - self.timestamps[tick-1]
            new_temp = m.advance_model(ðt, self.pwm_samples[pwm_idx][1], fan_power)
            model_temp_samples.append((time, new_temp))
        return (m, model_temp_samples)

    def _deriv_at(self, idx):
        before_idx = idx
        after_idx = idx
        global DELTA_T
        while self.timestamps[idx] - self.timestamps[before_idx] < DELTA_T:
            before_idx -= 1
        while self.timestamps[after_idx] - self.timestamps[idx] < DELTA_T:
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
        #  These are the variables we need to find
        self.env_temp = self.smoothed_samples[0]
        #  base_cooling_per_deg = None
        #  fan_extra_cooling_per_deg = None
        #  shell_conductivity = None
        #  heater_power = None
        return self._fit_model('heatup', 'cooldown')


    def _fit_model(self, heat_phase, cooldown_phase):
        # we'll measure passive convective cooling during the cooldown phase
        # and compensate our input data for the energy lost.
        # the resulting temp data should have roughly linear heating

        temp_range = range(self.calibrate_temp, COOLDOWN_TARGET_TMP-1,-1)
        measured_derivs = [self._deriv_at(self._find_temp(i, phase=cooldown_phase)) for i in temp_range]
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

        start, _ = self._get_index_range(heat_phase)
        _, end = self._get_index_range(cooldown_phase)
        compensated_temps = []
        loss = 0
        for idx in range(start, end+1):
            t = self.smoothed_samples[idx]
            compensated_temps.append(t-loss)
            ðt = self.timestamps[idx+1] - self.timestamps[idx]
            loss += ðt * (a*(t - self.env_temp) + b)

        config = {
            'heater_power': 134,
            'thermal_conductivity': 0.197,
            'initial_temp': self.smoothed_samples[start],
            'env_temp': self.env_temp,
            'base_cooling': 0.0105,
#            'base_cooling': 0.0,
            'fan_cooling': 0.0
            }
        # Order of calibration:
        # heater strength (initial guess, fit to compensated)
        # thermal mass (fit to compensated)
        # base_cooling (scale so peaks align vertically)
        # heater strength (to get scaling factor right)
        m, model_samples = self._simulate_model(config, start, end, fan_power=0.0)
        plt.plot(self.timestamps, self.smoothed_samples)
        plt.plot(self.timestamps[start:end+1], compensated_temps)
        plt.plot(self.timestamps[start:end+1], [i[1] for i in model_samples])
        plt.show()

        return a, b

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

    def _plot(self):
        plt.plot(self.timestamps, self.raw_samples, label='Trace temp')
        plt.plot(self.timestamps, self.smoothed_samples, label='smoothed temp')
        plt.legend(loc='upper left')
        plt.show()


def load_config(config):
    return ShellCalibrate(config)

def get():
    class FHeater:
        def get_max_power(self):
            return 1.0
    c = ControlAutoTune(FHeater(), 200)
    c.from_file()
    return c
