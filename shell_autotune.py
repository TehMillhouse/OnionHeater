# Calibration of model-based controller settings

import math, model

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
    shells = None
    heater_power = None

    # controller phases in order:
    # heatup, overshoot, cooldown, heatup_fan, overshoot_fan, cooldown_fan

    def __init__(self, heater, target):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.calibrate_temp = target
        # Sample recording
        self.last_pwm = 0.
        self.pwm_samples = []
        self.temp_samples = []
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
        if self.temp_samples:
            last_temp = self.temp_samples[-1]
        else:
            last_temp = temp
            self.env_temp = temp
        self.temp_samples.append((read_time, temp))

        # control code for phases should be listed in reverse order to prevent skipping a phase
        if self.phase == 'cooldown' and temp < target_temp:
            self.heater.alter_target(0)
            self.phase = 'done'

        if self.phase == 'overshoot' and temp < last_temp:
            self.phase = 'cooldown'

        if self.phase == 'heatup' and temp >= target_temp:
            self.phase = 'overshoot'
            self.heater.alter_target(self.env_temp + 20)

        if self.phase in ['heatup', 'heatup_fan']:
            self.set_pwm(read_time, self.heater_max_power)
        else:
            self.set_pwm(read_time, 0.)

        if self.phase not in self.phase_start:
            self.phase_start[self.phase] = read_time
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return not self.phase == 'done'
    # Offline analysis helper
    def write_file(self, filename):
        pwm = ["pwm: %.3f %.3f" % (time, value)
               for time, value in self.pwm_samples]
        out = ["%.3f %.3f" % (time, temp) for time, temp in self.temp_samples]
        phases = ["phase %s start: %.3f" % (p, t) for p, t in self.phase_start.items()]
        f = open(filename, "wb")
        f.write('\n'.join(pwm + out + phases))
        f.close()

def load_config(config):
    return ShellCalibrate(config)
