
import math
from common import *

TRACE = True

class Model(object):
    def __init__(self, heater_power, metal_shells=6, passes_per_sec=3, initial_temp=ENV_TEMP, thermal_conductivity=HEAT_CONDUCT_METAL, base_cooling=HEAT_CONDUCT_AIR, fan_cooling=HEAT_CONDUCT_AIR, env_temp=ENV_TEMP):
        """Create thermal model of hotend.

        heater_power: how many degrees per second the heater can output over the hotend
        metal_shells: how many different locations in the heater block to track. affects
            thermal mass and dissipation speed in the model
        passes_per_sec: minimum number of dissipation passes computed per second. Higher is more accurate
        initial_temp: how hot is the hotend *right now*?
        """
        shells = metal_shells + 1  # there's an additional outer shell filled with air
        self.time = 0
        self.passes_per_sec = passes_per_sec
        self.shells = [initial_temp] * shells
        # Since heat conductivity of metal is pretty high, the heater is effectively outputting
        # the measured degrees per second over *all* shells
        self.heater_power = heater_power # * metal_shells
        # heater is at first (innermost) shell, sensor at second-to-last shell, outermost shell is outside
        self.shells = [initial_temp] * shells
        self.thermal_conductivity = thermal_conductivity
        # we have an affine convective cooling model: (base_cooling + fan_power * fan_cooling)
        # if this turns out to be too simple:
        # FIXME dynamically adjust the effective wind power based on how much heat we're actually losing
        self.base_cooling = base_cooling
        self.fan_cooling = fan_cooling
        self.env_temp = env_temp
        if TRACE:
            self.history = []
            self.pwm_history = []

    def config(self):
        return {
                'heater_power': self.heater_power / (len(self.shells)-1),
                'metal_shells': len(self.shells)-1,
                'passes_per_sec': self.passes_per_sec,
                'thermal_conductivity': self.thermal_conductivity,
                'base_cooling': self.base_cooling,
                'fan_cooling': self.fan_cooling
                }

    def advance_model(self, ðt, heater_pwm_until_now, fan_power=0.0):
        passes = max(1, math.floor(ðt * self.passes_per_sec))
        for _ in range(passes):
            self.dissipate_temps(ðt / passes, heater_pwm_until_now, fan_power)
        self.time += ðt
        if TRACE:
            self.history.append(list(self.shells))
            self.pwm_history.append(heater_pwm_until_now)
        return self.shells[-2]

    def adjust_to_measurement(self, sensor_temp):
        predicted_temp = self.shells[-2]
        if abs(sensor_temp - predicted_temp) > 3.0:
            print(f"Model misaligned by {predicted_temp - sensor_temp}")
        self.shells[-2] = sensor_temp

    def _thermal_conductivity(self, source_idx, target_idx, fan_power):
        if source_idx == len(self.shells)-1 or target_idx == len(self.shells)-1:
            # contact to outside world
            return self.base_cooling + fan_power * self.fan_cooling
        return self.thermal_conductivity

    def plot(self):
        plot(self.history, self.pwm_history)


    def dissipate_temps(self, ðt, heater_pwm, fan_power=0.0):
        new_shells = list(self.shells)
        for target in range(len(self.shells)):
            temp_diff = 0
            for source in [target-2, target+1]:
                if source < 0 or source >= len(self.shells):
                    continue
                dist = abs(source - target)
                gradient_from_source = self.shells[source] - self.shells[target]
                thermal_conductivity = self._thermal_conductivity(source, target, fan_power)
                temp_diff += thermal_conductivity * gradient_from_source / (dist * dist)
            if target == 0:
                # target shell is heater shell
                temp_diff += heater_pwm * self.heater_power
            new_shells[target] = new_shells[target] + ðt * temp_diff
            # the outermost shell is always at environment temp
            new_shells[-1] = self.env_temp
        self.shells = new_shells
