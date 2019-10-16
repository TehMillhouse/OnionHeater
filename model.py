import math

TRACE = True

class Model(object):
    def __init__(self, heater_power, initial_temp, thermal_conductivity, base_cooling, fan_cooling, env_temp, metal_cells=6, passes_per_sec=3):
        """Create thermal model of hotend.

        heater_power: how many degrees per second the heater can output over the hotend
        metal_cells: how many different locations in the heater block to track. affects
            thermal mass and dissipation speed in the model
        passes_per_sec: minimum number of dissipation passes computed per second. Higher is more accurate
        initial_temp: how hot is the hotend *right now*?
        """
        cells = int(metal_cells + 1)  # there's an additional outer shell filled with air
        self.time = 0
        self.passes_per_sec = passes_per_sec
        self.cells = [initial_temp] * cells
        # Since heat conductivity of metal is pretty high, the heater is effectively outputting
        # the measured degrees per second over *all* cells
        self.heater_power = heater_power # * metal_cells
        # heater is at first (innermost) shell, sensor at second-to-last shell, outermost shell is outside
        self.cells = [initial_temp] * cells
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
                'heater_power': self.heater_power / (len(self.cells)-1),
                'metal_cells': len(self.cells)-1,
                'passes_per_sec': self.passes_per_sec,
                'thermal_conductivity': self.thermal_conductivity,
                'base_cooling': self.base_cooling,
                'fan_cooling': self.fan_cooling
                }

    def advance_model(self, dt, heater_pwm_until_now, fan_power=0.0):
        passes = int(max(1, math.floor(dt * self.passes_per_sec)))
        for _ in range(passes):
            self.dissipate_temps(dt / passes, heater_pwm_until_now, fan_power)
        self.time += dt
        if TRACE:
            self.history.append(list(self.cells))
            self.pwm_history.append(heater_pwm_until_now)
        return self.cells[-2]

    def adjust_to_measurement(self, sensor_temp):
        predicted_temp = self.cells[-2]
        if sensor_temp < self.env_temp:
            self.env_temp = sensor_temp
        self.cells[-2] = sensor_temp

    def _thermal_conductivity(self, source_idx, target_idx, fan_power):
        if source_idx == len(self.cells)-1 or target_idx == len(self.cells)-1:
            # contact to outside world
            return self.base_cooling + fan_power * self.fan_cooling
        return self.thermal_conductivity

    def plot(self):
        self._plot(self.history, self.pwm_history)

    def _plot(self, trace, pwm_output):
        import matplotlib.pyplot as plt
        # trace = list of tuples, one for each shell
        shells = len(trace[0])
        time = [ i * 0.833 for i in range(len(trace))]
        for i in range(shells):
            if i == shells-2:
                continue
            shell = [blip[i] for blip in trace]
            plt.plot(time, shell, label='c. shell ' + str(i), linestyle='--')
        plt.plot(time, [blip[-2] for blip in trace], label='Sensor temp')
        plt.bar(time, [y * 100 for y in pwm_output], color="#aaaaaa20", width=0.25, label='Heater power')
        plt.legend(loc='upper left')
        plt.show()

    def dissipate_temps(self, dt, heater_pwm, fan_power=0.0):
        new_cells = list(self.cells)
        for target in range(len(self.cells)):
            temp_diff = 0
            for source in [target-1, target+1]:
                if source < 0 or source >= len(self.cells):
                    continue
                dist = abs(source - target)
                gradient_from_source = self.cells[source] - self.cells[target]
                thermal_conductivity = self._thermal_conductivity(source, target, fan_power)
                temp_diff += thermal_conductivity * gradient_from_source / (dist * dist)
            if target == 0:
                # target shell is heater shell
                temp_diff += heater_pwm * self.heater_power
            new_cells[target] = new_cells[target] + dt * temp_diff
            # the outermost shell is always at environment temp
            new_cells[-1] = self.env_temp
        self.cells = new_cells
