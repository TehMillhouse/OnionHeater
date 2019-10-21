import math

TRACE = True

class Model(object):
    def __init__(self, heater_power, initial_temp, thermal_conductivity, base_cooling, metal_cells=6):
        cells = int(metal_cells + 1)  # there's an additional outer shell filled with air
        self.time = 0
        self.cells = [initial_temp] * cells
        # Since heat conductivity of metal is pretty high, the heater is effectively outputting
        # the measured degrees per second over *all* cells
        self.heater_power = heater_power # * metal_cells
        # heater is at first (innermost) shell, sensor at second-to-last shell, outermost shell is outside
        self.cells = [initial_temp] * cells
        self.thermal_conductivity = thermal_conductivity
        self.base_cooling = base_cooling
        self.egress_samples = [0.0, 0.0, 0.0]
        self._env_temp = None
        if TRACE:
            self.history = []
            self.pwm_history = []

    @property
    def env_temp(self):
        if self._env_temp is None:
            return 21.0
        return self._env_temp

    @property
    def egress_p_sec(self):
        return float(sum(self.egress_samples)) / len(self.egress_samples)

    def config(self):
        return {
                'heater_power': self.heater_power / (len(self.cells)-1),
                'metal_cells': len(self.cells)-1,
                'passes_per_sec': self.passes_per_sec,
                'thermal_conductivity': self.thermal_conductivity,
                'base_cooling': self.base_cooling,
                }

    def avg_energy(self):
        return sum(self.cells[:-1]) / (len(self.cells)-1)

    def advance_model(self, dt, heater_pwm_until_now, sensor_temp):
        if self._env_temp is None or sensor_temp < self._env_temp:
            self._env_temp = sensor_temp

        old_en = self.avg_energy()
        passes = int(max(1, math.floor(dt * 3)))  # ensure at least 3 dissipation passes per second
        for _ in range(passes):
            self.dissipate_temps(float(dt) / passes, heater_pwm_until_now)
        self.time += dt
        new_en = self.avg_energy() - (dt * heater_pwm_until_now * self.heater_power) / (len(self.cells)-1)

        # expected egress: old_en - new_en
        # actual nonmodelled egress: (old_en - sensor_temp) - expected_egress
        # assuming the model is brought to ground truth on every tick
        self.egress_samples.append((- sensor_temp + new_en) / dt)
        self.egress_samples = self.egress_samples[1:]

        predicted_temp = self.cells[-2]
        delta = (predicted_temp - sensor_temp)
        MAX_EXAGG = 5
        # cap exaggeration at MAX_EXAGG degrees
        if abs(1.3*delta) > MAX_EXAGG:
            sgn = -1 if delta < 0 else 1
            self.cells[-2] -= sgn * max(MAX_EXAGG, abs(delta))
        else:
            self.cells[-2] -= 1.3*delta
            self.cells[-3] -= 0.7*delta

        if TRACE:
            self.history.append(list(self.cells))
            self.pwm_history.append(heater_pwm_until_now)
        return self.cells[-2]

    def _thermal_conductivity(self, source_idx, target_idx):
        if source_idx == len(self.cells)-1 or target_idx == len(self.cells)-1:
            # contact to outside world
            return self.base_cooling
        return self.thermal_conductivity

    def dissipate_temps(self, dt, heater_pwm):
        new_cells = list(self.cells)
        for target in range(len(self.cells)):
            temp_diff = 0
            for source in [target-1, target+1]:
                if source < 0 or source >= len(self.cells):
                    continue
                gradient_from_source = self.cells[source] - self.cells[target]
                thermal_conductivity = self._thermal_conductivity(source, target)
                temp_diff += thermal_conductivity * gradient_from_source
            if target == 0:
                # target shell is heater shell
                temp_diff += heater_pwm * self.heater_power
            new_cells[target] = new_cells[target] + dt * temp_diff
            # the outermost shell is always at environment temp
            new_cells[-1] = self.env_temp
        self.cells = new_cells

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
