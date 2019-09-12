
import matplotlib.pyplot as plt
from common import *
from pid import OnionController

class Sim(object):
    def __init__(self, target, shells, power, dissipation_passes):
        self.controller = OnionController(shells, power, dissipation_passes)
        self.controller_decisions = [0.0]
        # heater is at first (innermost) shell, sensor at second-to-last shell, outermost shell is outside
        self.temp_shells = [ENV_TEMP] * shells
        self.data = []
        for i in range(shells):
            self.data.append(list([ENV_TEMP]))
        self.controller.set_target(target)
        self.modifications_todo = []

    def disturb(self, degrees=-10, in_ticks=10, duration=20):
        self.modifications_todo += [(in_ticks + i, degrees) for i in range(duration)]
        self.modifications_todo.sort(key=lambda mod: mod[0])

    def dissipate_temps(self):
        # heat dissipation within metal
        for i in range(len(self.temp_shells)-1):
            if self.temp_shells[i] > self.temp_shells[i+1]:
                hi = i
                lo = i+1
            else:
                hi = i+1
                lo = i
            temp_diff = self.temp_shells[hi] - self.temp_shells[lo]
            # different heat exchange factors for metal/metal and metal/air
            if i < len(self.temp_shells)-2:
                self.temp_shells[hi] -= HEAT_CONDUCT_METAL * temp_diff
                self.temp_shells[lo] += HEAT_CONDUCT_METAL * temp_diff
            else:
                # heat exchange between metal and cooling air
                self.temp_shells[hi] -= HEAT_CONDUCT_AIR * temp_diff
                self.temp_shells[lo] += HEAT_CONDUCT_AIR * temp_diff
        # the outermost shell is always at ENV_TEMP
        self.temp_shells[-1] = ENV_TEMP

    def _pop_disturbance(self):
        mod = 0
        occurred = 0
        for i in range(len(self.modifications_todo)):
            time_left, until_mod = self.modifications_todo[i]
            if time_left == 0:
                mod += until_mod
                occurred += 1
            else:
                self.modifications_todo[i] = (time_left - 1, until_mod)
        self.modifications_todo = self.modifications_todo[occurred:]
        return mod

    def simulate_tick(self):
        cont = self.controller
        cont.record_sample(self.sensor_temp() + self._pop_disturbance())
        heater_output = cont.make_decision()
        self.temp_shells[0] += heater_output * cont.power
        self.controller_decisions.append(heater_output)
        self.dissipate_temps()
        self.dissipate_temps()
        for i in range(len(self.temp_shells)):
            self.data[i].append(self.temp_shells[i])

    def simulate_ticks(self, n):
        for _ in range(n):
            self.simulate_tick()

    def plot(self):
        time = [ i * TICK_LEN for i in range(len(self.data[0]))]
        for i, shell in enumerate(self.data):
            plt.plot(time, shell, label='shell ' + str(i))
        # plt.plot(time, self.heater_data, label='Heater')
        plt.bar(time, [y * 100 for y in self.controller_decisions], color="#aaaaaa20", width=0.25, label='Heater power')
        plt.legend(loc='upper left')
        plt.show()

    def sensor_temp(self):
        return self.temp_shells[-2]
