
import random
import math
from model_based_controller import ModelBasedController

class FakeHeater(object):
    def __init__(self):
        self.pwm = 0

    def get_max_power(self):
        return 1.0

    def get_pwm(self):
        return self.pwm

    def set_pwm(self, time, value):
        self.pwm = value

class FakeConfig(object):
    def __init__(self, model_config):
        self.cfg = model_config

    class FakePrinter(object):
        def get_status(self, none):
            return {'speed': 0.0}

        def lookup_object(self, string):
            return self

    def get_printer(self):
        return FakeConfig.FakePrinter()

    def getfloat(self, string, *args, **kwargs):
        assert string.startswith('model_')
        string = string[len('model_'):]
        if string in self.cfg:
            return self.cfg[string]
        assert len(args)
        return args[0]

    def getint(self, string, *args, **kwargs):
        return self.getfloat(string, *args, **kwargs)

TICK_LEN = 0.833
ENV_TEMP = 21
HEATER_POWER = 2.0166
NOISE_AMP = 0.2
HEAT_CONDUCT_METAL = 0.05
HEAT_CONDUCT_AIR = 0.004345

DEFAULT_CFG = {
        'heater_power': HEATER_POWER,
        'thermal_conductivity': HEAT_CONDUCT_METAL,
        'base_cooling': HEAT_CONDUCT_AIR,
        'initial_temp': ENV_TEMP
        }

class Sim(object):
    def __init__(self, target, metal_cells=5, power=HEATER_POWER, randomness=NOISE_AMP, dissipation_passes=2, model_cfg=DEFAULT_CFG):
        cells = int(metal_cells + 1)  # one filled with air
        self.target = target
        self.power = power
        self.heater = FakeHeater()
        self.config = FakeConfig(model_cfg)
        self.controller = ModelBasedController(self.heater, self.config)
        # heater is at first (innermost) shell, sensor at second-to-last shell, outermost shell is outside
        self.temp_cells = [ENV_TEMP] * cells
        self.controller_data = []
        self.temperature_history = [0.0]
        for i in range(len(self.controller.model.cells)):
            self.controller_data.append(list([0.0]))
        self.controller_decisions = [0.0]
        self.modifications_todo = []
        self.randomness = randomness
        self.time = 0

    def _noise(self):
        return random.random() * self.randomness - 0.5 * self.randomness

    def _S(self, x):
        return 1 / (1 + math.exp(-x))

    def _bell(self,x):
        if x < 0:
            return self._S(x*2+6)
        return self._S(-x*2+6)

    def disturb(self, degrees=-4, in_ticks=10, duration=20):
        def disturbance(tick):
            if tick < 6:
                return degrees * self._bell(tick-6)
            if tick >= duration-6:
                return degrees * self._bell(6 - duration + tick)
            return degrees
        self.modifications_todo += [(in_ticks + i, disturbance(i) + self._noise()) for i in range(duration)]
        self.modifications_todo.sort(key=lambda mod: mod[0])

    def disturb_hard(self, degrees=-4, in_ticks=10, duration=20):
        self.modifications_todo += [(in_ticks + i, degrees + self._noise()) for i in range(duration)]
        self.modifications_todo.sort(key=lambda mod: mod[0])

    def dissipate_temps(self, env_cooling_factor=1.0):
        # heat dissipation as cellular automaton: each shell independently calculates how much
        # heat it exchanges with its neighbors
        cells = self.temp_cells
        next_cells = list(self.temp_cells)
        for i in range(len(cells)):
            conduct_left  = env_cooling_factor * HEAT_CONDUCT_AIR if i   == len(cells)-1 else HEAT_CONDUCT_METAL
            conduct_right = env_cooling_factor * HEAT_CONDUCT_AIR if i+1 == len(cells)-1 else HEAT_CONDUCT_METAL
            delta_left = 0 if i-1 < 0 else cells[i-1] - cells[i]
            delta_right = 0 if i+1 >= len(cells) else cells[i+1] - cells[i]
            # these values are >0 if shell[i] is colder than neighbors
            next_cells[i] = cells[i] + conduct_left*delta_left + conduct_right*delta_right

        for i in range(len(cells)):
            self.temp_cells[i] = next_cells[i]
        # the outermost shell is always at ENV_TEMP
        self.temp_cells[-1] = ENV_TEMP


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

    def tick(self):
        cont = self.controller
        effective_temp = self.sensor_temp() + self._pop_disturbance() + self._noise()
        self.temperature_history.append(effective_temp)
        cont.temperature_update(self.time, effective_temp, self.target)
        heater_output = self.heater.get_pwm()
        self.temp_cells[0] += TICK_LEN * heater_output * self.power * (len(self.temp_cells)-1)
        self.controller_decisions.append(heater_output)
        self.dissipate_temps()
        self.dissipate_temps()
        for i in range(len(cont.model.cells)):
            self.controller_data[i].append(cont.model.cells[i])
        self.time += TICK_LEN

    def ticks(self, n):
        for _ in range(n):
            self.tick()

    def plot(self):
        import matplotlib.pyplot as plt
        time = [ i * TICK_LEN for i in range(len(self.controller_data[0]))]
        for i, shell in enumerate(self.controller_data):
            plt.plot(time, shell, label='c. shell ' + str(i), linestyle='--')
        plt.plot(time, self.temperature_history, label='Simulator temp')
        plt.bar(time, [y * 100 for y in self.controller_decisions], color="#aaaaaa20", width=0.25, label='Heater power')
        plt.legend(loc='upper left')
        plt.show()

    def sensor_temp(self):
        return self.temp_cells[-2]

class NullSim(Sim):
    def __init__(self, target, config=DEFAULT_CFG, randomness=NOISE_AMP):
        self.target = target
        self.config = config
        self.heater = FakeHeater()
        self.config = FakeConfig(config)
        self.controller = ModelBasedController(self.heater, self.config)
        self.controller_data = []
        self.temperature_history = [0.0]
        for i in range(len(self.controller.model.cells)):
            self.controller_data.append(list([0.0]))
        self.controller_decisions = [0.0]
        self.modifications_todo = []
        self.randomness = randomness
        self.time = 0

    def sensor_temp(self):
        c = self.controller.model.cells
        return c[-2] # - (c[-4] - c[-3])

    def tick(self):
        cont = self.controller
        effective_temp = self.sensor_temp() + self._pop_disturbance() + self._noise()
        self.temperature_history.append(effective_temp)
        prev_en = sum(cont.model.cells[:-1])
        cont.temperature_update(self.time, effective_temp, self.target)
        new_en = sum(cont.model.cells[:-1])
        # print(prev_en - new_en)# - TICK_LEN * cont.current_heater_pwm * cont.model.heater_power)
        heater_output = self.heater.get_pwm()
        self.controller_decisions.append(heater_output)
        for i in range(len(cont.model.cells)):
            self.controller_data[i].append(cont.model.cells[i])
        self.time += TICK_LEN
