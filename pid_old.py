
class Controller(object):
    def __init__(self, power, deadtime):
        # Controller model: 
        # heater burns only get triggered after deadtime
        # a deadtime of 1 means we'll see the burn's effect at the next tick

        self.deadtime = deadtime  # time is quantized in ticks, deadtime is in this time unit
        self.power = power  # the max amount of heating we can apply (~ in degrees per tick)

        # the newest `deadtime` of these burns haven't dissipated to the sensor yet,
        # the rest is remembered to better estimate current thermal egress
        self.burn_history = RingBuffer([0.0] * (deadtime + WINDOW_SIZE) )

        # past temperature readings
        self.history = None

        self.target = 0.0

    def record_sample(self, temp):
        # record new sample, advance model by one tick
        if self.history is None:
            self.history = RingBuffer([temp] * WINDOW_SIZE)
        else:
            self.history.record_new(temp)
        self.burn_history.record_new(0.0)  # make sure both histories are synced

        needed_temp_delta = self.target - self.temp_at_t_plus_deadtime(0.0)
        self.burn_history[0] = max(0.0, min(1.0, needed_temp_delta / self.power))

    def set_target(self, t):
        self.target = t

    def get_decision(self):
        return self.burn_history[0]

    def current_thermal_egress(self):
        # look at past WINDOW_SIZE temperature readings to estimate how quickly
        # heat is lost through convection. We correct for the amount of power we
        # supplied `deadtime` ticks *before* those readings.

        # return type: ~degrees per tick
        # TODO: other return type? after all, cooling scales with temp differential
        thermal_ingress = 0.0
        temp_delta = 0.0
        for i in range(1, WINDOW_SIZE):
            temp_delta += self.history[i] - self.history[i-1]
            #  the amount of energy we pumped into the heater in that tick: max power * PWM duty cycle
            thermal_ingress += self.power * self.burn_history[self.deadtime + i-1]  # TODO: off-by-one?
        # thermal egress is a positive number
        return - (temp_delta - thermal_ingress) / WINDOW_SIZE

    def temp_at_t_plus_deadtime(self, expected_extra_egress):
        # The burn we may or may not perform now will only affect the hotend in `deadtime` ticks.
        # So to compensate for burns we've already performed, but haven't yet seen the effects of,
        # we simulate their future effect on the temperature of the sensor.

        # expected_extra_egress is supplied to compensate for fans we *just*
        # turned on, so we don't have to wait for the system to stabilize by itself
        egress = self.current_thermal_egress() + expected_extra_egress
        temp = self.history[0]
        for t in range(self.deadtime):
            temp += self.burn_history[t] * self.power - egress
        return temp
