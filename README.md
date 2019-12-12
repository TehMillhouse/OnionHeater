## What's this?

I couldn't get the PID loop in my 3D printer firmware (currently [klipper](https://github.com/KevinOConnor/klipper)) to keep the temperature stable, especially not if the fan speed varied.
I decided to fix this by implementing my own temperature controller, one that doesn't use a PID loop.

## How does it work?

It has an internal thermal model of how hot different parts of the hotend are, laid out as follows:
The hotend is modelled as a series of concentric zones (like the layers of an onion), with the innermost zone housing the heating cartridge, and the outermost zone being the outside air. The temperature sensor (thermistor/thermocouple) is located on the second-to-last layer.
It keeps track of how much heat the heater cartridge produces and runs a simple automata-based thermal simulation to keep its internal model in sync with sensor data coming in.

I tried to keep the number of parameters in the model as low as possible, but there's a couple of tunables:
* Number of cells in the thermal model
* Heater output
* Thermal conductivity across metal-metal boundaries and metal-air boundaries
* Fan strength
* Assumed environment temperature
* etc...  

Model and controller are separate, there's a simple simulator capable of driving it and drawing graphs, and there's a fairly complicated mechanism that jumps through several statistical hoops to be able to auto-tune the various parameters of the model to best match a temperature trace from real hardware.

## Does it work?

Yes... and no. When all is said and done, autotuning the model parameters yields a model that perfectly replicates the given trace, and within the simulator, the controller manages to *nail* the target temperature without any overshoot or oscillation.  
Running this controller on real hardware, however, fails to replicate these results. Usually, the real temperature drags some between 5-15°C below the target, and is anything but stable. The model has perfect replicatory power, but little predictive power.  

I have not conclusively found out why, yet, but the temperature loss by actually melting plastic seems like a major culprit.
The plan going forward is to instead use a cascade approach, incorporating a PID loop and a secondary controller that compensates for varying fan speeds.
