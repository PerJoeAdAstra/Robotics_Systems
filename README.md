### The README for robotics systems. ###

__TODO__
- [x] Setup git
- [x] Setup overleaf doc, link when done: https://www.overleaf.com/4863979513zbkvhxdvsqpd 
- [x] Check how infrared module works
- [x] Create baseline solution
- [ ] Create alternate baseline (constantly rotating)
- [x] Calibrate Sensors
- [ ] Characterise the sensors
"characterise the sensors, understand them, find and improvement, evaluate it."
Show how the sensors are flawed.
Measure reading vs wall at different distances
Reading at angle to wall (should give different behaviors)
Changing reflectance of wall should change results.

- [ ] Do Experiment.
Changable parameters: shape of environment, reflectance of environment and angle step.

Measured Metrics: Accuracy of measurement.
Either initial hit/miss chance,
Average offset, e.g. sum(abs(measured_value - actual_value)) / total
tally for individual sensor readings and threshold of map.

Improvement: using more sensors
- extra parameter of position of sensors*
- number of sensors
- How the data is combined

Also potentially look at how the snesors are mapped

- [ ] Figure out why improvement is slanted
- [ ] Update TODO list.
- [ ] Delete TODO list.
