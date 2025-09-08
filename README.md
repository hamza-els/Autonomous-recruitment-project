# FEB Autonomous Recruitment Project

The goal of this project is to design and implement something to drive a car around a track while avoiding cones.

More specifically, you must write a function that takes in a single argument (the state of the car) $[x \, y \, \phi \, v \, \theta ]$ and returns a single control command $[a \, \dot{\theta}]$.

These variables represent the following quantities:
- $x$ and $y$ are the position of the car, in meters.
- $\phi$ is the heading of the car, in radians (zero is pointing directly left)
- $v$ is the velocity of the car, in meters per second.
- $\theta$ is the angle of the front wheels, in radians.
- $a$ is the requested acceleration for the car. Think of this more as the normalized force $\frac{F}{m}$ - the car is subject to aerodynamic drag and will not keep accelerating infinitely.
- $\dot{\theta}$ is the time derivative of the steering angle $\theta$. it is a control input. We assume that the steer-by-wire system can acheive high accelerations here, so we don't need to worry about changing this value quickly.

There are also the following constraints:

| variable       | lower bound | upper bound |
| -------------- | ----------- | ----------- |
| $\theta$       | -0.7        | 0.7         |
| $\dot{\theta}$ | -1.0        | 1.0         |
| $a$            | -4          | 10          |

The only other important numbers that you do not get to choose are:
- the wheelbase (distance between front and rear wheels) is 1.58 meters.
- the maximum acceleration the car can handle (in $x$ and $y$ combined) is 12 meters per second per second.
- the outer geometry of the car is a convex polygon with vertices accesible through the `Simulator.car_vertices` property.

## Implementation

Please write your implementation in the `controller` method of `main.py`. You may read or modify anything you want in `simulator.py` for development but make sure your code works with the original simulator class before submission.

Provided in `simulator.py` is the `Simulator` class, which has:
- some basic visualization utilities (`.plot()` and `.animate()`)
- the variable bounds (`.steering_limits`, `.lbu`, and `.ubu`)
- the car outline vertices (`.car_vertices`)
- `.get_results` to see how the simulation went
- `.run()` to run the simulation.

In order to run this simulation, you'll need to install `casadi`, `matplotlib`, and `numpy`.
All can be installed with `pip` (ie, `pip install casadi matplotlib numpy`)

We recommend you fork this repo and develop your code there (so you can keep history with git/github), but you do not need to. You can just clone this repo or download the .zip file and work locally.

If you don't want to run anything locally (if you don't have python, or there's something funny with your default C compiler, or any other reason), this code works in google colab. Just make a new notebook and add the following as the first cell:

```
!pip install casadi
!git clone git@github.com:FEBAutonomous/control-recruitment-project.git
```

Then you can copy the contents of `main.py` in: 

```python
import numpy as np
from simulator import Simulator

def controller(x):
    """
    <docstring excluded for brevity>
    """
    ...

sim = Simulator(controller)
sim.run()
sim.animate()
sim.plot()
```

I'm not sure if `sim.animate()` will work in colab, but if it doesn't, you can just pass `sim.animate(save=True)` and look at the resulting gif.


## Presentation & Submission

When you present your project, you will have roughly 10-15 minutes to showcase everything you've done in just these two weeks (please do not show us anything not directly related to this project). The goal is to show:
- *why* you did what you did
- what you learned
- what problems you encountered and how you solved them

Whatever medium you think is best for this is fine; we're not particularly concerned about your graphic design or presentation skills beyond what is needed to communicate the core ideas here.

You will also submit a zip file of your finalized code.
