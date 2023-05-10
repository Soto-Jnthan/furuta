# Reinforcement Learning Control of a Physical Inverted Pendulum Model 

This repository has been used during the creation and testing of the software related to the Warsaw University of Technology's Bachelor Diploma Work, titled "Reinforcement Learning Control of a Physical Inverted Pendulum Model."

The repository contains both the code of the C application for the ARM® Cortex® MCU, which controls the low-level electronics of the system, and the Python code that handles the Gym environment and the RL model learning/inference scripts.

## Credits
The code presented was built on top preexisting code, including:
- Re-used bits from [Quanser's code](https://git.ias.informatik.tu-darmstadt.de/quanser/clients/-/tree/master/quanser_robots/qube). Notably: 
  * their VelocityFilter class to compute the angular speeds
  * their ActionLimiter class
- Re-used bits from [Armandpl](https://github.com/Armandpl/furuta/tree/master). Notably: 
  * HistoryWrapper and continuity cost

## Author
[Jonathan Soto Peguero](https://www.linkedin.com/in/soto-jnthan/)  