# Reinforcement Learning Control of a Physical Inverted Pendulum Model 

This repository has been used during the creation and testing of the software related to the Warsaw University of Technology's Bachelor Diploma Work titled "Reinforcement Learning Control of a Physical Inverted Pendulum Model".

It contains both the code of the C application for the ARM® Cortex® MCU, which controls the low-level electronics of the system, and the Python code that handles the Gym environment and the RL model learning/inference scripts.

## Credits
The software presented was built on top of a few pieces of preexisting code, including:
- The official expansion package for the X-NUCLEO-IHM06A1 Stepper Motor Driver from [X-CUBE-SPN6](https://www.st.com/en/embedded-software/x-cube-spn6.html).
- Re-used bits from [Quanser's code](https://git.ias.informatik.tu-darmstadt.de/quanser/clients/-/tree/master/quanser_robots/qube). Notably: 
  * their VelocityFilter class to compute the angular speeds
  * their ActionLimiter class
- Re-used bits from [Armandpl's code](https://github.com/Armandpl/furuta/tree/master). Notably: 
  * HistoryWrapper and continuity cost

## Author
[Jonathan Soto Peguero](https://www.linkedin.com/in/soto-jnthan/) 
