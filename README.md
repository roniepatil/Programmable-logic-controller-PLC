## Description
This repository includes all the PLC designs and code files generated for arduino uno.

## Author
Rohit M Patil

## Content
PLC (programmable logic controller), which is a device that can control industrial processes and machines using ladder logic.I learned about:

- Scan cycles: The process of scanning the input, executing the ladder logic, updating the output and performing housekeeping tasks in each cycle.
- Rung blocks: The basic elements of ladder logic, such as rungs, sub-rungs, contacts (normally open, normally closed, rising edge, falling edge) and coils (normal, negated, latched, unlatched).
- Files: The different types of data files that store the input, output, binary, timer, counter, control and numeric values (INT, DINT, real) in the PLC memory. I also learned about analog input/output and sequencer files (compare/load/output).
- Compare: The instructions that compare two values and set a bit based on the result. I learned about equal, not equal, less than, greater than, less than or equal, greater than or equal, limit and mask equal instructions.
- Timers: The instructions that measure time and set a bit based on the elapsed time. I learned about counter up, counter up-down, timer on-delay, timer off-delay, timer pulse and retentive timer on instructions.
- Bit operation: The instructions that manipulate bits in a file or a word. I learned about bit shift left, bit shift right, rotate left, rotate right, AND, OR , NOT, XOR, clear, copy, fill, move and masked move instructions.
- Mathematical operation: The instructions that perform arithmetic and trigonometric calculations on numeric values. I learned about addition, subtraction, multiplication, division, absolute value, square root, exponential function, cosine function, sine function, tangent function, logarithmic function, negate function,
increment function,
decrement function,
radians to degree conversion,
degree to radians conversion,
scale data function and
scale with parameters function.
- Subroutines & external interrupts: The instructions that allow branching and jumping to different parts of the ladder logic program. I learned about jump to label,
jump to subroutine,
label location,
immediate input mask,
immediate output mask,
I/O refresh and
temporary end instructions.
- I/O Manager: The software tool that allows configuring and monitoring the input/output modules of the PLC.

I also completed several projects using the PLC simulator to apply the concepts I learned. Some of the projects are:

- 2 stage pump: A project that controls the speed and direction of a pump using two switches and two LEDs.
- Tank levels: A project that monitors the level of water in two tanks using analog input and output modules and displays the level on a screen.
- Filling lines: A project that controls the filling of bottles using sensors and valves on a conveyor belt.
- Traffic lights: A project that simulates the operation of traffic lights at an intersection using timers and counters.
- Chocolate factory temperature management: A project that regulates the temperature of a chocolate melting tank using a heater and a fan.