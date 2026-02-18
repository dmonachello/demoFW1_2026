# Commands Notes

This folder contains command classes used by the robot.
Commands are scheduled by the CommandScheduler and run every ~20 ms.

Command style note:
1. use `initialize()` for one-shot commands that set a target and end,
2. use `execute()` for continuous commands that must run every cycle.
