# Overview
The contents of this file contain everything that was used to assist us in the creation of our demo 1 code, and the demo 1 code itself

# Matlab / Simulink

The apctrlsystem.slx and the apctrlsystem.slxc were files that we used to create and find 'K' variables for our motor. They are simulink control setups that we made
directly while modeling them after Professor Vincents.

Starting with tunemotor.mlx and tunectrl.mlx, these assisted us with proving the 'K' variables that we obtained from the apctrlsystem.slx and was able to ensur 
that our motor was able to produce realistic results.

The motor.slx, and motor.slxc are the tuning control parameters that we were able to set and obtain with matlab/simulink for our robot.

The steeringsimulation.slxc, is another matlab/simulink file that was created by Professor Vincent that we used as refernce and help in creating our own version.

Demo1_tune.m and animate.m are the final matlab files that we used to have proof of concept that our 'K' values did indeed work. Demo1_tune.m runs the 'k' values over a period 
of time and illustrates the results on various graphs. Animate.m takes the values of Demo1_tune.m and illustrates how the robot would move.

# Ardiuno

Finally, control.ino is the Ardiuno code that we used to have our robot complete demo1.
