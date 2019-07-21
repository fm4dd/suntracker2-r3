# SunTracker2 Revision 3

<img src="img/DSC_3030.png">

This project develops a photovoltaic (PV) solar tracking controller.
Pictured above is the current prototype. PV solar trackers control the PV
panel direction to keep the panel perpendicular to the Sun, maximizing the
power output. The power output difference for a fully aligned panel is
reported to be in a range of 15\~40%, which would be substantial.

## Design

The goal is a self-orienting dual-axis, scheduled, Arduino-based solar
tracker. I want to control PV panel orientation for both Elevation and
Azimuth angles (dual-axis), based on the calculated solar position of a PV
systems specific geographic location and time (scheduled).

<img src="img/SunTracker5.png width="320px"><img src="img/DSC_0226s.jpg width="320px">


Working under space, cost and time constraints, the prototype development is
done on a small scale model. Eventually, I want to control and optimize the
orientation of Mini-PV systems such as below.

## Functional Design

<img src="img/sun-position-schema.png">

In order to clearly understand and validate the sun tracking function, a 32
dual-color LED ring will display the horizon and move the sun position over
time.

### Board Components

<img src="img/suntracker2r3 block diagram.png">

### Board Layout

<img src="img/suntracker2-v1.7-overview.png">

The solar position data gets calculated off-system in advance, and saved into a
micro SD card. The pre-generated daily sun position files are then read by the
Arduino MKR Zero microcontroller through the onboard card reader. The
astronomical calculations of the suns position require sufficient CPU power, and
are taxing on Microcontrollers.

At the moment I am using a sun position dataset that covers two years of
operation, and uses approx. 20MB in size (46 MB with additional debug data). I
wrote the “suncalc” tool to pre-calculate the data set files for a specific
location, on a Linux PC.

### PCB

<img src="img/DSC_3001.png">
