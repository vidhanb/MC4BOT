# Robot Project
C++ code for the Ohio State Fundamentals of Engineering Honors robot project

Developers: Vidhan Bhardwaj and Kyle Westhaus
- Note: Majority of the coding was done in person together through VS live share, finished code was then pushed

## Table of Contents
1. [Introduction](#introduction)
    - [Background](#background)
    - [Results](#results)
2. [Setup](#setup)
3. [Capabilities](#capabilities)
4. [Future Improvements](#future-improvements)
5. [Permalinks](#permalinks)
6. [License](#license)


## Introduction
### Background
Each year, Ohio State's Fundamentals of Engineering Honors (FEH) classes participate in a semester-long project to build a robot. The goal of the robot is to complete that year's challenge course, constructed by the Engineering Education Department (EED) staff and teaching assistants, in under 2 minutes. I wrote most of the code for my group's robot, pictured below.

<img src="images/M4-Robot.jpg" height="400" alt="Team M4 Robot Picture" >

A picture of the course layout (courtesy of the EED) for our class (Spring 2019) is also included at the end of this paragraph. Tasks included reading light colors, pushing and holding buttons, flipping levers, moving score sliders, and depositing a token into a slot. A full description of the challenge scenario can be found in [this document from the EED.](doc/Robot_Scenario.pdf)

<img src="images/Arcade-Course-CAD-2019.jpg" height="600" style="transform:rotate(270deg);" alt="Course Layout Design" >


### Results
Our robot had noticeable mechanical issues from its tread-based drivetrain, but this code was able to drive the robot to a perfect run during the final competition. A video of this run can be found [here.](https://youtu.be/jPylJhgtDp8?t=7669) A full video of the competition can be found [on YouTube.](https://youtu.be/jPylJhgtDp8)


## Setup
This code is intended to be compiled to an S19 file for the Proteus controllers used in the FEH classrooms. The Proteus is built on a Freescale Kinetis K60 (ARM Cortex M-4) and other components as described on [this page.](https://u.osu.edu/fehproteus/introduction/hardware/)

[OSU's Proteus wiki page](https://u.osu.edu/fehproteus/qt-environment/install-guide/) describes how to set up the standard development environment. Other IDEs (I used VS Code) and platforms will also work through modification of the project Makefile and installation of the [GNU Arm Embedded Toolchain,](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) [OSU's FEH Proteus firmware,](https://code.osu.edu/fehelectronics/proteus_software/fehproteusfirmware) and [Freescale Semiconductor's Embedded Warrior Library (EWL).](https://community.nxp.com/docs/DOC-93277)


## Capabilities
This project implements many features for the robot, including:
* Per-course path variance adjustment during initialization
* Proportion-based adjustment for straight driving
* Both single-movement and pulsed correction for X-position, Y-position, and course Heading based on FEH's "Robot Positioning System" (RPS)
* Error tolerance and sampling for RPS X-position, Y-position, and Heading values
* Acceleration-mapped driving to speed up and slow down the robot based on percentage of movement complete, with safe fallbacks
* Timeouts for start light and encoder-based drive functions
* Micro SD card logging for post-run debugging
* Screen status updates
* Movement functions for driving(forwards/backwards), turning, and curving based on both time and distance (through encoder count calculations)
* Thorough test functions to show successful operation of the robot's drivetrain, servos, sensors, and other code functions


## Future Improvements
A number of possible features could improve operation of the robot, including:
* Increased motor speed values, ideally after testing to confirm effects on consistency
* Improved calculations for position checks around the lever. Currently the robot functions fine there, but loses some time
* Simulation of RPS lag, rather than costly (in terms of time) sleep statements. This idea was used by team M3 during the competition
* Full PID feedback for straight driving, after testing to confirm feasibility with treads
* Decreasing adjustment movements after overcorrections
* Better error tolerance for the case in which one encoder completely breaks during a run


## Permalinks
Links in the above text were live at the time I originally published this repository. In the event that link rot should occur, I have verified that the following links point to the intended content. The links make use of the Internet Archive's Wayback Machine.

[Proteus Hardware](https://web.archive.org/web/20200503214826/https://u.osu.edu/fehproteus/introduction/hardware/)

[Proteus Development Environment Setup](https://web.archive.org/web/20191025211221/https://u.osu.edu/fehproteus/qt-environment/install-guide/)

[GNU Arm Embedded Toolchain](https://web.archive.org/web/20200314195521/https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)

[Proteus Firmware](https://web.archive.org/web/20190129222412/https://code.osu.edu/fehelectronics/fehproteusfirmware)

[Embedded Warrior Library (EWL)](https://web.archive.org/web/20190821042720/https://community.nxp.com/docs/DOC-93277)


## License
This code is free to use and available under the GPLv3 license. Full license text for this project is located in the [LICENSE](./LICENSE) file.
