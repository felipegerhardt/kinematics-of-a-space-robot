# Kinematics and kinetics of a space robot - 2D Simulation
---

## Introduction

<br />

This repository aims to perform numerical simulation of a robot arm under different conditions using the following open source libraries:

- Sympy for symbolic mathematics;
- Scipy for scientific computing;
- Numpy for array manipulation;
- Pydy for multibody dynamics.

For animation and graphic visualization purposes, the following libraries are needed:

- Matplotlib;
- Pydy.viz.

---

## Inspiration

Aboard the International Space Station (ISS) there is a robotic system that plays a key role in station assembly and maintenance. The Mobile Servicing System (MSS) moves equipment, supplies, support astronauts, services instruments and other payloads. The MSS is composed of three main components:
- the Space Station Remote Manipulator System (SSRMS), known as Canadarm2, see Fig. 1.;
- the Mobile Remote Service Base System (MBS);
- the Special Purpose Dexterous Manipulator (SPDM - or "Dextre"/"Canada hand") [1].

<br />

<p align="center">
  <img img witdh="455" height="300" src="https://user-images.githubusercontent.com/60149913/105194211-ba1c8380-5b0f-11eb-8d72-e362e87db1cf.jpg">
</p>

<p align="center">
<strong>Fig. 1</strong> - Astronaut Stephen K. Robinson anchored to the end of Canadarm2 during STS-114, 2005. 
</p>

<br />

The focus of this simulaton is to model a simplificated version of the Canadarm2, which can be controled by astrounauts, by the ground team at the CSA headquarters or NASA. The Canadarm2 robotic arm can perform:
- Station maintenance;
- Movement of supplies, equipment, other robots and even astronauts;
- "Cosmic catches" by grappling visiting vehicles and berthing them to the ISS [2].

<br />

Therefore, the system has to be able to perform different movements including linear and rotational displacements among multiple axis. More technical information about the Canadarm2 can be seen in refernce [3].

---

## Modeling

To be able to model a two-dimensional version of Canadarm2, a lot of simplifications must be done. For instance, as seen in Fig. 2 the robotic arm has 7 degrees of freedom (DOF). The complexity of its movement is 

<p align="center">
  <img witdh="455" height="300" src="https://user-images.githubusercontent.com/60149913/105189997-f0580400-5b0b-11eb-99f7-35e8250f8d20.png">
</p>

## References
[1] https://en.wikipedia.org/wiki/Mobile_Servicing_System#Canadarm2
<br />
[2] https://www.nasa.gov/mission_pages/station/structure/elements/remote-manipulator-system-canadarm2/
<br /> 
[3] https://www.nasa.gov/mission_pages/station/structure/elements/subsystems.html