# Kinematics and kinetics of a space robot - 2D Simulation
---

## Introduction

<br />

This repository (still in progress) aims to perform numerical simulations of a simplificated version of a robot arm performing one specific movement under different gravity conditions. In adittion, the energy consumption is compared for all cases. The following Python open source libraries were assessed to perform the simulations:

- SymPy for symbolic mathematics;
- Scipy for scientific computing;
- Numpy for array manipulation;
- Pydy for multibody dynamics.

For animation and graphic visualization purposes, the following libraries are needed:

- Matplotlib.Pyplot;
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
<strong>Fig. 1</strong> - Astronaut Stephen K. Robinson anchored to the end of Canadarm2 during STS-114, 2005 [1]. 
</p>

<br />

The focus of this simulaton is to model a simplificated version of the Canadarm2, which can be controled by astrounauts, by the ground team at the CSA headquarters or NASA. The Canadarm2 robotic arm can perform:
- Station maintenance;
- Movement of supplies, equipment, other robots and even astronauts;
- "Cosmic catches" by grappling visiting vehicles and berthing them to the ISS [2].

<br />

Therefore, in order to accomplish these complex tasks, the system has to be able to perform different movements including linear and rotational displacements among multiple axis. More technical information about the Canadarm2 can be seen in refernce [3].

---

## Modeling

To be able to model a two-dimensional version of Canadarm2, a lot of simplifications must be done. As seen in Fig. 2 and Fig. 3, the robotic arm has 7 degrees of freedom (DOF), which can be challenging for modelling tasks and computational calculations. 

<br />

<p align="center">
  <img src="https://user-images.githubusercontent.com/60149913/105221882-18f1f500-5b30-11eb-9fb5-a7fb1d0d2857.png">
</p>

<p align="center">
<strong>Fig. 2</strong> - 3D representation of Canadarm2 and its joints [4].
</p>

<br />

<p align="center">
  <img src="https://user-images.githubusercontent.com/60149913/105223551-4dff4700-5b32-11eb-8785-6eee1a6fa936.jpg">
</p>

<p align="center">
<strong>Fig. 3</strong> - Canadarm2 diagram [5].
</p>

<br />

As the aim of this study is to evaluate the kinematics, kinetics and the energy consumption under different gravity conditions, the simplificated model is considered to be two-dimensional and the DOF are simplified as show Fig. 4. The 5 DOF characterize the movement of the Lower Arm, Upper Arm, Hand, Finger 1 (bottom) and Finger 2 (top).

<br />

<p align="center">
  <img witdh="455" height="300" src="https://user-images.githubusercontent.com/60149913/105635696-a4260000-5e3a-11eb-9f30-8431146db0ed.png">
</p>

<br />

<p align="center">
<strong>Fig. 4</strong> - The 2D model schematic and DOFs.
</p>


## References
[1] https://en.wikipedia.org/wiki/Mobile_Servicing_System#Canadarm2
<br />
[2] https://www.nasa.gov/mission_pages/station/structure/elements/remote-manipulator-system-canadarm2/
<br /> 
[3] https://www.nasa.gov/mission_pages/station/structure/elements/subsystems.html
<br /> 
[4] P. Fournier-Viger, R. Nkambou and A. Mayers, "Evaluating Spatial Representations and Skills in a Simulator-Based Tutoring System," in IEEE Transactions on Learning Technologies, vol. 1, no. 1, pp. 63-74, Jan.-March 2008, doi: 10.1109/TLT.2008.13. 
<br /> 
[5] https://spaceq.ca/spacewalk-needed-to-replace-another-degraded-canadarm2-hand-on-the-international-space-station/