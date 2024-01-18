# Traffic Lights - Embedded Systems
This code is written for the main project of the course 
Embedded Systems [IS1300] at KTH, Royal Institute of Technology.

The task of the main project was to program a KTH proprietary traffic lights shield attached to a Nucleo-L476RG development board with a STM32L476RG MPU.
STM32L476RG is a single core microcontroller based on the Arm Cortex-M4 32-bit core, set for this project to operate at 80 Hz.
The proprietary traffic lights shield provided by KTH represented a street crossing with LED traffic lights and pedestrian lights, connected to three shift registers.
The traffic lights were to change depending on different traffic scenarios based on the various states of traffic and pedestrian activity. 

The scenarios were controlled by buttons resembling pedestrians at the street crossings and switches resembling vehicles on the roads, including an idle scenario where no pedestrians or traffic activity is present.

The system had to operate in realtime and be able to execute tasks concurrently, despite being executed on a single core. To acheive the perception of concurrency, FreeRTOS was implemented and used.

## Relevant Files For The Reader
- Core
  - Inc
    -  pro1_funct.h
    -  pro1_test.h
  - Src
    - freertos.c
    - main.c
    - pro1_input.c
    - pro1_lights.c
    - pro1_sysconfig.c
    - pro1_tests.c
    - pro1_traffic.c
    - stm32l4xx_it.c
- PRO1_Love_Mitteregger.ioc
- PinConfiguration.txt 

## License
MIT license is applied to all code written by my self (Love Mitteregger), else licenses provided by FreeRTOS and ST must be considered. Who wrote what is clearly denoted in the code segments.

## Extras
Only the code, with comments, for the project is included in this github repository.
Planning, project documentation and presentation, applied testing practices and other general project related deliverables were taken into consideration by the faculty when grading the project, these files are not included in this project repository.
