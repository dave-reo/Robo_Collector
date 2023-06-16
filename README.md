# Robo_Collector

## Introduction
The Robo_Collector was inspired by the classic "Hungry Hippos" game where players complete to collect as many marbles as possible. However, our goal was to take this concept to the next level by creating a sophisticated robot capable of finding, picking up, and sorting ping pong balls. We aimed to develop the mechanical, electrical, and software systems for this robot by using concepts learned in the mechatronics concentration at Cal Poly.

## Mechanical Design
We began by creating a preliminary CAD model of the robot frame, but had to make necessary adjustments due to considerations such as PCB sizing and sensor placement, which influenced the design of individual components. To bring our vision to life, we utilized available 3D printers and crafted the entire frame using PLA material. The integration of different pieces was achieved through a combination of fasteners and adhesive bonds. Throughout the development process, we underwent multiple iterations for each component, including the base, ball retriever, ramp, and motor container.

One of the main challenges we encountered was ensuring that the ball retriever could handle the torque without allowing the ping pong balls to escape from the frame. To address this, we decided to reprint the ramp and base board, incorporating guard rails to keep the balls within the desired boundaries. While we were successful in capturing some ping pong balls, we faced difficulties in achieving instant torque for the motor due to the high friction associated with PLA. To overcome this, we initially employed a higher duty cycle to get the ball retriever spinning, and then gradually adjusted it to a suitable level considering the lightweight components.

Despite these efforts, we discovered that the ball retriever would dislocate from the collar supports on the ramp after approximately 5 seconds of spinning. To mitigate this issue in future iterations, we plan to extend the collars by 10 millimeters to account for any lateral translation that may occur. This adjustment will provide the necessary stability and ensure that the ball retriever remains securely attached to the ramp throughout its operation.

## Electronic Design
Our circuit board design catered to a 12V battery, featuring a 12V to 5V buck regulator to support various sensors and a 5V to 3.3V linear regulator for the STM32F411 microcontroller, which we chose for its versatility. To ensure flexibility, we also incorporated the option to use the STM32F411-based Blackpill Development Board if our PCB encountered any issues. Notable elements on the board included an indicator light for power status, a 3A fuse for battery protection, a MOSFET for reverse polarity, a 20MHz crystal, and three motor drivers. We soldered the components onto the board using a stencil and solder paste, with the hindsight that a frame would have saved time during the solder paste application. Additionally, labeling the ordered circuit components according to the PCB labels would have provided further time savings. 

Testing the board was an iterative process, involving techniques such as continuity testing and isopropyl alcohol application to locate shorts. Although we successfully addressed some shorts, the buck regulator and linear regulator remained problematic. After extensive testing and component replacements, we suspected a possible defect in the board itself, leading us to switch to the Blackpill for a functional robot within our tight timeline. Given more time, we would have further investigated component specifications and placement to identify the root cause of the short.

## Software Design
To meet the tight deadline and avoid unnecessarily complex code, our software implementation followed a straightforward approach, utilizing pre-written drivers whenever possible. Key code elements, written in C, included the motor driver, ultrasonic sensor setup, line sensor configuration, and the start/stop ("Deadman's") switch. The motor driver, adapted from a previous lab, featured essential functions for enabling, disabling, and controlling the motor object's duty cycle. The Deadman's switch, triggered by a transmitter input, set flags to enable or disable the motors accordingly. Another important function, HAL_TIM_IC_CaptureCallback, processed timer information and executed expected behaviors. The ultrasonic sensor, integrated using a pre-written driver, employed the Is_Dead flag to prevent collisions by disabling the motors when the calculated distance exceeded a threshold. The line sensors, initially analog but later switched to digital for efficiency, relied on GPIO readings and basic logic to detect lines. The main loop encapsulated continuous readings from the ultrasonic and line sensors, incorporating a FSM for motor control and driving logic based on line detection.

## Results
We successfully developed various individual functionalities for the robot, including line sensors, ultrasonic sensors, and motor drivers. However, when it came to integrating all these components together, we faced challenges and were unable to complete the ball sorting mechanism and its corresponding code within the given time constraints. Given more time, we would have revised both the mechanical and electrical design to create a more functional prototype on the next iteration.
