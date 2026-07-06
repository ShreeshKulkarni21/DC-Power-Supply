# STM32F407 PID Controlled DC Power Supply

A digitally controlled buck-converter based DC power supply built using the STM32F407 microcontroller. The output voltage is regulated using a closed-loop PID controller with ADC feedback synchronized to the PWM switching cycle.

## Features
- Closed-loop PID voltage regulation
- STM32F407 based controller
- Center-aligned PWM generation
- ADC sampling synchronized with PWM
- Settable output voltage and current (CV/CC mode)
- Soft-start implementation
- Duty cycle limiting and protection logic

## Hardware Used

- STM32F407 Discovery Board
- Buck Converter Topology for DC-DC Conversion
- Isolated MOSFET Driver (TLP250)
- Voltage Divider Feedback Network
- Op amp based Current to Voltage amplifier
- Output Capacitor and Inductor

## Software

- STM32CubeIDE
- C Programming

## Control Strategy

The controller samples the output voltage using the ADC at the center of each PWM period to minimize switching noise. The measured voltage is averaged before being fed into a PID controller. The PID computes the required duty cycle which updates the PWM driving the buck converter.



```
