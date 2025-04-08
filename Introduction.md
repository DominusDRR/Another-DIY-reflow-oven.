# Introduction

I needed to solder a PCB device whose process must be carried out using a correct temperature curve over time.

The cost of doing so, on request from a company that is dedicated to such processes, seemed very high for just a few units.

Initially I thought about buying a mini reflow oven, but I came across a video where several tests were done with said device where several problems and errors were shown both in the temperature and the times that each subprocess should last.

I found several options for DIY reflow oven controls on the internet, and I wanted to buy something from them, but they are discontinued, so after thinking about it a lot, I decided to do it myself.

This system is basically what is shown in the following image


![](https://pbs.twimg.com/media/GdlK7kVW0AAkiTA?format=png&name=medium)

The electric oven is basically an electric resistance that is heated.

To control the temperature, the controller activates a solid state electronic switch (SSR).

Also through a temperature sensor, which is a thermocouple, the system knows the temperature inside the oven.

The system also has a display (can be numeric, alphanumeric or graphic) to inform the user of the process being carried out and to calibrate or choose a work mode.

The keyboard allows to start, stop and configure the thermal reflow process.

The temperature should describe a curve over time similar to the one shown in this image. Also known as temperature profile.

![](https://ac-blog.panasonic.com/hs-fs/hubfs/20151130-1.png?width=700&name=20151130-1.png)

I have seen that the controller is of the PID type, and a microcontroller is generally used to carry out this control process, in addition to controlling and reading information from other devices.

