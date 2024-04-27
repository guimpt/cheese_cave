# Cheese cave
## The project
Are you ready to embark on your cheese-making journey? Picture this: crafting your own artisanal cheeses in the comfort of your own home. But wait, before you dive into the world of curds and whey, let's talk about the indispensable tool you'll need – the cheese cave.

Cheese caves provide the optimal environment for cheese aging, ensuring that your creations develop the perfect flavor and texture. While many suggest repurposing a wine fridge for this purpose, I've found that their temperature control leaves much to be desired.

Introducing this project: a complete overhaul of the control electronics in your standard wine fridge, transforming it into a precise cheese cave with an astonishing accuracy of ±0.1ºC.

Here's what the project entails:
 - Electronics Design and Firmware: all the necessary files for replicating the project, from the circuit schematics to the firmware code, are provided.
 - Fabrication Files: easily order the semi-assembled PCB from JLCPCB and gather the remaining components from LCSC.

![Front Electronics](https://github.com/guimpt/cheese_cave/blob/develop/doc/Front.png)
![Front Electronics](https://github.com/guimpt/cheese_cave/blob/develop/doc/Back.png)

## Specifications:
 - Utilizes STM32F042 for reliable and cheap performance.
 - Implements precise circuitry to maintain temperature control with PT100, NTC, or SHT30 sensors.
 - Features two MOSFET outputs for controlling mist makers, in order to implement humidity regulation (currently not implemented in the provided firmware).
 - Boasts a user-friendly OLED display with large numerical readouts and a control bar indicating the control action.
 - Equipped with touch sensors for convenient adjustment of fridge temperature and lighting.

## Compatible Fridges:
 - The controller is compatible with numerous wine fridges on the market.
 - Specifically designed as a replacement for the 3FX-136 / QC1218B wine fridge digital controllers.
 - If your fridge sports two seven-segment displays and three touch buttons, chances are it's equipped with the same controller, making it a seamless upgrade.

![](https://github.com/guimpt/cheese_cave/blob/develop/doc/Final_result.jpg)





