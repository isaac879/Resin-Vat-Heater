# Resin-Vat-Heater
This project is a vat heater for resin 3D printers. It is designed to keep the resin warm at an optimal temperature to ensure consistent and high quality prints.

The code is developed for a custom control board based on an ESP32-C3 and an E-Ink (E-Paper) display module.

Important:
The E-Ink display lines “RES” and “CS” are mislabelled on the control board and to have the wires swapped. This has already been implemented in the code (See: resin_vat_heater_control_board.ino Line 33/Pin declarations). This prevents the “RES” line from pulling GPIO2 low causing issues uploading the code.

Make sure the element resistance is correct in the code and matches the resistance of your heating elements (Line 90: ELEMENT_RESISTANCE). 

Project video: https://youtu.be/hyjGFaW69lo
 
