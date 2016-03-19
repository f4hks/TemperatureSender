Sample X25 transponder for use with a transceiver ,in this release there is implemented:
X25 stack 
FSK Stack
LCD driver 
The futures plan are 
Use FX25 with reed salomon error correction (implementing a wrapper that use the fx25.c and .h ) 
Implementing demodulation of a fsk signal and decoding AX25 (and FX25 error correction) 
Write a state machine for synchronisation of the tasks 
Implement NMEA protocol (or use an external library with a wrapper) 
Hardware 
Draw the PCB for a final product 
Miscellanious 
Optimizing the code for reduce the flash occupation and ram (or tune the FreeRtos kernel) 
