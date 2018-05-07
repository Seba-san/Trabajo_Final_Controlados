EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ESP8266
LIBS:adaptador_wifi-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ESP-12E U1
U 1 1 5A707B79
P 5800 3150
F 0 "U1" H 5800 3050 50  0000 C CNN
F 1 "ESP-12E" H 5800 3250 50  0000 C CNN
F 2 "ESP8266:ESP-12E_SMD" H 5800 3150 50  0001 C CNN
F 3 "" H 5800 3150 50  0001 C CNN
	1    5800 3150
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x08_Male J1
U 1 1 5A707C9B
P 4600 3150
F 0 "J1" H 4600 3550 50  0000 C CNN
F 1 "Conn_01x08_Male" H 4600 2650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 4600 3150 50  0001 C CNN
F 3 "" H 4600 3150 50  0001 C CNN
	1    4600 3150
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x08_Male J3
U 1 1 5A707DEE
P 7000 3250
F 0 "J3" H 7000 3650 50  0000 C CNN
F 1 "Conn_01x08_Male" H 7000 2750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 7000 3250 50  0001 C CNN
F 3 "" H 7000 3250 50  0001 C CNN
	1    7000 3250
	-1   0    0    1   
$EndComp
$Comp
L Conn_01x06_Male J2
U 1 1 5A707E35
P 5750 4400
F 0 "J2" H 5750 4700 50  0000 C CNN
F 1 "Conn_01x06_Male" H 5750 4000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 5750 4400 50  0001 C CNN
F 3 "" H 5750 4400 50  0001 C CNN
	1    5750 4400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4800 2850 4900 2850
Wire Wire Line
	4800 2950 4900 2950
Wire Wire Line
	4800 3050 4900 3050
Wire Wire Line
	4800 3150 4900 3150
Wire Wire Line
	4800 3250 4900 3250
Wire Wire Line
	4800 3350 4900 3350
Wire Wire Line
	4800 3450 4900 3450
Wire Wire Line
	4800 3550 4900 3550
Wire Wire Line
	5550 4200 5550 4050
Wire Wire Line
	5650 4200 5650 4050
Wire Wire Line
	5750 4200 5750 4050
Wire Wire Line
	5850 4200 5850 4050
Wire Wire Line
	5950 4200 5950 4050
Wire Wire Line
	6050 4050 6050 4200
Wire Wire Line
	6700 2850 6800 2850
Wire Wire Line
	6700 2950 6800 2950
Wire Wire Line
	6800 3050 6700 3050
Wire Wire Line
	6700 3150 6800 3150
Wire Wire Line
	6800 3250 6700 3250
Wire Wire Line
	6700 3350 6800 3350
Wire Wire Line
	6700 3450 6800 3450
Wire Wire Line
	6800 3550 6700 3550
$EndSCHEMATC
