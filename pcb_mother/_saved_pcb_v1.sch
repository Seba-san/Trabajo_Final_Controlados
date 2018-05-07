EESchema Schematic File Version 2
LIBS:pcb_v1-rescue
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
LIBS:pcb_v1-cache
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
L ARDUINO_NANO ARDUINO1
U 1 1 5A709ECE
P 4100 1900
F 0 "ARDUINO1" H 4100 2750 70  0000 C CNN
F 1 "ARDUINO_NANO" V 4100 1900 70  0000 C CNN
F 2 "Propias:Arduino_Nano" H 4100 1900 60  0001 C CNN
F 3 "" H 4100 1900 60  0000 C CNN
	1    4100 1900
	1    0    0    -1  
$EndComp
Text Label 2950 1450 0    60   ~ 0
gnd_logic
Text Label 2900 2800 0    60   ~ 0
gnd_logic
Text Label 900  2650 0    60   ~ 0
gnd_motores
Text Label 4800 1150 0    60   ~ 0
Vcc_bat
Text Label 4800 1250 0    60   ~ 0
gnd_logic
Text Label 1000 1950 0    60   ~ 0
Vcc_bat
Text Label 4900 1450 0    60   ~ 0
Vcc_5v
$Comp
L BC548 Q11
U 1 1 5A78AADE
P 8950 4450
F 0 "Q11" H 9150 4525 50  0000 L CNN
F 1 "2N2219" H 9150 4450 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 9150 4375 50  0001 L CIN
F 3 "" H 8950 4450 50  0001 L CNN
	1    8950 4450
	1    0    0    -1  
$EndComp
$Comp
L BC548 Q12
U 1 1 5A78AAEB
P 9500 4450
F 0 "Q12" H 9700 4525 50  0000 L CNN
F 1 "2N2219" H 9700 4450 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 9700 4375 50  0001 L CIN
F 3 "" H 9500 4450 50  0001 L CNN
	1    9500 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2250 3500 2250
Wire Wire Line
	3250 2250 3250 1950
Wire Wire Line
	3250 1950 2900 1950
Wire Wire Line
	2900 2550 3250 2550
Wire Wire Line
	3250 2550 3250 2350
Wire Wire Line
	3250 2350 3500 2350
Wire Wire Line
	3500 2600 3500 2550
Wire Wire Line
	1350 2050 900  2050
Wire Wire Line
	2900 2650 2900 2800
Wire Wire Line
	950  1950 1350 1950
Wire Wire Line
	900  2650 1350 2650
Wire Wire Line
	4700 1150 4800 1150
Wire Wire Line
	4700 1250 4800 1250
Wire Wire Line
	900  2550 1350 2550
Wire Wire Line
	4700 1450 5600 1450
Wire Wire Line
	9050 4050 9050 4250
Wire Wire Line
	9600 4050 9600 4250
Connection ~ 9050 4650
Text Label 9600 4200 0    60   ~ 0
E2
Text Label 9050 4200 0    60   ~ 0
E1
$Comp
L BC548 Q13
U 1 1 5A78D80A
P 1750 4200
F 0 "Q13" H 1950 4275 50  0000 L CNN
F 1 "2N2219" H 1950 4200 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 1950 4125 50  0001 L CIN
F 3 "" H 1750 4200 50  0001 L CNN
	1    1750 4200
	-1   0    0    -1  
$EndComp
Text Label 1650 4600 0    60   ~ 0
gnd_logic
Wire Wire Line
	1650 4000 1950 4000
Wire Wire Line
	1950 4000 1950 4200
Wire Wire Line
	1650 4400 1650 4600
Wire Wire Line
	1650 3400 1650 3550
Wire Wire Line
	1650 3850 1650 4000
$Comp
L R R7
U 1 1 5A78E300
P 1650 3700
F 0 "R7" V 1730 3700 50  0000 C CNN
F 1 "220" V 1650 3700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1580 3700 50  0001 C CNN
F 3 "" H 1650 3700 50  0001 C CNN
	1    1650 3700
	1    0    0    -1  
$EndComp
$Comp
L BC548 Q14
U 1 1 5A78E756
P 2150 4200
F 0 "Q14" H 2350 4275 50  0000 L CNN
F 1 "2N2219" H 2350 4200 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 2350 4125 50  0001 L CIN
F 3 "" H 2150 4200 50  0001 L CNN
	1    2150 4200
	1    0    0    -1  
$EndComp
Text Label 2250 4500 0    60   ~ 0
gnd_logic
Wire Wire Line
	2250 4400 2250 4500
Text Label 2250 3650 0    60   ~ 0
Vcc_bat
$Comp
L R R1
U 1 1 5A7BCF15
P 6500 1250
F 0 "R1" V 6580 1250 50  0000 C CNN
F 1 "6k8" V 6500 1250 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6430 1250 50  0001 C CNN
F 3 "" H 6500 1250 50  0001 C CNN
	1    6500 1250
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5A7BCF96
P 6500 1700
F 0 "R2" V 6580 1700 50  0000 C CNN
F 1 "10k" V 6500 1700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6430 1700 50  0001 C CNN
F 3 "" H 6500 1700 50  0001 C CNN
	1    6500 1700
	1    0    0    -1  
$EndComp
Text Label 6500 2000 3    60   ~ 0
gnd_logic
Wire Wire Line
	6500 2000 6500 1850
Text Label 6500 950  0    60   ~ 0
Vcc_bat
Wire Wire Line
	6500 950  6500 1100
Wire Wire Line
	6500 1400 6500 1550
Text Label 6500 1500 0    60   ~ 0
Nivel_bat
Text Label 4850 1550 0    60   ~ 0
Nivel_bat
Wire Wire Line
	4700 2250 4850 2250
Text Notes 6500 700  0    60   ~ 0
Es mejor llevar 8.4 a 5 que 6 a 0V porque le baja la amplitud de medicion.
$Comp
L BC548 Q10
U 1 1 5A7BF4CE
P 8200 4450
F 0 "Q10" H 8400 4525 50  0000 L CNN
F 1 "2N2219" H 8400 4450 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 8400 4375 50  0001 L CIN
F 3 "" H 8200 4450 50  0001 L CNN
	1    8200 4450
	-1   0    0    -1  
$EndComp
Text Label 8850 4800 0    60   ~ 0
gnd_logic
Text Label 6700 4050 0    60   ~ 0
Encoders
Text Label 8400 4250 0    60   ~ 0
Vref_encoders
Wire Wire Line
	8100 4250 8400 4250
Wire Wire Line
	8400 4250 8400 4450
Wire Wire Line
	8100 4050 8100 4250
$Comp
L R R6
U 1 1 5A7BF622
P 7900 4050
F 0 "R6" V 7980 4050 50  0000 C CNN
F 1 "100k" V 7900 4050 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7830 4050 50  0001 C CNN
F 3 "" H 7900 4050 50  0001 C CNN
	1    7900 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	7450 4650 9600 4650
Wire Wire Line
	8400 4450 9300 4450
Connection ~ 8750 4450
Text Label 4850 2250 0    60   ~ 0
E1
Text Label 4850 2150 0    60   ~ 0
E2
Wire Wire Line
	4700 2050 4850 2050
Wire Wire Line
	4850 2150 4700 2150
Text Label 4850 2050 0    60   ~ 0
L1_a
Text Label 4850 1950 0    60   ~ 0
L2_a
Text Label 4850 1850 0    60   ~ 0
L3_a
Text Label 4850 1750 0    60   ~ 0
L4_a
Text Label 4850 1650 0    60   ~ 0
L5_a
Wire Wire Line
	4700 1550 4850 1550
Wire Wire Line
	4850 1650 4700 1650
Wire Wire Line
	4700 1750 4850 1750
Wire Wire Line
	4850 1850 4700 1850
Wire Wire Line
	4850 1950 4700 1950
Text Label 3300 1750 0    60   ~ 0
L6_a
Text Label 3300 1650 0    60   ~ 0
L7_a
Text Label 3300 1550 0    60   ~ 0
L8_a
Wire Wire Line
	3300 1650 3500 1650
Wire Wire Line
	3300 1750 3500 1750
Wire Wire Line
	2950 1850 3500 1850
Text Label 3150 2450 0    60   ~ 0
Encoders
Text Label 4750 2550 0    60   ~ 0
S_linea
Text Label 3100 1150 0    60   ~ 0
Comunicacion_TX_a
Text Label 3100 1250 0    60   ~ 0
Comunicacion_RX_a
Wire Wire Line
	3000 1150 3500 1150
Wire Wire Line
	3000 1250 3500 1250
Wire Wire Line
	3500 2450 3150 2450
Wire Wire Line
	3300 1950 3500 1950
$Comp
L R R5
U 1 1 5A7BE257
P 7200 4050
F 0 "R5" V 7280 4050 50  0000 C CNN
F 1 "100k" V 7200 4050 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7130 4050 50  0001 C CNN
F 3 "" H 7200 4050 50  0001 C CNN
	1    7200 4050
	0    1    1    0   
$EndComp
$Comp
L C C6
U 1 1 5A7BE2F7
P 7450 4200
F 0 "C6" H 7475 4300 50  0000 L CNN
F 1 "2.2u" H 7475 4100 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 7488 4050 50  0001 C CNN
F 3 "" H 7450 4200 50  0001 C CNN
	1    7450 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 4350 7450 4650
Wire Wire Line
	7350 4050 7750 4050
Connection ~ 7450 4050
Wire Wire Line
	7050 4050 6700 4050
Text Notes 8750 3650 0    60   ~ 0
Fotodiodos de encoders\n\n
$Comp
L C_Small C3
U 1 1 5A7C0F86
P 1450 2300
F 0 "C3" H 1460 2370 50  0000 L CNN
F 1 "C_Small = 100n" H 1460 2220 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 1450 2300 50  0001 C CNN
F 3 "" H 1450 2300 50  0001 C CNN
	1    1450 2300
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 5A7C13E7
P 1200 2500
F 0 "C4" H 1210 2570 50  0000 L CNN
F 1 "C_Small = 100n" H 1210 2420 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 1200 2500 50  0001 C CNN
F 3 "" H 1200 2500 50  0001 C CNN
	1    1200 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  2350 1350 2350
Wire Wire Line
	900  2450 1350 2450
$Comp
L CP C1
U 1 1 5A7C1F25
P 5350 1600
F 0 "C1" H 5375 1700 50  0000 L CNN
F 1 "CP" H 5375 1500 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 5388 1450 50  0001 C CNN
F 3 "" H 5350 1600 50  0001 C CNN
	1    5350 1600
	1    0    0    -1  
$EndComp
Text Label 5350 1850 0    60   ~ 0
gnd_logic
Wire Wire Line
	5350 1750 5350 1850
$Comp
L C_Small C2
U 1 1 5A7C2558
P 5600 1600
F 0 "C2" H 5610 1670 50  0000 L CNN
F 1 "C_Small = 100n" H 5610 1520 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 5600 1600 50  0001 C CNN
F 3 "" H 5600 1600 50  0001 C CNN
	1    5600 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 1450 5600 1500
Connection ~ 5350 1450
Wire Wire Line
	5600 1700 5600 1800
Wire Wire Line
	5600 1800 5350 1800
Connection ~ 5350 1800
Connection ~ 1950 4200
$Comp
L Conn_01x02 J1
U 1 1 5A7CF181
P 10300 1100
F 0 "J1" H 10300 1200 50  0000 C CNN
F 1 "Conn_01x02" H 10300 900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 10300 1100 50  0001 C CNN
F 3 "" H 10300 1100 50  0001 C CNN
	1    10300 1100
	1    0    0    -1  
$EndComp
Text Label 9800 1100 0    60   ~ 0
Vcc_bat
Text Label 9800 1200 0    60   ~ 0
gnd_logic
Wire Wire Line
	9800 1100 10100 1100
Wire Wire Line
	10100 1200 9800 1200
$Comp
L Conn_01x02 J3
U 1 1 5A7CFAC2
P 10300 1500
F 0 "J3" H 10300 1600 50  0000 C CNN
F 1 "Conn_01x02" H 10300 1300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 10300 1500 50  0001 C CNN
F 3 "" H 10300 1500 50  0001 C CNN
	1    10300 1500
	1    0    0    -1  
$EndComp
Text Label 9750 1500 0    60   ~ 0
gnd_logic
Text Label 9750 1600 0    60   ~ 0
gnd_motores
Wire Wire Line
	9750 1600 10100 1600
Wire Wire Line
	10100 1500 9750 1500
Wire Wire Line
	2950 1450 3500 1450
Wire Wire Line
	8850 4800 8850 4650
Connection ~ 8850 4650
Wire Wire Line
	8050 4050 8100 4050
Wire Wire Line
	9050 3850 9600 3850
Connection ~ 8100 4650
Text Label 9200 3800 0    60   ~ 0
Vcc_5v
Wire Wire Line
	9200 3800 9200 3850
Connection ~ 9200 3850
Text Label 1650 3400 0    60   ~ 0
Vcc_5v
Wire Wire Line
	2900 2450 3100 2450
$Comp
L Conn_01x02_Male J4
U 1 1 5A7DF955
P 700 2250
F 0 "J4" H 700 2350 50  0000 C CNN
F 1 "Conn_01x02_Male" H 700 2050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 700 2250 50  0001 C CNN
F 3 "" H 700 2250 50  0001 C CNN
	1    700  2250
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x02_Male J5
U 1 1 5A7DFB7A
P 700 2450
F 0 "J5" H 700 2550 50  0000 C CNN
F 1 "Conn_01x02_Male" H 700 2250 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 700 2450 50  0001 C CNN
F 3 "" H 700 2450 50  0001 C CNN
	1    700  2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  2250 1350 2250
Wire Wire Line
	900  2150 1350 2150
Text Label 900  2050 0    60   ~ 0
Vcc_5v
Wire Wire Line
	1200 2400 1200 2450
Connection ~ 1200 2450
Wire Wire Line
	1200 2600 1200 2550
Connection ~ 1200 2550
Wire Wire Line
	1450 2200 1300 2200
Wire Wire Line
	1300 2200 1300 2250
Connection ~ 1300 2250
Wire Wire Line
	1300 2350 1300 2400
Wire Wire Line
	1300 2400 1450 2400
Connection ~ 1300 2350
Text Label 950  2150 0    60   ~ 0
gnd_motores
$Comp
L TB6612F U1
U 1 1 5A7E9351
P 2150 2350
F 0 "U1" H 2700 2950 60  0000 C CNN
F 1 "TB6612F" V 2150 2400 60  0000 C CNN
F 2 "Propias:tb6612" H 2150 2350 60  0001 C CNN
F 3 "" H 2150 2350 60  0001 C CNN
	1    2150 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 2450 3100 2600
Wire Wire Line
	3100 2600 3500 2600
Wire Wire Line
	2900 2350 3150 2350
Wire Wire Line
	3150 2350 3150 2150
Wire Wire Line
	3150 2150 3500 2150
Wire Wire Line
	2900 2250 3100 2250
Wire Wire Line
	3100 2250 3100 2050
Wire Wire Line
	3100 2050 3500 2050
Wire Wire Line
	2900 2150 3050 2150
Wire Wire Line
	3050 2150 3050 2000
Wire Wire Line
	3050 2000 3300 2000
Wire Wire Line
	3300 2000 3300 1950
Wire Wire Line
	2950 1850 2950 2050
Wire Wire Line
	2950 2050 2900 2050
Wire Wire Line
	4700 2550 4750 2550
Wire Wire Line
	3300 1550 3500 1550
$Comp
L Rws433mhz U2
U 1 1 5A7F1A3C
P 9800 2400
F 0 "U2" H 9800 2050 60  0000 C CNN
F 1 "Rws433mhz" H 9850 2700 60  0000 C CNN
F 2 "Propias:rws433" H 9800 2400 60  0001 C CNN
F 3 "" H 9800 2400 60  0001 C CNN
	1    9800 2400
	1    0    0    -1  
$EndComp
$Comp
L tws U3
U 1 1 5A7F2EC5
P 9900 3100
F 0 "U3" H 9900 2850 60  0000 C CNN
F 1 "tws" H 9900 3500 60  0000 C CNN
F 2 "Propias:tws433" H 9900 3100 60  0001 C CNN
F 3 "" H 9900 3100 60  0001 C CNN
	1    9900 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 2200 9100 2200
Wire Wire Line
	9100 2200 9100 3100
Wire Wire Line
	9100 3100 9350 3100
Wire Wire Line
	8200 2300 9350 2300
Wire Wire Line
	8200 2900 9350 2900
Text Label 8550 2300 0    60   ~ 0
Comunicacion_RX
Text Label 8600 2900 0    60   ~ 0
Comunicacion_TX
Text Label 9150 2500 0    60   ~ 0
Vcc_5v
Wire Wire Line
	9150 2500 9350 2500
Text Label 9150 3000 0    60   ~ 0
Vcc_bat
Wire Wire Line
	9150 3000 9350 3000
Text Label 9100 2750 0    60   ~ 0
gnd_logic
$Comp
L Conn_01x01 J7
U 1 1 5A7F6230
P 8800 2600
F 0 "J7" H 8800 2700 50  0000 C CNN
F 1 "Conn_01x01" H 8800 2500 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 8800 2600 50  0001 C CNN
F 3 "" H 8800 2600 50  0001 C CNN
	1    8800 2600
	-1   0    0    1   
$EndComp
$Comp
L Conn_01x01 J8
U 1 1 5A7F632A
P 8800 3200
F 0 "J8" H 8800 3300 50  0000 C CNN
F 1 "Conn_01x01" H 8800 3100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 8800 3200 50  0001 C CNN
F 3 "" H 8800 3200 50  0001 C CNN
	1    8800 3200
	-1   0    0    1   
$EndComp
Wire Wire Line
	9000 2600 9350 2600
Wire Wire Line
	9000 3200 9350 3200
$Comp
L Conn_01x02 J9
U 1 1 5A8084FE
P 9800 3950
F 0 "J9" H 9800 4050 50  0000 C CNN
F 1 "Conn_01x02" H 9800 3750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 9800 3950 50  0001 C CNN
F 3 "" H 9800 3950 50  0001 C CNN
	1    9800 3950
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x02 J10
U 1 1 5A8086E0
P 8850 4050
F 0 "J10" H 8850 4150 50  0000 C CNN
F 1 "Conn_01x02" H 8850 3850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 8850 4050 50  0001 C CNN
F 3 "" H 8850 4050 50  0001 C CNN
	1    8850 4050
	-1   0    0    1   
$EndComp
Wire Wire Line
	9050 3950 9050 3850
Wire Wire Line
	9600 3850 9600 3950
Text Label 5650 4450 0    60   ~ 0
L8_a
Text Label 5650 4350 0    60   ~ 0
L7_a
Text Label 5650 4250 0    60   ~ 0
L6_a
Text Label 6100 3100 0    60   ~ 0
L5_a
Text Label 6100 3200 0    60   ~ 0
L4_a
Text Label 6100 3300 0    60   ~ 0
L3_a
Text Label 6100 3400 0    60   ~ 0
L2_a
Text Label 6100 3500 0    60   ~ 0
L1_a
Text Label 6100 3000 0    60   ~ 0
Vcc_5v
Text Label 5650 3950 0    60   ~ 0
gnd_logic
Text Label 5650 4050 0    60   ~ 0
Vcc_bat
Wire Wire Line
	5650 4350 5850 4350
Wire Wire Line
	5850 4250 5650 4250
Wire Wire Line
	5650 4150 5850 4150
Wire Wire Line
	5650 4050 5850 4050
Wire Wire Line
	5850 3950 5650 3950
Wire Wire Line
	6100 3500 6300 3500
Wire Wire Line
	6300 3400 6100 3400
Wire Wire Line
	6100 3300 6300 3300
Wire Wire Line
	6100 3000 6300 3000
Wire Wire Line
	6300 3100 6100 3100
Wire Wire Line
	6100 3200 6300 3200
Text Label 5650 4150 0    60   ~ 0
S_lin_ref_a
Wire Wire Line
	5650 4450 5850 4450
$Comp
L Conn_01x02 J12
U 1 1 5A81776A
P 8000 2850
F 0 "J12" H 8000 2950 50  0000 C CNN
F 1 "Conn_01x02" H 8000 2650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 8000 2850 50  0001 C CNN
F 3 "" H 8000 2850 50  0001 C CNN
	1    8000 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	8200 2300 8200 2750
Wire Wire Line
	8200 2850 8200 2900
$Comp
L Conn_01x02 J13
U 1 1 5A80E04C
P 2450 3800
F 0 "J13" H 2450 3900 50  0000 C CNN
F 1 "Conn_01x02" H 2450 3600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 2450 3800 50  0001 C CNN
F 3 "" H 2450 3800 50  0001 C CNN
	1    2450 3800
	1    0    0    -1  
$EndComp
Connection ~ 2250 3900
Connection ~ 2250 3800
Wire Wire Line
	2250 3650 2250 3800
Wire Wire Line
	2250 4000 2250 3900
$Comp
L Conn_01x06 J14
U 1 1 5A8111EF
P 6050 4150
F 0 "J14" H 6050 4450 50  0000 C CNN
F 1 "Conn_01x06" H 6050 3750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 6050 4150 50  0001 C CNN
F 3 "" H 6050 4150 50  0001 C CNN
	1    6050 4150
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J11
U 1 1 5A8112C3
P 6500 3200
F 0 "J11" H 6500 3500 50  0000 C CNN
F 1 "Conn_01x06" H 6500 2800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 6500 3200 50  0001 C CNN
F 3 "" H 6500 3200 50  0001 C CNN
	1    6500 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1450 3000 1350
Connection ~ 3000 1450
$Comp
L Conn_01x03 J?
U 1 1 5AF0DEFE
P 2800 1250
F 0 "J?" H 2800 1450 50  0000 C CNN
F 1 "Conn_01x03" H 2800 1050 50  0000 C CNN
F 2 "" H 2800 1250 50  0001 C CNN
F 3 "" H 2800 1250 50  0001 C CNN
	1    2800 1250
	-1   0    0    1   
$EndComp
$EndSCHEMATC
