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
	9050 4050 9050 4250
Wire Wire Line
	9600 4050 9600 4250
Connection ~ 9050 4650
Text Label 9600 4200 0    60   ~ 0
E2
Text Label 9050 4200 0    60   ~ 0
E1
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
	8400 4450 9300 4450
Connection ~ 8750 4450
Text Label 3950 4250 0    60   ~ 0
E1
Text Label 3450 4250 0    60   ~ 0
E2
Text Label 2900 4250 0    60   ~ 0
L1_a
Text Label 2350 4250 0    60   ~ 0
L2_a
Text Label 1850 4250 0    60   ~ 0
L3_a
Text Label 1350 4250 0    60   ~ 0
L4_a
Text Label 850  4250 0    60   ~ 0
L5_a
Wire Wire Line
	4700 1550 4850 1550
Wire Wire Line
	4700 1650 4950 1650
Text Label 2350 5600 0    60   ~ 0
L6_a
Text Label 1650 5600 0    60   ~ 0
L7_a
Text Label 850  5600 0    60   ~ 0
L8_a
Wire Wire Line
	2950 1850 3500 1850
Text Label 3100 1150 0    60   ~ 0
Comunicacion_TX_a
Text Label 3100 1250 0    60   ~ 0
Comunicacion_RX_a
Wire Wire Line
	3000 1150 3500 1150
Wire Wire Line
	3000 1250 3500 1250
Wire Wire Line
	3300 1950 3500 1950
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
P 5450 900
F 0 "C1" H 5475 1000 50  0000 L CNN
F 1 "CP" H 5475 800 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 5488 750 50  0001 C CNN
F 3 "" H 5450 900 50  0001 C CNN
	1    5450 900 
	1    0    0    -1  
$EndComp
Text Label 5450 1150 0    60   ~ 0
gnd_logic
Wire Wire Line
	5450 1050 5450 1150
$Comp
L C_Small C2
U 1 1 5A7C2558
P 5700 900
F 0 "C2" H 5710 970 50  0000 L CNN
F 1 "C_Small = 100n" H 5710 820 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 5700 900 50  0001 C CNN
F 3 "" H 5700 900 50  0001 C CNN
	1    5700 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 750  5700 800 
Wire Wire Line
	5700 1000 5700 1100
Wire Wire Line
	5700 1100 5450 1100
Connection ~ 5450 1100
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
Text Label 9400 1100 0    60   ~ 0
Vcc_bat
Text Label 9800 1200 0    60   ~ 0
gnd_logic
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
L Conn_01x03 J2
U 1 1 5AF0DEFE
P 2800 1250
F 0 "J2" H 2800 1450 50  0000 C CNN
F 1 "Conn_01x03" H 2800 1050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 2800 1250 50  0001 C CNN
F 3 "" H 2800 1250 50  0001 C CNN
	1    2800 1250
	-1   0    0    1   
$EndComp
$Comp
L D D1
U 1 1 5AF0E572
P 9850 1100
F 0 "D1" H 9850 1200 50  0000 C CNN
F 1 "D" H 9850 1000 50  0000 C CNN
F 2 "Diodes_SMD:D_1206" H 9850 1100 50  0001 C CNN
F 3 "" H 9850 1100 50  0001 C CNN
	1    9850 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 1100 10000 1100
Wire Wire Line
	9700 1100 9400 1100
Wire Wire Line
	9600 4650 8100 4650
Text Label 7500 4050 0    60   ~ 0
Vcc_5v
Wire Wire Line
	7500 4050 7750 4050
Connection ~ 7650 4050
Wire Wire Line
	4700 1450 5250 1450
Wire Wire Line
	5250 1450 5250 750 
Wire Wire Line
	5250 750  5700 750 
Wire Wire Line
	4700 1750 4950 1750
Wire Wire Line
	4950 1850 4700 1850
Wire Wire Line
	4700 2250 4950 2250
Wire Wire Line
	4950 2150 4700 2150
Wire Wire Line
	4700 2050 4950 2050
Wire Wire Line
	4950 1950 4700 1950
Text Label 850  3350 0    60   ~ 0
A6_l5_A
Text Label 1350 3350 0    60   ~ 0
A5_l4_A
Text Label 1850 3350 0    60   ~ 0
A4_l3_A
Text Label 2350 3350 0    60   ~ 0
A3_l2_A
Text Label 2900 3350 0    60   ~ 0
A2_l1_A
Text Label 3450 3350 0    60   ~ 0
A1_E2
Text Label 3950 3350 0    60   ~ 0
A0_E1
Text Label 850  4700 0    60   ~ 0
D2_L8_a
Text Label 1650 4700 0    60   ~ 0
D3_L7_a
Text Label 2350 4700 0    60   ~ 0
D4_L6_a
$Comp
L 74HCT04 U3
U 6 1 5AF29A21
P 2350 5150
F 0 "U3" H 2500 5250 50  0000 C CNN
F 1 "74HCT04" H 2550 5050 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 2350 5150 50  0001 C CNN
F 3 "" H 2350 5150 50  0001 C CNN
	6    2350 5150
	0    -1   -1   0   
$EndComp
$Comp
L 74HCT04 U3
U 5 1 5AF29B07
P 1650 5150
F 0 "U3" H 1800 5250 50  0000 C CNN
F 1 "74HCT04" H 1850 5050 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 1650 5150 50  0001 C CNN
F 3 "" H 1650 5150 50  0001 C CNN
	5    1650 5150
	0    -1   -1   0   
$EndComp
$Comp
L 74HCT04 U3
U 4 1 5AF29B81
P 850 5150
F 0 "U3" H 1000 5250 50  0000 C CNN
F 1 "74HCT04" H 1050 5050 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 850 5150 50  0001 C CNN
F 3 "" H 850 5150 50  0001 C CNN
	4    850  5150
	0    -1   -1   0   
$EndComp
$Comp
L 74HCT04 U2
U 1 1 5AF29BF4
P 850 3800
F 0 "U2" H 1000 3900 50  0000 C CNN
F 1 "74HCT04" H 1050 3700 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 850 3800 50  0001 C CNN
F 3 "" H 850 3800 50  0001 C CNN
	1    850  3800
	0    -1   -1   0   
$EndComp
$Comp
L 74HCT04 U2
U 2 1 5AF29CD2
P 1350 3800
F 0 "U2" H 1500 3900 50  0000 C CNN
F 1 "74HCT04" H 1550 3700 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 1350 3800 50  0001 C CNN
F 3 "" H 1350 3800 50  0001 C CNN
	2    1350 3800
	0    -1   -1   0   
$EndComp
$Comp
L 74HCT04 U2
U 3 1 5AF29D47
P 1850 3800
F 0 "U2" H 2000 3900 50  0000 C CNN
F 1 "74HCT04" H 2050 3700 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 1850 3800 50  0001 C CNN
F 3 "" H 1850 3800 50  0001 C CNN
	3    1850 3800
	0    -1   -1   0   
$EndComp
$Comp
L 74HCT04 U2
U 4 1 5AF29DC5
P 3450 3800
F 0 "U2" H 3600 3900 50  0000 C CNN
F 1 "74HCT04" H 3650 3700 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 3450 3800 50  0001 C CNN
F 3 "" H 3450 3800 50  0001 C CNN
	4    3450 3800
	0    -1   -1   0   
$EndComp
$Comp
L 74HCT04 U2
U 5 1 5AF29FD4
P 2900 3800
F 0 "U2" H 3050 3900 50  0000 C CNN
F 1 "74HCT04" H 3100 3700 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 2900 3800 50  0001 C CNN
F 3 "" H 2900 3800 50  0001 C CNN
	5    2900 3800
	0    -1   -1   0   
$EndComp
$Comp
L 74HCT04 U2
U 6 1 5AF2A050
P 2350 3800
F 0 "U2" H 2500 3900 50  0000 C CNN
F 1 "74HCT04" H 2550 3700 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 2350 3800 50  0001 C CNN
F 3 "" H 2350 3800 50  0001 C CNN
	6    2350 3800
	0    -1   -1   0   
$EndComp
$Comp
L 74HCT04 U3
U 3 1 5AF2A0D1
P 3950 3800
F 0 "U3" H 4100 3900 50  0000 C CNN
F 1 "74HCT04" H 4150 3700 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 3950 3800 50  0001 C CNN
F 3 "" H 3950 3800 50  0001 C CNN
	3    3950 3800
	0    -1   -1   0   
$EndComp
Text Label 3150 1550 0    60   ~ 0
D2_L8_a
Text Label 3150 1650 0    60   ~ 0
D3_L7_a
Text Label 3150 1750 0    60   ~ 0
D4_L6_a
Wire Wire Line
	3150 1550 3500 1550
Wire Wire Line
	3500 1650 3150 1650
Wire Wire Line
	3150 1750 3500 1750
Text Label 4950 2250 0    60   ~ 0
A0_E1
Text Label 4950 2150 0    60   ~ 0
A1_E2
Text Label 4950 2050 0    60   ~ 0
A2_l1_A
Text Label 4950 1950 0    60   ~ 0
A3_l2_A
Text Label 4950 1850 0    60   ~ 0
A4_l3_A
Text Label 4950 1750 0    60   ~ 0
A5_l4_A
Text Label 4950 1650 0    60   ~ 0
A6_l5_A
$EndSCHEMATC
