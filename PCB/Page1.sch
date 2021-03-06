EESchema Schematic File Version 4
LIBS:PCB_Rev3-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 7
Title "CC3220MOD Schematic"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L XBP9B-DPST-001:CC3220MODSFMOB U2
U 1 1 5BBD8C52
P 5400 4750
F 0 "U2" H 5400 7138 60  0000 C CNN
F 1 "CC3220MODSFMOB" H 5400 7032 60  0000 C CNN
F 2 "WiFiMapSpecific:CC3220MODSF12MOBR" H 5400 4690 60  0001 C CNN
F 3 "" H 5400 4750 60  0000 C CNN
	1    5400 4750
	1    0    0    -1  
$EndComp
$Comp
L WiFiMapper_R1-cache:DEA202450BT-1294C1-H U1
U 1 1 5BC45518
P 3650 4600
F 0 "U1" H 3650 4831 60  0000 C CNN
F 1 "DEA202450BT-1294C1-H" H 3650 4831 60  0001 C CNN
F 2 "WiFiMapSpecific:DEA202450BT-1294C1-H" H 3650 4890 60  0001 C CNN
F 3 "" H 2650 4600 60  0000 C CNN
	1    3650 4600
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5BC4551F
P 3650 4850
F 0 "#PWR0101" H 3650 4600 50  0001 C CNN
F 1 "GND" H 3655 4677 50  0000 C CNN
F 2 "" H 3650 4850 50  0001 C CNN
F 3 "" H 3650 4850 50  0001 C CNN
	1    3650 4850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3350 4650 3300 4650
Wire Wire Line
	3300 4650 3300 4850
Wire Wire Line
	3300 4850 3650 4850
Wire Wire Line
	3950 4650 4000 4650
Wire Wire Line
	4000 4650 4000 4850
Wire Wire Line
	4000 4850 3650 4850
Connection ~ 3650 4850
$Comp
L Connector_Specialized:Conn_Coaxial J4
U 1 1 5BC4552F
P 2050 4550
F 0 "J4" H 2150 4526 50  0000 L CNN
F 1 "WIFI SMA" H 2150 4435 50  0000 L CNN
F 2 "WiFiMapSpecific:Amphenol-132289rp" H 2050 4550 50  0001 C CNN
F 3 "" H 2050 4550 50  0001 C CNN
	1    2050 4550
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5BC45536
P 2050 4950
F 0 "#PWR0102" H 2050 4700 50  0001 C CNN
F 1 "GND" H 2055 4777 50  0000 C CNN
F 2 "" H 2050 4950 50  0001 C CNN
F 3 "" H 2050 4950 50  0001 C CNN
	1    2050 4950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2050 4750 2050 4950
Wire Wire Line
	3350 4550 2200 4550
Text GLabel 6900 2850 2    50   Input ~ 0
CC_UART0_TX
Text GLabel 6900 2950 2    50   Input ~ 0
CC_UART0_RX
Wire Wire Line
	6600 2950 6900 2950
Wire Wire Line
	6600 2850 6900 2850
Text GLabel 6900 2750 2    50   Input ~ 0
STATUS_LED_1
Wire Wire Line
	6600 2750 6850 2750
Text GLabel 6900 3050 2    50   Input ~ 0
STATUS_LED_2
Wire Wire Line
	6600 3050 6700 3050
Text GLabel 6900 3150 2    50   Input ~ 0
CC_ADC_CH2
Wire Wire Line
	6600 3150 6900 3150
Text GLabel 6900 3250 2    50   Input ~ 0
CC_ADC_CH3
Wire Wire Line
	6600 3250 6900 3250
Text GLabel 6900 3350 2    50   Input ~ 0
CC_GPIO6
Wire Wire Line
	6600 3350 6900 3350
Text GLabel 6900 3450 2    50   Input ~ 0
CC_GPIO7
Wire Wire Line
	6600 3450 6900 3450
Text GLabel 6900 3550 2    50   Input ~ 0
CC_GPIO8
Wire Wire Line
	6600 3550 6900 3550
Text GLabel 6900 3650 2    50   Input ~ 0
CC_GPIO9
Wire Wire Line
	6600 3650 6900 3650
Text GLabel 6900 3750 2    50   Input ~ 0
CC_SCL0
Wire Wire Line
	6600 3750 6900 3750
Text GLabel 6900 3850 2    50   Input ~ 0
CC_SDA0
Wire Wire Line
	6600 3850 6900 3850
Text GLabel 6900 3950 2    50   Input ~ 0
CC_GPIO12
Wire Wire Line
	6600 3950 6900 3950
Text GLabel 6900 4050 2    50   Input ~ 0
CC_GPIO13
Wire Wire Line
	6600 4050 6900 4050
Text GLabel 6900 4150 2    50   Input ~ 0
CC_GPIO14
Wire Wire Line
	6600 4150 6900 4150
Text GLabel 6900 4250 2    50   Input ~ 0
CC_GPIO15
Wire Wire Line
	6600 4250 6900 4250
Text GLabel 6900 4350 2    50   Input ~ 0
CC_UART1_TX
Wire Wire Line
	6600 4350 6900 4350
Text GLabel 6900 4450 2    50   Input ~ 0
CC_UART1_RX
Wire Wire Line
	6600 4450 6900 4450
Text GLabel 6900 4550 2    50   Input ~ 0
CC_GPIO22
Wire Wire Line
	6600 4550 6900 4550
Text GLabel 6900 4650 2    50   Input ~ 0
CC_GPIO28
Wire Wire Line
	6600 4650 6900 4650
Text GLabel 6900 4750 2    50   Input ~ 0
CC_GPIO30
Wire Wire Line
	6600 4750 6900 4750
$Comp
L power:GND #PWR0103
U 1 1 5BEB4F02
P 6750 6850
F 0 "#PWR0103" H 6750 6600 50  0001 C CNN
F 1 "GND" H 6755 6677 50  0000 C CNN
F 2 "" H 6750 6850 50  0001 C CNN
F 3 "" H 6750 6850 50  0001 C CNN
	1    6750 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 6750 6750 6750
Wire Wire Line
	6750 6750 6750 6850
Wire Wire Line
	6600 6650 6750 6650
Wire Wire Line
	6750 6650 6750 6750
Connection ~ 6750 6750
Wire Wire Line
	6600 6550 6750 6550
Wire Wire Line
	6750 6550 6750 6650
Connection ~ 6750 6650
Wire Wire Line
	6600 6450 6750 6450
Wire Wire Line
	6750 6450 6750 6550
Connection ~ 6750 6550
Wire Wire Line
	6600 6350 6750 6350
Wire Wire Line
	6750 6350 6750 6450
Connection ~ 6750 6450
Wire Wire Line
	6600 6250 6750 6250
Wire Wire Line
	6750 6250 6750 6350
Connection ~ 6750 6350
Wire Wire Line
	6600 6050 6750 6050
Wire Wire Line
	6750 6050 6750 6150
Connection ~ 6750 6250
Wire Wire Line
	6600 6150 6750 6150
Connection ~ 6750 6150
Wire Wire Line
	6750 6150 6750 6250
Wire Wire Line
	6600 5950 6750 5950
Wire Wire Line
	6750 5950 6750 6050
Connection ~ 6750 6050
Wire Wire Line
	6600 5850 6750 5850
Wire Wire Line
	6750 5850 6750 5950
Connection ~ 6750 5950
Wire Wire Line
	6600 5750 6750 5750
Wire Wire Line
	6750 5750 6750 5850
Connection ~ 6750 5850
Wire Wire Line
	6600 5650 6750 5650
Wire Wire Line
	6750 5650 6750 5750
Connection ~ 6750 5750
Wire Wire Line
	6600 5550 6750 5550
Wire Wire Line
	6750 5550 6750 5650
Connection ~ 6750 5650
Wire Wire Line
	6600 5450 6750 5450
Wire Wire Line
	6750 5450 6750 5550
Connection ~ 6750 5550
Wire Wire Line
	6600 5350 6750 5350
Wire Wire Line
	6750 5350 6750 5450
Connection ~ 6750 5450
Wire Wire Line
	6600 5250 6750 5250
Wire Wire Line
	6750 5250 6750 5350
Connection ~ 6750 5350
Wire Wire Line
	6600 5150 6750 5150
Wire Wire Line
	6750 5150 6750 5250
Connection ~ 6750 5250
Wire Wire Line
	6600 5050 6750 5050
Wire Wire Line
	6750 5050 6750 5150
Connection ~ 6750 5150
Text GLabel 3850 5850 0    50   Input ~ 0
FLASH_CS
Text GLabel 3850 6050 0    50   Input ~ 0
FLASH_DIN
Text GLabel 3850 5950 0    50   Input ~ 0
FLASH_DOUT
Text GLabel 3850 5750 0    50   Input ~ 0
FLASH_CLK
Wire Wire Line
	4200 6050 3850 6050
Wire Wire Line
	4200 5950 3850 5950
Wire Wire Line
	4200 5850 3850 5850
Wire Wire Line
	4200 5750 3850 5750
$Comp
L Device:R R4
U 1 1 5C1BBEA6
P 1850 7000
F 0 "R4" H 1920 7046 50  0000 L CNN
F 1 "100k" H 1920 6955 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1780 7000 50  0001 C CNN
F 3 "~" H 1850 7000 50  0001 C CNN
	1    1850 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5C1BBEAD
P 2200 7000
F 0 "R5" H 2270 7046 50  0000 L CNN
F 1 "100k" H 2270 6955 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2130 7000 50  0001 C CNN
F 3 "~" H 2200 7000 50  0001 C CNN
	1    2200 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5C1BBEB4
P 2550 7000
F 0 "R7" H 2620 7046 50  0000 L CNN
F 1 "100k" H 2620 6955 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2480 7000 50  0001 C CNN
F 3 "~" H 2550 7000 50  0001 C CNN
	1    2550 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 5450 2900 6750
Wire Wire Line
	2800 5350 2800 6650
Wire Wire Line
	2800 6650 2200 6650
Wire Wire Line
	2700 5250 2700 6550
Wire Wire Line
	2700 6550 1850 6550
Wire Wire Line
	1850 6850 1850 6550
Wire Wire Line
	2200 6850 2200 6650
Wire Wire Line
	2550 6850 2550 6750
Wire Wire Line
	2550 6750 2900 6750
$Comp
L power:GND #PWR0105
U 1 1 5C1BBEE0
P 2200 7250
F 0 "#PWR0105" H 2200 7000 50  0001 C CNN
F 1 "GND" H 2205 7077 50  0000 C CNN
F 2 "" H 2200 7250 50  0001 C CNN
F 3 "" H 2200 7250 50  0001 C CNN
	1    2200 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 7150 1850 7250
Wire Wire Line
	1850 7250 2200 7250
Wire Wire Line
	2550 7150 2550 7250
Wire Wire Line
	2550 7250 2200 7250
Connection ~ 2200 7250
Wire Wire Line
	2200 7150 2200 7250
Text GLabel 2600 5450 0    50   Input ~ 0
SOP2
Wire Wire Line
	2900 5450 4200 5450
Wire Wire Line
	2800 5350 4200 5350
Wire Wire Line
	2700 5250 4200 5250
Wire Wire Line
	3950 4550 4200 4550
$Comp
L Device:R R3
U 1 1 5C5156F7
P 1250 4450
F 0 "R3" H 1320 4496 50  0000 L CNN
F 1 "100k" H 1320 4405 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1180 4450 50  0001 C CNN
F 3 "~" H 1250 4450 50  0001 C CNN
	1    1250 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5C5156FE
P 1250 4600
F 0 "#PWR0108" H 1250 4350 50  0001 C CNN
F 1 "GND" H 1255 4427 50  0000 C CNN
F 2 "" H 1250 4600 50  0001 C CNN
F 3 "" H 1250 4600 50  0001 C CNN
	1    1250 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5C57C0F6
P 2400 2850
F 0 "R6" V 2300 2800 50  0000 L CNN
F 1 "10k" V 2500 2800 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2330 2850 50  0001 C CNN
F 3 "~" H 2400 2850 50  0001 C CNN
	1    2400 2850
	0    -1   1    0   
$EndComp
$Comp
L Device:C C3
U 1 1 5C57C0FD
P 1900 3200
F 0 "C3" H 2015 3246 50  0000 L CNN
F 1 "0.1uF" H 2015 3155 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1938 3050 50  0001 C CNN
F 3 "~" H 1900 3200 50  0001 C CNN
	1    1900 3200
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5C57C104
P 1900 3350
F 0 "#PWR0109" H 1900 3100 50  0001 C CNN
F 1 "GND" H 1905 3177 50  0000 C CNN
F 2 "" H 1900 3350 50  0001 C CNN
F 3 "" H 1900 3350 50  0001 C CNN
	1    1900 3350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2850 3550 2850 3050
Text GLabel 2600 3550 0    50   Input ~ 0
CC_nRESET
Connection ~ 2850 3550
Wire Wire Line
	2850 3550 2600 3550
Wire Wire Line
	2850 3050 1900 3050
Wire Wire Line
	2850 3550 4200 3550
Wire Wire Line
	2550 2850 2850 2850
Wire Wire Line
	2850 2850 2850 3050
Connection ~ 2850 3050
$Comp
L power:+3.3V #PWR0110
U 1 1 5C6F095B
P 1900 2850
F 0 "#PWR0110" H 1900 2700 50  0001 C CNN
F 1 "+3.3V" H 1915 3023 50  0000 C CNN
F 2 "" H 1900 2850 50  0001 C CNN
F 3 "" H 1900 2850 50  0001 C CNN
	1    1900 2850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2250 2850 1900 2850
$Comp
L Device:R R8
U 1 1 5C7263DF
P 3750 3350
F 0 "R8" V 3650 3300 50  0000 L CNN
F 1 "DNP" V 3850 3300 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3680 3350 50  0001 C CNN
F 3 "~" H 3750 3350 50  0001 C CNN
	1    3750 3350
	0    1    -1   0   
$EndComp
Wire Wire Line
	4200 3350 3900 3350
Wire Wire Line
	3600 3350 3600 3150
$Comp
L Device:C C1
U 1 1 5C793565
P 1400 2150
F 0 "C1" H 1515 2196 50  0000 L CNN
F 1 "100uF" H 1515 2105 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1438 2000 50  0001 C CNN
F 3 "~" H 1400 2150 50  0001 C CNN
	1    1400 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5C79356C
P 1850 2150
F 0 "C2" H 1965 2196 50  0000 L CNN
F 1 "100uF" H 1965 2105 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1888 2000 50  0001 C CNN
F 3 "~" H 1850 2150 50  0001 C CNN
	1    1850 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5C793573
P 2300 2150
F 0 "C4" H 2415 2196 50  0000 L CNN
F 1 "0.1uF" H 2415 2105 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2338 2000 50  0001 C CNN
F 3 "~" H 2300 2150 50  0001 C CNN
	1    2300 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5C79357A
P 1850 2300
F 0 "#PWR0111" H 1850 2050 50  0001 C CNN
F 1 "GND" H 1855 2127 50  0000 C CNN
F 2 "" H 1850 2300 50  0001 C CNN
F 3 "" H 1850 2300 50  0001 C CNN
	1    1850 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 2300 1850 2300
Connection ~ 1850 2300
Wire Wire Line
	2300 2300 1850 2300
Wire Wire Line
	2300 2000 2300 1950
Wire Wire Line
	2300 1950 1850 1950
Wire Wire Line
	1850 1950 1850 2000
Wire Wire Line
	1400 2000 1400 1950
Wire Wire Line
	1400 1950 1850 1950
Connection ~ 1850 1950
$Comp
L power:+3.3V #PWR0112
U 1 1 5C793589
P 1850 1850
F 0 "#PWR0112" H 1850 1700 50  0001 C CNN
F 1 "+3.3V" H 1865 2023 50  0000 C CNN
F 2 "" H 1850 1850 50  0001 C CNN
F 3 "" H 1850 1850 50  0001 C CNN
	1    1850 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 1850 1850 1950
Connection ~ 2300 1950
$Comp
L Device:C C5
U 1 1 5C793591
P 2750 2150
F 0 "C5" H 2865 2196 50  0000 L CNN
F 1 "0.1uF" H 2865 2105 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2788 2000 50  0001 C CNN
F 3 "~" H 2750 2150 50  0001 C CNN
	1    2750 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 2300 2750 2300
Connection ~ 2300 2300
Wire Wire Line
	2750 1950 2750 2000
Wire Wire Line
	2300 1950 2750 1950
Connection ~ 2750 1950
Wire Wire Line
	2750 1950 3200 1950
$Comp
L Device:C C6
U 1 1 5C80838A
P 3200 2100
F 0 "C6" H 3315 2146 50  0000 L CNN
F 1 "0.1uF" H 3315 2055 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3238 1950 50  0001 C CNN
F 3 "~" H 3200 2100 50  0001 C CNN
	1    3200 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 2300 3200 2300
Wire Wire Line
	3200 2300 3200 2250
Connection ~ 2750 2300
Wire Wire Line
	3200 1950 3600 1950
Wire Wire Line
	3600 1950 3600 2750
Wire Wire Line
	3600 2750 4200 2750
Connection ~ 3200 1950
Wire Wire Line
	3600 2750 3600 2950
Wire Wire Line
	3600 2950 4200 2950
Connection ~ 3600 2750
$Comp
L Device:R R9
U 1 1 5C8F2D99
P 4000 3150
F 0 "R9" V 3900 3100 50  0000 L CNN
F 1 "DNP" V 4100 3100 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3930 3150 50  0001 C CNN
F 3 "~" H 4000 3150 50  0001 C CNN
	1    4000 3150
	0    1    -1   0   
$EndComp
Wire Wire Line
	3600 3150 3850 3150
Wire Wire Line
	4150 3150 4200 3150
Wire Wire Line
	3600 2950 3600 3150
Connection ~ 3600 2950
Connection ~ 3600 3150
Text GLabel 2600 5250 0    50   Input ~ 0
SOP0
Text GLabel 2600 5350 0    50   Input ~ 0
SOP1
Wire Wire Line
	2600 5450 2900 5450
Connection ~ 2900 5450
Wire Wire Line
	2600 5350 2800 5350
Connection ~ 2800 5350
Wire Wire Line
	2600 5250 2700 5250
Connection ~ 2700 5250
Wire Wire Line
	1250 4050 1250 4300
Text GLabel 4000 3850 0    50   Input ~ 0
JTAG_TDI
Wire Wire Line
	4200 3850 4000 3850
Text GLabel 4000 3950 0    50   Input ~ 0
JTAG_TDO
Text GLabel 4000 4150 0    50   Input ~ 0
JTAG_TMS
Text GLabel 1100 4050 0    50   Input ~ 0
JTAG_TCK
Wire Wire Line
	1100 4050 1250 4050
Wire Wire Line
	4000 3950 4200 3950
Wire Wire Line
	4000 4150 4200 4150
$Comp
L Device:R R2
U 1 1 5BA93929
P 7850 2000
F 0 "R2" V 7750 1950 50  0000 L CNN
F 1 "270" V 7950 1950 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7780 2000 50  0001 C CNN
F 3 "~" H 7850 2000 50  0001 C CNN
	1    7850 2000
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D6
U 1 1 5BA939ED
P 7400 2000
F 0 "D6" H 7392 1745 50  0000 C CNN
F 1 "LED" H 7392 1836 50  0000 C CNN
F 2 "LEDs:LED_0603_HandSoldering" H 7400 2000 50  0001 C CNN
F 3 "~" H 7400 2000 50  0001 C CNN
	1    7400 2000
	-1   0    0    1   
$EndComp
$Comp
L Transistor_FET:BSS138 Q2
U 1 1 5BA93B8B
P 8350 2100
F 0 "Q2" V 8693 2100 50  0000 C CNN
F 1 "BSS138" V 8602 2100 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 8550 2025 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 8350 2100 50  0001 L CNN
	1    8350 2100
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R13
U 1 1 5BA93D6E
P 8800 2300
F 0 "R13" V 8700 2250 50  0000 L CNN
F 1 "100k" V 8900 2250 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8730 2300 50  0001 C CNN
F 3 "~" H 8800 2300 50  0001 C CNN
	1    8800 2300
	0    1    -1   0   
$EndComp
Wire Wire Line
	8350 2300 8650 2300
$Comp
L power:GND #PWR04
U 1 1 5BA99681
P 9200 2300
F 0 "#PWR04" H 9200 2050 50  0001 C CNN
F 1 "GND" H 9205 2127 50  0000 C CNN
F 2 "" H 9200 2300 50  0001 C CNN
F 3 "" H 9200 2300 50  0001 C CNN
	1    9200 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 2300 9200 2300
Wire Wire Line
	8550 2000 9200 2000
Wire Wire Line
	9200 2000 9200 2300
Connection ~ 9200 2300
Wire Wire Line
	7550 2000 7700 2000
Wire Wire Line
	8000 2000 8150 2000
$Comp
L power:+3.3V #PWR02
U 1 1 5BAB0450
P 7100 2000
F 0 "#PWR02" H 7100 1850 50  0001 C CNN
F 1 "+3.3V" H 7115 2173 50  0000 C CNN
F 2 "" H 7100 2000 50  0001 C CNN
F 3 "" H 7100 2000 50  0001 C CNN
	1    7100 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 2000 7250 2000
Wire Wire Line
	8350 2300 6850 2300
Wire Wire Line
	6850 2300 6850 2750
Connection ~ 8350 2300
Connection ~ 6850 2750
Wire Wire Line
	6850 2750 6900 2750
$Comp
L Device:R R1
U 1 1 5BAC3284
P 7600 850
F 0 "R1" V 7500 800 50  0000 L CNN
F 1 "270" V 7700 800 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7530 850 50  0001 C CNN
F 3 "~" H 7600 850 50  0001 C CNN
	1    7600 850 
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D5
U 1 1 5BAC328B
P 7150 850
F 0 "D5" H 7142 595 50  0000 C CNN
F 1 "LED" H 7142 686 50  0000 C CNN
F 2 "LEDs:LED_0603_HandSoldering" H 7150 850 50  0001 C CNN
F 3 "~" H 7150 850 50  0001 C CNN
	1    7150 850 
	-1   0    0    1   
$EndComp
$Comp
L Transistor_FET:BSS138 Q1
U 1 1 5BAC3292
P 8100 950
F 0 "Q1" V 8443 950 50  0000 C CNN
F 1 "BSS138" V 8352 950 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 8300 875 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 8100 950 50  0001 L CNN
	1    8100 950 
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R10
U 1 1 5BAC3299
P 8550 1150
F 0 "R10" V 8450 1100 50  0000 L CNN
F 1 "100k" V 8650 1100 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8480 1150 50  0001 C CNN
F 3 "~" H 8550 1150 50  0001 C CNN
	1    8550 1150
	0    1    -1   0   
$EndComp
Wire Wire Line
	8100 1150 8400 1150
$Comp
L power:GND #PWR03
U 1 1 5BAC32A1
P 8950 1150
F 0 "#PWR03" H 8950 900 50  0001 C CNN
F 1 "GND" H 8955 977 50  0000 C CNN
F 2 "" H 8950 1150 50  0001 C CNN
F 3 "" H 8950 1150 50  0001 C CNN
	1    8950 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 1150 8950 1150
Wire Wire Line
	8300 850  8950 850 
Wire Wire Line
	8950 850  8950 1150
Connection ~ 8950 1150
Wire Wire Line
	7300 850  7450 850 
Wire Wire Line
	7750 850  7900 850 
$Comp
L power:+3.3V #PWR01
U 1 1 5BAC32AD
P 6850 850
F 0 "#PWR01" H 6850 700 50  0001 C CNN
F 1 "+3.3V" H 6865 1023 50  0000 C CNN
F 2 "" H 6850 850 50  0001 C CNN
F 3 "" H 6850 850 50  0001 C CNN
	1    6850 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 850  7000 850 
Connection ~ 8100 1150
Wire Wire Line
	6700 3050 6700 1150
Connection ~ 6700 3050
Wire Wire Line
	6700 3050 6900 3050
Wire Wire Line
	6700 1150 8100 1150
Wire Wire Line
	4200 4050 1250 4050
Connection ~ 1250 4050
$EndSCHEMATC
