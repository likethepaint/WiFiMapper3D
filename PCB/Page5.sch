EESchema Schematic File Version 4
LIBS:PCB_Rev3-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
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
L Logic_LevelTranslator:FSUSB42MUX U?
U 1 1 5B9B9A42
P 3450 3250
F 0 "U?" H 3425 4015 50  0000 C CNN
F 1 "FSUSB42MUX" H 3425 3924 50  0000 C CNN
F 2 "Housings_SSOP:MSOP-10_3x3mm_Pitch0.5mm" H 3450 3250 50  0001 C CNN
F 3 "" H 3450 3250 50  0001 C CNN
	1    3450 3250
	1    0    0    -1  
$EndComp
Text GLabel 2550 3050 0    50   Input ~ 0
XBEE_TX
Text GLabel 2550 3200 0    50   Input ~ 0
XBEE_RX
$Comp
L power:GND #PWR?
U 1 1 5B9B9CD9
P 2950 3500
F 0 "#PWR?" H 2950 3250 50  0001 C CNN
F 1 "GND" H 2955 3327 50  0000 C CNN
F 2 "" H 2950 3500 50  0001 C CNN
F 3 "" H 2950 3500 50  0001 C CNN
	1    2950 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 3500 2950 3350
Wire Wire Line
	2950 3350 3000 3350
Wire Wire Line
	3000 3050 2550 3050
Wire Wire Line
	3000 3200 2550 3200
$Comp
L Device:C C?
U 1 1 5B9B9DF5
P 8750 1600
F 0 "C?" H 8865 1646 50  0000 L CNN
F 1 "10uF" H 8865 1555 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 8788 1450 50  0001 C CNN
F 3 "~" H 8750 1600 50  0001 C CNN
	1    8750 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B9B9DFC
P 5500 2800
F 0 "#PWR?" H 5500 2550 50  0001 C CNN
F 1 "GND" H 5505 2627 50  0000 C CNN
F 2 "" H 5500 2800 50  0001 C CNN
F 3 "" H 5500 2800 50  0001 C CNN
	1    5500 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 1450 8500 1450
$Comp
L Device:R R?
U 1 1 5B9B9E03
P 9500 1650
F 0 "R?" H 9570 1696 50  0000 L CNN
F 1 "330" H 9570 1605 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 9430 1650 50  0001 C CNN
F 3 "~" H 9500 1650 50  0001 C CNN
	1    9500 1650
	-1   0    0    -1  
$EndComp
$Comp
L Device:LED D?
U 1 1 5B9B9E0A
P 9500 2050
F 0 "D?" V 9492 1932 50  0000 R CNN
F 1 "LG L29K-F2J1-24-Z " V 9447 1933 50  0001 R CNN
F 2 "LEDs:LED_0603_HandSoldering" H 9500 2050 50  0001 C CNN
F 3 "~" H 9500 2050 50  0001 C CNN
	1    9500 2050
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B9B9E11
P 9500 2300
F 0 "#PWR?" H 9500 2050 50  0001 C CNN
F 1 "GND" H 9505 2127 50  0000 C CNN
F 2 "" H 9500 2300 50  0001 C CNN
F 3 "" H 9500 2300 50  0001 C CNN
	1    9500 2300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9900 1450 9500 1450
Wire Wire Line
	9500 1500 9500 1450
Connection ~ 9500 1450
Wire Wire Line
	9500 1800 9500 1900
Wire Wire Line
	9500 2200 9500 2300
Wire Wire Line
	8750 1450 9500 1450
Connection ~ 8750 1450
$Comp
L Device:R R?
U 1 1 5B9B9F15
P 5000 2750
F 0 "R?" V 5100 2700 50  0000 L CNN
F 1 "10k" V 4900 2700 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4930 2750 50  0001 C CNN
F 3 "~" H 5000 2750 50  0001 C CNN
	1    5000 2750
	0    1    -1   0   
$EndComp
Wire Wire Line
	5150 2750 5500 2750
Wire Wire Line
	5500 2750 5500 2800
Text GLabel 4150 3200 2    50   Input ~ 0
CC_UART0_RX
Text GLabel 4150 3350 2    50   Input ~ 0
CC_UART0_TX
Wire Wire Line
	4150 3350 3850 3350
Wire Wire Line
	4150 3200 3850 3200
Text GLabel 4150 2900 2    50   Input ~ 0
XBEE_DEBUG_TX
Text GLabel 4150 3050 2    50   Input ~ 0
XBEE_DEBUG_RX
Wire Wire Line
	4150 2900 3850 2900
Wire Wire Line
	4150 3050 3850 3050
Wire Wire Line
	3850 2750 4850 2750
$Comp
L Device:C C?
U 1 1 5B9BACAA
P 1700 2350
F 0 "C?" H 1815 2396 50  0000 L CNN
F 1 "4.7uF" H 1815 2305 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1738 2200 50  0001 C CNN
F 3 "~" H 1700 2350 50  0001 C CNN
	1    1700 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5B9BADD6
P 2200 2350
F 0 "C?" H 2315 2396 50  0000 L CNN
F 1 "0.1uF" H 2315 2305 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2238 2200 50  0001 C CNN
F 3 "~" H 2200 2350 50  0001 C CNN
	1    2200 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B9BAEA1
P 1950 2500
F 0 "#PWR?" H 1950 2250 50  0001 C CNN
F 1 "GND" H 1955 2327 50  0000 C CNN
F 2 "" H 1950 2500 50  0001 C CNN
F 3 "" H 1950 2500 50  0001 C CNN
	1    1950 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 2500 1950 2500
Wire Wire Line
	1700 2500 1950 2500
Connection ~ 1950 2500
Wire Wire Line
	2200 2200 2800 2200
Wire Wire Line
	2800 2200 2800 2750
Wire Wire Line
	2800 2750 3000 2750
Wire Wire Line
	1700 2200 2200 2200
Connection ~ 2200 2200
Wire Wire Line
	1700 2200 1400 2200
Connection ~ 1700 2200
$Comp
L power:+3.3V #PWR?
U 1 1 5B9BBBCE
P 1400 2200
F 0 "#PWR?" H 1400 2050 50  0001 C CNN
F 1 "+3.3V" H 1415 2373 50  0000 C CNN
F 2 "" H 1400 2200 50  0001 C CNN
F 3 "" H 1400 2200 50  0001 C CNN
	1    1400 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5B9BBC48
P 1850 3100
F 0 "R?" H 1900 3050 50  0000 L CNN
F 1 "10k" H 1900 3150 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1780 3100 50  0001 C CNN
F 3 "~" H 1850 3100 50  0001 C CNN
	1    1850 3100
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B9BBD20
P 1850 3250
F 0 "#PWR?" H 1850 3000 50  0001 C CNN
F 1 "GND" H 1855 3077 50  0000 C CNN
F 2 "" H 1850 3250 50  0001 C CNN
F 3 "" H 1850 3250 50  0001 C CNN
	1    1850 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 2950 1850 2900
Wire Wire Line
	1850 2900 3000 2900
Text GLabel 1650 2900 0    50   Input ~ 0
XBEE_DEBUG_EN
Wire Wire Line
	1850 2900 1650 2900
Connection ~ 1850 2900
$Comp
L Logic_LevelTranslator:FSUSB42MUX U?
U 1 1 5B9BC750
P 3400 5400
F 0 "U?" H 3375 6165 50  0000 C CNN
F 1 "FSUSB42MUX" H 3375 6074 50  0000 C CNN
F 2 "Housings_SSOP:MSOP-10_3x3mm_Pitch0.5mm" H 3400 5400 50  0001 C CNN
F 3 "" H 3400 5400 50  0001 C CNN
	1    3400 5400
	1    0    0    -1  
$EndComp
Text GLabel 2500 5200 0    50   Input ~ 0
GPS_TX
Text GLabel 2500 5350 0    50   Input ~ 0
GPS_RX
$Comp
L power:GND #PWR?
U 1 1 5B9BC759
P 2900 5650
F 0 "#PWR?" H 2900 5400 50  0001 C CNN
F 1 "GND" H 2905 5477 50  0000 C CNN
F 2 "" H 2900 5650 50  0001 C CNN
F 3 "" H 2900 5650 50  0001 C CNN
	1    2900 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 5650 2900 5500
Wire Wire Line
	2900 5500 2950 5500
Wire Wire Line
	2950 5200 2500 5200
Wire Wire Line
	2950 5350 2500 5350
$Comp
L power:GND #PWR?
U 1 1 5B9BC763
P 5450 4950
F 0 "#PWR?" H 5450 4700 50  0001 C CNN
F 1 "GND" H 5455 4777 50  0000 C CNN
F 2 "" H 5450 4950 50  0001 C CNN
F 3 "" H 5450 4950 50  0001 C CNN
	1    5450 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5B9BC769
P 4950 4900
F 0 "R?" V 5050 4850 50  0000 L CNN
F 1 "10k" V 4850 4850 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4880 4900 50  0001 C CNN
F 3 "~" H 4950 4900 50  0001 C CNN
	1    4950 4900
	0    1    -1   0   
$EndComp
Wire Wire Line
	5100 4900 5450 4900
Wire Wire Line
	5450 4900 5450 4950
Text GLabel 4100 5350 2    50   Input ~ 0
CC_UART1_RX
Text GLabel 4100 5500 2    50   Input ~ 0
CC_UART1_TX
Wire Wire Line
	4100 5500 3800 5500
Wire Wire Line
	4100 5350 3800 5350
Text GLabel 4100 5050 2    50   Input ~ 0
GPS_DEBUG_TX
Text GLabel 4100 5200 2    50   Input ~ 0
GPS_DEBUG_RX
Wire Wire Line
	4100 5050 3800 5050
Wire Wire Line
	4100 5200 3800 5200
Wire Wire Line
	3800 4900 4800 4900
$Comp
L Device:C C?
U 1 1 5B9BC77B
P 1650 4500
F 0 "C?" H 1765 4546 50  0000 L CNN
F 1 "4.7uF" H 1765 4455 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1688 4350 50  0001 C CNN
F 3 "~" H 1650 4500 50  0001 C CNN
	1    1650 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5B9BC782
P 2150 4500
F 0 "C?" H 2265 4546 50  0000 L CNN
F 1 "0.1uF" H 2265 4455 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2188 4350 50  0001 C CNN
F 3 "~" H 2150 4500 50  0001 C CNN
	1    2150 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B9BC789
P 1900 4650
F 0 "#PWR?" H 1900 4400 50  0001 C CNN
F 1 "GND" H 1905 4477 50  0000 C CNN
F 2 "" H 1900 4650 50  0001 C CNN
F 3 "" H 1900 4650 50  0001 C CNN
	1    1900 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 4650 1900 4650
Wire Wire Line
	1650 4650 1900 4650
Connection ~ 1900 4650
Wire Wire Line
	2150 4350 2750 4350
Wire Wire Line
	2750 4350 2750 4900
Wire Wire Line
	2750 4900 2950 4900
Wire Wire Line
	1650 4350 2150 4350
Connection ~ 2150 4350
Wire Wire Line
	1650 4350 1350 4350
Connection ~ 1650 4350
$Comp
L power:+3.3V #PWR?
U 1 1 5B9BC799
P 1350 4350
F 0 "#PWR?" H 1350 4200 50  0001 C CNN
F 1 "+3.3V" H 1365 4523 50  0000 C CNN
F 2 "" H 1350 4350 50  0001 C CNN
F 3 "" H 1350 4350 50  0001 C CNN
	1    1350 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5B9BC79F
P 1800 5250
F 0 "R?" H 1850 5200 50  0000 L CNN
F 1 "10k" H 1850 5300 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1730 5250 50  0001 C CNN
F 3 "~" H 1800 5250 50  0001 C CNN
	1    1800 5250
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B9BC7A6
P 1800 5400
F 0 "#PWR?" H 1800 5150 50  0001 C CNN
F 1 "GND" H 1805 5227 50  0000 C CNN
F 2 "" H 1800 5400 50  0001 C CNN
F 3 "" H 1800 5400 50  0001 C CNN
	1    1800 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 5100 1800 5050
Wire Wire Line
	1800 5050 2950 5050
Text GLabel 1600 5050 0    50   Input ~ 0
GPS_DEBUG_EN
Wire Wire Line
	1800 5050 1600 5050
Connection ~ 1800 5050
$EndSCHEMATC
