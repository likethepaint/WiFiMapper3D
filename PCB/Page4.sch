EESchema Schematic File Version 4
LIBS:PCB_Rev3-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 7
Title "USB MUX's and Debug Header"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Logic_LevelTranslator:FSUSB42MUX U12
U 1 1 5B9B9A42
P 3450 3250
F 0 "U12" H 3425 4015 50  0000 C CNN
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
L power:GND #PWR0169
U 1 1 5B9B9CD9
P 2950 3500
F 0 "#PWR0169" H 2950 3250 50  0001 C CNN
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
L power:GND #PWR0170
U 1 1 5B9B9DFC
P 5500 2800
F 0 "#PWR0170" H 5500 2550 50  0001 C CNN
F 1 "GND" H 5505 2627 50  0000 C CNN
F 2 "" H 5500 2800 50  0001 C CNN
F 3 "" H 5500 2800 50  0001 C CNN
	1    5500 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R28
U 1 1 5B9B9F15
P 5000 2750
F 0 "R28" V 5100 2700 50  0000 L CNN
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
L Device:C C48
U 1 1 5B9BACAA
P 1700 2350
F 0 "C48" H 1815 2396 50  0000 L CNN
F 1 "4.7uF" H 1815 2305 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1738 2200 50  0001 C CNN
F 3 "~" H 1700 2350 50  0001 C CNN
	1    1700 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C50
U 1 1 5B9BADD6
P 2200 2350
F 0 "C50" H 2315 2396 50  0000 L CNN
F 1 "0.1uF" H 2315 2305 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2238 2200 50  0001 C CNN
F 3 "~" H 2200 2350 50  0001 C CNN
	1    2200 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0171
U 1 1 5B9BAEA1
P 1950 2500
F 0 "#PWR0171" H 1950 2250 50  0001 C CNN
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
L power:+3.3V #PWR0172
U 1 1 5B9BBBCE
P 1400 2200
F 0 "#PWR0172" H 1400 2050 50  0001 C CNN
F 1 "+3.3V" H 1415 2373 50  0000 C CNN
F 2 "" H 1400 2200 50  0001 C CNN
F 3 "" H 1400 2200 50  0001 C CNN
	1    1400 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R26
U 1 1 5B9BBC48
P 1850 3100
F 0 "R26" H 1900 3050 50  0000 L CNN
F 1 "10k" H 1900 3150 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1780 3100 50  0001 C CNN
F 3 "~" H 1850 3100 50  0001 C CNN
	1    1850 3100
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0173
U 1 1 5B9BBD20
P 1850 3250
F 0 "#PWR0173" H 1850 3000 50  0001 C CNN
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
L Logic_LevelTranslator:FSUSB42MUX U11
U 1 1 5B9BC750
P 3400 5400
F 0 "U11" H 3375 6165 50  0000 C CNN
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
L power:GND #PWR0174
U 1 1 5B9BC759
P 2900 5650
F 0 "#PWR0174" H 2900 5400 50  0001 C CNN
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
L power:GND #PWR0175
U 1 1 5B9BC763
P 5450 4950
F 0 "#PWR0175" H 5450 4700 50  0001 C CNN
F 1 "GND" H 5455 4777 50  0000 C CNN
F 2 "" H 5450 4950 50  0001 C CNN
F 3 "" H 5450 4950 50  0001 C CNN
	1    5450 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R27
U 1 1 5B9BC769
P 4950 4900
F 0 "R27" V 5050 4850 50  0000 L CNN
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
L Device:C C47
U 1 1 5B9BC77B
P 1650 4500
F 0 "C47" H 1765 4546 50  0000 L CNN
F 1 "4.7uF" H 1765 4455 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1688 4350 50  0001 C CNN
F 3 "~" H 1650 4500 50  0001 C CNN
	1    1650 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C49
U 1 1 5B9BC782
P 2150 4500
F 0 "C49" H 2265 4546 50  0000 L CNN
F 1 "0.1uF" H 2265 4455 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2188 4350 50  0001 C CNN
F 3 "~" H 2150 4500 50  0001 C CNN
	1    2150 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0176
U 1 1 5B9BC789
P 1900 4650
F 0 "#PWR0176" H 1900 4400 50  0001 C CNN
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
L power:+3.3V #PWR0177
U 1 1 5B9BC799
P 1350 4350
F 0 "#PWR0177" H 1350 4200 50  0001 C CNN
F 1 "+3.3V" H 1365 4523 50  0000 C CNN
F 2 "" H 1350 4350 50  0001 C CNN
F 3 "" H 1350 4350 50  0001 C CNN
	1    1350 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R25
U 1 1 5B9BC79F
P 1800 5250
F 0 "R25" H 1850 5200 50  0000 L CNN
F 1 "10k" H 1850 5300 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1730 5250 50  0001 C CNN
F 3 "~" H 1800 5250 50  0001 C CNN
	1    1800 5250
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0178
U 1 1 5B9BC7A6
P 1800 5400
F 0 "#PWR0178" H 1800 5150 50  0001 C CNN
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
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 5B9CAFC0
P 8600 5150
F 0 "J5" H 8520 5367 50  0000 C CNN
F 1 "Conn_01x02" H 8520 5276 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 8600 5150 50  0001 C CNN
F 3 "~" H 8600 5150 50  0001 C CNN
	1    8600 5150
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0179
U 1 1 5B9CAFC7
P 8900 5350
F 0 "#PWR0179" H 8900 5100 50  0001 C CNN
F 1 "GND" H 8905 5177 50  0000 C CNN
F 2 "" H 8900 5350 50  0001 C CNN
F 3 "" H 8900 5350 50  0001 C CNN
	1    8900 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 5250 8900 5250
Wire Wire Line
	8900 5250 8900 5350
Wire Wire Line
	8800 5150 8900 5150
Text GLabel 8900 5150 2    50   Input ~ 0
VBAT
Text GLabel 9750 2850 2    50   Input ~ 0
GPS_DEBUG_RX
Text GLabel 9250 2950 0    50   Input ~ 0
GPS_DEBUG_TX
Text GLabel 9250 2850 0    50   Input ~ 0
GPS_DEBUG_EN
Text GLabel 9750 2950 2    50   Input ~ 0
GPS_RESET_N
Text GLabel 9250 3150 0    50   Input ~ 0
XBEE_DEBUG_EN
Text GLabel 9750 3150 2    50   Input ~ 0
XBEE_RESET
Text GLabel 9750 3050 2    50   Input ~ 0
XBEE_DEBUG_RX
Text GLabel 9250 3050 0    50   Input ~ 0
XBEE_DEBUG_TX
Text GLabel 9750 3350 2    50   Input ~ 0
XBEE_RTS
Text GLabel 9750 3250 2    50   Input ~ 0
XBEE_CTS
Text GLabel 9250 3250 0    50   Input ~ 0
XBEE_DTR
Text GLabel 9250 3350 0    50   Input ~ 0
CC_nRESET
Text GLabel 9750 3450 2    50   Input ~ 0
CC_UART0_RX
Text GLabel 9250 3450 0    50   Input ~ 0
CC_UART0_TX
Text GLabel 9750 2750 2    50   Input ~ 0
VBUS
Text GLabel 9250 2650 0    50   Input ~ 0
VBUS
$Comp
L power:+3.3V #PWR0181
U 1 1 5B9D2B86
P 8600 4150
F 0 "#PWR0181" H 8600 4000 50  0001 C CNN
F 1 "+3.3V" H 8615 4323 50  0000 C CNN
F 2 "" H 8600 4150 50  0001 C CNN
F 3 "" H 8600 4150 50  0001 C CNN
	1    8600 4150
	1    0    0    -1  
$EndComp
Text GLabel 9250 3550 0    50   Input ~ 0
FLASH_CS
Text GLabel 9750 3550 2    50   Input ~ 0
FLASH_DIN
Text GLabel 9250 3650 0    50   Input ~ 0
FLASH_DOUT
Text GLabel 9750 3650 2    50   Input ~ 0
FLASH_CLK
Text GLabel 9750 3750 2    50   Input ~ 0
JTAG_TDI
Text GLabel 9250 3750 0    50   Input ~ 0
JTAG_TDO
Text GLabel 9750 3850 2    50   Input ~ 0
JTAG_TCK
Text GLabel 9250 3850 0    50   Input ~ 0
JTAG_TMS
$Comp
L power:+3.3V #PWR0116
U 1 1 5BA4C22D
P 10400 4150
F 0 "#PWR0116" H 10400 4000 50  0001 C CNN
F 1 "+3.3V" H 10415 4323 50  0000 C CNN
F 2 "" H 10400 4150 50  0001 C CNN
F 3 "" H 10400 4150 50  0001 C CNN
	1    10400 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 5BA4C277
P 9850 4450
F 0 "#PWR0117" H 9850 4200 50  0001 C CNN
F 1 "GND" H 9855 4277 50  0000 C CNN
F 2 "" H 9850 4450 50  0001 C CNN
F 3 "" H 9850 4450 50  0001 C CNN
	1    9850 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5BA4C2D4
P 9150 4450
F 0 "#PWR0118" H 9150 4200 50  0001 C CNN
F 1 "GND" H 9155 4277 50  0000 C CNN
F 2 "" H 9150 4450 50  0001 C CNN
F 3 "" H 9150 4450 50  0001 C CNN
	1    9150 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 4350 9850 4450
Wire Wire Line
	9150 4350 9150 4450
Wire Wire Line
	9250 4150 8600 4150
Wire Wire Line
	9750 4150 10400 4150
Text GLabel 9250 4050 0    50   Input ~ 0
SOP2
$Comp
L Connector_Generic:Conn_02x18_Odd_Even J1
U 1 1 5BA876CA
P 9450 3450
F 0 "J1" H 9500 4467 50  0000 C CNN
F 1 "Conn_02x18_Odd_Even" H 9500 4376 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x18_Pitch2.00mm_SMD" H 9450 3450 50  0001 C CNN
F 3 "~" H 9450 3450 50  0001 C CNN
	1    9450 3450
	1    0    0    -1  
$EndComp
Text GLabel 9250 3950 0    50   Input ~ 0
SOP0
Text GLabel 9750 3950 2    50   Input ~ 0
SOP1
Text GLabel 9250 2750 0    50   Input ~ 0
VBUS
Text GLabel 9750 2650 2    50   Input ~ 0
VBUS
Wire Wire Line
	9250 4350 9150 4350
Wire Wire Line
	9250 4250 9150 4250
Wire Wire Line
	9150 4250 9150 4350
Connection ~ 9150 4350
Wire Wire Line
	9750 4350 9850 4350
Wire Wire Line
	9750 4250 9850 4250
Wire Wire Line
	9850 4250 9850 4350
Connection ~ 9850 4350
$EndSCHEMATC
