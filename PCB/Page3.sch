EESchema Schematic File Version 4
LIBS:PCB_Rev3-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 7
Title "XBEE Downlink Radio Schematic"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L XBP9B-DPST-001:XBP9B-DPST-001 U8
U 1 1 5B981517
P 4250 3400
F 0 "U8" H 4250 4467 50  0000 C CNN
F 1 "XBP9B-DPST-001" H 4250 4376 50  0000 C CNN
F 2 "WiFiMapSpecific:XBEE_SMD" H 4250 3400 50  0001 L BNN
F 3 "Digi International" H 4250 3400 50  0001 L BNN
F 4 "XBee-PRO 900HP _S3B_ Point2Multipoint, 900MHz, 250mW, RPSMA, 10Kbps" H 4250 3400 50  0001 L BNN "Field4"
F 5 "38.27 USD" H 4250 3400 50  0001 L BNN "Field5"
F 6 "XBP9B-DPST-001" H 4250 3400 50  0001 L BNN "Field6"
F 7 "Module Digi International" H 4250 3400 50  0001 L BNN "Field7"
F 8 "Warning" H 4250 3400 50  0001 L BNN "Field8"
	1    4250 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0154
U 1 1 5B9854F7
P 5400 4250
F 0 "#PWR0154" H 5400 4000 50  0001 C CNN
F 1 "GND" H 5405 4077 50  0000 C CNN
F 2 "" H 5400 4250 50  0001 C CNN
F 3 "" H 5400 4250 50  0001 C CNN
	1    5400 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 4200 5400 4200
Wire Wire Line
	5400 4200 5400 4250
$Comp
L Device:C C41
U 1 1 5B985547
P 6250 2750
F 0 "C41" H 6365 2796 50  0000 L CNN
F 1 "10uF" H 6365 2705 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6288 2600 50  0001 C CNN
F 3 "~" H 6250 2750 50  0001 C CNN
	1    6250 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C40
U 1 1 5B985597
P 5750 2750
F 0 "C40" H 5865 2796 50  0000 L CNN
F 1 "0.01uF" H 5865 2705 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 5788 2600 50  0001 C CNN
F 3 "~" H 5750 2750 50  0001 C CNN
	1    5750 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0155
U 1 1 5B9855FA
P 5750 2900
F 0 "#PWR0155" H 5750 2650 50  0001 C CNN
F 1 "GND" H 5755 2727 50  0000 C CNN
F 2 "" H 5750 2900 50  0001 C CNN
F 3 "" H 5750 2900 50  0001 C CNN
	1    5750 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0156
U 1 1 5B985622
P 6250 2900
F 0 "#PWR0156" H 6250 2650 50  0001 C CNN
F 1 "GND" H 6255 2727 50  0000 C CNN
F 2 "" H 6250 2900 50  0001 C CNN
F 3 "" H 6250 2900 50  0001 C CNN
	1    6250 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 2600 5750 2600
Connection ~ 5750 2600
$Comp
L Power_Management:TPS22918 U10
U 1 1 5B98A906
P 8700 2700
F 0 "U10" H 8700 3065 50  0000 C CNN
F 1 "TPS22918" H 8700 2974 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-6_Handsoldering" H 8200 1950 50  0001 C CNN
F 3 "" H 8200 1950 50  0001 C CNN
	1    8700 2700
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0157
U 1 1 5B98A90D
P 9800 2600
F 0 "#PWR0157" H 9800 2450 50  0001 C CNN
F 1 "+3.3V" H 9815 2773 50  0000 C CNN
F 2 "" H 9800 2600 50  0001 C CNN
F 3 "" H 9800 2600 50  0001 C CNN
	1    9800 2600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9800 2600 9400 2600
$Comp
L power:GND #PWR0158
U 1 1 5B98A914
P 9150 2850
F 0 "#PWR0158" H 9150 2600 50  0001 C CNN
F 1 "GND" H 9155 2677 50  0000 C CNN
F 2 "" H 9150 2850 50  0001 C CNN
F 3 "" H 9150 2850 50  0001 C CNN
	1    9150 2850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9150 2850 9150 2800
Wire Wire Line
	9150 2800 9050 2800
$Comp
L Device:R R23
U 1 1 5B98A91C
P 7800 2700
F 0 "R23" V 7900 2700 50  0000 C CNN
F 1 "0" V 7800 2700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7730 2700 50  0001 C CNN
F 3 "~" H 7800 2700 50  0001 C CNN
	1    7800 2700
	0    -1   1    0   
$EndComp
Wire Wire Line
	8350 2700 7950 2700
Wire Wire Line
	8350 2600 7550 2600
Wire Wire Line
	7550 2600 7550 2700
Wire Wire Line
	7550 2700 7650 2700
$Comp
L Device:C C44
U 1 1 5B98A927
P 8100 3000
F 0 "C44" H 8215 3046 50  0000 L CNN
F 1 "0.001uF" H 8215 2955 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 8138 2850 50  0001 C CNN
F 3 "~" H 8100 3000 50  0001 C CNN
	1    8100 3000
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0159
U 1 1 5B98A92E
P 8100 3150
F 0 "#PWR0159" H 8100 2900 50  0001 C CNN
F 1 "GND" H 8105 2977 50  0000 C CNN
F 2 "" H 8100 3150 50  0001 C CNN
F 3 "" H 8100 3150 50  0001 C CNN
	1    8100 3150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8350 2800 8100 2800
Wire Wire Line
	8100 2800 8100 2850
Text GLabel 9600 2700 2    50   Input ~ 0
CC_GPIO7
$Comp
L Device:R R24
U 1 1 5B98A937
P 9550 2950
F 0 "R24" H 9620 2996 50  0000 L CNN
F 1 "100K" H 9620 2905 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 9480 2950 50  0001 C CNN
F 3 "~" H 9550 2950 50  0001 C CNN
	1    9550 2950
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0160
U 1 1 5B98A93E
P 9550 3100
F 0 "#PWR0160" H 9550 2850 50  0001 C CNN
F 1 "GND" H 9555 2927 50  0000 C CNN
F 2 "" H 9550 3100 50  0001 C CNN
F 3 "" H 9550 3100 50  0001 C CNN
	1    9550 3100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9600 2700 9550 2700
Wire Wire Line
	9550 2800 9550 2700
Connection ~ 9550 2700
Wire Wire Line
	9550 2700 9050 2700
Connection ~ 7550 2600
$Comp
L Device:C C46
U 1 1 5B98A94A
P 9400 2300
F 0 "C46" H 9515 2346 50  0000 L CNN
F 1 "1uF" H 9515 2255 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 9438 2150 50  0001 C CNN
F 3 "~" H 9400 2300 50  0001 C CNN
	1    9400 2300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9400 2600 9400 2450
Connection ~ 9400 2600
Wire Wire Line
	9400 2600 9050 2600
Wire Wire Line
	9400 2150 10150 2150
Wire Wire Line
	10150 2150 10150 3100
Wire Wire Line
	10150 3100 9550 3100
Connection ~ 9550 3100
$Comp
L Device:C C42
U 1 1 5B98A958
P 7400 3000
F 0 "C42" H 7515 3046 50  0000 L CNN
F 1 "22uF" H 7515 2955 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7438 2850 50  0001 C CNN
F 3 "~" H 7400 3000 50  0001 C CNN
	1    7400 3000
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0161
U 1 1 5B98A95F
P 7400 3150
F 0 "#PWR0161" H 7400 2900 50  0001 C CNN
F 1 "GND" H 7405 2977 50  0000 C CNN
F 2 "" H 7400 3150 50  0001 C CNN
F 3 "" H 7400 3150 50  0001 C CNN
	1    7400 3150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7400 2850 7400 2600
Connection ~ 7400 2600
Wire Wire Line
	7400 2600 7550 2600
Connection ~ 6250 2600
Text Label 5350 2600 0    50   ~ 0
XBEE_VCC
Text GLabel 2900 3500 0    50   Input ~ 0
XBEE_DTR
Text GLabel 2900 3700 0    50   Input ~ 0
XBEE_CTS
Wire Wire Line
	5350 3700 5550 3700
$Comp
L Connector_Specialized:Test_Point TP2
U 1 1 5B9A5DB7
P 5550 3700
F 0 "TP2" H 5608 3820 50  0000 L CNN
F 1 "Test_Point" H 5608 3729 50  0000 L CNN
F 2 "Connectors_TestPoints:Test_Point_Pad_d1.5mm" H 5750 3700 50  0001 C CNN
F 3 "~" H 5750 3700 50  0001 C CNN
	1    5550 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 3500 3150 3500
Wire Wire Line
	2900 3700 3150 3700
Text GLabel 2900 2600 0    50   Input ~ 0
XBEE_RTS
Wire Wire Line
	3150 2600 2900 2600
Wire Wire Line
	3150 2800 3050 2800
$Comp
L Device:R R21
U 1 1 5B9A7942
P 3000 4450
F 0 "R21" H 3070 4496 50  0000 L CNN
F 1 "330" H 3070 4405 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2930 4450 50  0001 C CNN
F 3 "~" H 3000 4450 50  0001 C CNN
	1    3000 4450
	-1   0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5B9A79D6
P 3000 4750
F 0 "D1" V 2992 4632 50  0000 R CNN
F 1 "LG L29K-F2J1-24-Z " V 2947 4633 50  0001 R CNN
F 2 "LEDs:LED_0603_HandSoldering" H 3000 4750 50  0001 C CNN
F 3 "~" H 3000 4750 50  0001 C CNN
	1    3000 4750
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0163
U 1 1 5B9A7A5C
P 3000 4900
F 0 "#PWR0163" H 3000 4650 50  0001 C CNN
F 1 "GND" H 3005 4727 50  0000 C CNN
F 2 "" H 3000 4900 50  0001 C CNN
F 3 "" H 3000 4900 50  0001 C CNN
	1    3000 4900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3150 3300 3000 3300
Wire Wire Line
	3000 3300 3000 4300
$Comp
L Logic_LevelTranslator:TXB0102DCU U9
U 1 1 5B9A91F0
P 7850 4600
F 0 "U9" H 7600 5050 50  0000 C CNN
F 1 "TXB0102DCU" H 8150 4150 50  0000 C CNN
F 2 "Housings_SSOP:VSSOP-8_2.4x2.1mm_Pitch0.5mm" H 8850 4000 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/txb0102.pdf" H 7850 4570 50  0001 C CNN
	1    7850 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0164
U 1 1 5B9A9700
P 7850 5100
F 0 "#PWR0164" H 7850 4850 50  0001 C CNN
F 1 "GND" H 7855 4927 50  0000 C CNN
F 2 "" H 7850 5100 50  0001 C CNN
F 3 "" H 7850 5100 50  0001 C CNN
	1    7850 5100
	1    0    0    -1  
$EndComp
Text Label 2600 3000 0    50   ~ 0
TX_ISOLATED
Wire Wire Line
	2600 3000 3150 3000
Text Label 2600 3100 0    50   ~ 0
RX_ISOLATED
Wire Wire Line
	2600 3100 3150 3100
Text Label 8700 4500 0    50   ~ 0
TX_ISOLATED
Text Label 8700 4700 0    50   ~ 0
RX_ISOLATED
Wire Wire Line
	8250 4500 8700 4500
Wire Wire Line
	8250 4700 8700 4700
Text GLabel 7100 4500 0    50   Input ~ 0
XBEE_TX
Text GLabel 7100 4700 0    50   Input ~ 0
XBEE_RX
Wire Wire Line
	7100 4500 7450 4500
Wire Wire Line
	7100 4700 7450 4700
$Comp
L Device:C C43
U 1 1 5B9AFCA1
P 7200 3950
F 0 "C43" H 7315 3996 50  0000 L CNN
F 1 "0.1uF" H 7315 3905 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7238 3800 50  0001 C CNN
F 3 "~" H 7200 3950 50  0001 C CNN
	1    7200 3950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0166
U 1 1 5B9AFD63
P 7200 4100
F 0 "#PWR0166" H 7200 3850 50  0001 C CNN
F 1 "GND" H 7205 3927 50  0000 C CNN
F 2 "" H 7200 4100 50  0001 C CNN
F 3 "" H 7200 4100 50  0001 C CNN
	1    7200 4100
	1    0    0    -1  
$EndComp
Text Label 8600 3700 0    50   ~ 0
XBEE_VCC
Wire Wire Line
	7950 3700 7950 4100
$Comp
L power:+3.3V #PWR0167
U 1 1 5B9B18CA
P 7200 3700
F 0 "#PWR0167" H 7200 3550 50  0001 C CNN
F 1 "+3.3V" H 7215 3873 50  0000 C CNN
F 2 "" H 7200 3700 50  0001 C CNN
F 3 "" H 7200 3700 50  0001 C CNN
	1    7200 3700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7200 3700 7200 3800
Wire Wire Line
	7200 3700 7750 3700
Wire Wire Line
	7750 3700 7750 4100
Connection ~ 7200 3700
$Comp
L Device:R R22
U 1 1 5B9B36E8
P 7250 5200
F 0 "R22" H 7320 5246 50  0000 L CNN
F 1 "100K" H 7320 5155 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7180 5200 50  0001 C CNN
F 3 "~" H 7250 5200 50  0001 C CNN
	1    7250 5200
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0168
U 1 1 5B9B3886
P 7250 5350
F 0 "#PWR0168" H 7250 5100 50  0001 C CNN
F 1 "GND" H 7255 5177 50  0000 C CNN
F 2 "" H 7250 5350 50  0001 C CNN
F 3 "" H 7250 5350 50  0001 C CNN
	1    7250 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 5050 7250 4900
Wire Wire Line
	7250 4900 7450 4900
Wire Wire Line
	7250 4900 6950 4900
Connection ~ 7250 4900
Text Label 6950 4900 2    50   ~ 0
XBEE_VCC
Text GLabel 2900 2700 0    50   Input ~ 0
XBEE_RESET
Wire Wire Line
	2900 2700 3150 2700
Wire Wire Line
	6250 2600 7400 2600
Wire Wire Line
	5750 2600 6250 2600
$Comp
L power:GND #PWR0188
U 1 1 5BB7B0DE
P 3400 2150
F 0 "#PWR0188" H 3400 1900 50  0001 C CNN
F 1 "GND" H 3405 1977 50  0000 C CNN
F 2 "" H 3400 2150 50  0001 C CNN
F 3 "" H 3400 2150 50  0001 C CNN
	1    3400 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 2100 3400 2100
Wire Wire Line
	3400 2100 3400 2150
Wire Wire Line
	3050 2100 3050 2800
Wire Wire Line
	7950 3700 8600 3700
$EndSCHEMATC
