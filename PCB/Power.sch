EESchema Schematic File Version 4
LIBS:PCB_Rev3-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 7
Title "TPS63070 Buck Boost Schematic"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L XBP9B-DPST-001:TPS63070RNM U17
U 1 1 5B99D16C
P 6550 4200
F 0 "U17" H 6550 5437 60  0000 C CNN
F 1 "TPS63070RNM" H 6550 5331 60  0000 C CNN
F 2 "WiFiMapSpecific:TPS63070RNMR" H 6550 4140 60  0001 C CNN
F 3 "" H 6550 4200 60  0000 C CNN
	1    6550 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:D D2
U 1 1 5B99DC40
P 1650 3350
F 0 "D2" H 1650 3134 50  0000 C CNN
F 1 "D" H 1650 3225 50  0000 C CNN
F 2 "Diodes_SMD:D_SMC" H 1650 3350 50  0001 C CNN
F 3 "VS-30BQ015-M3/9AT" H 1650 3350 50  0001 C CNN
	1    1650 3350
	-1   0    0    1   
$EndComp
Text GLabel 1100 3350 0    50   Input ~ 0
VBUS
Text GLabel 1100 3750 0    50   Input ~ 0
VBAT
Wire Wire Line
	1100 3350 1500 3350
Wire Wire Line
	1100 3750 1500 3750
Wire Wire Line
	1800 3350 2000 3350
Wire Wire Line
	2000 3350 2000 3550
Wire Wire Line
	2000 3750 1800 3750
$Comp
L Device:C C64
U 1 1 5B99DD01
P 3750 3750
F 0 "C64" H 3865 3796 50  0000 L CNN
F 1 "10uF" H 3865 3705 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3788 3600 50  0001 C CNN
F 3 "~" H 3750 3750 50  0001 C CNN
	1    3750 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C65
U 1 1 5B99DD81
P 4200 3750
F 0 "C65" H 4315 3796 50  0000 L CNN
F 1 "10uF" H 4315 3705 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4238 3600 50  0001 C CNN
F 3 "~" H 4200 3750 50  0001 C CNN
	1    4200 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C66
U 1 1 5B99DDAB
P 4650 3750
F 0 "C66" H 4765 3796 50  0000 L CNN
F 1 "10uF" H 4765 3705 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4688 3600 50  0001 C CNN
F 3 "~" H 4650 3750 50  0001 C CNN
	1    4650 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0201
U 1 1 5B99DE69
P 4650 3900
F 0 "#PWR0201" H 4650 3650 50  0001 C CNN
F 1 "GND" H 4655 3727 50  0000 C CNN
F 2 "" H 4650 3900 50  0001 C CNN
F 3 "" H 4650 3900 50  0001 C CNN
	1    4650 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0202
U 1 1 5B99DE97
P 4200 3900
F 0 "#PWR0202" H 4200 3650 50  0001 C CNN
F 1 "GND" H 4205 3727 50  0000 C CNN
F 2 "" H 4200 3900 50  0001 C CNN
F 3 "" H 4200 3900 50  0001 C CNN
	1    4200 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0203
U 1 1 5B99DEC5
P 3750 3900
F 0 "#PWR0203" H 3750 3650 50  0001 C CNN
F 1 "GND" H 3755 3727 50  0000 C CNN
F 2 "" H 3750 3900 50  0001 C CNN
F 3 "" H 3750 3900 50  0001 C CNN
	1    3750 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 3550 3750 3600
Connection ~ 2000 3550
Wire Wire Line
	2000 3550 2000 3750
Wire Wire Line
	3750 3550 4200 3550
Wire Wire Line
	4200 3550 4200 3600
Wire Wire Line
	4200 3550 4650 3550
Wire Wire Line
	4650 3550 4650 3600
Connection ~ 4200 3550
Wire Wire Line
	4650 3550 5300 3550
Connection ~ 4650 3550
Wire Wire Line
	6100 3700 5950 3700
Wire Wire Line
	5950 3700 5950 3550
Connection ~ 5950 3550
Wire Wire Line
	5950 3550 6100 3550
$Comp
L power:GND #PWR0204
U 1 1 5B99E159
P 5950 4550
F 0 "#PWR0204" H 5950 4300 50  0001 C CNN
F 1 "GND" H 5955 4377 50  0000 C CNN
F 2 "" H 5950 4550 50  0001 C CNN
F 3 "" H 5950 4550 50  0001 C CNN
	1    5950 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4050 5950 4050
Wire Wire Line
	5950 4050 5950 4550
$Comp
L Device:C C67
U 1 1 5B99E25A
P 5450 4400
F 0 "C67" H 5565 4446 50  0000 L CNN
F 1 "0.1uF" H 5565 4355 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 5488 4250 50  0001 C CNN
F 3 "~" H 5450 4400 50  0001 C CNN
	1    5450 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0205
U 1 1 5B99E2BB
P 5450 4550
F 0 "#PWR0205" H 5450 4300 50  0001 C CNN
F 1 "GND" H 5455 4377 50  0000 C CNN
F 2 "" H 5450 4550 50  0001 C CNN
F 3 "" H 5450 4550 50  0001 C CNN
	1    5450 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4200 5450 4200
Wire Wire Line
	5450 4200 5450 4250
$Comp
L Device:R R38
U 1 1 5B99E423
P 5550 3900
F 0 "R38" V 5343 3900 50  0000 C CNN
F 1 "10k" V 5434 3900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5480 3900 50  0001 C CNN
F 3 "~" H 5550 3900 50  0001 C CNN
	1    5550 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	5400 3900 5300 3900
Wire Wire Line
	5300 3900 5300 3550
Connection ~ 5300 3550
Wire Wire Line
	5300 3550 5950 3550
Wire Wire Line
	5700 3900 5850 3900
Wire Wire Line
	6100 4350 5850 4350
Wire Wire Line
	5850 4350 5850 3900
Connection ~ 5850 3900
Wire Wire Line
	5850 3900 6100 3900
$Comp
L power:GND #PWR0206
U 1 1 5B99EA64
P 7150 4550
F 0 "#PWR0206" H 7150 4300 50  0001 C CNN
F 1 "GND" H 7155 4377 50  0000 C CNN
F 2 "" H 7150 4550 50  0001 C CNN
F 3 "" H 7150 4550 50  0001 C CNN
	1    7150 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 4300 7150 4300
Wire Wire Line
	7150 4300 7150 4450
Wire Wire Line
	7000 4450 7150 4450
Connection ~ 7150 4450
Wire Wire Line
	7150 4450 7150 4550
$Comp
L Device:C C69
U 1 1 5B99F04D
P 8800 3700
F 0 "C69" H 8915 3746 50  0000 L CNN
F 1 "22uF" H 8915 3655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8838 3550 50  0001 C CNN
F 3 "~" H 8800 3700 50  0001 C CNN
	1    8800 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C70
U 1 1 5B99F11E
P 9250 3700
F 0 "C70" H 9365 3746 50  0000 L CNN
F 1 "22uF" H 9365 3655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9288 3550 50  0001 C CNN
F 3 "~" H 9250 3700 50  0001 C CNN
	1    9250 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C71
U 1 1 5B99F150
P 9700 3700
F 0 "C71" H 9815 3746 50  0000 L CNN
F 1 "22uF" H 9815 3655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9738 3550 50  0001 C CNN
F 3 "~" H 9700 3700 50  0001 C CNN
	1    9700 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C68
U 1 1 5B99F1F0
P 8350 3700
F 0 "C68" H 8465 3746 50  0000 L CNN
F 1 "10uF" H 8465 3655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 8388 3550 50  0001 C CNN
F 3 "~" H 8350 3700 50  0001 C CNN
	1    8350 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R41
U 1 1 5B99F61F
P 7950 3700
F 0 "R41" H 7880 3654 50  0000 R CNN
F 1 "100k" H 7880 3745 50  0000 R CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7880 3700 50  0001 C CNN
F 3 "~" H 7950 3700 50  0001 C CNN
	1    7950 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	7000 4000 7150 4000
Connection ~ 7150 4300
Wire Wire Line
	7950 4150 7950 3850
Wire Wire Line
	7950 3550 8350 3550
Connection ~ 7950 3550
Wire Wire Line
	8350 3550 8800 3550
Connection ~ 8350 3550
Wire Wire Line
	8800 3550 9250 3550
Connection ~ 8800 3550
$Comp
L power:GND #PWR0207
U 1 1 5B9A11F2
P 8350 3850
F 0 "#PWR0207" H 8350 3600 50  0001 C CNN
F 1 "GND" H 8355 3677 50  0000 C CNN
F 2 "" H 8350 3850 50  0001 C CNN
F 3 "" H 8350 3850 50  0001 C CNN
	1    8350 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0208
U 1 1 5B9A122E
P 8800 3850
F 0 "#PWR0208" H 8800 3600 50  0001 C CNN
F 1 "GND" H 8805 3677 50  0000 C CNN
F 2 "" H 8800 3850 50  0001 C CNN
F 3 "" H 8800 3850 50  0001 C CNN
	1    8800 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0209
U 1 1 5B9A1277
P 9250 3850
F 0 "#PWR0209" H 9250 3600 50  0001 C CNN
F 1 "GND" H 9255 3677 50  0000 C CNN
F 2 "" H 9250 3850 50  0001 C CNN
F 3 "" H 9250 3850 50  0001 C CNN
	1    9250 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0210
U 1 1 5B9A12B3
P 9700 3850
F 0 "#PWR0210" H 9700 3600 50  0001 C CNN
F 1 "GND" H 9705 3677 50  0000 C CNN
F 2 "" H 9700 3850 50  0001 C CNN
F 3 "" H 9700 3850 50  0001 C CNN
	1    9700 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 3550 9700 3550
Connection ~ 9250 3550
Wire Wire Line
	10450 3550 10850 3550
$Comp
L power:+3.3V #PWR0211
U 1 1 5B9A1DFA
P 10850 3550
F 0 "#PWR0211" H 10850 3400 50  0001 C CNN
F 1 "+3.3V" H 10865 3723 50  0000 C CNN
F 2 "" H 10850 3550 50  0001 C CNN
F 3 "" H 10850 3550 50  0001 C CNN
	1    10850 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R40
U 1 1 5B9A1E8E
P 7550 3850
F 0 "R40" V 7757 3850 50  0000 C CNN
F 1 "470k" V 7666 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7480 3850 50  0001 C CNN
F 3 "~" H 7550 3850 50  0001 C CNN
	1    7550 3850
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R39
U 1 1 5B9A1EE9
P 7400 4400
F 0 "R39" H 7470 4446 50  0000 L CNN
F 1 "150k" H 7470 4355 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7330 4400 50  0001 C CNN
F 3 "~" H 7400 4400 50  0001 C CNN
	1    7400 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 4000 7150 4300
Wire Wire Line
	7000 4150 7950 4150
Wire Wire Line
	7000 3550 7150 3550
Wire Wire Line
	7000 3700 7150 3700
Wire Wire Line
	7150 3700 7150 3550
Connection ~ 7150 3550
Wire Wire Line
	7150 3550 7750 3550
Wire Wire Line
	7000 3850 7400 3850
Wire Wire Line
	7700 3850 7750 3850
Wire Wire Line
	7750 3850 7750 3550
Connection ~ 7750 3550
Wire Wire Line
	7750 3550 7950 3550
Wire Wire Line
	7400 4250 7400 3850
Connection ~ 7400 3850
$Comp
L power:GND #PWR0212
U 1 1 5B9A4F6B
P 7400 4550
F 0 "#PWR0212" H 7400 4300 50  0001 C CNN
F 1 "GND" H 7405 4377 50  0000 C CNN
F 2 "" H 7400 4550 50  0001 C CNN
F 3 "" H 7400 4550 50  0001 C CNN
	1    7400 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:L L9
U 1 1 5B9A5016
P 6550 2650
F 0 "L9" V 6740 2650 50  0000 C CNN
F 1 "1.5uH" V 6649 2650 50  0000 C CNN
F 2 "WiFiMapSpecific:XFL4020-152ME" H 6550 2650 50  0001 C CNN
F 3 "XFL4020-152ME" H 6550 2650 50  0001 C CNN
	1    6550 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7000 3350 7150 3350
Wire Wire Line
	7150 3350 7150 2650
Wire Wire Line
	7150 2650 6700 2650
Wire Wire Line
	6400 2650 5950 2650
Wire Wire Line
	5950 2650 5950 3350
Wire Wire Line
	5950 3350 6100 3350
$Comp
L Device:D D3
U 1 1 5B9A6887
P 1650 3750
F 0 "D3" H 1650 3534 50  0000 C CNN
F 1 "D" H 1650 3625 50  0000 C CNN
F 2 "Diodes_SMD:D_SMC" H 1650 3750 50  0001 C CNN
F 3 "VS-30BQ015-M3/9AT" H 1650 3750 50  0001 C CNN
	1    1650 3750
	-1   0    0    1   
$EndComp
$Comp
L Device:R R42
U 1 1 5B9C54AF
P 10450 3750
F 0 "R42" H 10520 3796 50  0000 L CNN
F 1 "330" H 10520 3705 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 10380 3750 50  0001 C CNN
F 3 "~" H 10450 3750 50  0001 C CNN
	1    10450 3750
	-1   0    0    -1  
$EndComp
$Comp
L Device:LED D4
U 1 1 5B9C54B6
P 10450 4150
F 0 "D4" V 10442 4032 50  0000 R CNN
F 1 "LG L29K-F2J1-24-Z " V 10397 4033 50  0001 R CNN
F 2 "LEDs:LED_0603_HandSoldering" H 10450 4150 50  0001 C CNN
F 3 "~" H 10450 4150 50  0001 C CNN
	1    10450 4150
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0213
U 1 1 5B9C54BD
P 10450 4400
F 0 "#PWR0213" H 10450 4150 50  0001 C CNN
F 1 "GND" H 10455 4227 50  0000 C CNN
F 2 "" H 10450 4400 50  0001 C CNN
F 3 "" H 10450 4400 50  0001 C CNN
	1    10450 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10450 3600 10450 3550
Wire Wire Line
	10450 3900 10450 4000
Wire Wire Line
	10450 4300 10450 4400
Wire Wire Line
	9700 3550 10450 3550
Connection ~ 9700 3550
Connection ~ 10450 3550
$Comp
L Amplifier_Operational:OPA2340 U16
U 2 1 5BA9A10F
P 3000 5550
F 0 "U16" H 3000 5400 50  0000 L CNN
F 1 "OPA2340" H 3000 5300 50  0000 L CNN
F 2 "Housings_SSOP:VSSOP-8_3.0x3.0mm_Pitch0.65mm" H 3000 5550 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa4340.pdf" H 3000 5550 50  0001 C CNN
	2    3000 5550
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:OPA2340 U16
U 1 1 5BA9A2ED
P 3000 1850
F 0 "U16" H 3000 1700 50  0000 L CNN
F 1 "OPA2340" H 3000 1600 50  0000 L CNN
F 2 "Housings_SSOP:VSSOP-8_3.0x3.0mm_Pitch0.65mm" H 3000 1850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa4340.pdf" H 3000 1850 50  0001 C CNN
	1    3000 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0153
U 1 1 5BA9A447
P 2900 2150
F 0 "#PWR0153" H 2900 1900 50  0001 C CNN
F 1 "GND" H 2905 1977 50  0000 C CNN
F 2 "" H 2900 2150 50  0001 C CNN
F 3 "" H 2900 2150 50  0001 C CNN
	1    2900 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0162
U 1 1 5BA9A511
P 2900 5850
F 0 "#PWR0162" H 2900 5600 50  0001 C CNN
F 1 "GND" H 2905 5677 50  0000 C CNN
F 2 "" H 2900 5850 50  0001 C CNN
F 3 "" H 2900 5850 50  0001 C CNN
	1    2900 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C62
U 1 1 5BA9B5E4
P 3300 1300
F 0 "C62" H 3415 1346 50  0000 L CNN
F 1 "0.01uF" H 3415 1255 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3338 1150 50  0001 C CNN
F 3 "~" H 3300 1300 50  0001 C CNN
	1    3300 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0200
U 1 1 5BA9B6B6
P 3300 1450
F 0 "#PWR0200" H 3300 1200 50  0001 C CNN
F 1 "GND" H 3305 1277 50  0000 C CNN
F 2 "" H 3300 1450 50  0001 C CNN
F 3 "" H 3300 1450 50  0001 C CNN
	1    3300 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0214
U 1 1 5BA9C5FE
P 2550 1150
F 0 "#PWR0214" H 2550 1000 50  0001 C CNN
F 1 "+3.3V" H 2565 1323 50  0000 C CNN
F 2 "" H 2550 1150 50  0001 C CNN
F 3 "" H 2550 1150 50  0001 C CNN
	1    2550 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 1150 2900 1150
Wire Wire Line
	2900 1150 2900 1550
Wire Wire Line
	2900 1150 3300 1150
Connection ~ 2900 1150
Wire Wire Line
	2700 1950 2550 1950
Wire Wire Line
	2550 1950 2550 2500
Wire Wire Line
	2550 2500 3650 2500
Wire Wire Line
	3650 2500 3650 1850
Wire Wire Line
	3650 1850 3300 1850
$Comp
L Device:R R34
U 1 1 5BAA0965
P 2200 2900
F 0 "R34" V 1993 2900 50  0000 C CNN
F 1 "???" V 2084 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2130 2900 50  0001 C CNN
F 3 "~" H 2200 2900 50  0001 C CNN
	1    2200 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R36
U 1 1 5BAA1EED
P 2900 3550
F 0 "R36" V 2693 3550 50  0000 C CNN
F 1 "0.05" V 2784 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2830 3550 50  0001 C CNN
F 3 "~" H 2900 3550 50  0001 C CNN
	1    2900 3550
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 2150 2900 2150
Connection ~ 2900 2150
Wire Wire Line
	2200 1750 2700 1750
Wire Wire Line
	3650 1850 3950 1850
Connection ~ 3650 1850
$Comp
L Device:C C63
U 1 1 5BAAA764
P 3300 5000
F 0 "C63" H 3415 5046 50  0000 L CNN
F 1 "0.01uF" H 3415 4955 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3338 4850 50  0001 C CNN
F 3 "~" H 3300 5000 50  0001 C CNN
	1    3300 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0215
U 1 1 5BAAA76B
P 3300 5150
F 0 "#PWR0215" H 3300 4900 50  0001 C CNN
F 1 "GND" H 3305 4977 50  0000 C CNN
F 2 "" H 3300 5150 50  0001 C CNN
F 3 "" H 3300 5150 50  0001 C CNN
	1    3300 5150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0216
U 1 1 5BAAA771
P 2550 4850
F 0 "#PWR0216" H 2550 4700 50  0001 C CNN
F 1 "+3.3V" H 2565 5023 50  0000 C CNN
F 2 "" H 2550 4850 50  0001 C CNN
F 3 "" H 2550 4850 50  0001 C CNN
	1    2550 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 4850 2900 4850
Wire Wire Line
	2900 4850 2900 5250
Wire Wire Line
	2900 4850 3300 4850
Connection ~ 2900 4850
Wire Wire Line
	2700 5650 2550 5650
Wire Wire Line
	2550 5650 2550 6150
Wire Wire Line
	2550 6150 3650 6150
Wire Wire Line
	3650 6150 3650 5550
Wire Wire Line
	3650 5550 3300 5550
Wire Wire Line
	3650 5550 4000 5550
Connection ~ 3650 5550
Wire Wire Line
	2200 1750 2200 2150
Connection ~ 3750 3550
$Comp
L Device:R R35
U 1 1 5BAB3DBF
P 2350 2150
F 0 "R35" V 2143 2150 50  0000 C CNN
F 1 "???" V 2234 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2280 2150 50  0001 C CNN
F 3 "~" H 2350 2150 50  0001 C CNN
	1    2350 2150
	0    1    1    0   
$EndComp
Connection ~ 2200 2150
Wire Wire Line
	2200 2150 2200 2750
$Comp
L Device:R R37
U 1 1 5BAB3E77
P 3350 4050
F 0 "R37" V 3143 4050 50  0000 C CNN
F 1 "???" V 3234 4050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3280 4050 50  0001 C CNN
F 3 "~" H 3350 4050 50  0001 C CNN
	1    3350 4050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R33
U 1 1 5BAB3FFD
P 2150 5600
F 0 "R33" V 1943 5600 50  0000 C CNN
F 1 "???" V 2034 5600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2080 5600 50  0001 C CNN
F 3 "~" H 2150 5600 50  0001 C CNN
	1    2150 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 5750 2150 5850
Wire Wire Line
	2150 5850 2900 5850
Connection ~ 2900 5850
Wire Wire Line
	2150 5450 2700 5450
Wire Wire Line
	3350 4200 2150 4200
Wire Wire Line
	2150 4200 2150 5450
Connection ~ 2150 5450
Text GLabel 4000 5550 2    50   Input ~ 0
CC_ADC_CH3
Text GLabel 3950 1850 2    50   Input ~ 0
CC_ADC_CH2
Wire Wire Line
	3050 3550 3250 3550
Wire Wire Line
	2000 3550 2250 3550
Wire Wire Line
	2200 3050 2200 3150
Wire Wire Line
	2200 3150 3250 3150
Wire Wire Line
	3250 3150 3250 3550
Connection ~ 3250 3550
Wire Wire Line
	3250 3550 3750 3550
Wire Wire Line
	3350 3900 2250 3900
Wire Wire Line
	2250 3900 2250 3550
Connection ~ 2250 3550
Wire Wire Line
	2250 3550 2750 3550
$EndSCHEMATC