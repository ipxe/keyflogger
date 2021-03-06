EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "keyflogger PIC16F145x"
Date ""
Rev ""
Comp "Fen Systems Ltd."
Comment1 ""
Comment2 "https://github.com/ipxe/keyflogger"
Comment3 ""
Comment4 "USB keyboard/mouse injector (PIC16F145x-based)"
$EndDescr
$Comp
L MCU_Microchip_PIC16:PIC16F1454-IP U1
U 1 1 60365657
P 4950 2000
F 0 "U1" H 5250 2650 50  0000 C CNN
F 1 "PIC16F1454-IP" H 5500 2550 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 4950 2000 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/41639A.pdf" H 4950 2000 50  0001 C CNN
	1    4950 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 6036924D
P 1100 2500
F 0 "#PWR01" H 1100 2250 50  0001 C CNN
F 1 "GND" H 1105 2327 50  0000 C CNN
F 2 "" H 1100 2500 50  0001 C CNN
F 3 "" H 1100 2500 50  0001 C CNN
	1    1100 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 2300 1000 2400
Wire Wire Line
	1000 2400 1100 2400
Wire Wire Line
	1100 2300 1100 2400
Connection ~ 1100 2400
Wire Wire Line
	1100 2400 1100 2500
Wire Wire Line
	1550 1700 1550 1500
Wire Wire Line
	4950 1100 4950 1200
$Comp
L power:GND #PWR07
U 1 1 6036CC6B
P 4350 1250
F 0 "#PWR07" H 4350 1000 50  0001 C CNN
F 1 "GND" H 4355 1077 50  0000 C CNN
F 2 "" H 4350 1250 50  0001 C CNN
F 3 "" H 4350 1250 50  0001 C CNN
	1    4350 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 6036E100
P 4650 1200
F 0 "C3" V 4398 1200 50  0000 C CNN
F 1 "470n" V 4489 1200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4688 1050 50  0001 C CNN
F 3 "~" H 4650 1200 50  0001 C CNN
	1    4650 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 1250 4350 1200
Wire Wire Line
	4350 1200 4500 1200
Wire Wire Line
	4800 1200 4950 1200
Connection ~ 4950 1200
Wire Wire Line
	4950 1200 4950 1400
$Comp
L Connector:Conn_PIC_ICSP_ICD J3
U 1 1 6036F92E
P 10550 1700
F 0 "J3" H 10750 2150 50  0000 R CNN
F 1 "Conn_PIC_ICSP_ICD" H 11150 2050 50  0000 R CNN
F 2 "Connector:Tag-Connect_TC2030-IDC-NL_2x03_P1.27mm_Vertical" H 10600 1850 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/devicedoc/30277d.pdf" V 10250 1550 50  0001 C CNN
	1    10550 1700
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 60370F7A
P 10750 2200
F 0 "#PWR018" H 10750 1950 50  0001 C CNN
F 1 "GND" H 10755 2027 50  0000 C CNN
F 2 "" H 10750 2200 50  0001 C CNN
F 3 "" H 10750 2200 50  0001 C CNN
	1    10750 2200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10750 2100 10750 2200
$Comp
L power:GND #PWR010
U 1 1 6037157A
P 4950 2700
F 0 "#PWR010" H 4950 2450 50  0001 C CNN
F 1 "GND" H 4955 2527 50  0000 C CNN
F 2 "" H 4950 2700 50  0001 C CNN
F 3 "" H 4950 2700 50  0001 C CNN
	1    4950 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 2600 4950 2700
NoConn ~ 10050 1900
NoConn ~ 2950 2200
NoConn ~ 6950 2100
$Comp
L MCU_Microchip_PIC16:PIC16F1454-IP U2
U 1 1 60392DFE
P 4950 5350
F 0 "U2" H 5250 6000 50  0000 C CNN
F 1 "PIC16F1454-IP" H 5500 5900 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 4950 5350 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/41639A.pdf" H 4950 5350 50  0001 C CNN
	1    4950 5350
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J2
U 1 1 60392F2E
P 1100 5250
F 0 "J2" H 1157 5717 50  0000 C CNN
F 1 "USB_B_Micro" H 1157 5626 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Amphenol_10118194_Horizontal" H 1250 5200 50  0001 C CNN
F 3 "~" H 1250 5200 50  0001 C CNN
	1    1100 5250
	1    0    0    -1  
$EndComp
NoConn ~ 1400 5450
$Comp
L power:GND #PWR02
U 1 1 60392F39
P 1100 5850
F 0 "#PWR02" H 1100 5600 50  0001 C CNN
F 1 "GND" H 1105 5677 50  0000 C CNN
F 2 "" H 1100 5850 50  0001 C CNN
F 3 "" H 1100 5850 50  0001 C CNN
	1    1100 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 5650 1000 5750
Wire Wire Line
	1000 5750 1100 5750
Wire Wire Line
	1100 5650 1100 5750
Connection ~ 1100 5750
Wire Wire Line
	1100 5750 1100 5850
Wire Wire Line
	1400 5050 1550 5050
Wire Wire Line
	1550 5050 1550 4850
Wire Wire Line
	4950 4450 4950 4550
$Comp
L power:GND #PWR08
U 1 1 60392F5F
P 4350 4600
F 0 "#PWR08" H 4350 4350 50  0001 C CNN
F 1 "GND" H 4355 4427 50  0000 C CNN
F 2 "" H 4350 4600 50  0001 C CNN
F 3 "" H 4350 4600 50  0001 C CNN
	1    4350 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 60392F69
P 4650 4550
F 0 "C4" V 4398 4550 50  0000 C CNN
F 1 "470n" V 4489 4550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4688 4400 50  0001 C CNN
F 3 "~" H 4650 4550 50  0001 C CNN
	1    4650 4550
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 4600 4350 4550
Wire Wire Line
	4350 4550 4500 4550
Wire Wire Line
	4800 4550 4950 4550
Connection ~ 4950 4550
Wire Wire Line
	4950 4550 4950 4750
$Comp
L power:GND #PWR012
U 1 1 60392F7A
P 4950 6050
F 0 "#PWR012" H 4950 5800 50  0001 C CNN
F 1 "GND" H 4955 5877 50  0000 C CNN
F 2 "" H 4950 6050 50  0001 C CNN
F 3 "" H 4950 6050 50  0001 C CNN
	1    4950 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 5950 4950 6050
$Comp
L Device:C C2
U 1 1 60392F89
P 3250 4550
F 0 "C2" V 2998 4550 50  0000 C CNN
F 1 "470n" V 3089 4550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3288 4400 50  0001 C CNN
F 3 "~" H 3250 4550 50  0001 C CNN
	1    3250 4550
	0    1    1    0   
$EndComp
NoConn ~ 2950 5550
NoConn ~ 6950 5450
$Comp
L local:VDD1 #PWR03
U 1 1 6036E3C6
P 1550 1500
F 0 "#PWR03" H 1550 1350 50  0001 C CNN
F 1 "VDD1" H 1565 1673 50  0000 C CNN
F 2 "" H 1550 1500 50  0001 C CNN
F 3 "" H 1550 1500 50  0001 C CNN
	1    1550 1500
	1    0    0    -1  
$EndComp
$Comp
L local:VDD1 #PWR09
U 1 1 6036EE33
P 4950 1100
F 0 "#PWR09" H 4950 950 50  0001 C CNN
F 1 "VDD1" H 4965 1273 50  0000 C CNN
F 2 "" H 4950 1100 50  0001 C CNN
F 3 "" H 4950 1100 50  0001 C CNN
	1    4950 1100
	1    0    0    -1  
$EndComp
$Comp
L local:VDD2 #PWR04
U 1 1 6036FB70
P 1550 4850
F 0 "#PWR04" H 1550 4700 50  0001 C CNN
F 1 "VDD2" H 1565 5023 50  0000 C CNN
F 2 "" H 1550 4850 50  0001 C CNN
F 3 "" H 1550 4850 50  0001 C CNN
	1    1550 4850
	1    0    0    -1  
$EndComp
$Comp
L local:VDD2 #PWR011
U 1 1 603703E3
P 4950 4450
F 0 "#PWR011" H 4950 4300 50  0001 C CNN
F 1 "VDD2" H 4965 4623 50  0000 C CNN
F 2 "" H 4950 4450 50  0001 C CNN
F 3 "" H 4950 4450 50  0001 C CNN
	1    4950 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 603787CE
P 7750 2200
F 0 "D1" H 7750 2100 50  0000 C CNN
F 1 "LED" H 7750 2000 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 7750 2200 50  0001 C CNN
F 3 "~" H 7750 2200 50  0001 C CNN
	1    7750 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 6037A5C4
P 7350 2200
F 0 "R1" V 7450 2200 50  0000 C CNN
F 1 "1K5" V 7550 2200 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 7280 2200 50  0001 C CNN
F 3 "~" H 7350 2200 50  0001 C CNN
	1    7350 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	7500 2200 7600 2200
Text Label 8400 3250 2    50   ~ 0
SCL
Text Label 9100 3250 0    50   ~ 0
SDA
$Comp
L local:VDD1 #PWR013
U 1 1 6038C989
P 8000 2100
F 0 "#PWR013" H 8000 1950 50  0001 C CNN
F 1 "VDD1" H 8150 2150 50  0000 C CNN
F 2 "" H 8000 2100 50  0001 C CNN
F 3 "" H 8000 2100 50  0001 C CNN
	1    8000 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 2100 8000 2200
Wire Wire Line
	8000 2200 7900 2200
$Comp
L Device:LED D2
U 1 1 6038F120
P 7750 5550
F 0 "D2" H 7750 5450 50  0000 C CNN
F 1 "LED" H 7750 5350 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 7750 5550 50  0001 C CNN
F 3 "~" H 7750 5550 50  0001 C CNN
	1    7750 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 6038F32F
P 7350 5550
F 0 "R2" V 7450 5550 50  0000 C CNN
F 1 "1K5" V 7550 5550 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 7280 5550 50  0001 C CNN
F 3 "~" H 7350 5550 50  0001 C CNN
	1    7350 5550
	0    1    1    0   
$EndComp
Wire Wire Line
	7500 5550 7600 5550
Wire Wire Line
	8000 5450 8000 5550
Wire Wire Line
	8000 5550 7900 5550
$Comp
L local:VDD2 #PWR014
U 1 1 60394761
P 8000 5450
F 0 "#PWR014" H 8000 5300 50  0001 C CNN
F 1 "VDD2" H 8150 5500 50  0000 C CNN
F 2 "" H 8000 5450 50  0001 C CNN
F 3 "" H 8000 5450 50  0001 C CNN
	1    8000 5450
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q1
U 1 1 6039BE8E
P 8500 2350
F 0 "Q1" H 8700 2300 50  0000 L CNN
F 1 "2N7002" H 8700 2400 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8700 2275 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 8500 2350 50  0001 L CNN
	1    8500 2350
	-1   0    0    1   
$EndComp
Wire Wire Line
	6950 1700 8400 1700
Wire Wire Line
	8400 1700 8400 2150
$Comp
L Transistor_FET:2N7002 Q2
U 1 1 603B789A
P 8500 4250
F 0 "Q2" H 8700 4300 50  0000 L CNN
F 1 "2N7002" H 8700 4200 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8700 4175 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 8500 4250 50  0001 L CNN
	1    8500 4250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6950 5050 8400 5050
Wire Wire Line
	8400 5050 8400 4450
$Comp
L Transistor_FET:2N7002 Q3
U 1 1 603CD14D
P 9000 2350
F 0 "Q3" H 9200 2300 50  0000 L CNN
F 1 "2N7002" H 9200 2400 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9200 2275 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 9000 2350 50  0001 L CNN
	1    9000 2350
	1    0    0    1   
$EndComp
$Comp
L Transistor_FET:2N7002 Q4
U 1 1 603D9144
P 9000 4250
F 0 "Q4" H 9200 4300 50  0000 L CNN
F 1 "2N7002" H 9200 4200 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9200 4175 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 9000 4250 50  0001 L CNN
	1    9000 4250
	1    0    0    -1  
$EndComp
Text Label 7050 1700 0    50   ~ 0
SCL_ICSPDAT1
Text Label 7050 1800 0    50   ~ 0
SDA_ICSPCLK1
Text Label 7050 5050 0    50   ~ 0
SCL_ICSPDAT2
Text Label 7050 5150 0    50   ~ 0
SDA_ICSPCLK2
Wire Wire Line
	6950 1800 9100 1800
Wire Wire Line
	9100 1800 9100 2150
Wire Wire Line
	6950 5150 9100 5150
Wire Wire Line
	9100 5150 9100 4450
Wire Wire Line
	8700 4250 8750 4250
Connection ~ 8750 4250
Wire Wire Line
	8750 4250 8800 4250
Text Label 1500 1900 0    50   ~ 0
USB1_D+
Text Label 1500 2000 0    50   ~ 0
USB1_D-
Wire Wire Line
	9850 1500 10050 1500
Text Label 1500 5250 0    50   ~ 0
USB2_D+
Text Label 1500 5350 0    50   ~ 0
USB2_D-
Text Label 7050 2200 0    50   ~ 0
LED1
Text Label 7050 5550 0    50   ~ 0
LED2
NoConn ~ 6950 5250
$Comp
L Connector:USB_A J1
U 1 1 6051C542
P 1100 1900
F 0 "J1" H 1157 2367 50  0000 C CNN
F 1 "USB_A" H 1157 2276 50  0000 C CNN
F 2 "Connector_USB:USB_A_CNCTech_1001-011-01101_Horizontal" H 1250 1850 50  0001 C CNN
F 3 " ~" H 1250 1850 50  0001 C CNN
	1    1100 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 603A7947
P 8750 2600
F 0 "R3" H 8820 2646 50  0000 L CNN
F 1 "1M" H 8820 2555 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 8680 2600 50  0001 C CNN
F 3 "~" H 8750 2600 50  0001 C CNN
	1    8750 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 603A83FD
P 8750 2850
F 0 "#PWR015" H 8750 2600 50  0001 C CNN
F 1 "GND" H 8755 2677 50  0000 C CNN
F 2 "" H 8750 2850 50  0001 C CNN
F 3 "" H 8750 2850 50  0001 C CNN
	1    8750 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 603B9288
P 8750 4500
F 0 "R4" H 8820 4546 50  0000 L CNN
F 1 "1M" H 8820 4455 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 8680 4500 50  0001 C CNN
F 3 "~" H 8750 4500 50  0001 C CNN
	1    8750 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 2550 9100 4050
Wire Wire Line
	8400 2550 8400 4050
$Comp
L local:VDD1 #PWR017
U 1 1 6043B491
P 10750 1200
F 0 "#PWR017" H 10750 1050 50  0001 C CNN
F 1 "VDD1" H 10765 1373 50  0000 C CNN
F 2 "" H 10750 1200 50  0001 C CNN
F 3 "" H 10750 1200 50  0001 C CNN
	1    10750 1200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10750 1200 10750 1300
$Comp
L Connector:Conn_PIC_ICSP_ICD J4
U 1 1 604489A2
P 10550 5050
F 0 "J4" H 10750 5500 50  0000 R CNN
F 1 "Conn_PIC_ICSP_ICD" H 11150 5400 50  0000 R CNN
F 2 "Connector:Tag-Connect_TC2030-IDC-NL_2x03_P1.27mm_Vertical" H 10600 5200 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/devicedoc/30277d.pdf" V 10250 4900 50  0001 C CNN
	1    10550 5050
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 60448B46
P 10750 5550
F 0 "#PWR020" H 10750 5300 50  0001 C CNN
F 1 "GND" H 10755 5377 50  0000 C CNN
F 2 "" H 10750 5550 50  0001 C CNN
F 3 "" H 10750 5550 50  0001 C CNN
	1    10750 5550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10750 5450 10750 5550
NoConn ~ 10050 5250
Wire Wire Line
	9850 4850 10050 4850
Wire Wire Line
	10750 4550 10750 4650
$Comp
L local:VDD2 #PWR019
U 1 1 604D48B3
P 10750 4550
F 0 "#PWR019" H 10750 4400 50  0001 C CNN
F 1 "VDD2" H 10765 4723 50  0000 C CNN
F 2 "" H 10750 4550 50  0001 C CNN
F 3 "" H 10750 4550 50  0001 C CNN
	1    10750 4550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1400 1700 1550 1700
Wire Wire Line
	1400 5350 2950 5350
Wire Wire Line
	1400 1900 2950 1900
Wire Wire Line
	1400 2000 2950 2000
Wire Wire Line
	1400 5250 2950 5250
Wire Wire Line
	8400 5050 10050 5050
Connection ~ 8400 5050
Wire Wire Line
	10050 5150 9100 5150
Connection ~ 9100 5150
Wire Wire Line
	8400 1700 10050 1700
Connection ~ 8400 1700
Wire Wire Line
	9100 1800 10050 1800
Connection ~ 9100 1800
Wire Wire Line
	2850 1700 2950 1700
$Comp
L Device:C C1
U 1 1 60372C1D
P 3250 1200
F 0 "C1" V 2998 1200 50  0000 C CNN
F 1 "470n" V 3089 1200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3288 1050 50  0001 C CNN
F 3 "~" H 3250 1200 50  0001 C CNN
	1    3250 1200
	0    -1   1    0   
$EndComp
Wire Wire Line
	2850 1700 2850 1200
Wire Wire Line
	2850 1200 3100 1200
$Comp
L power:GND #PWR05
U 1 1 6065A589
P 3550 1250
F 0 "#PWR05" H 3550 1000 50  0001 C CNN
F 1 "GND" H 3555 1077 50  0000 C CNN
F 2 "" H 3550 1250 50  0001 C CNN
F 3 "" H 3550 1250 50  0001 C CNN
	1    3550 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 1200 3550 1200
Wire Wire Line
	3550 1200 3550 1250
Wire Wire Line
	2950 5050 2850 5050
Wire Wire Line
	2850 5050 2850 4550
Wire Wire Line
	2850 4550 3100 4550
$Comp
L power:GND #PWR06
U 1 1 6044EC15
P 3550 4600
F 0 "#PWR06" H 3550 4350 50  0001 C CNN
F 1 "GND" H 3555 4427 50  0000 C CNN
F 2 "" H 3550 4600 50  0001 C CNN
F 3 "" H 3550 4600 50  0001 C CNN
	1    3550 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 4550 3550 4550
Wire Wire Line
	3550 4550 3550 4600
Wire Wire Line
	8700 2350 8750 2350
Connection ~ 8750 2350
Wire Wire Line
	8750 2350 8800 2350
Wire Wire Line
	8750 2850 8750 2750
$Comp
L power:GND #PWR016
U 1 1 604A37D4
P 8750 4750
F 0 "#PWR016" H 8750 4500 50  0001 C CNN
F 1 "GND" H 8755 4577 50  0000 C CNN
F 2 "" H 8750 4750 50  0001 C CNN
F 3 "" H 8750 4750 50  0001 C CNN
	1    8750 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 4750 8750 4650
Wire Wire Line
	8750 4250 8750 4350
Wire Wire Line
	8750 4250 8750 3800
Wire Wire Line
	8750 3800 2350 3800
Wire Wire Line
	2350 3800 2350 5650
Wire Wire Line
	2350 5650 2950 5650
Text Label 7050 1900 0    50   ~ 0
I2C_EN1
Text Label 7050 3800 0    50   ~ 0
I2C_EN2
Wire Wire Line
	2650 5450 2950 5450
Wire Wire Line
	2650 5450 2650 6350
Wire Wire Line
	2650 6350 9850 6350
Wire Wire Line
	9850 6350 9850 4850
Wire Wire Line
	2950 2100 2650 2100
Wire Wire Line
	2650 2100 2650 800 
Wire Wire Line
	2650 800  9850 800 
Wire Wire Line
	9850 800  9850 1500
Text Label 6050 800  0    50   ~ 0
~MCLR~_VPP1
Text Label 6050 6350 0    50   ~ 0
~MCLR~_VPP2
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 60555845
P 1650 1700
F 0 "#FLG0101" H 1650 1775 50  0001 C CNN
F 1 "PWR_FLAG" V 1650 1828 50  0000 L CNN
F 2 "" H 1650 1700 50  0001 C CNN
F 3 "~" H 1650 1700 50  0001 C CNN
	1    1650 1700
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 1700 1550 1700
Connection ~ 1550 1700
Text Label 2850 1450 0    50   ~ 0
VUSB1
Text Label 2850 4800 0    50   ~ 0
VUSB2
Wire Wire Line
	6950 2200 7200 2200
NoConn ~ 6950 2000
Wire Wire Line
	6950 1900 8750 1900
Wire Wire Line
	8750 1900 8750 2350
Wire Wire Line
	8750 2350 8750 2450
NoConn ~ 2950 2300
Wire Wire Line
	7200 5550 6950 5550
NoConn ~ 6950 5350
$EndSCHEMATC
