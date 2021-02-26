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
P 6000 2050
F 0 "U1" H 6300 2700 50  0000 C CNN
F 1 "PIC16F1454-IP" H 6550 2600 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 6000 2050 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/41639A.pdf" H 6000 2050 50  0001 C CNN
	1    6000 2050
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J1
U 1 1 60366616
P 1700 1950
F 0 "J1" H 1757 2417 50  0000 C CNN
F 1 "USB_B_Micro" H 1757 2326 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Amphenol_10118194_Horizontal" H 1850 1900 50  0001 C CNN
F 3 "~" H 1850 1900 50  0001 C CNN
	1    1700 1950
	1    0    0    -1  
$EndComp
NoConn ~ 2000 2150
$Comp
L power:GND #PWR07
U 1 1 6036924D
P 1700 2550
F 0 "#PWR07" H 1700 2300 50  0001 C CNN
F 1 "GND" H 1705 2377 50  0000 C CNN
F 2 "" H 1700 2550 50  0001 C CNN
F 3 "" H 1700 2550 50  0001 C CNN
	1    1700 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 2350 1600 2450
Wire Wire Line
	1600 2450 1700 2450
Wire Wire Line
	1700 2350 1700 2450
Connection ~ 1700 2450
Wire Wire Line
	1700 2450 1700 2550
Wire Wire Line
	2150 1750 2150 1550
Wire Wire Line
	6000 1150 6000 1250
$Comp
L power:GND #PWR02
U 1 1 6036CC6B
P 5400 1300
F 0 "#PWR02" H 5400 1050 50  0001 C CNN
F 1 "GND" H 5405 1127 50  0000 C CNN
F 2 "" H 5400 1300 50  0001 C CNN
F 3 "" H 5400 1300 50  0001 C CNN
	1    5400 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 6036E100
P 5700 1250
F 0 "C1" V 5448 1250 50  0000 C CNN
F 1 "470n" V 5539 1250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5738 1100 50  0001 C CNN
F 3 "~" H 5700 1250 50  0001 C CNN
	1    5700 1250
	0    1    1    0   
$EndComp
Wire Wire Line
	5400 1300 5400 1250
Wire Wire Line
	5400 1250 5550 1250
Wire Wire Line
	5850 1250 6000 1250
Connection ~ 6000 1250
Wire Wire Line
	6000 1250 6000 1450
$Comp
L Connector:Conn_PIC_ICSP_ICD J2
U 1 1 6036F92E
P 3700 3750
F 0 "J2" H 3900 4200 50  0000 R CNN
F 1 "Conn_PIC_ICSP_ICD" H 4300 4100 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 3750 3900 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/devicedoc/30277d.pdf" V 3400 3600 50  0001 C CNN
	1    3700 3750
	1    0    0    -1  
$EndComp
NoConn ~ 3500 3350
$Comp
L power:GND #PWR011
U 1 1 60370F7A
P 3500 4250
F 0 "#PWR011" H 3500 4000 50  0001 C CNN
F 1 "GND" H 3505 4077 50  0000 C CNN
F 2 "" H 3500 4250 50  0001 C CNN
F 3 "" H 3500 4250 50  0001 C CNN
	1    3500 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4150 3500 4250
$Comp
L power:GND #PWR08
U 1 1 6037157A
P 6000 2750
F 0 "#PWR08" H 6000 2500 50  0001 C CNN
F 1 "GND" H 6005 2577 50  0000 C CNN
F 2 "" H 6000 2750 50  0001 C CNN
F 3 "" H 6000 2750 50  0001 C CNN
	1    6000 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 2650 6000 2750
$Comp
L Device:C C2
U 1 1 60372C1D
P 3750 1750
F 0 "C2" V 3498 1750 50  0000 C CNN
F 1 "470n" V 3589 1750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3788 1600 50  0001 C CNN
F 3 "~" H 3750 1750 50  0001 C CNN
	1    3750 1750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 60373024
P 3450 1800
F 0 "#PWR04" H 3450 1550 50  0001 C CNN
F 1 "GND" H 3300 1750 50  0000 C CNN
F 2 "" H 3450 1800 50  0001 C CNN
F 3 "" H 3450 1800 50  0001 C CNN
	1    3450 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 1750 4000 1750
NoConn ~ 4200 3950
NoConn ~ 4000 2250
NoConn ~ 8000 2150
Wire Wire Line
	3600 1750 3450 1750
Wire Wire Line
	3450 1750 3450 1800
Wire Wire Line
	2000 1950 4000 1950
Wire Wire Line
	2000 2050 4000 2050
$Comp
L MCU_Microchip_PIC16:PIC16F1454-IP U2
U 1 1 60392DFE
P 6000 5650
F 0 "U2" H 6300 6300 50  0000 C CNN
F 1 "PIC16F1454-IP" H 6550 6200 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 6000 5650 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/41639A.pdf" H 6000 5650 50  0001 C CNN
	1    6000 5650
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J3
U 1 1 60392F2E
P 1700 5550
F 0 "J3" H 1757 6017 50  0000 C CNN
F 1 "USB_B_Micro" H 1757 5926 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Amphenol_10118194_Horizontal" H 1850 5500 50  0001 C CNN
F 3 "~" H 1850 5500 50  0001 C CNN
	1    1700 5550
	1    0    0    -1  
$EndComp
NoConn ~ 2000 5750
$Comp
L power:GND #PWR018
U 1 1 60392F39
P 1700 6150
F 0 "#PWR018" H 1700 5900 50  0001 C CNN
F 1 "GND" H 1705 5977 50  0000 C CNN
F 2 "" H 1700 6150 50  0001 C CNN
F 3 "" H 1700 6150 50  0001 C CNN
	1    1700 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 5950 1600 6050
Wire Wire Line
	1600 6050 1700 6050
Wire Wire Line
	1700 5950 1700 6050
Connection ~ 1700 6050
Wire Wire Line
	1700 6050 1700 6150
Wire Wire Line
	2000 5350 2150 5350
Wire Wire Line
	2150 5350 2150 5150
Wire Wire Line
	6000 4750 6000 4850
$Comp
L power:GND #PWR013
U 1 1 60392F5F
P 5400 4900
F 0 "#PWR013" H 5400 4650 50  0001 C CNN
F 1 "GND" H 5405 4727 50  0000 C CNN
F 2 "" H 5400 4900 50  0001 C CNN
F 3 "" H 5400 4900 50  0001 C CNN
	1    5400 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 60392F69
P 5700 4850
F 0 "C3" V 5448 4850 50  0000 C CNN
F 1 "470n" V 5539 4850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5738 4700 50  0001 C CNN
F 3 "~" H 5700 4850 50  0001 C CNN
	1    5700 4850
	0    1    1    0   
$EndComp
Wire Wire Line
	5400 4900 5400 4850
Wire Wire Line
	5400 4850 5550 4850
Wire Wire Line
	5850 4850 6000 4850
Connection ~ 6000 4850
Wire Wire Line
	6000 4850 6000 5050
$Comp
L power:GND #PWR019
U 1 1 60392F7A
P 6000 6350
F 0 "#PWR019" H 6000 6100 50  0001 C CNN
F 1 "GND" H 6005 6177 50  0000 C CNN
F 2 "" H 6000 6350 50  0001 C CNN
F 3 "" H 6000 6350 50  0001 C CNN
	1    6000 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 6250 6000 6350
$Comp
L Device:C C4
U 1 1 60392F89
P 3750 5350
F 0 "C4" V 3498 5350 50  0000 C CNN
F 1 "470n" V 3589 5350 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3788 5200 50  0001 C CNN
F 3 "~" H 3750 5350 50  0001 C CNN
	1    3750 5350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR015
U 1 1 60392F93
P 3450 5400
F 0 "#PWR015" H 3450 5150 50  0001 C CNN
F 1 "GND" H 3300 5350 50  0000 C CNN
F 2 "" H 3450 5400 50  0001 C CNN
F 3 "" H 3450 5400 50  0001 C CNN
	1    3450 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 5350 4000 5350
NoConn ~ 4000 5850
NoConn ~ 8000 5750
Wire Wire Line
	3600 5350 3450 5350
Wire Wire Line
	3450 5350 3450 5400
Wire Wire Line
	2000 5550 4000 5550
Wire Wire Line
	2000 5650 4000 5650
$Comp
L local:VDD1 #PWR03
U 1 1 6036E3C6
P 2150 1550
F 0 "#PWR03" H 2150 1400 50  0001 C CNN
F 1 "VDD1" H 2165 1723 50  0000 C CNN
F 2 "" H 2150 1550 50  0001 C CNN
F 3 "" H 2150 1550 50  0001 C CNN
	1    2150 1550
	1    0    0    -1  
$EndComp
$Comp
L local:VDD1 #PWR01
U 1 1 6036EE33
P 6000 1150
F 0 "#PWR01" H 6000 1000 50  0001 C CNN
F 1 "VDD1" H 6015 1323 50  0000 C CNN
F 2 "" H 6000 1150 50  0001 C CNN
F 3 "" H 6000 1150 50  0001 C CNN
	1    6000 1150
	1    0    0    -1  
$EndComp
$Comp
L local:VDD2 #PWR014
U 1 1 6036FB70
P 2150 5150
F 0 "#PWR014" H 2150 5000 50  0001 C CNN
F 1 "VDD2" H 2165 5323 50  0000 C CNN
F 2 "" H 2150 5150 50  0001 C CNN
F 3 "" H 2150 5150 50  0001 C CNN
	1    2150 5150
	1    0    0    -1  
$EndComp
$Comp
L local:VDD2 #PWR012
U 1 1 603703E3
P 6000 4750
F 0 "#PWR012" H 6000 4600 50  0001 C CNN
F 1 "VDD2" H 6015 4923 50  0000 C CNN
F 2 "" H 6000 4750 50  0001 C CNN
F 3 "" H 6000 4750 50  0001 C CNN
	1    6000 4750
	1    0    0    -1  
$EndComp
$Comp
L local:VDD1 #PWR06
U 1 1 60371884
P 3750 2350
F 0 "#PWR06" H 3750 2200 50  0001 C CNN
F 1 "VDD1" H 3600 2400 50  0000 C CNN
F 2 "" H 3750 2350 50  0001 C CNN
F 3 "" H 3750 2350 50  0001 C CNN
	1    3750 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2350 3750 2350
$Comp
L power:GND #PWR017
U 1 1 6037496F
P 3750 5950
F 0 "#PWR017" H 3750 5700 50  0001 C CNN
F 1 "GND" H 3755 5777 50  0000 C CNN
F 2 "" H 3750 5950 50  0001 C CNN
F 3 "" H 3750 5950 50  0001 C CNN
	1    3750 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 5950 3750 5950
$Comp
L Device:LED D1
U 1 1 603787CE
P 8400 2250
F 0 "D1" H 8400 2150 50  0000 C CNN
F 1 "LED" H 8400 2050 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 8400 2250 50  0001 C CNN
F 3 "~" H 8400 2250 50  0001 C CNN
	1    8400 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 6037A5C4
P 8800 2250
F 0 "R1" V 8900 2250 50  0000 C CNN
F 1 "1K5" V 9000 2250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 8730 2250 50  0001 C CNN
F 3 "~" H 8800 2250 50  0001 C CNN
	1    8800 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	8550 2250 8650 2250
Wire Wire Line
	4200 3750 9450 3750
Text Label 8150 3750 0    50   ~ 0
SCL_ICSPDAT
Text Label 8150 3850 0    50   ~ 0
SDA_ICSPCLK
$Comp
L local:VDD1 #PWR05
U 1 1 6038C989
P 9050 2150
F 0 "#PWR05" H 9050 2000 50  0001 C CNN
F 1 "VDD1" H 9200 2200 50  0000 C CNN
F 2 "" H 9050 2150 50  0001 C CNN
F 3 "" H 9050 2150 50  0001 C CNN
	1    9050 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 2150 9050 2250
Wire Wire Line
	9050 2250 8950 2250
$Comp
L Device:LED D3
U 1 1 6038F120
P 8400 5850
F 0 "D3" H 8400 5750 50  0000 C CNN
F 1 "LED" H 8400 5650 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 8400 5850 50  0001 C CNN
F 3 "~" H 8400 5850 50  0001 C CNN
	1    8400 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 6038F32F
P 8800 5850
F 0 "R2" V 8900 5850 50  0000 C CNN
F 1 "1K5" V 9000 5850 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 8730 5850 50  0001 C CNN
F 3 "~" H 8800 5850 50  0001 C CNN
	1    8800 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	8550 5850 8650 5850
Wire Wire Line
	9050 5750 9050 5850
Wire Wire Line
	9050 5850 8950 5850
$Comp
L local:VDD2 #PWR016
U 1 1 60394761
P 9050 5750
F 0 "#PWR016" H 9050 5600 50  0001 C CNN
F 1 "VDD2" H 9200 5800 50  0000 C CNN
F 2 "" H 9050 5750 50  0001 C CNN
F 3 "" H 9050 5750 50  0001 C CNN
	1    9050 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 1750 2150 1750
$Comp
L Diode:BAT54C D2
U 1 1 6038E22E
P 2850 3000
F 0 "D2" V 2804 3088 50  0000 L CNN
F 1 "BAT54C" V 2895 3088 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2925 3125 50  0001 L CNN
F 3 "http://www.diodes.com/_files/datasheets/ds11005.pdf" H 2770 3000 50  0001 C CNN
	1    2850 3000
	0    -1   1    0   
$EndComp
Wire Wire Line
	2850 2700 2850 2150
Wire Wire Line
	2850 2150 4000 2150
Wire Wire Line
	4000 5750 2850 5750
Wire Wire Line
	2850 5750 2850 3300
$Comp
L Transistor_FET:2N7002 Q1
U 1 1 6039BE8E
P 9550 3350
F 0 "Q1" H 9750 3300 50  0000 L CNN
F 1 "2N7002" H 9750 3400 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9750 3275 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 9550 3350 50  0001 L CNN
	1    9550 3350
	-1   0    0    1   
$EndComp
Wire Wire Line
	8000 1750 9450 1750
Wire Wire Line
	9450 1750 9450 3150
Wire Wire Line
	9450 3550 9450 3750
$Comp
L Transistor_FET:2N7002 Q3
U 1 1 603B789A
P 9550 4250
F 0 "Q3" H 9750 4300 50  0000 L CNN
F 1 "2N7002" H 9750 4200 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9750 4175 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 9550 4250 50  0001 L CNN
	1    9550 4250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8000 5350 9450 5350
Wire Wire Line
	9450 5350 9450 4450
Wire Wire Line
	9450 4050 9450 3750
Connection ~ 9450 3750
$Comp
L Transistor_FET:2N7002 Q2
U 1 1 603CD14D
P 10050 3350
F 0 "Q2" H 10250 3300 50  0000 L CNN
F 1 "2N7002" H 10250 3400 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10250 3275 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 10050 3350 50  0001 L CNN
	1    10050 3350
	1    0    0    1   
$EndComp
$Comp
L Transistor_FET:2N7002 Q4
U 1 1 603D9144
P 10050 4250
F 0 "Q4" H 10250 4300 50  0000 L CNN
F 1 "2N7002" H 10250 4200 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10250 4175 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 10050 4250 50  0001 L CNN
	1    10050 4250
	1    0    0    -1  
$EndComp
$Comp
L local:VDD1 #PWR09
U 1 1 6040E69D
P 9800 3250
F 0 "#PWR09" H 9800 3100 50  0001 C CNN
F 1 "VDD1" H 9815 3423 50  0000 C CNN
F 2 "" H 9800 3250 50  0001 C CNN
F 3 "" H 9800 3250 50  0001 C CNN
	1    9800 3250
	1    0    0    -1  
$EndComp
Text Label 8150 1750 0    50   ~ 0
SCL_ICSPDAT_1
Text Label 8150 1850 0    50   ~ 0
SDA_ICSPCLK_1
Text Label 8150 5350 0    50   ~ 0
SCL_ICSPDAT_2
Text Label 8150 5450 0    50   ~ 0
SDA_ICSPCLK_2
Wire Wire Line
	9750 3350 9800 3350
Wire Wire Line
	9800 3250 9800 3350
Connection ~ 9800 3350
Wire Wire Line
	9800 3350 9850 3350
Wire Wire Line
	8000 1850 10150 1850
Wire Wire Line
	10150 1850 10150 3150
Wire Wire Line
	10150 3850 10150 3550
Wire Wire Line
	4200 3850 10150 3850
Wire Wire Line
	10150 3850 10150 4050
Connection ~ 10150 3850
Wire Wire Line
	8000 5450 10150 5450
Wire Wire Line
	10150 5450 10150 4450
Wire Wire Line
	9750 4250 9800 4250
$Comp
L local:VDD2 #PWR010
U 1 1 604F93F5
P 9800 4150
F 0 "#PWR010" H 9800 4000 50  0001 C CNN
F 1 "VDD2" H 9815 4323 50  0000 C CNN
F 2 "" H 9800 4150 50  0001 C CNN
F 3 "" H 9800 4150 50  0001 C CNN
	1    9800 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 4150 9800 4250
Connection ~ 9800 4250
Wire Wire Line
	9800 4250 9850 4250
Text Label 2250 1950 0    50   ~ 0
USB1_D+
Text Label 2250 2050 0    50   ~ 0
USB1_D-
Text Label 3050 5750 0    50   ~ 0
~MCLR2~
Text Label 3050 2150 0    50   ~ 0
~MCLR1~
Text Label 3050 3000 0    50   ~ 0
~MCLR~
Wire Wire Line
	3050 3000 4500 3000
Wire Wire Line
	4500 3000 4500 3550
Wire Wire Line
	4500 3550 4200 3550
Text Label 2250 5550 0    50   ~ 0
USB2_D+
Text Label 2250 5650 0    50   ~ 0
USB2_D-
NoConn ~ 8000 2250
Wire Wire Line
	8150 2250 8250 2250
Text Label 8150 2150 0    50   ~ 0
STATUS1
NoConn ~ 8000 5850
Wire Wire Line
	8250 5850 8150 5850
Wire Wire Line
	8150 5850 8150 5650
Text Label 8150 5750 0    50   ~ 0
STATUS2
Wire Wire Line
	8000 2050 8150 2050
Wire Wire Line
	8150 2050 8150 2250
NoConn ~ 8000 1950
Wire Wire Line
	8000 5650 8150 5650
NoConn ~ 8000 5550
$EndSCHEMATC
