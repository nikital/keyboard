EESchema Schematic File Version 4
EELAYER 30 0
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
L Connector:RJ12 J1
U 1 1 6134DF61
P 1050 1450
F 0 "J1" H 1050 1550 50  0000 C CNN
F 1 "RJ12" H 1000 1350 50  0000 C CNN
F 2 "local_footprint:RJ12_Custom1" V 1050 1475 50  0001 C CNN
F 3 "~" V 1050 1475 50  0001 C CNN
	1    1050 1450
	1    0    0    -1  
$EndComp
NoConn ~ 1450 1150
NoConn ~ 1450 1650
$Comp
L power:GND #PWR03
U 1 1 6134F4BC
P 1600 1550
F 0 "#PWR03" H 1600 1300 50  0001 C CNN
F 1 "GND" V 1605 1422 50  0000 R CNN
F 2 "" H 1600 1550 50  0001 C CNN
F 3 "" H 1600 1550 50  0001 C CNN
	1    1600 1550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1450 1550 1600 1550
$Comp
L power:+5V #PWR02
U 1 1 613504AB
P 1600 1350
F 0 "#PWR02" H 1600 1200 50  0001 C CNN
F 1 "+5V" V 1615 1478 50  0000 L CNN
F 2 "" H 1600 1350 50  0001 C CNN
F 3 "" H 1600 1350 50  0001 C CNN
	1    1600 1350
	0    1    1    0   
$EndComp
Wire Wire Line
	1600 1350 1450 1350
Text Label 1450 1250 0    50   ~ 0
SDA
Text Label 1450 1450 0    50   ~ 0
SCL
$Comp
L symbols:MCP23018_SO U1
U 1 1 61357520
P 1900 3800
F 0 "U1" H 1950 4000 50  0000 C CNN
F 1 "MCP23018_SO" H 1800 3350 50  0000 C CNN
F 2 "Package_SO:SOIC-28W_7.5x17.9mm_P1.27mm" H 2100 2800 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf" H 2100 2700 50  0001 L CNN
	1    1900 3800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR05
U 1 1 61358D70
P 1900 2400
F 0 "#PWR05" H 1900 2250 50  0001 C CNN
F 1 "+5V" H 1915 2573 50  0000 C CNN
F 2 "" H 1900 2400 50  0001 C CNN
F 3 "" H 1900 2400 50  0001 C CNN
	1    1900 2400
	1    0    0    -1  
$EndComp
NoConn ~ 1200 3500
NoConn ~ 1200 3600
Text Label 1200 3100 2    50   ~ 0
SDA
Text Label 1200 3000 2    50   ~ 0
SCL
$Comp
L Device:R R2
U 1 1 6135C1BB
P 1050 3950
F 0 "R2" V 843 3950 50  0000 C CNN
F 1 "10K" V 934 3950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 980 3950 50  0001 C CNN
F 3 "~" H 1050 3950 50  0001 C CNN
	1    1050 3950
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR08
U 1 1 6135C94B
P 800 3850
F 0 "#PWR08" H 800 3700 50  0001 C CNN
F 1 "+5V" H 815 4023 50  0000 C CNN
F 2 "" H 800 3850 50  0001 C CNN
F 3 "" H 800 3850 50  0001 C CNN
	1    800  3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  3950 800  3950
Wire Wire Line
	800  3950 800  3850
$Comp
L Device:R R3
U 1 1 61360243
P 1050 4600
F 0 "R3" V 843 4600 50  0000 C CNN
F 1 "10K" V 934 4600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 980 4600 50  0001 C CNN
F 3 "~" H 1050 4600 50  0001 C CNN
	1    1050 4600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 613612F4
P 800 4700
F 0 "#PWR09" H 800 4450 50  0001 C CNN
F 1 "GND" H 805 4527 50  0000 C CNN
F 2 "" H 800 4700 50  0001 C CNN
F 3 "" H 800 4700 50  0001 C CNN
	1    800  4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  4700 800  4600
Wire Wire Line
	800  4600 900  4600
$Comp
L power:GND #PWR010
U 1 1 61361F23
P 1900 4900
F 0 "#PWR010" H 1900 4650 50  0001 C CNN
F 1 "GND" H 1905 4727 50  0000 C CNN
F 2 "" H 1900 4900 50  0001 C CNN
F 3 "" H 1900 4900 50  0001 C CNN
	1    1900 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 61365CC0
P 2100 2550
F 0 "C1" H 2215 2596 50  0000 L CNN
F 1 "100n" H 2215 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2138 2400 50  0001 C CNN
F 3 "~" H 2100 2550 50  0001 C CNN
	1    2100 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 61366260
P 2100 2700
F 0 "#PWR06" H 2100 2450 50  0001 C CNN
F 1 "GND" H 2300 2600 50  0000 C CNN
F 2 "" H 2100 2700 50  0001 C CNN
F 3 "" H 2100 2700 50  0001 C CNN
	1    2100 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 2400 1900 2700
Wire Wire Line
	2100 2400 1900 2400
Connection ~ 1900 2400
$Comp
L Power_Protection:PRTR5V0U2X D36
U 1 1 61369C2B
P 2750 1450
F 0 "D36" H 3000 1050 50  0000 L CNN
F 1 "PRTR5V0U2X" H 3000 950 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-143" H 2810 1450 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/PRTR5V0U2X.pdf" H 2810 1450 50  0001 C CNN
	1    2750 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 6136B027
P 2750 950
F 0 "#PWR01" H 2750 800 50  0001 C CNN
F 1 "+5V" H 2765 1123 50  0000 C CNN
F 2 "" H 2750 950 50  0001 C CNN
F 3 "" H 2750 950 50  0001 C CNN
	1    2750 950 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 6136B804
P 2750 1950
F 0 "#PWR04" H 2750 1700 50  0001 C CNN
F 1 "GND" H 2755 1777 50  0000 C CNN
F 2 "" H 2750 1950 50  0001 C CNN
F 3 "" H 2750 1950 50  0001 C CNN
	1    2750 1950
	1    0    0    -1  
$EndComp
Text Label 3250 1450 0    50   ~ 0
SDA
Text Label 2250 1450 2    50   ~ 0
SCL
$Comp
L Device:D_Small D1
U 1 1 61381CE0
P 4350 1750
F 0 "D1" V 4350 1680 50  0000 R CNN
F 1 "D_Small" V 4305 1680 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4350 1750 50  0001 C CNN
F 3 "~" V 4350 1750 50  0001 C CNN
	1    4350 1750
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW2
U 1 1 61388F27
P 4900 1550
F 0 "SW2" V 4800 1600 50  0000 R CNN
F 1 "SW_Push_45deg" V 4855 1406 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4900 1550 50  0001 C CNN
F 3 "~" H 4900 1550 50  0001 C CNN
	1    4900 1550
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D2
U 1 1 6138903B
P 4800 1750
F 0 "D2" V 4800 1680 50  0000 R CNN
F 1 "D_Small" V 4755 1680 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4800 1750 50  0001 C CNN
F 3 "~" V 4800 1750 50  0001 C CNN
	1    4800 1750
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW3
U 1 1 6138C259
P 5350 1550
F 0 "SW3" V 5250 1600 50  0000 R CNN
F 1 "SW_Push_45deg" V 5305 1406 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5350 1550 50  0001 C CNN
F 3 "~" H 5350 1550 50  0001 C CNN
	1    5350 1550
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D3
U 1 1 6138C39B
P 5250 1750
F 0 "D3" V 5250 1680 50  0000 R CNN
F 1 "D_Small" V 5205 1680 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5250 1750 50  0001 C CNN
F 3 "~" V 5250 1750 50  0001 C CNN
	1    5250 1750
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW4
U 1 1 6138C3A5
P 5800 1550
F 0 "SW4" V 5700 1600 50  0000 R CNN
F 1 "SW_Push_45deg" V 5755 1406 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5800 1550 50  0001 C CNN
F 3 "~" H 5800 1550 50  0001 C CNN
	1    5800 1550
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D4
U 1 1 6138C3AF
P 5700 1750
F 0 "D4" V 5700 1680 50  0000 R CNN
F 1 "D_Small" V 5655 1680 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5700 1750 50  0001 C CNN
F 3 "~" V 5700 1750 50  0001 C CNN
	1    5700 1750
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW5
U 1 1 613959E1
P 6250 1550
F 0 "SW5" V 6150 1600 50  0000 R CNN
F 1 "SW_Push_45deg" V 6205 1406 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6250 1550 50  0001 C CNN
F 3 "~" H 6250 1550 50  0001 C CNN
	1    6250 1550
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D5
U 1 1 61395B7F
P 6150 1750
F 0 "D5" V 6150 1680 50  0000 R CNN
F 1 "D_Small" V 6105 1680 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6150 1750 50  0001 C CNN
F 3 "~" V 6150 1750 50  0001 C CNN
	1    6150 1750
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW6
U 1 1 61395B89
P 6700 1550
F 0 "SW6" V 6600 1600 50  0000 R CNN
F 1 "SW_Push_45deg" V 6655 1406 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6700 1550 50  0001 C CNN
F 3 "~" H 6700 1550 50  0001 C CNN
	1    6700 1550
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D6
U 1 1 61395B93
P 6600 1750
F 0 "D6" V 6600 1680 50  0000 R CNN
F 1 "D_Small" V 6555 1680 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6600 1750 50  0001 C CNN
F 3 "~" V 6600 1750 50  0001 C CNN
	1    6600 1750
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW1
U 1 1 61380334
P 4450 1550
F 0 "SW1" V 4350 1600 50  0000 R CNN
F 1 "SW_Push_45deg" V 4405 1406 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4450 1550 50  0001 C CNN
F 3 "~" H 4450 1550 50  0001 C CNN
	1    4450 1550
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D7
U 1 1 613A74F6
P 4350 2300
F 0 "D7" V 4350 2230 50  0000 R CNN
F 1 "D_Small" V 4305 2230 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4350 2300 50  0001 C CNN
F 3 "~" V 4350 2300 50  0001 C CNN
	1    4350 2300
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW8
U 1 1 613A773A
P 4900 2100
F 0 "SW8" V 4800 2150 50  0000 R CNN
F 1 "SW_Push_45deg" V 4855 1956 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4900 2100 50  0001 C CNN
F 3 "~" H 4900 2100 50  0001 C CNN
	1    4900 2100
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D8
U 1 1 613A7744
P 4800 2300
F 0 "D8" V 4800 2230 50  0000 R CNN
F 1 "D_Small" V 4755 2230 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4800 2300 50  0001 C CNN
F 3 "~" V 4800 2300 50  0001 C CNN
	1    4800 2300
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW9
U 1 1 613A774E
P 5350 2100
F 0 "SW9" V 5250 2150 50  0000 R CNN
F 1 "SW_Push_45deg" V 5305 1956 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5350 2100 50  0001 C CNN
F 3 "~" H 5350 2100 50  0001 C CNN
	1    5350 2100
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D9
U 1 1 613A7758
P 5250 2300
F 0 "D9" V 5250 2230 50  0000 R CNN
F 1 "D_Small" V 5205 2230 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5250 2300 50  0001 C CNN
F 3 "~" V 5250 2300 50  0001 C CNN
	1    5250 2300
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW10
U 1 1 613A7762
P 5800 2100
F 0 "SW10" V 5700 2150 50  0000 R CNN
F 1 "SW_Push_45deg" V 5755 1956 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5800 2100 50  0001 C CNN
F 3 "~" H 5800 2100 50  0001 C CNN
	1    5800 2100
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D10
U 1 1 613A776C
P 5700 2300
F 0 "D10" V 5700 2230 50  0000 R CNN
F 1 "D_Small" V 5655 2230 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5700 2300 50  0001 C CNN
F 3 "~" V 5700 2300 50  0001 C CNN
	1    5700 2300
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW11
U 1 1 613A7776
P 6250 2100
F 0 "SW11" V 6150 2150 50  0000 R CNN
F 1 "SW_Push_45deg" V 6205 1956 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6250 2100 50  0001 C CNN
F 3 "~" H 6250 2100 50  0001 C CNN
	1    6250 2100
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D11
U 1 1 613A7780
P 6150 2300
F 0 "D11" V 6150 2230 50  0000 R CNN
F 1 "D_Small" V 6105 2230 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6150 2300 50  0001 C CNN
F 3 "~" V 6150 2300 50  0001 C CNN
	1    6150 2300
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW12
U 1 1 613A778A
P 6700 2100
F 0 "SW12" V 6600 2150 50  0000 R CNN
F 1 "SW_Push_45deg" V 6655 1956 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6700 2100 50  0001 C CNN
F 3 "~" H 6700 2100 50  0001 C CNN
	1    6700 2100
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D12
U 1 1 613A7794
P 6600 2300
F 0 "D12" V 6600 2230 50  0000 R CNN
F 1 "D_Small" V 6555 2230 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6600 2300 50  0001 C CNN
F 3 "~" V 6600 2300 50  0001 C CNN
	1    6600 2300
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW7
U 1 1 613A779E
P 4450 2100
F 0 "SW7" V 4350 2150 50  0000 R CNN
F 1 "SW_Push_45deg" V 4405 1956 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4450 2100 50  0001 C CNN
F 3 "~" H 4450 2100 50  0001 C CNN
	1    4450 2100
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D13
U 1 1 613BD1CC
P 4350 2850
F 0 "D13" V 4350 2780 50  0000 R CNN
F 1 "D_Small" V 4305 2780 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4350 2850 50  0001 C CNN
F 3 "~" V 4350 2850 50  0001 C CNN
	1    4350 2850
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW14
U 1 1 613BD524
P 4900 2650
F 0 "SW14" V 4800 2700 50  0000 R CNN
F 1 "SW_Push_45deg" V 4855 2506 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4900 2650 50  0001 C CNN
F 3 "~" H 4900 2650 50  0001 C CNN
	1    4900 2650
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D14
U 1 1 613BD52E
P 4800 2850
F 0 "D14" V 4800 2780 50  0000 R CNN
F 1 "D_Small" V 4755 2780 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4800 2850 50  0001 C CNN
F 3 "~" V 4800 2850 50  0001 C CNN
	1    4800 2850
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW15
U 1 1 613BD538
P 5350 2650
F 0 "SW15" V 5250 2700 50  0000 R CNN
F 1 "SW_Push_45deg" V 5305 2506 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5350 2650 50  0001 C CNN
F 3 "~" H 5350 2650 50  0001 C CNN
	1    5350 2650
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D15
U 1 1 613BD542
P 5250 2850
F 0 "D15" V 5250 2780 50  0000 R CNN
F 1 "D_Small" V 5205 2780 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5250 2850 50  0001 C CNN
F 3 "~" V 5250 2850 50  0001 C CNN
	1    5250 2850
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW16
U 1 1 613BD54C
P 5800 2650
F 0 "SW16" V 5700 2700 50  0000 R CNN
F 1 "SW_Push_45deg" V 5755 2506 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5800 2650 50  0001 C CNN
F 3 "~" H 5800 2650 50  0001 C CNN
	1    5800 2650
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D16
U 1 1 613BD556
P 5700 2850
F 0 "D16" V 5700 2780 50  0000 R CNN
F 1 "D_Small" V 5655 2780 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5700 2850 50  0001 C CNN
F 3 "~" V 5700 2850 50  0001 C CNN
	1    5700 2850
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW17
U 1 1 613BD560
P 6250 2650
F 0 "SW17" V 6150 2700 50  0000 R CNN
F 1 "SW_Push_45deg" V 6205 2506 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6250 2650 50  0001 C CNN
F 3 "~" H 6250 2650 50  0001 C CNN
	1    6250 2650
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D17
U 1 1 613BD56A
P 6150 2850
F 0 "D17" V 6150 2780 50  0000 R CNN
F 1 "D_Small" V 6105 2780 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6150 2850 50  0001 C CNN
F 3 "~" V 6150 2850 50  0001 C CNN
	1    6150 2850
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW18
U 1 1 613BD574
P 6700 2650
F 0 "SW18" V 6600 2700 50  0000 R CNN
F 1 "SW_Push_45deg" V 6655 2506 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6700 2650 50  0001 C CNN
F 3 "~" H 6700 2650 50  0001 C CNN
	1    6700 2650
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D18
U 1 1 613BD57E
P 6600 2850
F 0 "D18" V 6600 2780 50  0000 R CNN
F 1 "D_Small" V 6555 2780 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6600 2850 50  0001 C CNN
F 3 "~" V 6600 2850 50  0001 C CNN
	1    6600 2850
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW13
U 1 1 613BD588
P 4450 2650
F 0 "SW13" V 4350 2700 50  0000 R CNN
F 1 "SW_Push_45deg" V 4405 2506 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4450 2650 50  0001 C CNN
F 3 "~" H 4450 2650 50  0001 C CNN
	1    4450 2650
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D19
U 1 1 613BD592
P 4350 3400
F 0 "D19" V 4350 3330 50  0000 R CNN
F 1 "D_Small" V 4305 3330 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4350 3400 50  0001 C CNN
F 3 "~" V 4350 3400 50  0001 C CNN
	1    4350 3400
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW20
U 1 1 613BD59C
P 4900 3200
F 0 "SW20" V 4800 3250 50  0000 R CNN
F 1 "SW_Push_45deg" V 4855 3056 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4900 3200 50  0001 C CNN
F 3 "~" H 4900 3200 50  0001 C CNN
	1    4900 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D20
U 1 1 613BD5A6
P 4800 3400
F 0 "D20" V 4800 3330 50  0000 R CNN
F 1 "D_Small" V 4755 3330 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4800 3400 50  0001 C CNN
F 3 "~" V 4800 3400 50  0001 C CNN
	1    4800 3400
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW21
U 1 1 613BD5B0
P 5350 3200
F 0 "SW21" V 5250 3250 50  0000 R CNN
F 1 "SW_Push_45deg" V 5305 3056 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5350 3200 50  0001 C CNN
F 3 "~" H 5350 3200 50  0001 C CNN
	1    5350 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D21
U 1 1 613BD5BA
P 5250 3400
F 0 "D21" V 5250 3330 50  0000 R CNN
F 1 "D_Small" V 5205 3330 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5250 3400 50  0001 C CNN
F 3 "~" V 5250 3400 50  0001 C CNN
	1    5250 3400
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW22
U 1 1 613BD5C4
P 5800 3200
F 0 "SW22" V 5700 3250 50  0000 R CNN
F 1 "SW_Push_45deg" V 5755 3056 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5800 3200 50  0001 C CNN
F 3 "~" H 5800 3200 50  0001 C CNN
	1    5800 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D22
U 1 1 613BD5CE
P 5700 3400
F 0 "D22" V 5700 3330 50  0000 R CNN
F 1 "D_Small" V 5655 3330 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5700 3400 50  0001 C CNN
F 3 "~" V 5700 3400 50  0001 C CNN
	1    5700 3400
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW23
U 1 1 613BD5D8
P 6250 3200
F 0 "SW23" V 6150 3250 50  0000 R CNN
F 1 "SW_Push_45deg" V 6205 3056 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6250 3200 50  0001 C CNN
F 3 "~" H 6250 3200 50  0001 C CNN
	1    6250 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D23
U 1 1 613BD5E2
P 6150 3400
F 0 "D23" V 6150 3330 50  0000 R CNN
F 1 "D_Small" V 6105 3330 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6150 3400 50  0001 C CNN
F 3 "~" V 6150 3400 50  0001 C CNN
	1    6150 3400
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW24
U 1 1 613BD5EC
P 6700 3200
F 0 "SW24" V 6600 3250 50  0000 R CNN
F 1 "SW_Push_45deg" V 6655 3056 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6700 3200 50  0001 C CNN
F 3 "~" H 6700 3200 50  0001 C CNN
	1    6700 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D24
U 1 1 613BD5F6
P 6600 3400
F 0 "D24" V 6600 3330 50  0000 R CNN
F 1 "D_Small" V 6555 3330 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6600 3400 50  0001 C CNN
F 3 "~" V 6600 3400 50  0001 C CNN
	1    6600 3400
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW19
U 1 1 613BD600
P 4450 3200
F 0 "SW19" V 4350 3250 50  0000 R CNN
F 1 "SW_Push_45deg" V 4405 3056 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4450 3200 50  0001 C CNN
F 3 "~" H 4450 3200 50  0001 C CNN
	1    4450 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D25
U 1 1 61408C0C
P 4350 3950
F 0 "D25" V 4350 3880 50  0000 R CNN
F 1 "D_Small" V 4305 3880 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4350 3950 50  0001 C CNN
F 3 "~" V 4350 3950 50  0001 C CNN
	1    4350 3950
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW26
U 1 1 6140915C
P 4900 3750
F 0 "SW26" V 4800 3800 50  0000 R CNN
F 1 "SW_Push_45deg" V 4855 3606 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4900 3750 50  0001 C CNN
F 3 "~" H 4900 3750 50  0001 C CNN
	1    4900 3750
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D26
U 1 1 61409166
P 4800 3950
F 0 "D26" V 4800 3880 50  0000 R CNN
F 1 "D_Small" V 4755 3880 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4800 3950 50  0001 C CNN
F 3 "~" V 4800 3950 50  0001 C CNN
	1    4800 3950
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW27
U 1 1 61409170
P 5350 3750
F 0 "SW27" V 5250 3800 50  0000 R CNN
F 1 "SW_Push_45deg" V 5305 3606 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5350 3750 50  0001 C CNN
F 3 "~" H 5350 3750 50  0001 C CNN
	1    5350 3750
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D27
U 1 1 6140917A
P 5250 3950
F 0 "D27" V 5250 3880 50  0000 R CNN
F 1 "D_Small" V 5205 3880 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5250 3950 50  0001 C CNN
F 3 "~" V 5250 3950 50  0001 C CNN
	1    5250 3950
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW28
U 1 1 61409184
P 5800 3750
F 0 "SW28" V 5700 3800 50  0000 R CNN
F 1 "SW_Push_45deg" V 5755 3606 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5800 3750 50  0001 C CNN
F 3 "~" H 5800 3750 50  0001 C CNN
	1    5800 3750
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D28
U 1 1 6140918E
P 5700 3950
F 0 "D28" V 5700 3880 50  0000 R CNN
F 1 "D_Small" V 5655 3880 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5700 3950 50  0001 C CNN
F 3 "~" V 5700 3950 50  0001 C CNN
	1    5700 3950
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW29
U 1 1 61409198
P 6250 3750
F 0 "SW29" V 6150 3800 50  0000 R CNN
F 1 "SW_Push_45deg" V 6205 3606 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6250 3750 50  0001 C CNN
F 3 "~" H 6250 3750 50  0001 C CNN
	1    6250 3750
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D29
U 1 1 614091A2
P 6150 3950
F 0 "D29" V 6150 3880 50  0000 R CNN
F 1 "D_Small" V 6105 3880 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6150 3950 50  0001 C CNN
F 3 "~" V 6150 3950 50  0001 C CNN
	1    6150 3950
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW30
U 1 1 614091AC
P 6700 3750
F 0 "SW30" V 6600 3800 50  0000 R CNN
F 1 "SW_Push_45deg" V 6655 3606 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6700 3750 50  0001 C CNN
F 3 "~" H 6700 3750 50  0001 C CNN
	1    6700 3750
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D30
U 1 1 614091B6
P 6600 3950
F 0 "D30" V 6600 3880 50  0000 R CNN
F 1 "D_Small" V 6555 3880 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6600 3950 50  0001 C CNN
F 3 "~" V 6600 3950 50  0001 C CNN
	1    6600 3950
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW25
U 1 1 614091C0
P 4450 3750
F 0 "SW25" V 4350 3800 50  0000 R CNN
F 1 "SW_Push_45deg" V 4405 3606 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4450 3750 50  0001 C CNN
F 3 "~" H 4450 3750 50  0001 C CNN
	1    4450 3750
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D31
U 1 1 614091CA
P 4350 4500
F 0 "D31" V 4350 4430 50  0000 R CNN
F 1 "D_Small" V 4305 4430 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4350 4500 50  0001 C CNN
F 3 "~" V 4350 4500 50  0001 C CNN
	1    4350 4500
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW32
U 1 1 614091D4
P 4900 4300
F 0 "SW32" V 4800 4350 50  0000 R CNN
F 1 "SW_Push_45deg" V 4855 4156 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4900 4300 50  0001 C CNN
F 3 "~" H 4900 4300 50  0001 C CNN
	1    4900 4300
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D32
U 1 1 614091DE
P 4800 4500
F 0 "D32" V 4800 4430 50  0000 R CNN
F 1 "D_Small" V 4755 4430 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 4800 4500 50  0001 C CNN
F 3 "~" V 4800 4500 50  0001 C CNN
	1    4800 4500
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW33
U 1 1 614091E8
P 5350 4300
F 0 "SW33" V 5250 4350 50  0000 R CNN
F 1 "SW_Push_45deg" V 5305 4156 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 5350 4300 50  0001 C CNN
F 3 "~" H 5350 4300 50  0001 C CNN
	1    5350 4300
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D33
U 1 1 614091F2
P 5250 4500
F 0 "D33" V 5250 4430 50  0000 R CNN
F 1 "D_Small" V 5205 4430 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 5250 4500 50  0001 C CNN
F 3 "~" V 5250 4500 50  0001 C CNN
	1    5250 4500
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW34
U 1 1 61409210
P 6250 4300
F 0 "SW34" V 6150 4350 50  0000 R CNN
F 1 "SW_Push_45deg" V 6205 4156 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6250 4300 50  0001 C CNN
F 3 "~" H 6250 4300 50  0001 C CNN
	1    6250 4300
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D34
U 1 1 6140921A
P 6150 4500
F 0 "D34" V 6150 4430 50  0000 R CNN
F 1 "D_Small" V 6105 4430 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6150 4500 50  0001 C CNN
F 3 "~" V 6150 4500 50  0001 C CNN
	1    6150 4500
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_45deg SW31
U 1 1 61409238
P 4450 4300
F 0 "SW31" V 4350 4350 50  0000 R CNN
F 1 "SW_Push_45deg" V 4405 4156 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 4450 4300 50  0001 C CNN
F 3 "~" H 4450 4300 50  0001 C CNN
	1    4450 4300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4350 1850 4800 1850
Connection ~ 4800 1850
Wire Wire Line
	4800 1850 5250 1850
Connection ~ 5250 1850
Wire Wire Line
	5250 1850 5700 1850
Connection ~ 5700 1850
Wire Wire Line
	5700 1850 6150 1850
Connection ~ 6150 1850
Wire Wire Line
	6150 1850 6600 1850
Wire Wire Line
	6600 2400 6150 2400
Connection ~ 4800 2400
Wire Wire Line
	4800 2400 4350 2400
Connection ~ 5250 2400
Wire Wire Line
	5250 2400 4800 2400
Connection ~ 5700 2400
Wire Wire Line
	5700 2400 5250 2400
Connection ~ 6150 2400
Wire Wire Line
	6150 2400 5700 2400
Wire Wire Line
	6600 2950 6150 2950
Connection ~ 4800 2950
Wire Wire Line
	4800 2950 4350 2950
Connection ~ 5250 2950
Wire Wire Line
	5250 2950 4800 2950
Connection ~ 5700 2950
Wire Wire Line
	5700 2950 5250 2950
Connection ~ 6150 2950
Wire Wire Line
	6150 2950 5700 2950
Wire Wire Line
	4350 3500 4800 3500
Connection ~ 4800 3500
Wire Wire Line
	4800 3500 5250 3500
Connection ~ 5250 3500
Wire Wire Line
	5250 3500 5700 3500
Connection ~ 5700 3500
Wire Wire Line
	5700 3500 6150 3500
Connection ~ 6150 3500
Wire Wire Line
	6150 3500 6600 3500
Wire Wire Line
	6600 4050 6150 4050
Connection ~ 4800 4050
Wire Wire Line
	4800 4050 4350 4050
Connection ~ 5250 4050
Wire Wire Line
	5250 4050 4800 4050
Connection ~ 5700 4050
Wire Wire Line
	5700 4050 5250 4050
Connection ~ 6150 4050
Wire Wire Line
	6150 4050 5700 4050
Wire Wire Line
	4350 4600 4800 4600
Connection ~ 4800 4600
Wire Wire Line
	4800 4600 5250 4600
Connection ~ 5250 4600
Connection ~ 6150 4600
Wire Wire Line
	4550 3100 4550 3650
Connection ~ 4550 3650
Wire Wire Line
	4550 3650 4550 4200
Connection ~ 4550 4200
Wire Wire Line
	4550 4200 4550 4900
Wire Wire Line
	5000 3100 5000 3650
Connection ~ 5000 3650
Wire Wire Line
	5000 3650 5000 4200
Connection ~ 5000 4200
Wire Wire Line
	5000 4200 5000 4900
Wire Wire Line
	5450 3100 5450 3650
Connection ~ 5450 3650
Wire Wire Line
	5450 3650 5450 4200
Connection ~ 5450 4200
Wire Wire Line
	5450 4200 5450 4900
Wire Wire Line
	5900 3100 5900 3650
Connection ~ 5900 3650
Wire Wire Line
	6350 3100 6350 3650
Connection ~ 6350 3650
Wire Wire Line
	6350 3650 6350 4200
Connection ~ 6350 4200
Wire Wire Line
	6350 4200 6350 4900
Wire Wire Line
	6800 3100 6800 3650
Connection ~ 6800 3650
Wire Wire Line
	4550 2550 4550 2000
Connection ~ 4550 1450
Wire Wire Line
	4550 1450 4550 1100
Connection ~ 4550 2000
Wire Wire Line
	4550 2000 4550 1450
Wire Wire Line
	5000 2550 5000 2000
Connection ~ 5000 1450
Wire Wire Line
	5000 1450 5000 1100
Connection ~ 5000 2000
Wire Wire Line
	5000 2000 5000 1450
Wire Wire Line
	5450 2550 5450 2000
Connection ~ 5450 1450
Wire Wire Line
	5450 1450 5450 1100
Connection ~ 5450 2000
Wire Wire Line
	5450 2000 5450 1450
Wire Wire Line
	5900 2550 5900 2000
Connection ~ 5900 1450
Wire Wire Line
	5900 1450 5900 1100
Connection ~ 5900 2000
Wire Wire Line
	5900 2000 5900 1450
Wire Wire Line
	6350 2550 6350 2000
Connection ~ 6350 1450
Wire Wire Line
	6350 1450 6350 1100
Connection ~ 6350 2000
Wire Wire Line
	6350 2000 6350 1450
Wire Wire Line
	6800 2550 6800 2000
Connection ~ 6800 1450
Wire Wire Line
	6800 1450 6800 1100
Connection ~ 6800 2000
Wire Wire Line
	6800 2000 6800 1450
Wire Wire Line
	4350 1850 4050 1850
Connection ~ 4350 1850
Wire Wire Line
	4350 2400 4050 2400
Connection ~ 4350 2400
Wire Wire Line
	4350 2950 4050 2950
Connection ~ 4350 2950
Wire Wire Line
	6600 3500 6900 3500
Wire Wire Line
	6900 3500 6900 2950
Wire Wire Line
	6900 2950 6600 2950
Connection ~ 6600 3500
Connection ~ 6600 2950
Wire Wire Line
	6600 4050 6950 4050
Wire Wire Line
	6950 4050 6950 2400
Wire Wire Line
	6950 2400 6600 2400
Connection ~ 6600 4050
Connection ~ 6600 2400
Wire Wire Line
	7000 4600 7000 1850
Wire Wire Line
	7000 1850 6600 1850
Connection ~ 6600 1850
Text Label 4050 1850 2    50   ~ 0
ROW0
Text Label 4050 2400 2    50   ~ 0
ROW1
Text Label 4050 2950 2    50   ~ 0
ROW2
Text Label 4550 1100 1    50   ~ 0
COL0
Text Label 5000 1100 1    50   ~ 0
COL1
Text Label 5450 1100 1    50   ~ 0
COL2
Text Label 5900 1100 1    50   ~ 0
COL3
Text Label 6350 1100 1    50   ~ 0
COL4
Text Label 6800 1100 1    50   ~ 0
COL5
Text Label 4550 4900 3    50   ~ 0
COL6
Text Label 5000 4900 3    50   ~ 0
COL7
Text Label 5450 4900 3    50   ~ 0
COL8
Text Label 5900 4900 3    50   ~ 0
COL9
Text Label 6350 4900 3    50   ~ 0
COL10
Text Label 6800 4900 3    50   ~ 0
COL11
$Comp
L Device:LED D37
U 1 1 6152B15D
P 3150 3550
F 0 "D37" V 3143 3432 50  0000 R CNN
F 1 "LED" V 3098 3432 50  0001 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3150 3550 50  0001 C CNN
F 3 "~" H 3150 3550 50  0001 C CNN
	1    3150 3550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 6152D409
P 3150 3250
F 0 "R1" H 3220 3296 50  0000 L CNN
F 1 "470" H 3220 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3080 3250 50  0001 C CNN
F 3 "~" H 3150 3250 50  0001 C CNN
	1    3150 3250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR07
U 1 1 6152E45E
P 3150 3100
F 0 "#PWR07" H 3150 2950 50  0001 C CNN
F 1 "+5V" H 3165 3273 50  0000 C CNN
F 2 "" H 3150 3100 50  0001 C CNN
F 3 "" H 3150 3100 50  0001 C CNN
	1    3150 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3700 2600 3700
Text Label 2600 4000 0    50   ~ 0
ROW0
Text Label 2600 4100 0    50   ~ 0
ROW1
Text Label 2600 4200 0    50   ~ 0
ROW2
Text Label 2600 4300 0    50   ~ 0
COL0
Text Label 2600 4400 0    50   ~ 0
COL1
Text Label 2600 4500 0    50   ~ 0
COL2
Text Label 2600 4600 0    50   ~ 0
COL3
Text Label 2600 3000 0    50   ~ 0
COL4
Text Label 2600 3100 0    50   ~ 0
COL5
Text Label 2600 3200 0    50   ~ 0
COL6
Text Label 2600 3300 0    50   ~ 0
COL7
Text Label 2600 3400 0    50   ~ 0
COL8
Text Label 2600 3500 0    50   ~ 0
COL9
Text Label 2600 3600 0    50   ~ 0
COL10
Text Label 2600 3900 0    50   ~ 0
COL11
Wire Wire Line
	5900 3650 5900 4900
Wire Wire Line
	5250 4600 6150 4600
Wire Wire Line
	6800 3650 6800 4200
Wire Wire Line
	6150 4600 6600 4600
$Comp
L Switch:SW_Push_45deg SW35
U 1 1 61596B0E
P 6700 4300
F 0 "SW35" V 6600 4350 50  0000 R CNN
F 1 "SW_Push_45deg" V 6655 4156 50  0001 R CNN
F 2 "keyswitches:Kailh_socket_MX" H 6700 4300 50  0001 C CNN
F 3 "~" H 6700 4300 50  0001 C CNN
	1    6700 4300
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small D35
U 1 1 615972C0
P 6600 4500
F 0 "D35" V 6600 4430 50  0000 R CNN
F 1 "D_Small" V 6555 4430 50  0001 R CNN
F 2 "Diode_SMD:D_SOD-123" V 6600 4500 50  0001 C CNN
F 3 "~" V 6600 4500 50  0001 C CNN
	1    6600 4500
	0    -1   -1   0   
$EndComp
Connection ~ 6600 4600
Wire Wire Line
	6600 4600 7000 4600
Connection ~ 6800 4200
Wire Wire Line
	6800 4200 6800 4900
$EndSCHEMATC
