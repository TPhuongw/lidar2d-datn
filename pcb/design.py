"""
design.py - Nguon du lieu DUY NHAT cho mach mang LiDAR 2D (shield cam len NUCLEO-F411RE qua header Arduino).
Moi linh kien: ref, (thu vien, ky hieu), gia tri, footprint, vi tri tren so do (mm, luoi 2.54), xoay, {so chan: net}.
gen_sch.py va gen_pcb.py deu doc file nay -> so do va PCB luon khop nhau.

Phan chan Nucleo-F411RE (header Arduino, theo UM1724):
  D10=PB6 TIM4_CH1 -> PUL      D7=PA8  -> DIR        D6=PB10 -> ENA
  D4 =PB5 <- ALM (EXTI5)       D3=PB3 <- INDEX (EXTI3)
  D8 =PA9 USART1_TX -> TF03 RX D2=PA10 USART1_RX <- TF03 TX
  A0 =PA0 TIM2_CH1 <- ENC_A    A1=PA1 TIM2_CH2 <- ENC_B (tuy chon)
  D12=PA6 <- nut START (EXTI6) D5=PB4 -> LED1       D15=PB8 -> LED2
  D0/D1 (PA3/PA2) de trong: ST-Link VCP gui du lieu len PC.  D13 (PA5) = LED LD2 tren Nucleo.
"""

FP_R = "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"
FP_C = "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder"
FP_LED = "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder"
FP_XH3 = "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical"
FP_XH4 = "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical"

R = ("Device", "R")
C = ("Device", "C")
CP = ("Device", "C_Polarized")
LED = ("Device", "LED")
DS = ("Device", "D_Schottky")

# (ref, symbol, value, footprint, x, y, rot, {pin: net})   - x,y la toa do tren so do (mm)
PARTS = [
    # ---------------- 1. Nguon vao 24 V + bao ve ----------------
    ("J1", ("Connector", "Screw_Terminal_01x02"), "24V_IN", "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal",
     30.48, 50.8, 0, {"1": "+24V_IN", "2": "GND"}),
    ("F1", ("Device", "Polyfuse"), "1.5A/33V 1812", "Fuse:Fuse_1812_4532Metric_Pad1.30x3.40mm_HandSolder",
     50.8, 40.64, 90, {"1": "+24V_IN", "2": "+24V_F"}),
    ("D1", DS, "SS34 (chong nguoc cuc)", "Diode_SMD:D_SMA_Handsoldering",
     71.12, 40.64, 180, {"2": "+24V_F", "1": "+24V"}),
    ("D2", ("Device", "D_TVS"), "SMBJ28CA", "Diode_SMD:D_SMB_Handsoldering",
     86.36, 55.88, 90, {"1": "+24V", "2": "GND"}),
    ("C1", CP, "470uF/50V", "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm",
     99.06, 55.88, 0, {"1": "+24V", "2": "GND"}),
    ("C2", C, "100nF/50V", FP_C, 111.76, 55.88, 0, {"1": "+24V", "2": "GND"}),

    # ---------------- 2. Buck LM2596S-5.0 (gia tri theo datasheet TI SNVS124) ----------------
    ("U1", ("Regulator_Switching", "LM2596S-5"), "LM2596S-5.0", "Package_TO_SOT_SMD:TO-263-5_TabPin3",
     139.7, 50.8, 0, {"1": "+24V", "2": "SW", "3": "GND", "4": "+5V", "5": "GND"}),
    ("D3", DS, "SS34", "Diode_SMD:D_SMA_Handsoldering",
     160.02, 63.5, 90, {"1": "SW", "2": "GND"}),
    ("L1", ("Device", "L"), "68uH Isat>=2A", "Inductor_SMD:L_12x12mm_H8mm",
     172.72, 43.18, 90, {"1": "SW", "2": "+5V"}),
    ("C3", CP, "220uF/25V", "Capacitor_THT:CP_Radial_D8.0mm_P3.50mm",
     187.96, 55.88, 0, {"1": "+5V", "2": "GND"}),
    ("C4", C, "100nF", FP_C, 198.12, 55.88, 0, {"1": "+5V", "2": "GND"}),
    ("R1", R, "1k", FP_R, 210.82, 50.8, 0, {"1": "+5V", "2": "LED_PWR"}),
    ("D4", LED, "PWR (do)", FP_LED, 210.82, 63.5, 90, {"2": "LED_PWR", "1": "GND"}),
    ("J2", ("Connector_Generic", "Conn_01x02"), "E5V_OUT (toi CN7-6)", "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
     231.14, 53.34, 0, {"1": "+5V", "2": "GND"}),

    # ---------------- 3. Nucleo-F411RE (header Arduino) ----------------
    ("A1", ("MCU_Module", "Arduino_UNO_R3"), "NUCLEO-F411RE", "Module:Arduino_UNO_R3",
     139.7, 129.54, 0,
     {"17": "UART_RX", "18": "INDEX", "19": "ALM", "20": "LED1", "21": "ENA_CTRL", "22": "DIR_CTRL",
      "23": "UART_TX", "25": "PUL_CTRL", "27": "BTN", "9": "ENC_A", "10": "ENC_B", "32": "LED2",
      "4": "+3V3", "6": "GND", "7": "GND", "29": "GND",
      # chan khong dung (no-connect)
      "1": None, "2": None, "3": None, "5": None, "8": None, "11": None, "12": None, "13": None, "14": None,
      "15": None, "16": None, "24": None, "26": None, "28": None, "30": None, "31": None}),

    # ---------------- 4. Giao tiep driver CL42C (chung duong, MOSFET keo xuong) ----------------
    ("Q1", ("Transistor_FET", "AO3400A"), "AO3400A", "Package_TO_SOT_SMD:SOT-23",
     45.72, 111.76, 0, {"1": "PUL_G", "2": "GND", "3": "PUL_N"}),
    ("Q2", ("Transistor_FET", "AO3400A"), "AO3400A", "Package_TO_SOT_SMD:SOT-23",
     45.72, 132.08, 0, {"1": "DIR_G", "2": "GND", "3": "DIR_N"}),
    ("Q3", ("Transistor_FET", "AO3400A"), "AO3400A", "Package_TO_SOT_SMD:SOT-23",
     45.72, 152.4, 0, {"1": "ENA_G", "2": "GND", "3": "ENA_N"}),
    ("R2", R, "100", FP_R, 25.4, 111.76, 90, {"1": "PUL_CTRL", "2": "PUL_G"}),
    ("R3", R, "100", FP_R, 25.4, 132.08, 90, {"1": "DIR_CTRL", "2": "DIR_G"}),
    ("R4", R, "100", FP_R, 25.4, 152.4, 90, {"1": "ENA_CTRL", "2": "ENA_G"}),
    ("R5", R, "100k", FP_R, 35.56, 119.38, 0, {"1": "PUL_G", "2": "GND"}),
    ("R6", R, "100k", FP_R, 35.56, 139.7, 0, {"1": "DIR_G", "2": "GND"}),
    ("R7", R, "100k", FP_R, 35.56, 160.02, 0, {"1": "ENA_G", "2": "GND"}),
    ("J3", ("Connector", "Screw_Terminal_01x08"), "DRIVER CL42C", "TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-8-3.5-H_1x08_P3.50mm_Horizontal",
     76.2, 129.54, 0,
     {"1": "+5V", "2": "PUL_N", "3": "+5V", "4": "DIR_N", "5": "+5V", "6": "ENA_N", "7": "ALM_IN", "8": "GND"}),
    ("R8", R, "10k", FP_R, 96.52, 152.4, 0, {"1": "+3V3", "2": "ALM_IN"}),
    ("R9", R, "1k", FP_R, 96.52, 165.1, 90, {"1": "ALM_IN", "2": "ALM"}),
    ("C5", C, "1nF", FP_C, 106.68, 172.72, 0, {"1": "ALM", "2": "GND"}),

    # ---------------- 5. TF03 (UART LVTTL 3.3V, nguon 5V ~150 mA) ----------------
    ("J4", ("Connector_Generic", "Conn_01x04"), "TF03", FP_XH4,
     195.58, 106.68, 180, {"1": "+5V", "2": "GND", "3": "TF_TX", "4": "TF_RX"}),
    ("R10", R, "100", FP_R, 177.8, 109.22, 90, {"1": "TF_TX", "2": "UART_RX"}),
    ("R11", R, "100", FP_R, 177.8, 99.06, 90, {"1": "UART_TX", "2": "TF_RX"}),
    ("C6", C, "10uF", FP_C, 205.74, 119.38, 0, {"1": "+5V", "2": "GND"}),

    # ---------------- 6. Cam bien khe (moc 0 do) ----------------
    ("J5", ("Connector_Generic", "Conn_01x03"), "SLOT_SENSOR", FP_XH3,
     195.58, 139.7, 180, {"1": "+3V3", "2": "GND", "3": "IDX_IN"}),
    ("R12", R, "1k", FP_R, 177.8, 139.7, 90, {"1": "IDX_IN", "2": "INDEX"}),
    ("R13", R, "10k", FP_R, 185.42, 129.54, 0, {"1": "+3V3", "2": "IDX_IN"}),
    ("C7", C, "1nF", FP_C, 167.64, 147.32, 0, {"1": "INDEX", "2": "GND"}),

    # ---------------- 7. Encoder dong co (tuy chon, DNP) ----------------
    ("J6", ("Connector_Generic", "Conn_01x03"), "ENC_TAP (tuy chon)", FP_XH3,
     195.58, 167.64, 180, {"1": "ENC_A_IN", "2": "ENC_B_IN", "3": "GND"}),
    ("R14", R, "10k", FP_R, 177.8, 162.56, 90, {"1": "ENC_A_IN", "2": "ENC_A"}),
    ("R15", R, "20k", FP_R, 167.64, 167.64, 0, {"1": "ENC_A", "2": "GND"}),
    ("R16", R, "10k", FP_R, 177.8, 175.26, 90, {"1": "ENC_B_IN", "2": "ENC_B"}),
    ("R17", R, "20k", FP_R, 167.64, 187.96, 0, {"1": "ENC_B", "2": "GND"}),

    # ---------------- 8. Giao dien nguoi dung ----------------
    ("R18", R, "1k", FP_R, 99.06, 99.06, 90, {"1": "LED1", "2": "LED1_A"}),
    ("D5", LED, "LED1 (xanh)", FP_LED, 99.06, 88.9, 90, {"2": "LED1_A", "1": "GND"}),
    ("R19", R, "1k", FP_R, 121.92, 99.06, 90, {"1": "LED2", "2": "LED2_A"}),
    ("D6", LED, "LED2 (vang)", FP_LED, 121.92, 88.9, 90, {"2": "LED2_A", "1": "GND"}),
    ("SW1", ("Switch", "SW_Push"), "START", "Button_Switch_THT:SW_PUSH_6mm",
     78.74, 88.9, 90, {"1": "BTN", "2": "GND"}),
    ("R20", R, "10k", FP_R, 66.04, 91.44, 0, {"1": "+3V3", "2": "BTN"}),
    ("C8", C, "100nF", FP_C, 88.9, 99.06, 0, {"1": "BTN", "2": "GND"}),

    # ---------------- 9. Diem do + lo bat vit ----------------
    ("TP1", ("Connector", "TestPoint"), "+5V", "TestPoint:TestPoint_Pad_D1.5mm", 241.3, 78.74, 0, {"1": "+5V"}),
    ("TP2", ("Connector", "TestPoint"), "+3V3", "TestPoint:TestPoint_Pad_D1.5mm", 251.46, 78.74, 0, {"1": "+3V3"}),
    ("TP3", ("Connector", "TestPoint"), "PUL", "TestPoint:TestPoint_Pad_D1.5mm", 261.62, 78.74, 0, {"1": "PUL_CTRL"}),
    ("TP4", ("Connector", "TestPoint"), "INDEX", "TestPoint:TestPoint_Pad_D1.5mm", 271.78, 78.74, 0, {"1": "INDEX"}),
    ("TP5", ("Connector", "TestPoint"), "GND", "TestPoint:TestPoint_Pad_D1.5mm", 281.94, 78.74, 0, {"1": "GND"}),
    ("H1", ("Mechanical", "MountingHole"), "M3", "MountingHole:MountingHole_3.2mm_M3", 241.3, 101.6, 0, {}),
    ("H2", ("Mechanical", "MountingHole"), "M3", "MountingHole:MountingHole_3.2mm_M3", 251.46, 101.6, 0, {}),
    ("H3", ("Mechanical", "MountingHole"), "M3", "MountingHole:MountingHole_3.2mm_M3", 261.62, 101.6, 0, {}),
    ("H4", ("Mechanical", "MountingHole"), "M3", "MountingHole:MountingHole_3.2mm_M3", 271.78, 101.6, 0, {}),
]

# PWR_FLAG cho cac net nhan nguon tu dau noi (de ERC hieu net co nguon)
PWR_FLAGS = [("+24V_IN", 30.48, 30.48), ("GND", 40.64, 66.04), ("+24V", 81.28, 35.56), ("+5V", 190.5, 38.1)]

DNP = {"J6", "R14", "R15", "R16", "R17"}   # khu ma hoa encoder: lap sau neu can
