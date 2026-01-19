Sample code demonstrating automatic continuous conversions for U545 ADC1 with GPDMA using "Standard Request Mode". 
ADC1 : VREFINT, IN1, IN2, Temperature Sensor
ADC2 : IN3, IN4
ADC calibration done at start-up

TESTED USING NUCLEO-U545RE-Q

Connected the Test Signals between the ADCx_INy Pins and the GND Pins of the Nucleo Board.
Nucleo-64 Pin out
CN7 Top View -------------------------
..
19 GND				20 GND
..					..
35 PC2  ADC4_IN3	36 PC1 ADC1_IN2
37 PC3  ADC4_IN4	38 PC0 ADC1_IN1

CN10 Top View -------------------------
..
					10 GND
..
					20 GND
..
                    32 AGND
..					..
					38



2^14 = 16384 = NUM_CODES_ADC1
2^12 = 4096 = NUM_CODES_ADC2

 VREF+ = 3.0 V × VREFINT_CAL ⁄ VREFINT_DATA
 VCHANNELx = ((VREF+)/NUM_CODES_ADC) × ADC_DATA

TCONV per channel = TSMPL + TSAR = [814_min + 17_for_14bits] × Tadc
TCONV per channel = 831 × (ADC_Clk_PreScaler/Fadc) = 22*(1/4Mhz) = 207.75us
For ADC1 with 4 channels =  207.75us x 4 = 831us ~ 1ms assuming delays for the DMA buffer complete call-backs.
Now accumulating 100 samples = 1ms x 100 = 100ms. So every 100ms we hand-over a new accumulated count.
Hence the earlier accumulated count should be processed before this new accumulated count arrives to ensure
that we do not miss some readings. I plan to move this checking / processing to a 10ms timer loop in production code.