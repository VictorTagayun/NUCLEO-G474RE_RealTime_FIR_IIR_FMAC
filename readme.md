# FIR processing of ADC data in realtime

## Target Block diagram  

## Step process  

	- Generate 2 freq of sinewaves, one low freq. like 1kHz and high freq like 10-20kHz. These signals will be fed to FIR filter and will be applied a LPF (low pass filter) and HPF (high pass filter)
	- Trigger the ADC and display the ADC data by DAC
	- Trigger the ADC and feed to FIR filter (by FMAC) and apply either LPF and HPF and send the data out to DAC

## Project files  

	- 
	- 
	- 
	
## Step 1  

From previous project [DAC by DMA](https://github.com/VictorTagayun/NUCLEO-G474RE_DAC_DMA_LL-HAL_TIM6), it is possible to generate sinewaves of two freqs.



## Step 2  

Activate ADC and generate DAC output from ADC data

## Step 3  

Insert FMAC in between ADC and DAC output so we can apply FIR filter

