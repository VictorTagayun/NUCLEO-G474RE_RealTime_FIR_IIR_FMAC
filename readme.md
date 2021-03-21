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
	
## Step 1 Generate 2 freq of sinewaves  

From previous project [DAC by DMA](https://github.com/VictorTagayun/NUCLEO-G474RE_DAC_DMA_LL-HAL_TIM6), it is possible to generate sinewaves of two freqs.

### GPIO  

* GPIOC6/8/11 is used for troubleshooting  

### TIM6 as 2MHz to trigger DAC3  

* Activate TIM6
* Prescaler = 17-1  
* ARR = 5-1
* TRGO Event = Update event
* setup in main.c 

	/*##- Enable TIM peripheral counter ######################################*/
	if(HAL_OK != HAL_TIM_Base_Start(&htim6))
	{
		Error_Handler();
	}

### DAC3 for 2 Freq generator using DMA  

* Set DAC High Freq. = 160MHz 
* Trigger TIM6 Out Event
* DMA Settings
	* Mode = Circular
	* Increment Adress = Memory
	* Data width = Word
* Do not Disable DMA IT!
* add #include "waveforms.h"
* add High freq to become 2 freq in main.c 

	for (uint16_t cntr = 0; cntr < MySine2000_SIZE; cntr++)
	{
		MySine2000[cntr] += 682;
		MySine2000[cntr] += MySine200[cntr % MySine200_SIZE];
	}
	
* setup DAC3 in main.c  

	/*##- Enable DAC Channel and associated DMA ##############################*/
	if(HAL_OK != HAL_DAC_Start_DMA(&hdac3, DAC_CHANNEL_1,
				   (uint32_t*)MySine2000, MySine2000_SIZE, DAC_ALIGN_12B_R))
	{
		/* Start DMA Error */
		Error_Handler();
	}

### OpAmp6  

* Mode = Follower DAC3 output1, input P
* Power Mode = High Speed
* Setup OpAmp6 in main.c  

	/*##- Start OPAMP    #####################################################*/
	/* Enable OPAMP */
	if(HAL_OK != HAL_OPAMP_Start(&hopamp6))
	{
		Error_Handler();
	}

### check output on PB11 

## Step 2 use HRTIM Master to trigger ADC (and use DAC output to display ADC data for testing if triggered) 

### Master HRTIM  

* Setup Master HRTIM 
* ADC trigger1 on Master Period  
* Setup HRTIM Master in main.c 

	if(HAL_OK != HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER))
	{
		Error_Handler();
	}
	
### ADC DMA to DAC4 to display ADC data  

* Setup 1 Regular Conversion mode   
	* External trigger by HRTIM trig 1 event (Master Period)  
* Add DMA
	* Mode Circular @ Memory, data width Word
* ADC_Settings
	* DMA Continous Request = Enable
	* Overrun behaviour = overwritten
* NVIC
	* Enable DMA global IT with Call handler
	* Disable ADC1-2 Global IT
* Add callback for Regular Conversion mode, later will be used for DAC1 output

	HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
	{
	  /* Prevent unused argument(s) compilation warning */
	  UNUSED(hadc);

	  /* NOTE : This function should not be modified. When the callback is needed,
				function HAL_ADC_ConvCpltCallback must be implemented in the user file.
	   */

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
		adc_data = HAL_ADC_GetValue(hadc);
		HAL_DAC_SetValue(&hdac4, DAC_CHANNEL_1, DAC_ALIGN_12B_R, adc_data);
	}

* Calibrate then enable ADC with DMA in main.c 

	/* Perform an ADC automatic self-calibration and enable ADC */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	
	/*##- Enable ADC Channel and associated DMA ##############################*/
	if(HAL_OK != HAL_ADC_Start_DMA(&hadc1, &adc_data, 1))
	{
		/* Start DMA Error */
		Error_Handler();
	}

### DAC4 for ADC data display   

* Enable DAC4  
* Set DAC High Freq. = 160MHz 
* Do not set any Trigger, DAC will not output with this command _HAL_DAC_SetValue_
* Enable DAC4 in main.c 

	/*##- Enable DAC Channel ##############################*/
	if(HAL_OK != HAL_DAC_Start(&hdac4, DAC_CHANNEL_1))
	{
		/* Start Error */
		Error_Handler();
	}
	
### OpAmp4  

* Mode = Follower DAC3 output1, input P
* Power Mode = High Speed
* Setup OpAmp4 in main.c  

	/*##- Start OPAMP    #####################################################*/
	/* Enable OPAMP */
	if(HAL_OK != HAL_OPAMP_Start(&hopamp4))
	{
		Error_Handler();
	}
	
### check output on PB11 

## Step 3 Feed ADC data to FMAC for FIR

Insert FMAC in between ADC and DAC output so we can apply FIR filter

### FMAC for FIR filter     

* Enable FMAC 
* Enable IT
* include fmac.h
* add VT_FMAC_init(void) 

	/*## Configure the FMAC peripheral ###########################################*/
	sFmacConfig.InputBaseAddress  = INPUT_BUFFER_BASE; 	// COEFF_VECTOR_B_SIZE = COEFFICIENT_BUFFER_SIZE = 5
	sFmacConfig.InputBufferSize   = INPUT_BUFFER_SIZE; 	// COEFF_VECTOR_B_SIZE (5) + MEMORY_PARAMETER_D1 (1)
	sFmacConfig.InputThreshold    = INPUT_THRESHOLD;  	// FMAC_THRESHOLD_1 = 0x00000000U
	sFmacConfig.CoeffBaseAddress  = COEFFICIENT_BUFFER_BASE; // = 0
	sFmacConfig.CoeffBufferSize   = COEFFICIENT_BUFFER_SIZE; // = 5
	sFmacConfig.OutputBaseAddress = OUTPUT_BUFFER_BASE;	// COEFFICIENT_BUFFER_SIZE + INPUT_BUFFER_SIZE
	sFmacConfig.OutputBufferSize  = OUTPUT_BUFFER_SIZE;	// MEMORY_PARAMETER_D2 = 2
	sFmacConfig.OutputThreshold   = OUTPUT_THRESHOLD; 	//
	sFmacConfig.pCoeffA           = NULL;					// no A coeffs
	sFmacConfig.CoeffASize        = 0;					// no A coeffs
	sFmacConfig.pCoeffB           = aFilterCoeffB;		//
	sFmacConfig.CoeffBSize        = COEFF_VECTOR_B_SIZE;	// 5
	sFmacConfig.Filter            = FMAC_FUNC_CONVO_FIR;  //
	sFmacConfig.InputAccess       = FMAC_BUFFER_ACCESS_NONE; /*!< Buffer handled by an external IP (ADC for instance) */
	sFmacConfig.OutputAccess      = FMAC_BUFFER_ACCESS_IT;// /*!< Buffer accessed through interruptions */
	sFmacConfig.Clip              = FMAC_CLIP_ENABLED;	//
	sFmacConfig.P                 = COEFF_VECTOR_B_SIZE;	// 5
	sFmacConfig.Q                 = FILTER_PARAM_Q_NOT_USED; // 0
	sFmacConfig.R                 = GAIN;					//

	if (HAL_FMAC_FilterConfig(&hfmac, &sFmacConfig) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}

	/*## Preload the input and output buffers ##################################*/
	if (HAL_FMAC_FilterPreload(&hfmac, NULL, INPUT_BUFFER_SIZE, NULL, 0) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}

	/*## Start calculation of FIR filter in polling/IT mode ####################*/
	if (HAL_FMAC_FilterStart(&hfmac,&Fmac_output,&ExpectedCalculatedOutputSize) != HAL_OK)
	{
		/* Processing Error */
		Error_Handler();
	}
	  
### DAC1 for FMAC data display   

* Enable DAC1  
* Enable Output Buffer
* Set DAC High Freq. = 160MHz 
* Do not set any Trigger, DAC will not output with this command _HAL_DAC_SetValue_
* Enable DAC1 in main.c 

	/*##- Enable DAC Channel ##############################*/
	if(HAL_OK != HAL_DAC_Start(&hdac1, DAC_CHANNEL_1))
	{
		/* Start Error */
		Error_Handler();
	}
	
FMAC init process flow

HAL_FMAC_FilterConfig, wait until finished
HAL_FMAC_FilterPreload, wait until finished
HAL_FMAC_GetState, check the current state of the peripheral
HAL_FMAC_FilterStart, Output data is also passed here
and
HAL_FMAC_AppendFilterData, 
	if from ADC by DMA not needed
	need to call by callback HAL_FMAC_OutputDataReadyCallback
		and get data HAL_FMAC_ConfigFilterOutputBuffer to get data
HAL_FMAC_PollFilterData, before add/append new data
HAL_FMAC_AppendFilterData

Call backs when already running,if from ADC by DMA not needed

HAL_FMAC_GetDataCallback >> HAL_FMAC_AppendFilterData
HAL_FMAC_OutputDataReadyCallback >> HAL_FMAC_ConfigFilterOutputBuffer