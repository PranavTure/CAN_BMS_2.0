

// 						MESSAGE ID   :   TYPE OF DATA THEY CARRY

/*
 * 		TRANSMISSIONS
 *
 *		0x001 : Input acceleration structure
 * 		0x111 : First choice message
 * 		0x222 : Battery configurations
 * 		0x333 : Battery Status
 * 		0x444 : mirror Rx data
 *
 * 		0x201 : update temperature
 * 		0x202 : update SOH
 * 		0x203 : update SOC
 *
 * 		0x211 : update degradation_rate cell [0]
 * 		0x212 : u d r cell[1]
 * 		0x213 : udr cell[2]
 *
 *
 *
 *
 * 		RECEPTION
 *
 * 		0x555 : Receive user's choice
 * 		0x666 : Acceleration
 * 		0x777 : config data
 *
 *
 * 		FAULT FLAG BIT MAPINGS
 *
 *     0th bit : LOW SOC < 20 %
 *     1st bit : Over temperature > 60;
 *     2nd bit : under temperature fault < 0
 *     3rd bit : LOW SOH of battery < 70 %
 *     4th bit : Over current fault (pack current > max discharge current)
 *     5-6 th  : Indicate which cell has degraded beyond the limit
 *     7th bit : general fault if any fault flag is set
 */



/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// DECLARATION OF USER CONFIG FUNCTIONS;

typedef struct {
	float battery_capacity_ah;
	float num_cells;
	float cell_capacity_ah[3];
	float cell_power_rating[3];
	float initial_soc[3];
	float initial_soh[3];
	float degradation_rate[3];
	float max_discharge_current;
	float max_charge_current;
} BatteryConfig;

BatteryConfig battery_config;

// DECLARATION OF OUTPUT VARIABLES

typedef struct {
	float pack_soc;
	float pack_soh;
	float cell_soc[3];
	float cell_soh[3];
	float pack_current;
	float pack_temperature;
	float pack_range;
	uint8_t fault_flags;
} BatteryStatus;

BatteryStatus battery_status;

// CONTINUOUS INPUTS VITUAL/USER

typedef struct {
	float acceleration;
} InputData;

InputData inputdata;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t txData[8];
uint8_t rxData[8];
uint8_t temp_rxData[8];
uint8_t init_complete = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void can_filter_config(void);
void default_init(void);
void calculate_outputs(void);
void send_battery_status(BatteryStatus *status);
void send_first_message(void);

void test_pwm(void);

void send_rx_data(void);
void send_struct(uint32_t id, void *data, uint16_t size);

void choice_handler(uint8_t choice);
void custom_input_handler(uint8_t* rxdata);

void update_cell_soh(void);
void update_cell_soc(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  can_filter_config();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  test_pwm();

  send_first_message();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t new_time = 0;
  uint32_t last_time = 0;
  uint32_t config_counter = 0;
  uint32_t config_counter_last = 0;
  while (1)
  {
    /* USER CODE END WHILE */
	  new_time = HAL_GetTick();
	  calculate_outputs();
	  if (new_time - last_time >= 2500) {
		  if(init_complete) send_struct(0x333, &battery_status, sizeof(battery_status));
		  else {
			  config_counter = HAL_GetTick();
			  if(config_counter - config_counter_last >= 10000){
				  send_first_message();
				  config_counter_last = config_counter;
			  }
		  }
		  last_time = new_time;
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

static void can_filter_config() {
    CAN_FilterTypeDef filter;

    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
}


/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;

    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {

    	send_rx_data();
        switch (rxHeader.StdId) {

			case 0x555: {                              		// 0X555   to choose CUSTOM or DEFAULT inits
				choice_handler(rxData[0]);
				break;
			}


            case 0x666: {                                  // 0X666 GIVES THE ACCELERATION VALUE OF THE VEHICLE
            	inputdata.acceleration = rxData[0];
                uint32_t pwm_val = (uint32_t)((inputdata.acceleration/100)*65535);
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_val);
                send_struct(0x001, &inputdata, sizeof(inputdata));
                break;
            }

            case 0x777: {                                  // 0x777 to get custom values for the initialization of the bms.
            	/*
            	 *             	memcpy(temp_rxData, rxData, 8);
            	custom_input_handler(temp_rxData);
            	 */
            	break;
			}

            case 0x201: {                                 // 0x201 : to set the pack temperature at any instance of time.
            	memcpy(&battery_status.pack_temperature, &rxData[0], 4);
            	break;
            }

            case 0x202: {                                 // 0x202 : to set the pack soh at any instance
            	memcpy(&battery_status.pack_soh, &rxData[0], 4);
            	update_cell_soh();
            	break;
            }

            case 0x203: {                                 // 0x203 : send a message with this id to change pack soc
            	memcpy(&battery_status.pack_soc, &rxData[0], 4);
            	update_cell_soc();
            	break;
            }

            case 0x211: {                                 // 0x211 : use this id to edit degradation rate of cell 0
            	memcpy(&battery_config.degradation_rate[0], &rxData[0], 4);
            	break;
            }

            case 0x212: {									// 0x212 : use this id to edit degradation rate of cell 1
            	memcpy(&battery_config.degradation_rate[1], &rxData[0], 4);
            	break;
            }

            case 0x213: {									// 0x213 : use this id to edit degradation rate of cell 2
            	memcpy(&battery_config.degradation_rate[2], &rxData[0], 4);
            	break;
            }
        }
       HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    }
    else {
    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    	HAL_Delay(1000);
    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    }
}


void test_pwm(){
	for(int i=0; i<65535; i+=500) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, i);
		HAL_Delay(8);
	}
	for(int i=65535; i>0; i-=500) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, i);
		HAL_Delay(8);
	}
}

void send_first_message() {				// first message to get the choice selection
	CAN_TxHeaderTypeDef txheader;
	uint8_t txChoiceData[8] = {1, 12, 2, 13, 3, 15, 0, 0};
	uint32_t txMailbox = 0;

	txheader.StdId = 0x111;                      // 0x111 TO send choice to the receiver
	txheader.IDE = CAN_ID_STD;
	txheader.RTR = CAN_RTR_DATA;
	txheader.DLC = 8;

	if (HAL_CAN_AddTxMessage(&hcan1, &txheader, txChoiceData, &txMailbox) != HAL_OK) {
		Error_Handler();
	}
}


void send_rx_data() {

	for(int i=0; i<8; i++) {
		txData[i] = rxData[i];
	}

	CAN_TxHeaderTypeDef txHeader;

	uint32_t txMailbox;


	txHeader.StdId = 0x444;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 8;

	 if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {

		 if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) == HAL_OK) {
			 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		 }

		 else {
		    HAL_GPIO_TogglePin(GPIOD, LED);
		    HAL_Delay(1000);
		    HAL_GPIO_TogglePin(GPIOD, LED);
		 }
	 }
}

uint8_t expect_custom_input = 0;
void choice_handler(uint8_t choice){
	if (choice == 1) {
		default_init();
		expect_custom_input = 1;
	}
	else if (choice == 2) {
		default_init();
		init_complete = 1;
		send_struct(0x222, &battery_config, sizeof(battery_config));
	}
	else if (choice == 3) {
		//send_first_message();
		init_complete = 0;
	}
}


void default_init() {                                         // CODE FOR DEFAULT INITIALIZATION OF THE BMS
	battery_config.battery_capacity_ah = 100.0f;
	battery_config.num_cells = 3;

	battery_config.cell_capacity_ah[0] = 33.3f;
	battery_config.cell_capacity_ah[1] = 33.3f;
	battery_config.cell_capacity_ah[2] = 33.3f;

	battery_config.cell_power_rating[0] = 50.0f;
	battery_config.cell_power_rating[1] = 50.0f;
	battery_config.cell_power_rating[2] = 50.0f;

	battery_config.degradation_rate[0] = 0.001f;
	battery_config.degradation_rate[1] = 0.001f;
	battery_config.degradation_rate[2] = 0.001f;

	battery_config.initial_soc[0] = 100.0f;
	battery_config.initial_soc[1] = 100.0f;
	battery_config.initial_soc[2] = 100.0f;

	battery_config.initial_soh[0] = 100.0f;
	battery_config.initial_soh[1] = 100.0f;
	battery_config.initial_soh[2] = 100.0f;

	battery_config.max_charge_current = 50.0f;
	battery_config.max_discharge_current = 1666.5f;  // 100 - 1 hour @ 100% Acc, 1666.5 - 6 mins @ 60% Acc

	battery_status.pack_soc = 100.0f;
	battery_status.pack_soh = 100.0f;
	battery_status.pack_temperature = 27.0f;
}


void send_struct(uint32_t id, void *data, uint16_t size) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8];
    uint8_t *ptr = (uint8_t*)data;
    uint8_t num_frames = (size + 7) / 8;

    txHeader.StdId = id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;

    for (uint8_t i = 0; i < num_frames; i++) {
        uint8_t len = (i == num_frames - 1) ? (size - i * 8) : 8;
        memcpy(txData, ptr + (i * 8), len);
        if (len < 8) memset(txData + len, 0, 8 - len);

        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
        HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
    }
}


void calculate_outputs() {
    static uint32_t last_degradation_tick = 0;
    static uint32_t last_temp_tick = 0;
    float delta_time_sec = 2.0f; // since we call every 2 sec

    // 1. Current draw based on acceleration
    float current_draw = battery_config.max_discharge_current * (inputdata.acceleration / 100.0f);
    battery_status.pack_current = current_draw;

    // 2. Pack SOC
    float soc_delta = (current_draw * delta_time_sec / 3600.0f) / battery_config.battery_capacity_ah * 100.0f;
    battery_status.pack_soc -= soc_delta;
    if(battery_status.pack_soc < 0) battery_status.pack_soc = 0;
    if(battery_status.pack_soc > 100) battery_status.pack_soc = 100;

    // 2a. Cell SOC
    for(int i = 0; i < battery_config.num_cells; i++) {
        float cell_delta = (current_draw * delta_time_sec / 3600.0f) / battery_config.cell_capacity_ah[i] * 100.0f;
        battery_status.cell_soc[i] -= cell_delta;
        if(battery_status.cell_soc[i] < 0) battery_status.cell_soc[i] = 0;
        if(battery_status.cell_soc[i] > 100) battery_status.cell_soc[i] = 100;
    }

    // 3. Range
    float full_range = 300.0f;
    battery_status.pack_range = full_range * (battery_status.pack_soc / 100.0f);

    // 4. SOH degradation every 15 seconds
    last_degradation_tick += 2000;
    if(last_degradation_tick >= 3000) {                // 15000, 3000
        for(int i = 0; i < battery_config.num_cells; i++) {
            float usage_factor = (100.0f - battery_status.cell_soc[i]) / 100.0f;
            battery_status.cell_soh[i] -= battery_config.degradation_rate[i] * usage_factor;
            if(battery_status.cell_soh[i] < 0) battery_status.cell_soh[i] = 0;
        }

        float soh_sum = 0;
        for(int i = 0; i < battery_config.num_cells; i++)
            soh_sum += battery_status.cell_soh[i];
        battery_status.pack_soh = soh_sum / battery_config.num_cells;

        last_degradation_tick = 0;
    }

    // 5. Temperature rise every 10 sec by 2°C
    last_temp_tick += 2000;
    if(last_temp_tick >= 10000) {
        battery_status.pack_temperature += 2.0f;
        last_temp_tick = 0;
    }

    // 6. Fault flag updates (auto reset if condition clears)
    battery_status.fault_flags = 0; // clear all each update

    // --- 0: Low SOC Fault (< 20%) ---
    if (battery_status.pack_soc < 20.0f)
        battery_status.fault_flags |= (1 << 0);

    // --- 1: Over Temperature Fault (> 60°C) ---
    if (battery_status.pack_temperature > 60.0f)
        battery_status.fault_flags |= (1 << 1);

    // --- 2: Under Temperature Fault (< 0°C) ---
    if (battery_status.pack_temperature < 0.0f)
        battery_status.fault_flags |= (1 << 2);

    // --- 3: Low SOH Fault (< 70%) ---
    if (battery_status.pack_soh < 70.0f)
        battery_status.fault_flags |= (1 << 3);

    // --- 4: Over Current Fault (> max discharge current) ---
    if (fabs(battery_status.pack_current) > battery_config.max_discharge_current)
        battery_status.fault_flags |= (1 << 4);

    //--- 5 & 6 bit in binary indicate how many cells have been degraded
    if (battery_status.cell_soh[0] < 70.0f) {
    	battery_status.fault_flags |= (1<<5);
    }

    if (battery_status.cell_soh[1] < 70.0f) {
    	battery_status.fault_flags |= (1<<6);
    }

    if (battery_status.cell_soh[2] < 70.0f) {
    	battery_status.fault_flags |= ((1<<5)|(1<<6));
    }

    // --- 7: General Fault (if any individual fault present) ---
    if (battery_status.fault_flags)
        battery_status.fault_flags |= (1 << 7);

}


void custom_input_handler(uint8_t* rxdata) {
    static uint8_t frame_count = 0;

    // Each CAN frame contains 2 floats (8 bytes)
    switch(frame_count) {

        case 0: // battery_capacity_ah + num_cells
            memcpy(&battery_config.battery_capacity_ah, &rxdata[0], 4);
            memcpy(&battery_config.num_cells, &rxdata[4], 4);
            frame_count++;
            break;

        case 1: // cell_capacity_ah[0..1]
            memcpy(&battery_config.cell_capacity_ah[0], rxdata, 8);
            frame_count++;
            break;

        case 2: // cell_capacity_ah[2] + cell_power_rating[0]
            memcpy(&battery_config.cell_capacity_ah[2], &rxdata[0], 4);
            memcpy(&battery_config.cell_power_rating[0], &rxdata[4], 4);
            frame_count++;
            break;

        case 3: // cell_power_rating[1..2]
            memcpy(&battery_config.cell_power_rating[1], rxdata, 8);
            frame_count++;
            break;

        case 4: // initial_soc[0..1]
            memcpy(&battery_config.initial_soc[0], rxdata, 8);
            frame_count++;
            break;

        case 5: // initial_soc[2] + initial_soh[0]
            memcpy(&battery_config.initial_soc[2], &rxdata[0], 4);
            memcpy(&battery_config.initial_soh[0], &rxdata[4], 4);
            frame_count++;
            break;

        case 6: // initial_soh[1..2]
            memcpy(&battery_config.initial_soh[1], rxdata, 8);
            frame_count++;
            break;

        case 7: // degradation_rate[0..1]
            memcpy(&battery_config.degradation_rate[0], rxdata, 8);
            frame_count++;
            break;

        case 8: // degradation_rate[2] + max_discharge_current
            memcpy(&battery_config.degradation_rate[2], &rxdata[0], 4);
            memcpy(&battery_config.max_discharge_current, &rxdata[4], 4);
            frame_count++;
            break;

        case 9: // max_charge_current + padding/unused
            memcpy(&battery_config.max_charge_current, &rxdata[0], 4);
            // remaining 4 bytes unused
            frame_count = 0; // Reset for next sequence
            send_struct(0x222, &battery_config, sizeof(battery_config));
            init_complete = 1;
            break;
    }
}


void update_cell_soh() {
	battery_status.cell_soh[0] = battery_status.pack_soh/3;
	battery_status.cell_soh[1] = battery_status.cell_soh[0];
	battery_status.cell_soh[2] = battery_status.cell_soh[0];
}

void update_cell_soc() {
	battery_status.cell_soc[0] = battery_status.pack_soc/3;
	battery_status.cell_soc[1] = battery_status.cell_soc[0];
	battery_status.cell_soc[2] = battery_status.cell_soc[0];
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) {
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14);
	  HAL_Delay(250);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
