

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
	int battery_capacity_ah;
	int num_cells;
	int cell_capacity_ah[3];
	int cell_power_rating[3];
	int initial_soc[3];
	int initial_soh[3];
	int degradation_rate[3];
	int max_discharge_current;
	int max_charge_current;
} BatteryConfig;

BatteryConfig battery_config;

// DECLARATION OF OUTPUT VARIABLES

typedef struct {
	int pack_soc;
	int pack_soh;
	int cell_soc[3];
	int cell_soh[3];
	int pack_current;
	int pack_temperature;
	int pack_range;
	int fault_flags;
} BatteryStatus;

BatteryStatus battery_status;

// CONTINUOUS INPUTS VIRTUAL/USER

typedef struct {
	int acceleration;
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

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {

        send_rx_data();

        switch (rxHeader.StdId) {

            case 0x555: {                                 // 0x555 : choose CUSTOM or DEFAULT init
                choice_handler(rxData[0]);
                break;
            }

            case 0x666: {                                 // 0x666 : vehicle acceleration input
                inputdata.acceleration = (int)rxData[0];
                uint32_t pwm_val = (uint32_t)((inputdata.acceleration / 100.0f) * 65535);
                rxData[0] = (uint8_t)inputdata.acceleration;
                send_rx_data();
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_val);
                send_struct(0x001, &inputdata, sizeof(inputdata));
                break;
            }

            case 0x777: {                                 // 0x777 : custom initialization inputs
                // memcpy(temp_rxData, rxData, 8);
                // custom_input_handler(temp_rxData);
                break;
            }

            case 0x201: {                                 // 0x201 : set pack temperature
                memcpy(&battery_status.pack_temperature, &rxData[0], sizeof(int));
                break;
            }

            case 0x202: {                                 // 0x202 : set pack SOH
                memcpy(&battery_status.pack_soh, &rxData[0], sizeof(int));
                update_cell_soh();
                break;
            }

            case 0x203: {                                 // 0x203 : set pack SOC
                memcpy(&battery_status.pack_soc, &rxData[0], sizeof(int));
                update_cell_soc();
                break;
            }

            case 0x211: {                                 // 0x211 : set degradation rate for cell 0
                memcpy(&battery_config.degradation_rate[0], &rxData[0], sizeof(int));
                break;
            }

            case 0x212: {                                 // 0x212 : set degradation rate for cell 1
                memcpy(&battery_config.degradation_rate[1], &rxData[0], sizeof(int));
                break;
            }

            case 0x213: {                                 // 0x213 : set degradation rate for cell 2
                memcpy(&battery_config.degradation_rate[2], &rxData[0], sizeof(int));
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
		send_first_message();
		init_complete = 0;
	}
}


void default_init() {  // CODE FOR DEFAULT INITIALIZATION OF THE BMS
    // CONFIGURATION STRUCT INIT
    battery_config.battery_capacity_ah = 100;
    battery_config.num_cells = 3;

    battery_config.cell_capacity_ah[0] = 33;
    battery_config.cell_capacity_ah[1] = 33;
    battery_config.cell_capacity_ah[2] = 33;

    battery_config.cell_power_rating[0] = 50;
    battery_config.cell_power_rating[1] = 50;
    battery_config.cell_power_rating[2] = 50;

    battery_config.degradation_rate[0] = 1;   // scaled from 0.001f (use 1 = 0.001%)
    battery_config.degradation_rate[1] = 1;
    battery_config.degradation_rate[2] = 1;

    battery_config.initial_soc[0] = 100;
    battery_config.initial_soc[1] = 100;
    battery_config.initial_soc[2] = 100;

    battery_config.initial_soh[0] = 100;
    battery_config.initial_soh[1] = 100;
    battery_config.initial_soh[2] = 100;

    battery_config.max_charge_current = 1000;
    battery_config.max_discharge_current = 1000; // 100 - 1 hour @ 100% Acc, 1666 - 6 mins @ 60% Acc

    // STATUS STRUCT INIT
    battery_status.pack_soc = 100;
    battery_status.pack_soh = 100;

    battery_status.cell_soc[0] = battery_config.initial_soc[0];
    battery_status.cell_soc[1] = battery_config.initial_soc[1];
    battery_status.cell_soc[2] = battery_config.initial_soc[2];

    battery_status.cell_soh[0] = battery_config.initial_soh[0];
    battery_status.cell_soh[1] = battery_config.initial_soh[1];
    battery_status.cell_soh[2] = battery_config.initial_soh[2];

    battery_status.pack_current = 0;
    battery_status.pack_temperature = 27;
    battery_status.pack_range = 300 * (battery_status.pack_soc / 100);
    battery_status.fault_flags = 0;
}


void calculate_outputs(void) {
    int full_range = 300;
    static uint32_t last_temp_tick = 0;
    static uint32_t last_soc_tick  = 0;
    static uint32_t last_soh_tick  = 0;

    uint32_t now = HAL_GetTick(); // current system tick in ms

    // 1. Current draw based on acceleration (% of max current)
    battery_status.pack_current =
        (battery_config.max_discharge_current * inputdata.acceleration) / 100;

    // 2. Temperature rise: +1°C every 2 sec
    if (now - last_temp_tick >= 2000) {
        battery_status.pack_temperature += 1;
        last_temp_tick = now;
    }

    // 3. SOC drop: -1% every 3 sec
    if (now - last_soc_tick >= 3000) {
        battery_status.pack_soc -= 1;
        if (battery_status.pack_soc < 0) battery_status.pack_soc = 0;
        update_cell_soc();
        last_soc_tick = now;
    }

    // 4. SOH drop: -1% every 5 sec
    if (now - last_soh_tick >= 5000) {
        battery_status.pack_soh -= 1;
        if (battery_status.pack_soh < 0) battery_status.pack_soh = 0;
        update_cell_soh();
        last_soh_tick = now;
    }

    // 5. Range update (based on SOC)
    battery_status.pack_range = (full_range * battery_status.pack_soc) / 100;

    // 6. Fault flag updates
    battery_status.fault_flags = 0;

    if (battery_status.pack_temperature > 60)
            battery_status.fault_flags |= (1 << 0); // Overtemperature
    if (battery_status.pack_soc <= 5)
            battery_status.fault_flags |= (1 << 1); // Low SOC
    if (battery_status.pack_soh <= 50)
            battery_status.fault_flags |= (1 << 2); // Low SOH
    if (battery_status.pack_current > battery_config.max_discharge_current)
            battery_status.fault_flags |= (1 << 3); // Overcurrent
    if (battery_status.pack_temperature > 60)
    		battery_status.fault_flags |= (1<<4);

    if (battery_status.fault_flags)
    		battery_status.fault_flags |= (1<<7);
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


void custom_input_handler(uint8_t* rxdata) {
    static uint8_t frame_count = 0;

    // Each CAN frame now contains 2 integers (8 bytes)
    switch(frame_count) {

        case 0: // battery_capacity_ah + num_cells
            memcpy(&battery_config.battery_capacity_ah, &rxdata[0], 4);
            memcpy(&battery_config.num_cells, &rxdata[4], 4);
            frame_count++;
            break;

        case 1: // cell_capacity_ah[0..1]
            memcpy(&battery_config.cell_capacity_ah[0], &rxdata[0], 4);
            memcpy(&battery_config.cell_capacity_ah[1], &rxdata[4], 4);
            frame_count++;
            break;

        case 2: // cell_capacity_ah[2] + cell_power_rating[0]
            memcpy(&battery_config.cell_capacity_ah[2], &rxdata[0], 4);
            memcpy(&battery_config.cell_power_rating[0], &rxdata[4], 4);
            frame_count++;
            break;

        case 3: // cell_power_rating[1..2]
            memcpy(&battery_config.cell_power_rating[1], &rxdata[0], 4);
            memcpy(&battery_config.cell_power_rating[2], &rxdata[4], 4);
            frame_count++;
            break;

        case 4: // initial_soc[0..1]
            memcpy(&battery_config.initial_soc[0], &rxdata[0], 4);
            memcpy(&battery_config.initial_soc[1], &rxdata[4], 4);
            frame_count++;
            break;

        case 5: // initial_soc[2] + initial_soh[0]
            memcpy(&battery_config.initial_soc[2], &rxdata[0], 4);
            memcpy(&battery_config.initial_soh[0], &rxdata[4], 4);
            frame_count++;
            break;

        case 6: // initial_soh[1..2]
            memcpy(&battery_config.initial_soh[1], &rxdata[0], 4);
            memcpy(&battery_config.initial_soh[2], &rxdata[4], 4);
            frame_count++;
            break;

        case 7: // degradation_rate[0..1]
            memcpy(&battery_config.degradation_rate[0], &rxdata[0], 4);
            memcpy(&battery_config.degradation_rate[1], &rxdata[4], 4);
            frame_count++;
            break;

        case 8: // degradation_rate[2] + max_discharge_current
            memcpy(&battery_config.degradation_rate[2], &rxdata[0], 4);
            memcpy(&battery_config.max_discharge_current, &rxdata[4], 4);
            frame_count++;
            break;

        case 9: // max_charge_current + padding/unused
            memcpy(&battery_config.max_charge_current, &rxdata[0], 4);
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
