/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	const float Mass;
	const float Damping;
	const float SpringConstant;
	const float EncoderGain;
	const uint16_t EncoderOffset;
	volatile float SpringForce[65];

} MechanicalPlantTypeDef;

typedef struct {
	volatile uint32_t Counter;
	volatile float Position;
	volatile float PositionHistory[24];
	volatile float Velocity;
	volatile float springForce;
} MechanicalMeasurementTypeDef;

typedef struct {
	volatile uint8_t Active;
	volatile uint8_t ControlMode;
	volatile float DesiredValue;
	volatile float Error[3];
	volatile float Saturation;
	volatile float ControlEffort[3];
	volatile float ControlEffortZP[2];
	volatile float DesiredValueZP[3];
	volatile float DesiredValueZP_P1;
	volatile float FFControlEffort;
	volatile uint8_t Updated;
	volatile float SoftTouchThreshold;
	volatile uint8_t SoftTouch;
	volatile float posSetX;
	volatile float posSetY;
	volatile float startPos;
	volatile uint32_t velCounter;
} MechanicalControllerTypeDef;

////////////////////////////////////////////////////////////////////////////////
/* CC: Current Control -------------------------------------------------------*/
/* LEM is soldered to use three primary turns resulting in 0.625 V / 2 A -----*/
////////////////////////////////////////////////////////////////////////////////
typedef struct {
	const float Resistance;
	const float Inductance;
	const float LEMGain;
	volatile float LEMOffset;
} ElectricalPlantTypeDef;

typedef struct {
	volatile uint32_t Counter;
	volatile float Current;
	volatile float CurrentHistory[24];
} ElectricalMeasurementTypeDef;

typedef struct {
//	const float IndSaturation;

	volatile uint8_t ControlMode;
	volatile float Saturation;
	volatile float DesiredValue;
	volatile float Error[2];
	volatile float ControlEffort[2];
	volatile float ExtForceSaturation;
} ElectricalControllerTypeDef;

typedef struct {
	volatile uint8_t ControllerActive;
	MechanicalPlantTypeDef MP;
	MechanicalMeasurementTypeDef MM;
	MechanicalMeasurementTypeDef MMLog;
	MechanicalControllerTypeDef MC;
	MechanicalControllerTypeDef MCLog;
	ElectricalPlantTypeDef EP;
	ElectricalMeasurementTypeDef EM;
	ElectricalMeasurementTypeDef EMLog;
	ElectricalControllerTypeDef EC;
	ElectricalControllerTypeDef ECLog;

} ActuatorTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IDLE_COMMAND						0
#define	HOME_COMMAND						1
#define	START_COMMAND						2
#define STOP_COMMAND						3
#define MEASUREMENT_COMMAND					4
#define CURRENT_OFFSET_COMMAND				5
#define SPRING_CHARACTERIZATION_COMMAND		6
#define IDENTIFICATION_COMMAND				7
#define FORCE_CONSTANT_CHARACTERIZATION		8

#define LINEAR_VOLTAGE_STEP_COMMAND			9
#define LINEAR_CURRENT_STEP_COMMAND			10
#define LINEAR_POSITION_STEP_COMMAND		     11
#define LINEAR_POSITION_RAMP_COMMAND			 12

#define ROTARY_VOLTAGE_STEP_COMMAND			13
#define ROTARY_CURRENT_STEP_COMMAND			14
#define ROTARY_POSITION_STEP_COMMAND		     15
#define ROTARY_RAMP_COMMAND					16


#define STAIR_COMMAND						17
#define CIRCLE_COMMAND						18
#define SQUARE_COMMAND					    19
#define TRIANGLE_COMMAND				    20
#define SINESTEP_COMMAND				    21
#define PRINT_COMMAND						22
////////////////////////////////////////////////////////////////////////////////
/* Control Mode --------------------------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////
#define VOLTAGE_CONTROL						0
#define	CURRENT_CONTROL						1
#define	FORCE_CONTROL						2
#define ACCELERATION_CONTROL				3
#define VELOCITY_CONTROL					4
#define POSITION_CONTROL					5
#define MEASUREMENT_ONLY					6
#define DONT_CHANGE_IT						7

#define UART_RX_BUFFER_SIZE 				256
#define UART_TX_BUFFER_SIZE 				2048

#define LOG_IDLE 							0
#define LOG_NEW_DATA_READY					1
#define LOG_OVERRUN							2
#define Frequency							24000
#define PWMARR								1500.0f
#define PPR									1000.0f
#define BusVoltage							24.0f
#define V								0
#define U								1
#define Rotary_Encoder_Offset				32768.0f

#define RAMP_STROKE							20.000f // mm
#define Linear_Ramp_Vel						5.000f // mm/s
#define Linear_Ramp_Duration				RAMP_STROKE/Linear_Ramp_Vel	 //
#define LINEAR_RAMP_CYCLES					1

#define Rotary_Ramp_Vel						150.000f // deg/s
#define Rotary_Ramp_Duration				180.000f/Rotary_Ramp_Vel	 //
#define KNOB_TEST_CYCLES					1
#define Transformation                      -0.70710678118f //cos(M_PI_4)
#define DesiredTransformation                0.70710678118f //cos(M_PI_4)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* UART ----------------------------------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////
__attribute__ ((section(".buffers"), used))
char UARTTXBuffer[UART_TX_BUFFER_SIZE];
uint8_t UARTTXLen;
uint8_t UARTRXBuffer[UART_RX_BUFFER_SIZE];

volatile uint8_t ActiveCommand;
volatile uint8_t HalfTrigger;
volatile uint8_t ControlLog;
volatile uint16_t SerialOverrun;
volatile uint16_t LogOverrun;
volatile uint8_t DataLogStatus;
volatile float TEMP;

volatile uint32_t LogCounter = 0;
volatile uint32_t PrintNumbers ;


float CURRENTLOG_V[2000];
float CURRENTLOG_U[2000];
float PosLog_X[2000];
float PosLog_Y[2000];
float PosLog_U[2000];
float PosLog_V[2000];

//uint32_t LogC[1000];
//float EC_ConEffX_LOG[2000] ;
//float EC_ConEffY_LOG[2000] ;

//float MC_ConEffX_LOG[1000];
//float MC_ConEffY_LOG[1000];
//float EC_ConEffX_LOG[1000];
//float EC_ConEffY_LOG[1000];
//float MC_ErrX_LOG[1000];
//float MC_ErrY_LOG[1000];
//float EC_ErrX_LOG[1000];
//float EC_ErrY_LOG[1000];
//volatile float x_cos[2000];
//volatile float y_sin[2000];


volatile uint32_t PrintNumbers = 2000; ;
const float R_circle =  1.0f;                 //radius od circle
const float freq_circle = 2.0f *M_PI*2.0f;   //frequency


const float amplitude_square = 5.0f;  		 // half side length
const float freq_square   = 0.5f;     		 //  Hz

const float amplitude_triangle = 5.0f;  // distance from center to vertex in Y
const float freq_triangle  = 0.5f; // convert F (rad/s) to Hz


volatile uint32_t PrintCounter = 0;
volatile uint32_t STPOS = 0;
volatile float PosU;
volatile float PosV;
volatile uint32_t timeS = 0;
volatile uint32_t timeE = 0;
volatile uint32_t timeCode = 0;
// V Actuator
ActuatorTypeDef YActuator = {

.MP = { .Mass = 0.387, .Damping = 12, .SpringConstant = 272.4,
		.SpringForce = { 0.9131, 0.9650, 0.9701, 0.9748, 1.0031, 1.0222, 1.0343,
				1.0447, 1.0545, 1.0877, 1.1104, 1.1264, 1.1361, 1.1590, 1.1793,
				1.2015, 1.2047, 1.2270, 1.2300, 1.2724, 1.2959, 1.2889, 1.3176,
				1.3214, 1.3534, 1.3804, 1.3771, 1.3929, 1.4204, 1.4529, 1.4629,
				1.4766, 1.4763, 1.5222, 1.5490, 1.5514, 1.5599, 1.5828, 1.6353,
				1.6720, 1.6408, 1.6679, 1.6981, 1.7413, 1.8062, 1.7907, 1.8090,
				1.7915, 1.8280, 1.8629, 1.9222, 1.9268, 1.9679, 2.0286 },
		.EncoderGain = 2.0f / 1024.0f, .EncoderOffset = 28385 }, .EP = {
		.Resistance = 7.3, .Inductance = 0.0075, .LEMGain = 8.0f }, .EC = {
		.ExtForceSaturation = 3.0f } };

ActuatorTypeDef XActuator = {

.MP = { .Mass = 0.387, .Damping = 12, .SpringConstant = 272.4, .EncoderGain =
		2.0f / 1024.0f, .EncoderOffset = 32776 }, .EP = { .Resistance = 7.3,
		.Inductance = 0.0075, .LEMGain = 8.0f } };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

volatile float ForceConstant(float Position);
volatile float BridgeDriver(uint8_t Axis, float ControlEffort);
void StartActuators();
void SetControlParameters(uint8_t Active, uint8_t MControlMode,
		float MDesiredValueX, float MDesiredValueY, uint8_t EControlMode,
		float EDesiredValueU, float EDesiredValueV, uint8_t Log);
volatile uint8_t ProcessSerialCommand(uint8_t *SerialCommand);
void StopActuators();
void MMeasurement();
void EMeasurement();
void LogData();
void ReInitMechanicalController();

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
  MX_DMA_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, UARTRXBuffer, UART_RX_BUFFER_SIZE);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

////  HOME
//	__HAL_TIM_SET_COUNTER(&htim4, XActuator.MP.EncoderOffset);
//	__HAL_TIM_SET_COUNTER(&htim3, YActuator.MP.EncoderOffset);

	HAL_OPAMP_SelfCalibrate(&hopamp2);
	HAL_OPAMP_Start(&hopamp2);

	HAL_OPAMP_SelfCalibrate(&hopamp3);
	HAL_OPAMP_Start(&hopamp3);

	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

//		XActuator.MM.Position = ((float) XActuator.MP.EncoderOffset
//				- __HAL_TIM_GET_COUNTER(&htim4)) * XActuator.MP.EncoderGain + 8;
//
//		YActuator.MM.Position = -((float) YActuator.MP.EncoderOffset
//				- __HAL_TIM_GET_COUNTER(&htim3)) * YActuator.MP.EncoderGain;
//
////		PosU = XActuator.MM.Position * -1.0f * Transformation
////				+ YActuator.MM.Position * Transformation;
////
////		PosV = XActuator.MM.Position * Transformation
////				+ YActuator.MM.Position * Transformation;
//
//
//		PosU = XActuator.MM.Position * (float) DesiredTransformation
//				+ YActuator.MM.Position * (float) DesiredTransformation;
//
//		PosV = XActuator.MM.Position * -1.0f * (float) DesiredTransformation
//				+ YActuator.MM.Position * (float) DesiredTransformation;
//
//
//
//		YActuator.MC.Error[0] = 10.0f * DesiredTransformation - PosV;
//		XActuator.MC.Error[0] = 10.0f * Transformation - PosU;
//
//		YActuator.EM.Current = -((float) HAL_ADC_GetValue(&hadc1)
//				- YActuator.EP.LEMOffset) / 4096.0f * 3.3f
//				* YActuator.EP.LEMGain;
//
//		XActuator.EM.Current = (HAL_ADC_GetValue(&hadc2)
//				- XActuator.EP.LEMOffset) / 4096.0f * 3.3
//				* XActuator.EP.LEMGain;

		switch (ActiveCommand) {

		case HOME_COMMAND:

			if (LogCounter == 0) {
				HAL_UART_DMAStop(&huart2);
				UARTTXLen = sprintf((char*) UARTTXBuffer, "Home Command\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);

				StartActuators();
				//HAL_Delay(500);
				__HAL_TIM_SET_COUNTER(&htim3, YActuator.MP.EncoderOffset);
				__HAL_TIM_SET_COUNTER(&htim4, XActuator.MP.EncoderOffset);

				STPOS = 1;

			} else if (LogCounter <= 0.5f * 24000.f) {
				// 8.0f is home target
				float position_now  = ( 8.0f / (0.5f * 24000.f) ) * (LogCounter);
				SetControlParameters(1, POSITION_CONTROL,
						8.0f-position_now, 0,
									DONT_CHANGE_IT, 0, 0, 0);


			} else if (LogCounter > 1.0f * 24000.f) {

				SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
				MEASUREMENT_ONLY, 0, 0, 0);
//				HAL_Delay(100);
				StopActuators();
				ActiveCommand = IDLE_COMMAND;
				STPOS = 0;
				UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);

			}

			break;

		case PRINT_COMMAND:

			HAL_UART_DMAStop(&huart2);
			for (int ii = 0; ii < PrintNumbers; ii++) {
				UARTTXLen = snprintf((char*) UARTTXBuffer,
				UART_TX_BUFFER_SIZE - 1, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",

				CURRENTLOG_U[ii], CURRENTLOG_V[ii],
				PosLog_U[ii],PosLog_V[ii],
						PosLog_X[ii], PosLog_Y[ii]);
//						,LogC[ii]);
//				XActuator.MC.DesiredValueX,YActuator.MC.DesiredValueX,
//				XActuator.MC.DesiredValueY,YActuator.MC.DesiredValueY,
//				MC_ErrX_LOG[ii],MC_ErrY_LOG[ii]);
//				EC_ErrX_LOG[ii],EC_ErrY_LOG[ii]);
//				EC_ConEffX_LOG[ii], EC_ConEffY_LOG[ii]);



				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen,
						100);
			}
			ActiveCommand = IDLE_COMMAND;

			break;

		case CURRENT_OFFSET_COMMAND:
			HAL_UART_DMAStop(&huart2);
			UARTTXLen = sprintf((char*) UARTTXBuffer,
					"Current Offset Command\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
			StopActuators();

			float LEMTempOffsetX = 0;
			float LEMTempOffsetY = 0;

			HAL_TIM_Base_Start(&htim1);
			HAL_ADC_Start(&hadc2);
			HAL_ADC_Start(&hadc3);


			LEMTempOffsetY = 0;
			LEMTempOffsetX = 0;

			for (uint32_t i = 0; i < 1000; i++) {
				HAL_ADC_PollForConversion(&hadc3, 1);
				uint16_t ADCTempValueX = HAL_ADC_GetValue(&hadc3);
				LEMTempOffsetX += ADCTempValueX / 1000.0f;

				HAL_ADC_PollForConversion(&hadc2, 1);
				uint16_t ADCTempValueY = HAL_ADC_GetValue(&hadc2);
				LEMTempOffsetY += ADCTempValueY / 1000.0f;
			}
			YActuator.EP.LEMOffset = LEMTempOffsetY;
			XActuator.EP.LEMOffset = LEMTempOffsetX;

			HAL_TIM_Base_Stop(&htim1);
			HAL_ADC_Stop(&hadc2);
			HAL_ADC_Stop(&hadc3);

			//StopActuators();
			UARTTXLen = sprintf((char*) UARTTXBuffer,
					"Finished, LEM Offset: V = %f , U = %f\n",
					YActuator.EP.LEMOffset, XActuator.EP.LEMOffset);

			HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
			ActiveCommand = IDLE_COMMAND;
			LogCounter = 0;
			break;

			////////////////////////////////////////////////////////////////////////////////
			/*------------------------- V ACTUATOR COMMANDS -------------------------*/
			////////////////////////////////////////////////////////////////////////////////
		case LINEAR_VOLTAGE_STEP_COMMAND:

			if (LogCounter == 0) {
				HAL_UART_DMAStop(&huart2);
				UARTTXLen = sprintf((char*) UARTTXBuffer,
						"V AXIS VOLTAGE STEP COMMAND\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen,
						100);
				StartActuators();

				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				VOLTAGE_CONTROL, 3, 0, 0);

			} else if (LogCounter >= 0.5 * 24000) {

				SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
				MEASUREMENT_ONLY, 0, 0, 0);
				StopActuators();
				ActiveCommand = IDLE_COMMAND;

				UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen,
						100);
			}
			break;

		case LINEAR_CURRENT_STEP_COMMAND:

			if (LogCounter == 0) {
				HAL_UART_DMAStop(&huart2);
				UARTTXLen = sprintf((char*) UARTTXBuffer,
						"V AXIS CURRENT STEP COMMAND\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen,
						100);

				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0, 0.25, 0);
				STPOS = 1;
				StartActuators();

			} else if (LogCounter >= 0.5 * 24000) {

				SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
				MEASUREMENT_ONLY, 0, 0, 0);
				StopActuators();
				ActiveCommand = IDLE_COMMAND;
				STPOS = 0;
				UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen,
						100);
			}
			break;

		case SINESTEP_COMMAND:

					if (LogCounter == 0) {
						HAL_UART_DMAStop(&huart2);
						UARTTXLen = sprintf((char*) UARTTXBuffer,
								"U AXIS CURRENT STEP COMMAND\n");
						HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
						STPOS = 1;
						StartActuators();

					} else if (LogCounter <= 2.0 * 24000) {
//						float omegaT = (float) LogCounter / 24000.0f * freq_circle;
						float omegaT = (float) (LogCounter) / 24000.0f * freq_circle;
										SetControlParameters(1, POSITION_CONTROL,
												0, R_circle * sinf(omegaT),
												DONT_CHANGE_IT, 0, 0, 0);


					} else if (LogCounter > 2.0 * 24000) {

						SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
						MEASUREMENT_ONLY, 0, 0, 0);
						StopActuators();
						ActiveCommand = IDLE_COMMAND;
						STPOS = 0;
						UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
						HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen,
								100);
					}
					break;
		case STAIR_COMMAND:

			if (LogCounter == 0) {
				HAL_UART_DMAStop(&huart2);
				UARTTXLen = sprintf((char*) UARTTXBuffer,
						"U AXIS CURRENT STEP COMMAND\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
				STPOS = 1;
				StartActuators();

			} else if (LogCounter <= 1.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0, 0, 0);

			}else if (LogCounter <= 3.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.25, 0, 0);

			}else if (LogCounter <= 4.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.0, 0, 0);

			}else if (LogCounter <= 6.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.5, 0, 0);

			}else if (LogCounter <= 7.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.0, 0, 0);

			}else if (LogCounter <= 9.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.75, 0, 0);

			}else if (LogCounter <= 10.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.0, 0, 0);

			}else if (LogCounter <= 13.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 1.0, 0, 0);

			} else if (LogCounter <= 14.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.0, 0, 0);

			} else if (LogCounter <= 16.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.75, 0, 0);

			} else if (LogCounter <= 17.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.0, 0, 0);

			} else if (LogCounter <= 19.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.5, 0, 0);

			} else if (LogCounter <= 20.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.0, 0, 0);

			} else if (LogCounter <= 22.0 * 24000) {
				SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
				CURRENT_CONTROL, 0.25, 0, 0);

			} else if (LogCounter > 22.0 * 24000) {
				SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
							MEASUREMENT_ONLY, 0, 0, 0);
				StopActuators();
				ActiveCommand = IDLE_COMMAND;
				STPOS = 0;
				UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
			}
			break;



		case LINEAR_POSITION_STEP_COMMAND:

			if (LogCounter == 0) {
				HAL_UART_DMAStop(&huart2);
				UARTTXLen = sprintf((char*) UARTTXBuffer, "STEP COMMAND\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
				StartActuators();
				STPOS = 1;
			} else if (LogCounter <= 2.0 * 24000.f) {
				SetControlParameters(1, POSITION_CONTROL,
						0, 3,
									DONT_CHANGE_IT, 0, 0, 0);
			} else if (LogCounter > 2.0 * 24000.f) {

				SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
							MEASUREMENT_ONLY, 0, 0, 0);
				StopActuators();
				ActiveCommand = IDLE_COMMAND;
				STPOS = 0;
				UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen,
									100);
		    }
			break;

		case LINEAR_POSITION_RAMP_COMMAND:

			if (LogCounter == 0) {
				HAL_UART_DMAStop(&huart2);
				UARTTXLen = sprintf((char*) UARTTXBuffer, "RAMP COMMAND\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
				StartActuators();
				STPOS = 1;
			} else if (LogCounter < 4.0 * 24000.f) {
				// 5.0f is home target
				float position_now  = ( 5.0f / (4.0f * 24000.f) ) * LogCounter;
				SetControlParameters(1, POSITION_CONTROL,
						position_now, 0,
									DONT_CHANGE_IT, 0, 0, 0);


			} else if (LogCounter >= 4.0 * 24000.f) {

				SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
							MEASUREMENT_ONLY, 0, 0, 0);
				StopActuators();
				ActiveCommand = IDLE_COMMAND;
				STPOS = 0;
				UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen,
									100);
		    }
			break;


		case CIRCLE_COMMAND:

			if (LogCounter == 0) {
				HAL_UART_DMAStop(&huart2);
				UARTTXLen = sprintf((char*) UARTTXBuffer, "SINE COMMAND\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen,
						100);
				StartActuators();
				STPOS = 1;
			} else if (LogCounter < 0.25f * 24000.f) {

				float omegaT_dummy = 0.25f * freq_circle;
				float position_start_X  = ( R_circle * sinf(omegaT_dummy) / (0.25f * 24000.f) ) * LogCounter;
				float position_start_Y  = ( R_circle * cosf(omegaT_dummy) / (0.25f * 24000.f) ) * LogCounter;
				SetControlParameters(1, POSITION_CONTROL,
						position_start_X, position_start_Y,
									DONT_CHANGE_IT, 0, 0, 0);


			} else if (LogCounter <= 4 * 24000.f) {

				float omegaT = (float) LogCounter / 24000.0f * freq_circle;
				SetControlParameters(1, POSITION_CONTROL,
						R_circle * sinf(omegaT), R_circle * cosf(omegaT),
						DONT_CHANGE_IT, 0, 0, 0);

			} else if (LogCounter > 4 * 24000.f) {

				SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
				MEASUREMENT_ONLY, 0, 0, 0);
				StopActuators();
				ActiveCommand = IDLE_COMMAND;
				STPOS = 0;
				UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen,
						100);
			}

			break;

		case SQUARE_COMMAND:

		    if (LogCounter == 0) {
		        HAL_UART_DMAStop(&huart2);
		        UARTTXLen = sprintf((char*) UARTTXBuffer, "SQUARE COMMAND\n");
		        HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
		        StartActuators();
		        STPOS = 1;
		    } else if (LogCounter < 0.25f * 24000.f) {

		        uint32_t square_time = 0.25f * 24000.f ;

		        uint32_t period_ticks = (uint32_t)( 24000.0f / freq_square);
		        uint32_t ticks_per_side = period_ticks / 4;
		        uint32_t ticks_in_side = square_time % ticks_per_side;
		        float progress_dummy = (float)ticks_in_side / (float)ticks_per_side; // 0..1 along side
		        uint32_t corner = (square_time / ticks_per_side) % 4;

		        float pos_x, pos_y;

		        switch (corner) {
		            case 0: // Top edge: (A, A) → (-A, A)
		                pos_x =  amplitude_square - 2.0f * amplitude_square * progress_dummy;
		                pos_y =  amplitude_square;
		                break;
		            case 1: // Left edge: (-A, A) → (-A, -A)
		                pos_x = -1.0f*amplitude_square;
		                pos_y =  amplitude_square - 2.0f * amplitude_square * progress_dummy;
		                break;
		            case 2: // Bottom edge: (-A, -A) → (A, -A)
		                pos_x = -1.0f*amplitude_square + 2.0f * amplitude_square * progress_dummy;
		                pos_y = -1.0f*amplitude_square;
		                break;
		            case 3: // Right edge: (A, -A) → (A, A)
		                pos_x =  amplitude_square;
		                pos_y = -1.0f*amplitude_square + 2.0f * amplitude_square * progress_dummy;
		                break;
		        }

				float position_start_X  = ( pos_x  / square_time ) * LogCounter;
				float position_start_Y  = ( pos_y / square_time ) * LogCounter;


				SetControlParameters(1, POSITION_CONTROL,
						position_start_X, position_start_Y,
									DONT_CHANGE_IT, 0, 0, 0);

			} else if (LogCounter < 3 * 24000.f) {

		        // ticks for a full square
		        uint32_t period_ticks = (uint32_t)( 24000.0f / freq_square);
		        uint32_t ticks_per_side = period_ticks / 4;

		        uint32_t corner = (LogCounter / ticks_per_side) % 4;
		        uint32_t ticks_in_side = LogCounter % ticks_per_side;
		        float progress = (float)ticks_in_side / (float)ticks_per_side; // 0..1 along side

		        float pos_x, pos_y;

		        switch (corner) {
		            case 0: // Top edge: (A, A) → (-A, A)
		                pos_x =  amplitude_square - 2.0f * amplitude_square * progress;
		                pos_y =  amplitude_square;
		                break;
		            case 1: // Left edge: (-A, A) → (-A, -A)
		                pos_x = -1.0f*amplitude_square;
		                pos_y =  amplitude_square - 2.0f * amplitude_square * progress;
		                break;
		            case 2: // Bottom edge: (-A, -A) → (A, -A)
		                pos_x = -1.0f*amplitude_square + 2.0f * amplitude_square * progress;
		                pos_y = -1.0f*amplitude_square;
		                break;
		            case 3: // Right edge: (A, -A) → (A, A)
		                pos_x =  amplitude_square;
		                pos_y = -1.0f*amplitude_square + 2.0f * amplitude_square * progress;
		                break;
		        }

		        SetControlParameters(1, POSITION_CONTROL,
		                             pos_x, pos_y,
		                             DONT_CHANGE_IT, 0, 0, 0);
		    }
		    else if (LogCounter >= 3 * 24000.f) {
		        SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
		                             MEASUREMENT_ONLY, 0, 0, 0);
		        StopActuators();
		        ActiveCommand = IDLE_COMMAND;
		        STPOS = 0;
		        UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
		        HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
		    }

		    break;

		case TRIANGLE_COMMAND:

		    if (LogCounter == 0) {
		        HAL_UART_DMAStop(&huart2);
		        UARTTXLen = sprintf((char*) UARTTXBuffer, "TRIANGLE COMMAND\n");
		        HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
		        StartActuators();
		        STPOS = 1;
		    }else if (LogCounter < 0.25 * 24000.f) {

		    	uint32_t triangle_time = 0.25f * 24000.f ;
		        const float height = amplitude_triangle * sqrtf(3.0f);
		        // Vertices (centered at origin)
		        float vx[3] = { 0.0f,     -amplitude_triangle,  amplitude_triangle };
		        float vy[3] = { height/2.0f, -height/2.0f, -height/2.0f };

		        // ticks for a full triangle
		        uint32_t period_ticks = (uint32_t)(24000.0f / freq_triangle);
		        uint32_t ticks_per_side = period_ticks / 3;

		        uint32_t side = (triangle_time / ticks_per_side) % 3;
		        uint32_t ticks_in_side = triangle_time % ticks_per_side;
		        float progress = (float)ticks_in_side / (float)ticks_per_side; // 0..1 along side

		        // Start and end vertex for this side
		        uint32_t next_vertex = (side + 1) % 3;

		        // Linear interpolation between vertices
		        float pos_x = vx[side] + (vx[next_vertex] - vx[side]) * progress;
		        float pos_y = vy[side] + (vy[next_vertex] - vy[side]) * progress;

		        float pos_x_start = ( pos_x / triangle_time ) * LogCounter;
		        float pos_y_start = ( pos_y / triangle_time ) * LogCounter;
		        SetControlParameters(1, POSITION_CONTROL,
		        		pos_x_start, pos_y_start,
		                             DONT_CHANGE_IT, 0, 0, 0);




		    }else if (LogCounter < 3 * 24000.f) {

		        // Triangle geometry
		        // Height of equilateral triangle
		        const float height = amplitude_triangle * sqrtf(3.0f);
		        // Vertices (centered at origin)
		        float vx[3] = { 0.0f,     -amplitude_triangle,  amplitude_triangle };
		        float vy[3] = { height/2.0f, -height/2.0f, -height/2.0f };

		        // ticks for a full triangle
		        uint32_t period_ticks = (uint32_t)(24000.0f / freq_triangle);
		        uint32_t ticks_per_side = period_ticks / 3;

		        uint32_t side = (LogCounter / ticks_per_side) % 3;
		        uint32_t ticks_in_side = LogCounter % ticks_per_side;
		        float progress = (float)ticks_in_side / (float)ticks_per_side; // 0..1 along side

		        // Start and end vertex for this side
		        uint32_t next_vertex = (side + 1) % 3;

		        // Linear interpolation between vertices
		        float pos_x = vx[side] + (vx[next_vertex] - vx[side]) * progress;
		        float pos_y = vy[side] + (vy[next_vertex] - vy[side]) * progress;

		        SetControlParameters(1, POSITION_CONTROL,
		                             pos_x, pos_y,
		                             DONT_CHANGE_IT, 0, 0, 0);
		    }
		    else if (LogCounter >= 3 * 24000.f) {
		        SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
		                             MEASUREMENT_ONLY, 0, 0, 0);
		        StopActuators();
		        ActiveCommand = IDLE_COMMAND;
		        STPOS = 0;
		        UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
		        HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
		    }

		    break;

			////////////////////////////////////////////////////////////////////////////////
			/*------------------------- END OF ACTUATION COMMANDS ---------------------------*/
			////////////////////////////////////////////////////////////////////////////////
		case MEASUREMENT_COMMAND:
			UARTTXLen = sprintf((char*) UARTTXBuffer, "Measurement Command\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen);

			SetControlParameters(1, MEASUREMENT_ONLY, 0, 0,
			MEASUREMENT_ONLY, 0, 0, 0);

			StartActuators();

			ActiveCommand = IDLE_COMMAND;
			break;

		case STOP_COMMAND:
			HAL_UART_DMAStop(&huart2);
			UARTTXLen = sprintf((char*) UARTTXBuffer, "STOP Command\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
			StopActuators();
			UARTTXLen = sprintf((char*) UARTTXBuffer, "Finished\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen, 100);
			ActiveCommand = IDLE_COMMAND;
			DataLogStatus = LOG_IDLE;
			break;
		}

//		switch (DataLogStatus) {
//
//		case LOG_NEW_DATA_READY:
//			HAL_UART_DMAStop(&huart2);
//			UARTTXLen = snprintf((char*) UARTTXBuffer,
//			UART_TX_BUFFER_SIZE - 1, "%.2f,%.2f,%.2f\n",
//			YActuator.MC.ControlEffort[0],YActuator.EC.Error[0],YActuator.EM.Current);
//
//
//			HAL_UART_Transmit_DMA(&huart2, (uint8_t*) UARTTXBuffer, UARTTXLen);
//
//			DataLogStatus = LOG_IDLE;
//			break;
//
//		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC34
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1500-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_PIN_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_PIN_Pin PA9 */
  GPIO_InitStruct.Pin = LED_PIN_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_UART5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_UART5;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SetControlParameters(uint8_t Active, uint8_t MControlMode,
		float MDesiredValueX, float MDesiredValueY, uint8_t EControlMode,
		float EDesiredValueU, float EDesiredValueV, uint8_t Log) {

	YActuator.ControllerActive = Active;

	if (MControlMode != DONT_CHANGE_IT) {

//			if (MControlMode
//					!= YActuator.MC.ControlMode && MControlMode != POSITION_CONTROL) {
//				ReInitMechanicalController(V);
//				YActuator.MC.startPos = YActuator.MC.posSetY;
//				YActuator.MC.velCounter = 0;
//			}
//			if ((MControlMode == YActuator.MC.ControlMode
//					&& MControlMode == VELOCITY_CONTROL
//					&& MDesiredValue != YActuator.MC.DesiredValueY)) {
//				YActuator.MC.startPos = YActuator.MC.posSetY;
//				YActuator.MC.velCounter = 0;
//			}

		YActuator.MC.ControlMode = MControlMode;

//		XActuator.MC.DesiredValue = MDesiredValueX
//				* (float) DesiredTransformation
//				+ MDesiredValueY * (float) DesiredTransformation;
//
//		YActuator.MC.DesiredValue = MDesiredValueX * -1.0f
//				* (float) DesiredTransformation
//				+ MDesiredValueY * (float) DesiredTransformation;

		XActuator.MC.DesiredValue = MDesiredValueX;

		YActuator.MC.DesiredValue = MDesiredValueY;

	}

	if (EControlMode != DONT_CHANGE_IT) {
		if (EControlMode != YActuator.EC.ControlMode) {
			YActuator.EC.Error[1] = 0;
			YActuator.EC.ControlEffort[1] = 0;
			XActuator.EC.Error[1] = 0;
			XActuator.EC.ControlEffort[1] = 0;
		}
		YActuator.EC.ControlMode = EControlMode;
		YActuator.EC.DesiredValue = EDesiredValueV;
		XActuator.EC.DesiredValue = EDesiredValueU;
	}
	ControlLog = Log;

}
void ReInitMechanicalController() {

	YActuator.MC.Error[1] = 0;
	YActuator.MC.Error[2] = 0;
	YActuator.MC.ControlEffort[1] = 0;
	YActuator.MC.ControlEffort[2] = 0;

	// ZPETC Y
	YActuator.MC.ControlEffortZP[1] = 0;
	YActuator.MC.ControlEffortZP[2] = 0;
	YActuator.MC.DesiredValueZP[0] = 0;
	YActuator.MC.DesiredValueZP[1] = 0;
	YActuator.MC.DesiredValueZP[2] = 0;

	XActuator.MC.Error[1] = 0;
	XActuator.MC.Error[2] = 0;
	XActuator.MC.ControlEffort[1] = 0;
	XActuator.MC.ControlEffort[2] = 0;

	// ZPETC X
	XActuator.MC.ControlEffortZP[1] = 0;
	XActuator.MC.ControlEffortZP[2] = 0;
	XActuator.MC.DesiredValueZP[0] = 0;
	XActuator.MC.DesiredValueZP[1] = 0;
	XActuator.MC.DesiredValueZP[2] = 0;

}
void StartActuators() {

// Linear
	BridgeDriver(V, 0);
	MMeasurement();
	YActuator.MC.Updated = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	HAL_ADC_Start_IT(&hadc2);

// Rotary
	BridgeDriver(U, 0);
	XActuator.MC.Updated = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_ADC_Start_IT(&hadc3);

}
void StopActuators() {

//Linear
	HAL_ADC_Stop_IT(&hadc2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	YActuator.MM.Counter = 0;
	YActuator.MC.ControlEffort[1] = 0;
	YActuator.MC.ControlEffort[2] = 0;
	YActuator.MC.Error[1] = 0;
	YActuator.MC.Error[2] = 0;
	YActuator.EM.Counter = 0;
	YActuator.EC.ControlEffort[1] = 0;
	YActuator.EC.Error[1] = 0;
	YActuator.MC.posSetY = 0;
	YActuator.MC.startPos = 0;
	// ZPETC Y
	YActuator.MC.ControlEffortZP[1] = 0;
	YActuator.MC.ControlEffortZP[2] = 0;
	YActuator.MC.DesiredValueZP[1] = 0;
	YActuator.MC.DesiredValueZP[2] = 0;
	YActuator.MC.DesiredValueZP[3] = 0;
	SetControlParameters(0, MEASUREMENT_ONLY, 0, 0,
	MEASUREMENT_ONLY, 0, 0, 0);
	BridgeDriver(V, 0);

//Rotary
	HAL_ADC_Stop_IT(&hadc3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	XActuator.MM.Counter = 0;
	XActuator.MC.ControlEffort[1] = 0;
	XActuator.MC.ControlEffort[2] = 0;
	XActuator.MC.Error[1] = 0;
	XActuator.MC.Error[2] = 0;
	XActuator.EM.Counter = 0;
	XActuator.EC.ControlEffort[1] = 0;
	XActuator.EC.Error[1] = 0;
	XActuator.MC.posSetY = 0;
	XActuator.MC.startPos = 0;
	// ZPETC X
	XActuator.MC.ControlEffortZP[1] = 0;
	XActuator.MC.ControlEffortZP[2] = 0;
	XActuator.MC.DesiredValueZP[1] = 0;
	XActuator.MC.DesiredValueZP[2] = 0;
	XActuator.MC.DesiredValueZP[3] = 0;
//	SetControlParameters(0, MEASUREMENT_ONLY, 0 ,0,
//	MEASUREMENT_ONLY, 0, 0, 0);
	BridgeDriver(U, 0);

	LogCounter = 0;
	PrintCounter = 0;


}
volatile uint8_t ProcessSerialCommand(uint8_t *SerialCommand) {
	uint8_t ActiveCommand;
	if (!strncmp("HOME", (const char*) SerialCommand, 4))
		ActiveCommand = HOME_COMMAND;
	else if (!strncmp("STRT", (const char*) SerialCommand, 4))
		ActiveCommand = START_COMMAND;
	else if (!strncmp("STOP", (const char*) SerialCommand, 4))
		ActiveCommand = STOP_COMMAND;
	else if (!strncmp("MSRM", (const char*) SerialCommand, 4))
		ActiveCommand = MEASUREMENT_COMMAND;
	else if (!strncmp("COFF", (const char*) SerialCommand, 4))
		ActiveCommand = CURRENT_OFFSET_COMMAND;

	else if (!strncmp("LVST", (const char*) SerialCommand, 4))
		ActiveCommand = LINEAR_VOLTAGE_STEP_COMMAND;
	else if (!strncmp("LCST", (const char*) SerialCommand, 4))
		ActiveCommand = LINEAR_CURRENT_STEP_COMMAND;
	else if (!strncmp("LPST", (const char*) SerialCommand, 4))
		ActiveCommand = LINEAR_POSITION_STEP_COMMAND;
	else if (!strncmp("LPRA", (const char*) SerialCommand, 4))
		ActiveCommand = LINEAR_POSITION_RAMP_COMMAND;
	else if (!strncmp("STAR", (const char*) SerialCommand, 4))
		ActiveCommand = STAIR_COMMAND;

	else if (!strncmp("CIRC", (const char*) SerialCommand, 4))
		ActiveCommand = CIRCLE_COMMAND;
	else if (!strncmp("SQUR", (const char*) SerialCommand, 4))
		ActiveCommand = SQUARE_COMMAND;
	else if (!strncmp("TRIA", (const char*) SerialCommand, 4))
		ActiveCommand = TRIANGLE_COMMAND;
	else if (!strncmp("SIST", (const char*) SerialCommand, 4))
		ActiveCommand = SINESTEP_COMMAND;
	else if (!strncmp("PRNT", (const char*) SerialCommand, 4))
		ActiveCommand = PRINT_COMMAND;

	else
		ActiveCommand = IDLE_COMMAND;
	memset(SerialCommand, '\0', 25);
	return ActiveCommand;
}

void MMeasurement() {

	if (YActuator.MC.Updated == 1)
		LogOverrun++;

	XActuator.MM.Position = ((float) XActuator.MP.EncoderOffset
			- __HAL_TIM_GET_COUNTER(&htim4)) * XActuator.MP.EncoderGain + 8;

	YActuator.MM.Position = -((float) YActuator.MP.EncoderOffset
			- __HAL_TIM_GET_COUNTER(&htim3)) * YActuator.MP.EncoderGain;

//	PosU = XActuator.MM.Position * -1.0f * Transformation
//			+ YActuator.MM.Position * Transformation;
//
//	PosV = XActuator.MM.Position * Transformation
//			+ YActuator.MM.Position * Transformation;

	PosU = XActuator.MM.Position * (float) DesiredTransformation
			+ YActuator.MM.Position * (float) DesiredTransformation;

	PosV = XActuator.MM.Position * -1.0f * (float) DesiredTransformation
			+ YActuator.MM.Position * (float) DesiredTransformation;

}
void EMeasurement() {

	YActuator.EM.Current = -((float) HAL_ADC_GetValue(&hadc2)
			- YActuator.EP.LEMOffset) / 4096.0f * 3.3f * YActuator.EP.LEMGain;

	XActuator.EM.Current = (HAL_ADC_GetValue(&hadc3) - XActuator.EP.LEMOffset)
			/ 4096.0f * 3.3 * XActuator.EP.LEMGain;


}
void LogData() {
	if (LogCounter % 48 == 0) {

		if (YActuator.MC.Updated == 1 && ControlLog == 1) {

//			data[LogCounter / 24 - 1][0] = YActuator.MM.Position;
////			data[LogCounter / 24 - 1][0] = YActuator.MC.posSet;
//
//			for (uint8_t i = 0; i < 24; i++)
//				YActuator.EMLog.Current +=
//						YActuator.EM.CurrentHistory[i];
//			data[LogCounter / 24 - 1][1] = YActuator.EMLog.Current /=
//					(float) 24.000f;

//			data[LogCounter / 48 - 1][0] = XActuator.MC.posSet;
//			data[LogCounter / 48 - 1][1] = XyActuator.EM.Current;

		}

		if (XActuator.MC.Updated == 1 && ControlLog == 1) {
//			memcpy(&XActuator.MMLog, &XActuator.MM,
//					sizeof(XActuator.MM));
//			memcpy(&XActuator.EMLog, &XActuator.EM,
//					sizeof(XActuator.EM));
//			memcpy(&XActuator.MCLog, &XActuator.MC,
//					sizeof(XActuator.MC));
//			memcpy(&XActuator.ECLog, &XActuator.EC,
//					sizeof(XActuator.EC));

//			for (uint8_t i = 0; i < 24; i++)
//				XActuator.EMLog.Current +=
//						XActuator.EM.CurrentHistory[i];
//			XActuator.EMLog.Current /= (float) 24.0f;
			//		memcpy(&ControlParametersLog, &ControlParameters,
			//				sizeof(ControlParameters));
			//		for (uint8_t i = 0; i < (EM.Frequency / MM.Frequency); i++)
			//			EMLog.Current += EM.CurrentHistory[i];
			//		EMLog.Current /= (float) (EM.Frequency / MM.Frequency);
//			if (DataLogStatus == LOG_NEW_DATA_READY)
//				SerialOverrun++;

//			data[LogCounter / 48 - 1][0] = XActuator.MM.Position;
//			data[LogCounter / 48 - 1][1] = XActuator.EM.Current;

			DataLogStatus = LOG_NEW_DATA_READY;
			XActuator.MC.Updated = 0;

		}
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	if (hadc->Instance != ADC2) return;
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_9);

	if (YActuator.ControllerActive) {

		switch (YActuator.MC.ControlMode) {

		case POSITION_CONTROL:

			MMeasurement();

////			 ZPETC
//
//			YActuator.MC.DesiredValueZP_P1 = (float)YActuator.MC.DesiredValue;
//			XActuator.MC.DesiredValueZP_P1 = (float)XActuator.MC.DesiredValue;
//
//
//			YActuator.MC.ControlEffortZP[0] =
//					0.997974517019591f * YActuator.MC.ControlEffortZP[1]
//									+ 4.941519171539334e+03f * YActuator.MC.DesiredValueZP_P1
//										 - 1.462844624891079e+04f * YActuator.MC.DesiredValueZP[0]
//											+ 1.443338521062161e+04f * YActuator.MC.DesiredValueZP[1]
//											 - 4.746456143558051e+03f * YActuator.MC.DesiredValueZP[2];
//
//			XActuator.MC.ControlEffortZP[0] =
//					0.996596193075758f * XActuator.MC.ControlEffortZP[1]
//									+ 5.154520945144142e+03f * XActuator.MC.DesiredValueZP_P1
//										 - 1.531443963750262e+04f * XActuator.MC.DesiredValueZP[0]
//											+ 1.516656955389555e+04f * XActuator.MC.DesiredValueZP[1]
//											  - 5.006647582173812e+03f * XActuator.MC.DesiredValueZP[2];
//
//			YActuator.MC.ControlEffortZP[1] = YActuator.MC.ControlEffortZP[0];
//			XActuator.MC.ControlEffortZP[1] = XActuator.MC.ControlEffortZP[0];
//
//			YActuator.MC.DesiredValueZP[2] = YActuator.MC.DesiredValueZP[1];
//			YActuator.MC.DesiredValueZP[1] = YActuator.MC.DesiredValueZP[0];
//			YActuator.MC.DesiredValueZP[0] = YActuator.MC.DesiredValueZP_P1;
//
//			XActuator.MC.DesiredValueZP[2] = XActuator.MC.DesiredValueZP[1];
//			XActuator.MC.DesiredValueZP[1] = XActuator.MC.DesiredValueZP[0];
//			XActuator.MC.DesiredValueZP[0] = XActuator.MC.DesiredValueZP_P1;
//
//			YActuator.MC.posSetY = YActuator.MC.ControlEffortZP[0];
//			XActuator.MC.posSetY = XActuator.MC.ControlEffortZP[0];
////
						YActuator.MC.posSetY = (double)YActuator.MC.DesiredValue;
						XActuator.MC.posSetY = (double)XActuator.MC.DesiredValue;

			// X-Y to U-V Transform
			float targetU = XActuator.MC.posSetY
					* (float) DesiredTransformation
					+ YActuator.MC.posSetY * (float) DesiredTransformation;

			float targetV = XActuator.MC.posSetY * -1.0f
					* (float) DesiredTransformation
					+ YActuator.MC.posSetY * (float) DesiredTransformation;

//
//			YActuator.MC.posSetY = YActuator.MC.DesiredValue;
//			XActuator.MC.posSetY = XActuator.MC.DesiredValue;


			YActuator.MC.Error[0] = targetV - PosV;
			XActuator.MC.Error[0] = targetU - PosU;




//// Controller#2 for tracking
			YActuator.MC.ControlEffort[0] =
					1.485726401317451f * YActuator.MC.ControlEffort[1]
							- 0.485746749177129f * YActuator.MC.ControlEffort[2]
							+ 300.0f
									* (YActuator.MC.Error[0]
											- 1.997800000000000f
													* YActuator.MC.Error[1]
											+ 0.997801050000000f
													* YActuator.MC.Error[2]);

			XActuator.MC.ControlEffort[0] =
					1.485726401317451f * XActuator.MC.ControlEffort[1]
							- 0.485746749177129f * XActuator.MC.ControlEffort[2]
							+ 300.0f
									* (XActuator.MC.Error[0]
											- 1.997800000000000f
													* XActuator.MC.Error[1]
											+ 0.997801050000000f
													* XActuator.MC.Error[2]);

			YActuator.MC.ControlEffort[2] = YActuator.MC.ControlEffort[1];
			YActuator.MC.ControlEffort[1] = YActuator.MC.ControlEffort[0];
			XActuator.MC.ControlEffort[2] = XActuator.MC.ControlEffort[1];
			XActuator.MC.ControlEffort[1] = XActuator.MC.ControlEffort[0];

			YActuator.MC.Error[2] = YActuator.MC.Error[1];
			YActuator.MC.Error[1] = YActuator.MC.Error[0];
			XActuator.MC.Error[2] = XActuator.MC.Error[1];
			XActuator.MC.Error[1] = XActuator.MC.Error[0];


			SetControlParameters(1, DONT_CHANGE_IT, 0, 0,
			CURRENT_CONTROL, XActuator.MC.ControlEffort[0],
					YActuator.MC.ControlEffort[0], 0);

			YActuator.MM.Counter++;
			YActuator.MC.Updated = 1;

			break;

		case MEASUREMENT_ONLY:

			MMeasurement();
			YActuator.MM.Counter++;
			YActuator.MC.Updated = 1;
			break;

		}

		switch (YActuator.EC.ControlMode) {

		case VOLTAGE_CONTROL:
			EMeasurement();
			BridgeDriver(V, YActuator.EC.DesiredValue);
			BridgeDriver(U, XActuator.EC.DesiredValue);
			YActuator.EM.Counter++;
			break;

		case CURRENT_CONTROL:

			EMeasurement();

			YActuator.EC.Error[0] = YActuator.EC.DesiredValue
					- YActuator.EM.Current;
			XActuator.EC.Error[0] = XActuator.EC.DesiredValue
					- XActuator.EM.Current;

			YActuator.EC.ControlEffort[0] = 1.0f * YActuator.EC.ControlEffort[1]
					+ 2.00f
							* (YActuator.EC.Error[0]
									- 0.8569f * YActuator.EC.Error[1]);

//			YActuator.EC.ControlEffort[0] = 0.999833865388806f * YActuator.EC.ControlEffort[1]
//					+ 6.00f *
//							(YActuator.EC.Error[0] - 0.982100133684100f * YActuator.EC.Error[1]);

			XActuator.EC.ControlEffort[0] = 1.0f * XActuator.EC.ControlEffort[1]
					+ 2.00f
							* (XActuator.EC.Error[0]
									- 0.8569f * XActuator.EC.Error[1]);

			YActuator.EC.ControlEffort[1] = BridgeDriver(V,
					YActuator.EC.ControlEffort[0]);
			XActuator.EC.ControlEffort[1] = BridgeDriver(U,
					XActuator.EC.ControlEffort[0]);

			YActuator.EC.Error[1] = YActuator.EC.Error[0];
			XActuator.EC.Error[1] = XActuator.EC.Error[0];
			YActuator.EM.Counter++;

			break;

		case MEASUREMENT_ONLY:

			EMeasurement();
//			LogData();
			YActuator.EM.Counter++;
			break;
		}
	}


	if (LogCounter % 48 == 0 && STPOS == 1) {
		//&& (LogCounter > 1*24000)

		CURRENTLOG_V[PrintCounter] = YActuator.EM.Current;
//		CURRENTLOG_U[PrintCounter] = XActuator.EM.Current;
		CURRENTLOG_U[PrintCounter] = YActuator.MC.DesiredValueZP[0];
		PosLog_U[PrintCounter] = XActuator.MC.posSetY;
	    PosLog_V[PrintCounter] = YActuator.MC.posSetY;
		PosLog_X[PrintCounter] = YActuator.MC.DesiredValueZP_P1;
//		PosLog_X[PrintCounter] = XActuator.MM.Position;
		PosLog_Y[PrintCounter] = YActuator.MM.Position;

//		LogC[PrintCounter] = LogCounter;

//		EC_ConEffX_LOG[PrintCounter] = XActuator.EC.ControlEffort[0];
//		EC_ConEffY_LOG[PrintCounter] = YActuator.EC.ControlEffort[0];
//		MC_ErrX_LOG[PrintCounter] = XActuator.MC.Error[0];
//		MC_ErrY_LOG[PrintCounter] = YActuator.MC.Error[0];

//		EC_ErrX_LOG[PrintCounter] = XActuator.EC.Error[0];
//		EC_ErrY_LOG[PrintCounter] = YActuator.EC.Error[0];

		PrintCounter++;
	}

	if (PrintCounter >= PrintNumbers) {
		STPOS = 0;
	}

	LogCounter++;
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_9);

}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	ActiveCommand = ProcessSerialCommand(UARTRXBuffer);
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, UARTRXBuffer,
	UART_RX_BUFFER_SIZE);
}

volatile float BridgeDriver(uint8_t Axis, float ControlEffort) {

	ControlEffort *= PWMARR / BusVoltage;
	if (ControlEffort >= 0) {
		ControlEffort = (ControlEffort > PWMARR) ?
		PWMARR :
													ControlEffort;
		if (Axis == V) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ControlEffort);
		} else if (Axis == U) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ControlEffort);
		}
		return ControlEffort * BusVoltage / PWMARR;
	} else {
		ControlEffort = (-ControlEffort > PWMARR) ?
		PWMARR :
													-ControlEffort;
		if (Axis == V) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ControlEffort);
		} else if (Axis == U) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ControlEffort);
		}
		return -ControlEffort * BusVoltage / PWMARR;
	}
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
	__disable_irq();
	while (1) {
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
