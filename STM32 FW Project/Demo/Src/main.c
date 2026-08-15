/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : STEVAL-FCU001V1 - tethered auto-hover build
  ******************************************************************************
  *
  * Stripped from the BLE remocon firmware. No radio, no BLE, no USB, no ADC.
  * On power-up the board runs a fixed, open-loop throttle profile while the
  * existing cascade PID holds attitude level:
  *
  *     calibrate (2 s)  ->  settle (2 s)  ->  spin-up (1 s)
  *                      ->  hover (10 s)  ->  descend (2 s)  ->  off (latched)
  *
  * THE BOARD MUST BE FLAT AND COMPLETELY STILL FOR THE FIRST 4 SECONDS.
  * The gyro/accel offsets and the level reference are captured in that window;
  * if it moves, the drone flies off at an angle.
  *
  * This holds ATTITUDE, not ALTITUDE. There is no altitude controller in this
  * codebase. PWM_HOVER below is an open-loop guess that must be calibrated on
  * the tether - see the bring-up procedure in the notes at the bottom.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "debug.h"
#include "config_drone.h"
#include "steval_fcu001_v1.h"
#include "steval_fcu001_v1_accelero.h"
#include "steval_fcu001_v1_gyro.h"
#include "sensor_data.h"
#include "quaternion.h"
#include "ahrs.h"
#include "flight_control.h"
#include "motor.h"
#include <math.h>

/* ==========================================================================
 *  FLIGHT PROFILE - the only numbers you should need to touch
 * ========================================================================== */

/* TIM9 tick rate: 84 MHz / (51+1) / 2000 */
#define TICK_HZ                 807.7f
#define TICKS(sec)              ((uint32_t)((sec) * TICK_HZ))

#define T_SETTLE_S              2.0f    /* motors off, AHRS pulls level at KP_BIG */
#define T_SPINUP_S              1.0f    /* ramp idle -> hover                     */
#define T_HOVER_S              10.0f    /* the 10 seconds you asked for           */
#define T_DESCEND_S             2.0f    /* ramp hover -> off                      */

/* PWM counts. TIM4 period is 1999; set_motor_pwm() clamps to [0, 1900]. */
#define PWM_IDLE                 0
#define PWM_SPINUP_FLOOR       700      /* props just begin to turn - verify!     */
#define PWM_HOVER             1100      /* <<< CALIBRATE THIS ON THE TETHER <<<   */

/* Abort if attitude runs away. ~50 deg. */
#define TILT_ABORT_RAD          0.87f

/* Belt-and-braces: nothing may ever run longer than this, whatever happens. */
#define HARD_CEILING_TICKS      TICKS(T_SETTLE_S + T_SPINUP_S + T_HOVER_S + T_DESCEND_S + 3.0f)

/* Phase boundaries, in ticks since calibration finished */
#define TK_SETTLE_END           TICKS(T_SETTLE_S)
#define TK_SPINUP_END           (TK_SETTLE_END  + TICKS(T_SPINUP_S))
#define TK_HOVER_END            (TK_SPINUP_END  + TICKS(T_HOVER_S))
#define TK_DESCEND_END          (TK_HOVER_END   + TICKS(T_DESCEND_S))

/* Synthetic "throttle demand". This is NOT a PWM value - it only tells ahrs.c
 * and flight_control.c whether we are armed, via their gTHR < MIN_THR tests.
 * MIN_THR is 200 in the MOTOR_DC build. */
#define GTHR_DISARMED            0
#define GTHR_ARMED            1000

typedef enum
{
  ST_CALIBRATING = 0,
  ST_SETTLE,
  ST_SPINUP,
  ST_HOVER,
  ST_DESCEND,
  ST_DONE,
  ST_ABORT
} FlightState;

/* ========================================================================== */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;          /* sensor SPI - configured by the BSP        */
SPI_HandleTypeDef hspi2;          /* referenced by board.c / stm32f4xx_it.c    */
TIM_HandleTypeDef htim4;          /* motor PWM out                             */
TIM_HandleTypeDef htim9;          /* 807.7 Hz control tick                     */
UART_HandleTypeDef huart1;        /* optional telemetry (PRINTF)               */

static void *LSM6DSL_X_0_handle = NULL;
static void *LSM6DSL_G_0_handle = NULL;

/* gTHR is externed by ahrs.c and flight_control.c - keep the name. */
int16_t gTHR = GTHR_DISARMED;
extern int16_t motor_thr;         /* now owned by us, see flight_control.c     */

volatile uint32_t tim9_event_flag = 0;
uint32_t tim9_cnt2 = 0;

volatile FlightState flight_state = ST_CALIBRATING;
volatile uint32_t flight_tick = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);
static void initializeAllSensors(void);
static void enableAllSensors(void);

/* USER CODE BEGIN 0 */
P_PI_PIDControlTypeDef pid;
EulerAngleTypeDef euler_rc, euler_ahrs;
AxesRaw_TypeDef acc, gyro, mag;
AxesRaw_TypeDef_Float acc_ahrs_FIFO[FIFO_Order], acc_FIFO[FIFO_Order], acc_ahrs;
AxesRaw_TypeDef_Float gyro_fil, gyro_y_pre[4], gyro_x_pre[4];
AxesRaw_TypeDef_Float gyro_ahrs_FIFO[FIFO_Order], gyro_FIFO[FIFO_Order], gyro_ahrs;
AxesRaw_TypeDef acc_off_calc, gyro_off_calc, acc_offset, gyro_offset;

int sensor_init_cali = 0, sensor_init_cali_count = 0;

typedef struct
{
  float a1, a2, b0, b1, b2;
} IIR_Coeff;

/* 100 Hz cutoff @ 800 Hz sample rate */
IIR_Coeff gyro_fil_coeff = {0.94280904158206336, -0.33333333333333343,
                            0.09763107293781749, 0.19526214587563498,
                            0.09763107293781749};

Gyro_Rad gyro_rad;
MotorControlTypeDef motor_pwm;
AHRS_State_TypeDef ahrs;
float press;                      /* unused - ReadSensorRawData needs the arg  */
/* USER CODE END 0 */

/* Linear ramp between two PWM values across a tick window */
static int16_t ramp(uint32_t t, uint32_t t0, uint32_t t1, int16_t v0, int16_t v1)
{
  if (t <= t0) return v0;
  if (t >= t1) return v1;
  return (int16_t)(v0 + ((int32_t)(v1 - v0) * (int32_t)(t - t0)) / (int32_t)(t1 - t0));
}

int main(void)
{
  int i;

  /* Zero the filter and offset state */
  acc_offset.AXIS_X  = 0; acc_offset.AXIS_Y  = 0; acc_offset.AXIS_Z  = 1000;
  gyro_offset.AXIS_X = 0; gyro_offset.AXIS_Y = 0; gyro_offset.AXIS_Z = 0;
  acc_off_calc.AXIS_X  = 0; acc_off_calc.AXIS_Y  = 0; acc_off_calc.AXIS_Z  = 0;
  gyro_off_calc.AXIS_X = 0; gyro_off_calc.AXIS_Y = 0; gyro_off_calc.AXIS_Z = 0;
  gyro_fil.AXIS_X = 0; gyro_fil.AXIS_Y = 0; gyro_fil.AXIS_Z = 0;

  for (i = 0; i < 4; i++)
  {
    gyro_x_pre[i].AXIS_X = 0; gyro_x_pre[i].AXIS_Y = 0; gyro_x_pre[i].AXIS_Z = 0;
    gyro_y_pre[i].AXIS_X = 0; gyro_y_pre[i].AXIS_Y = 0; gyro_y_pre[i].AXIS_Z = 0;
  }

  /* Attitude setpoint is level, always. This is the whole "remote control". */
  euler_rc.thx = 0.0f;
  euler_rc.thy = 0.0f;
  euler_rc.thz = 0.0f;

  /* MCU configuration -------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();

  PRINTF("STEVAL-FCU001V1 - tethered auto-hover build\n");

  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);

  /* Sensors: accelerometer + gyroscope only. Magnetometer and barometer are
   * not initialised - neither was ever used for control. */
  Sensor_IO_SPI_CS_Init_All();
  initializeAllSensors();
  enableAllSensors();

  /* Accelerometer: ODR 6.6 kHz, FS 4g, analog BW 1500 Hz, LPF2 @ ODR/400 */
  BSP_ACCELERO_Set_ODR_Value(LSM6DSL_X_0_handle, 6660.0);
  BSP_ACCELERO_Set_FS(LSM6DSL_X_0_handle, FS_MID);
  LSM6DSL_ACC_GYRO_W_InComposit(LSM6DSL_X_0_handle, LSM6DSL_ACC_GYRO_IN_ODR_DIV_2);
  LSM6DSL_ACC_GYRO_W_LowPassFiltSel_XL(LSM6DSL_X_0_handle, LSM6DSL_ACC_GYRO_LPF2_XL_ENABLE);
  LSM6DSL_ACC_GYRO_W_HPCF_XL(LSM6DSL_X_0_handle, LSM6DSL_ACC_GYRO_HPCF_XL_DIV400);
  {
    uint8_t reg;
    BSP_ACCELERO_Read_Reg(LSM6DSL_X_0_handle, 0x10, &reg);
    reg = reg & 0xFE;                     /* analog filter 1500 Hz */
    BSP_ACCELERO_Write_Reg(LSM6DSL_X_0_handle, 0x10, reg);
  }

  /* Gyroscope: FS 2000 dps, ODR 416 Hz, LPF1 narrow */
  LSM6DSL_ACC_GYRO_W_LP_BW_G(LSM6DSL_G_0_handle, LSM6DSL_ACC_GYRO_LP_G_NARROW);
  BSP_GYRO_Write_Reg(LSM6DSL_G_0_handle, 0x11, 0x6C);

  /* Motor PWM out */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  PIDControlInit(&pid);
  set_motor_pwm_zero(&motor_pwm);
  set_motor_pwm(&motor_pwm);

  /* Start the 807.7 Hz control tick. Everything happens from here. */
  BSP_LED_On(LED1);                       /* LED1 on = calibrating, hold still */
  HAL_TIM_Base_Start_IT(&htim9);

  /* Infinite loop -----------------------------------------------------------*/
  while (1)
  {
    if (tim9_event_flag)
    {
      /* Released by the ISR every 5th tick -> 161.5 Hz */
      tim9_event_flag = 0;

      acc_ahrs.AXIS_X = 0; acc_ahrs.AXIS_Y = 0; acc_ahrs.AXIS_Z = 0;
      gyro_ahrs.AXIS_X = 0; gyro_ahrs.AXIS_Y = 0; gyro_ahrs.AXIS_Z = 0;

      for (i = 0; i < FIFO_Order; i++)
      {
        acc_ahrs.AXIS_X  += acc_ahrs_FIFO[i].AXIS_X;
        acc_ahrs.AXIS_Y  += acc_ahrs_FIFO[i].AXIS_Y;
        acc_ahrs.AXIS_Z  += acc_ahrs_FIFO[i].AXIS_Z;
        gyro_ahrs.AXIS_X += gyro_ahrs_FIFO[i].AXIS_X;
        gyro_ahrs.AXIS_Y += gyro_ahrs_FIFO[i].AXIS_Y;
        gyro_ahrs.AXIS_Z += gyro_ahrs_FIFO[i].AXIS_Z;
      }

      acc_ahrs.AXIS_X  *= FIFO_Order_Recip;
      acc_ahrs.AXIS_Y  *= FIFO_Order_Recip;
      acc_ahrs.AXIS_Z  *= FIFO_Order_Recip;
      gyro_ahrs.AXIS_X *= FIFO_Order_Recip;
      gyro_ahrs.AXIS_Y *= FIFO_Order_Recip;
      gyro_ahrs.AXIS_Z *= FIFO_Order_Recip;

      /* Attitude estimate */
      ahrs_fusion_ag(&acc_ahrs, &gyro_ahrs, &ahrs);
      QuaternionToEuler(&ahrs.q, &euler_ahrs);

      /* Outer angle loop -> body rate command in pid->x_s1 / y_s1 / z_s1 */
      FlightControlPID_OuterLoop(&euler_rc, &euler_ahrs, &ahrs, &pid);
    }
  }
}

/**
 * TIM9 period elapsed - 807.7 Hz. This is the whole fast path.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  int i;

  /* ---- Phase 0: offset capture. 1 s settle, then 1 s of averaging. -------- */
  if (sensor_init_cali == 0)
  {
    sensor_init_cali_count++;

    if (sensor_init_cali_count > 800)
    {
      ReadSensorRawData(LSM6DSL_X_0_handle, LSM6DSL_G_0_handle, NULL, NULL,
                        &acc, &gyro, &mag, &press);

      acc_off_calc.AXIS_X  += acc.AXIS_X;
      acc_off_calc.AXIS_Y  += acc.AXIS_Y;
      acc_off_calc.AXIS_Z  += acc.AXIS_Z;
      gyro_off_calc.AXIS_X += gyro.AXIS_X;
      gyro_off_calc.AXIS_Y += gyro.AXIS_Y;
      gyro_off_calc.AXIS_Z += gyro.AXIS_Z;

      if (sensor_init_cali_count >= 1600)
      {
        acc_offset.AXIS_X  = (int32_t)(acc_off_calc.AXIS_X  * 0.00125f);
        acc_offset.AXIS_Y  = (int32_t)(acc_off_calc.AXIS_Y  * 0.00125f);
        acc_offset.AXIS_Z  = (int32_t)(acc_off_calc.AXIS_Z  * 0.00125f);
        gyro_offset.AXIS_X = (int32_t)(gyro_off_calc.AXIS_X * 0.00125f);
        gyro_offset.AXIS_Y = (int32_t)(gyro_off_calc.AXIS_Y * 0.00125f);
        gyro_offset.AXIS_Z = (int32_t)(gyro_off_calc.AXIS_Z * 0.00125f);

        sensor_init_cali_count = 0;
        sensor_init_cali = 1;
        flight_state = ST_SETTLE;
        flight_tick  = 0;
        BSP_LED_Off(LED1);                /* LED1 off = calibration done */
      }
    }
    return;
  }

  /* ---- Sensor read, offset removal, filtering ---------------------------- */
  tim9_cnt2++;

  ReadSensorRawData(LSM6DSL_X_0_handle, LSM6DSL_G_0_handle, NULL, NULL,
                    &acc, &gyro, &mag, &press);

  acc.AXIS_X  -= acc_offset.AXIS_X;
  acc.AXIS_Y  -= acc_offset.AXIS_Y;
  acc.AXIS_Z  -= (acc_offset.AXIS_Z - 1000);
  gyro.AXIS_X -= gyro_offset.AXIS_X;
  gyro.AXIS_Y -= gyro_offset.AXIS_Y;
  gyro.AXIS_Z -= gyro_offset.AXIS_Z;

  acc_FIFO[tim9_cnt2 - 1].AXIS_X = acc.AXIS_X;
  acc_FIFO[tim9_cnt2 - 1].AXIS_Y = acc.AXIS_Y;
  acc_FIFO[tim9_cnt2 - 1].AXIS_Z = acc.AXIS_Z;

  /* 2nd-order IIR on the gyro, 100 Hz */
  gyro_fil.AXIS_X = gyro_fil_coeff.b0 * gyro.AXIS_X
                  + gyro_fil_coeff.b1 * gyro_x_pre[0].AXIS_X
                  + gyro_fil_coeff.b2 * gyro_x_pre[1].AXIS_X
                  + gyro_fil_coeff.a1 * gyro_y_pre[0].AXIS_X
                  + gyro_fil_coeff.a2 * gyro_y_pre[1].AXIS_X;
  gyro_fil.AXIS_Y = gyro_fil_coeff.b0 * gyro.AXIS_Y
                  + gyro_fil_coeff.b1 * gyro_x_pre[0].AXIS_Y
                  + gyro_fil_coeff.b2 * gyro_x_pre[1].AXIS_Y
                  + gyro_fil_coeff.a1 * gyro_y_pre[0].AXIS_Y
                  + gyro_fil_coeff.a2 * gyro_y_pre[1].AXIS_Y;
  gyro_fil.AXIS_Z = gyro_fil_coeff.b0 * gyro.AXIS_Z
                  + gyro_fil_coeff.b1 * gyro_x_pre[0].AXIS_Z
                  + gyro_fil_coeff.b2 * gyro_x_pre[1].AXIS_Z
                  + gyro_fil_coeff.a1 * gyro_y_pre[0].AXIS_Z
                  + gyro_fil_coeff.a2 * gyro_y_pre[1].AXIS_Z;

  for (i = 1; i > 0; i--)
  {
    gyro_x_pre[i].AXIS_X = gyro_x_pre[i - 1].AXIS_X;
    gyro_x_pre[i].AXIS_Y = gyro_x_pre[i - 1].AXIS_Y;
    gyro_x_pre[i].AXIS_Z = gyro_x_pre[i - 1].AXIS_Z;
    gyro_y_pre[i].AXIS_X = gyro_y_pre[i - 1].AXIS_X;
    gyro_y_pre[i].AXIS_Y = gyro_y_pre[i - 1].AXIS_Y;
    gyro_y_pre[i].AXIS_Z = gyro_y_pre[i - 1].AXIS_Z;
  }
  gyro_x_pre[0].AXIS_X = gyro.AXIS_X;
  gyro_x_pre[0].AXIS_Y = gyro.AXIS_Y;
  gyro_x_pre[0].AXIS_Z = gyro.AXIS_Z;
  gyro_y_pre[0].AXIS_X = gyro_fil.AXIS_X;
  gyro_y_pre[0].AXIS_Y = gyro_fil.AXIS_Y;
  gyro_y_pre[0].AXIS_Z = gyro_fil.AXIS_Z;

  gyro_FIFO[tim9_cnt2 - 1].AXIS_X = gyro_fil.AXIS_X;
  gyro_FIFO[tim9_cnt2 - 1].AXIS_Y = gyro_fil.AXIS_Y;
  gyro_FIFO[tim9_cnt2 - 1].AXIS_Z = gyro_fil.AXIS_Z;

  if (tim9_cnt2 == FIFO_Order)
  {
    tim9_cnt2 = 0;
    for (i = 0; i < FIFO_Order; i++)
    {
      acc_ahrs_FIFO[i].AXIS_X  = acc_FIFO[i].AXIS_X;
      acc_ahrs_FIFO[i].AXIS_Y  = acc_FIFO[i].AXIS_Y;
      acc_ahrs_FIFO[i].AXIS_Z  = acc_FIFO[i].AXIS_Z;
      gyro_ahrs_FIFO[i].AXIS_X = gyro_FIFO[i].AXIS_X;
      gyro_ahrs_FIFO[i].AXIS_Y = gyro_FIFO[i].AXIS_Y;
      gyro_ahrs_FIFO[i].AXIS_Z = gyro_FIFO[i].AXIS_Z;
    }
    tim9_event_flag = 1;                  /* release the main loop */
  }

  gyro_rad.gx = ((float)gyro_fil.AXIS_X) * ((float)COE_MDPS_TO_RADPS);
  gyro_rad.gy = ((float)gyro_fil.AXIS_Y) * ((float)COE_MDPS_TO_RADPS);
  gyro_rad.gz = ((float)gyro_fil.AXIS_Z) * ((float)COE_MDPS_TO_RADPS);

  euler_ahrs.thz += gyro_rad.gz * PID_SAMPLING_TIME;

  /* ---- Flight profile state machine -------------------------------------- */
  flight_tick++;

  /* Abort on attitude runaway or if we have simply been alive too long. */
  if (flight_state == ST_SPINUP || flight_state == ST_HOVER || flight_state == ST_DESCEND)
  {
    if (fabsf(euler_ahrs.thx) > TILT_ABORT_RAD ||
        fabsf(euler_ahrs.thy) > TILT_ABORT_RAD ||
        flight_tick > HARD_CEILING_TICKS)
    {
      flight_state = ST_ABORT;
    }
  }

  switch (flight_state)
  {
    case ST_SETTLE:
      gTHR      = GTHR_DISARMED;          /* AHRS at KP_BIG, integrators held  */
      motor_thr = PWM_IDLE;
      if (flight_tick >= TK_SETTLE_END)
      {
        flight_state = ST_SPINUP;
        BSP_LED_On(LED2);                 /* LED2 on = motors live             */
      }
      break;

    case ST_SPINUP:
      gTHR      = GTHR_ARMED;
      motor_thr = ramp(flight_tick, TK_SETTLE_END, TK_SPINUP_END,
                       PWM_SPINUP_FLOOR, PWM_HOVER);
      if (flight_tick >= TK_SPINUP_END) flight_state = ST_HOVER;
      break;

    case ST_HOVER:
      gTHR      = GTHR_ARMED;
      motor_thr = PWM_HOVER;
      if (flight_tick >= TK_HOVER_END) flight_state = ST_DESCEND;
      break;

    case ST_DESCEND:
      gTHR      = GTHR_ARMED;
      motor_thr = ramp(flight_tick, TK_HOVER_END, TK_DESCEND_END,
                       PWM_HOVER, PWM_IDLE);
      if (flight_tick >= TK_DESCEND_END) flight_state = ST_DONE;
      break;

    case ST_DONE:
    case ST_ABORT:
    default:
      gTHR      = GTHR_DISARMED;
      motor_thr = PWM_IDLE;
      BSP_LED_Off(LED2);
      break;
  }

  /* ---- Inner rate loop + mixer ------------------------------------------- */
  if (flight_state == ST_SPINUP || flight_state == ST_HOVER || flight_state == ST_DESCEND)
  {
    FlightControlPID_innerLoop(&euler_rc, &gyro_rad, &ahrs, &pid, &motor_pwm);
  }
  else
  {
    set_motor_pwm_zero(&motor_pwm);
    euler_ahrs.thz = 0.0f;                /* park the drifting yaw estimate    */
  }

  set_motor_pwm(&motor_pwm);
}

/** System Clock Configuration - HSE, 84 MHz */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 16;
  RCC_OscInitStruct.PLL.PLLN       = 336;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ       = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM4 - 4 x motor PWM. DC build: 494 Hz, period 1999. */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
#ifdef MOTOR_DC
  htim4.Init.Prescaler = 84;
  htim4.Init.Period    = 1999;
#endif
#ifdef MOTOR_ESC
  htim4.Init.Prescaler = 100;
  htim4.Init.Period    = 2075;
#endif
  htim4.Init.CounterMode   = TIM_COUNTERMODE_UP;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
}

/* TIM9 - control tick. 84 MHz / 52 / 2000 = 807.7 Hz */
static void MX_TIM9_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance           = TIM9;
  htim9.Init.Prescaler     = 51;
  htim9.Init.CounterMode   = TIM_COUNTERMODE_UP;
  htim9.Init.Period        = 1999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim9);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void initializeAllSensors(void)
{
  if (BSP_ACCELERO_Init(LSM6DSL_X_0, &LSM6DSL_X_0_handle) != COMPONENT_OK) while (1);
  if (BSP_GYRO_Init(LSM6DSL_G_0, &LSM6DSL_G_0_handle) != COMPONENT_OK)     while (1);
}

static void enableAllSensors(void)
{
  BSP_ACCELERO_Sensor_Enable(LSM6DSL_X_0_handle);
  BSP_GYRO_Sensor_Enable(LSM6DSL_G_0_handle);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
