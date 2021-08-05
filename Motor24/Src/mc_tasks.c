
/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes tasks definition
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "mc_config.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "digital_output.h"
#include "state_machine.h"
#include "pwm_common.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"

/* USER CODE BEGIN Includes */
#include "spi_pwm_encoder.h"
#include "mc_position.h"
#include "tle5012b.h"
#include "abs_encoder_pos_fdbk.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)

/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

/* USER CODE END Private define */
// #define VBUS_TEMP_ERR_MASK ~(0 | 0 | MC_OVER_TEMP)
#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t * oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
RDivider_Handle_t *pBusSensorM1;
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref
                                                    during the start-up switch over.*/

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

uint8_t bMCBootCompleted = 0;

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
static void TSK_MediumFrequencyTaskM1(void);
static void FOC_Clear(uint8_t bMotor);
static void FOC_InitAdditionalMethods(uint8_t bMotor);
static void FOC_CalcCurrRef(uint8_t bMotor);
static uint16_t FOC_CurrController(uint8_t bMotor);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
static void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
static bool TSK_StopPermanencyTimeHasElapsedM1(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);

void UI_Scheduler(void);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @param  pMCTList pointer to the vector of MCTuning objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @retval None
  */
void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS],MCT_Handle_t* pMCTList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

  bMCBootCompleted = 0;
  pCLM[M1] = &CircleLimitationM1;

  /**********************************************************/
  /*    PWM and current sensing component initialization    */
  /**********************************************************/
  pwmcHandle[M1] = &PWM_Handle_M1._Super;
  R1VL1_Init(&PWM_Handle_M1);
  /* USER CODE BEGIN MCboot 1 */

  /* USER CODE END MCboot 1 */

  /**************************************/
  /*    Start timers synchronously      */
  /**************************************/
  startTimers();    

  /**************************************/
  /*    State machine initialization    */
  /**************************************/
  STM_Init(&STM[M1]);
  
  /******************************************************/
  /*   PID component initialization: speed regulation   */
  /******************************************************/
  PID_HandleInit(&PIDSpeedHandle_M1);
	
  
  /******************************************************/
  /*   Main speed sensor component initialization       */
  /******************************************************/
  pPIDSpeed[M1] = &PIDSpeedHandle_M1;
  pSTC[M1] = &SpeednTorqCtrlM1;
	
  // STO_PLL_Init (&STO_PLL_M1);
	Abs_Encoder_Init(&Abs_Encoder_M1);
  
  /******************************************************/
  /*   Speed & torque component initialization          */
  /******************************************************/
  //STC_Init(pSTC[M1],pPIDSpeed[M1], &STO_PLL_M1._Super);
	STC_Init(pSTC[M1],pPIDSpeed[M1], &Abs_Encoder_M1._Super);
  
//  /****************************************************/
//  /*   Virtual speed sensor component initialization  */
//  /****************************************************/ 
//  VSS_Init (&VirtualSpeedSensorM1);
//  
//  /**************************************/
//  /*   Rev-up component initialization  */
//  /**************************************/
//  RUC_Init(&RevUpControlM1,pSTC[M1],&VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);  
      
  /********************************************************/
  /*   PID component initialization: current regulation   */
  /********************************************************/
  PID_HandleInit(&PIDIqHandle_M1);
  PID_HandleInit(&PIDIdHandle_M1);
  pPIDIq[M1] = &PIDIqHandle_M1;
  pPIDId[M1] = &PIDIdHandle_M1;
	
	//---Ken An Add---//
  PID_HandleInit(&PIDAngleHandle_M1); // Initial position PID	
	Abs_Encoder_Init(&Abs_Encoder_M1);	// Initial absolute encoder sensor
#if defined(ENCODER_SPI_MODE)
	Abs_Encoder_M1.mode = SENSOR_SPI;
#elif defined(ENCODER_PWM_MODE)  
	Abs_Encoder_M1.mode = SENSOR_PWM;
#endif
  //---End add---//
  
  /********************************************************/
  /*   Bus voltage sensor component initialization        */
  /********************************************************/
  pBusSensorM1 = &RealBusVoltageSensorParamsM1;
  RVBS_Init(pBusSensorM1);
  
  /*************************************************/
  /*   Power measurement component initialization  */
  /*************************************************/
  pMPM[M1] = &PQD_MotorPowMeasM1;
  pMPM[M1]->pVBS = &(pBusSensorM1->_Super);
  pMPM[M1]->pFOCVars = &FOCVars[M1];
  
  /*******************************************************/
  /*   Temperature measurement component initialization  */
  /*******************************************************/
  NTC_Init(&TempSensorParamsM1);    
  pTemperatureSensor[M1] = &TempSensorParamsM1;
    

  pREMNG[M1] = &RampExtMngrHFParamsM1;
  REMNG_Init(pREMNG[M1]);

  FOC_Clear(M1);
  FOCVars[M1].bDriveInput = EXTERNAL;
  FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
  FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).qI_Component2;
  oMCInterface[M1] = & Mci[M1];
  MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1]);
  MCI_ExecSpeedRamp(oMCInterface[M1],
  STC_GetMecSpeedRef01HzDefault(pSTC[M1]),0); /*First command to STC*/
  pMCIList[M1] = oMCInterface[M1];
  MCT[M1].pPIDSpeed = pPIDSpeed[M1];
  MCT[M1].pPIDIq = pPIDIq[M1];
  MCT[M1].pPIDId = pPIDId[M1];
  MCT[M1].pPIDFluxWeakening = MC_NULL; /* if M1 doesn't has FW */
  MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
  MCT[M1].pRevupCtrl = MC_NULL;// &RevUpControlM1;              /* only if M1 is sensorless*/
  MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *)&Abs_Encoder_M1 ;// &STO_PLL_M1; 
  MCT[M1].pSpeedSensorAux = MC_NULL;
  MCT[M1].pSpeedSensorVirtual = MC_NULL; // &VirtualSpeedSensorM1;  /* only if M1 is sensorless*/
  MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
  MCT[M1].pStateMachine = &STM[M1];
  MCT[M1].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M1];
  MCT[M1].pBusVoltageSensor = &(pBusSensorM1->_Super);
  MCT[M1].pBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM1*/
  MCT[M1].pNTCRelay = MC_NULL;             /* relay is defined, oRelayM1*/
  MCT[M1].pMPM =  (MotorPowMeas_Handle_t*)pMPM[M1];
  MCT[M1].pFW = MC_NULL;
  MCT[M1].pFF = MC_NULL;
  MCT[M1].pSCC = MC_NULL;
  MCT[M1].pOTT = MC_NULL;
  pMCTList[M1] = &MCT[M1];
 

  /* USER CODE BEGIN MCboot 2 */

  /* USER CODE END MCboot 2 */

  bMCBootCompleted = 1;
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is 
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task. 
 */
void MC_RunMotorControlTasks(void)
{
  if ( bMCBootCompleted ) {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();

    /* Safety task is run after Medium Frequency task so that  
     * it can overcome actions they initiated if needed. */
    TSK_SafetyTask();
    

    /* ** User Interface Task ** */
    UI_Scheduler();
  }
}

/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance. 
 *
 * It is to be clocked at the Systick frequency.
 */
void MC_Scheduler(void)
{
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

  if (bMCBootCompleted == 1)
  {    
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();
      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
      hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
    }
    if(hBootCapDelayCounterM1 > 0u)
    {
      hBootCapDelayCounterM1--;
    }
    if(hStopPermanencyCounterM1 > 0u)
    {
      hStopPermanencyCounterM1--;
    }
  }
  else
  {
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the 
  * present state of its state machine. In particular, duties requiring a periodic 
  * execution at a medium frequency rate (such as the speed controller for instance) 
  * are executed here.
  */
void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  State_t StateM1;
  int16_t wAux = 0;

  bool IsSpeedReliable ; //= STO_PLL_CalcAvrgMecSpeed01Hz( &STO_PLL_M1, &wAux );
  PQD_CalcElMotorPower( pMPM[M1] );
	
	
	//---Ken An add---//
	/* Calculate average speed in 0.1Hz, Also read angle from abs encoder */
#if defined(ABS_ENCODER_MODE)	
	Abs_Encoder_CalcAvrgMecSpeed01Hz(&Abs_Encoder_M1, &wAux );
	IsSpeedReliable = 1;
#else
	Abs_Encoder_CalcAvrgMecSpeed01Hz(&Abs_Encoder_M1, &wAux );
  IsSpeedReliable = STO_PLL_CalcAvrgMecSpeed01Hz( &STO_PLL_M1, &wAux );
#endif

	//---End add---//	
	

  StateM1 = STM_GetState( &STM[M1] );

  switch ( StateM1 )
  {
  case IDLE_START:
    R1VL1_TurnOnLowSides( pwmcHandle[M1] );
    TSK_SetChargeBootCapDelayM1( CHARGE_BOOT_CAP_TICKS );
    STM_NextState( &STM[M1], CHARGE_BOOT_CAP );
    break;

  case CHARGE_BOOT_CAP:
    if ( TSK_ChargeBootCapDelayHasElapsedM1() )
    {
      PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_START );

      /* USER CODE BEGIN MediumFrequencyTask M1 Charge BootCap elapsed */

      /* USER CODE END MediumFrequencyTask M1 Charge BootCap elapsed */

      STM_NextState(&STM[M1],OFFSET_CALIB);
    }
    break;

  case OFFSET_CALIB:
    if ( PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_EXEC ) )
    {
      STM_NextState( &STM[M1], CLEAR );
    }
    break;

  case CLEAR:
    /* In a sensorless configuration. Initiate the Revup procedure */
    //---Ken An add---//
#if defined(ABS_ENCODER_MODE)	
	FOCVars[M1].bDriveInput = INTERNAL;
	STC_SetSpeedSensor( pSTC[M1], &Abs_Encoder_M1._Super ); // Set sensor to abs encoder
#else
    FOCVars[M1].bDriveInput = EXTERNAL;
    STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
    RUC_Clear( &RevUpControlM1, MCI_GetImposedMotorDirection( oMCInterface[M1] ) );

    STO_PLL_Clear( &STO_PLL_M1 );
#endif
//---End add---//

    if ( STM_NextState( &STM[M1], START ) == true )
    {
      FOC_Clear( M1 );

      R1VL1_SwitchOnPWM( pwmcHandle[M1] );
    }
    break;

  case START:
		//---Ken An add---//
#if defined(ABS_ENCODER_MODE)
    STM_NextState( &STM[M1], START_RUN );
#else
    {
  
      /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
      int16_t hForcedMecSpeed01Hz;
      Curr_Components IqdRef;
      bool ObserverConverged = false;

      /* Execute the Rev Up procedure */
      if( ! RUC_Exec( &RevUpControlM1 ) )
      {
        /* The time allowed for the startup sequence has expired */
        STM_FaultProcessing( &STM[M1], MC_START_UP, 0 );  
      }
      else
      {
        /* Execute the torque open loop current start-up ramp:
         * Compute the Iq reference current as configured in the Rev Up sequence */
        IqdRef.qI_Component1 = STC_CalcTorqueReference( pSTC[M1] );
        IqdRef.qI_Component2 = FOCVars[M1].UserIdref;
        /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
        FOCVars[M1].Iqdref = IqdRef;
      }

      (void) VSS_CalcAvrgMecSpeed01Hz( &VirtualSpeedSensorM1, &hForcedMecSpeed01Hz );

      ObserverConverged = STO_PLL_IsObserverConverged( &STO_PLL_M1,hForcedMecSpeed01Hz );
      (void) VSS_SetStartTransition( &VirtualSpeedSensorM1, ObserverConverged );

      if ( ObserverConverged )
      {
        int16_t Iq = 0;
        Curr_Components StatorCurrent = MCM_Park( FOCVars[M1].Ialphabeta, SPD_GetElAngle( &STO_PLL_M1._Super ) );
        Iq = StatorCurrent.qI_Component1;

        /* Start switch over ramp. This ramp will transition from the revup to the closed loop FOC. */
        REMNG_Init( pREMNG[M1] );
        REMNG_ExecRamp( pREMNG[M1], FOCVars[M1].Iqdref.qI_Component1, 0 );
        REMNG_ExecRamp( pREMNG[M1], Iq, TRANSITION_DURATION );
        
        STM_NextState( &STM[M1], SWITCH_OVER );
      }
    }
#endif
//---Add end---//
    break;
    
#ifndef ABS_ENCODER_MODE

  case SWITCH_OVER:
    {
      bool LoopClosed;
      int16_t hForcedMecSpeed01Hz;
      
       
      if( ! RUC_Exec( &RevUpControlM1 ) )
      {
          /* The time allowed for the startup sequence has expired */
          STM_FaultProcessing( &STM[M1], MC_START_UP, 0 );  
      } 
      else
      { 
        /* Compute the virtual speed and positions of the rotor. 
           The function returns true if the virtual speed is in the reliability range */
        LoopClosed = VSS_CalcAvrgMecSpeed01Hz(&VirtualSpeedSensorM1,&hForcedMecSpeed01Hz);
        /* Check if the transition ramp has completed. */ 
        LoopClosed |= VSS_TransitionEnded( &VirtualSpeedSensorM1 );
        
        /* If any of the above conditions is true, the loop is considered closed. 
           The state machine transitions to the START_RUN state. */
        if ( LoopClosed == true ) 
        {
          #if ( PID_SPEED_INTEGRAL_INIT_DIV == 0 )  
          PID_SetIntegralTerm( pPIDSpeed[M1], 0 );
          #else
          PID_SetIntegralTerm( pPIDSpeed[M1],
                               (int32_t) ( FOCVars[M1].Iqdref.qI_Component1 * PID_GetKIDivisor(pPIDSpeed[M1]) /
                               PID_SPEED_INTEGRAL_INIT_DIV ) );
          #endif
          
          STM_NextState( &STM[M1], START_RUN );
        }  
      }
    }

    break;
		
#endif

  case START_RUN:
//---Ken an add---//
#if defined(ABS_ENCODER_MODE)
	
		FOC_InitAdditionalMethods(M1);
    FOC_CalcCurrRef( M1 );
    STM_NextState( &STM[M1], RUN );
		STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1] ); /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands( oMCInterface[M1] ); /* Exec the speed ramp after changing of the speed sensor */
	
#else		
 /* only for sensor-less control */
    STC_SetSpeedSensor(pSTC[M1], &STO_PLL_M1._Super); /*Observer has converged*/
    {
      /* USER CODE BEGIN MediumFrequencyTask M1 1 */

      /* USER CODE END MediumFrequencyTask M1 1 */      
	  FOC_InitAdditionalMethods(M1);
      FOC_CalcCurrRef( M1 );
      STM_NextState( &STM[M1], RUN );
    }
    STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1] ); /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands( oMCInterface[M1] ); /* Exec the speed ramp after changing of the speed sensor */
#endif	
//---Add end---//		
    break;

  case RUN:
    /* USER CODE BEGIN MediumFrequencyTask M1 2 */
#if defined(POSITION_CONTROL)
		//---Ken An add---//
		/* 获取误差位置/角度，单位为rad*10000 */
		Position_GetErrorAngle(&Abs_Encoder_M1,Target_Angle);
	
		/* 速度控制模式执行 */
		if(Position_M1.Mode_Flag == P_SPEED_MODE)
		{
			Position_M1.Set_Speed = Position_CalcSpeedReferrence();
		}

		/* 电流控制模式, 且切换到电流控制时第一次执行到此处 */
		if((Position_M1.Mode_Flag == P_TORQUE_MODE) && (Position_M1.Torque_First_Flag == 0))
		{
			qd_t temp_qd;
			/* 获取当前的电流参考值, Id, Iq */
			temp_qd = MC_GetIqdrefMotor1();
      /* 设置控制力矩 */
			MC_ProgramTorqueRampMotor1(temp_qd.q,0);
      /* 完成一次执行, 设置标志位 */
			Position_M1.Torque_First_Flag = 1;	
      /* 给位置环PID设置新的积分增益 */
			PID_SetIntegralTerm(&PIDAngleHandle_M1,0);
		}
	
		Position_M1.Speed_ACC_Count++;
		/* 在速度控制模式，计算速度加速时间，也执行速度斜坡 */
    /* 使用20ms的周期执行速度控制 */
		if((Position_M1.Speed_ACC_Count > SPEED_ACC_RATE) && (Position_M1.Mode_Flag == P_SPEED_MODE))
		{
			Position_M1.Speed_ACC_Count = 0;
			Position_M1.Control_Speed = MC_GetLastRampFinalSpeedMotor1(); // 获取速度值
			Position_M1.Real_Speed = MC_GetMecSpeedAverageMotor1();	// 获取平均速度
      /* 下面这条没看懂什么含义 */
			Position_M1.Duration_ms = (uint16_t)(Abs_Value(Position_M1.Control_Speed - Position_M1.Real_Speed) * Position_M1.Speed_ACC_Gain / Position_M1.Speed_ACC_Div);
			
      /* 设置速度斜率控制 */
			MC_ProgramSpeedRampMotor1(Position_M1.Set_Speed/6,Position_M1.Duration_ms);
		}
#endif
		//---End add---//	

    /* USER CODE END MediumFrequencyTask M1 2 */

    MCI_ExecBufferedCommands( oMCInterface[M1] );
    FOC_CalcCurrRef( M1 );
 
#ifndef ABS_ENCODER_MODE
    if( !IsSpeedReliable )
    {
      STM_FaultProcessing( &STM[M1], MC_SPEED_FDBK, 0 );
    }

#endif

    /* USER CODE BEGIN MediumFrequencyTask M1 3 */

    /* USER CODE END MediumFrequencyTask M1 3 */
    break;

  case ANY_STOP:
    R1VL1_SwitchOffPWM( pwmcHandle[M1] );
    FOC_Clear( M1 );
    MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M1] );
    TSK_SetStopPermanencyTimeM1( STOPPERMANENCY_TICKS );

    /* USER CODE BEGIN MediumFrequencyTask M1 4 */

    /* USER CODE END MediumFrequencyTask M1 4 */

    STM_NextState( &STM[M1], STOP );
    break;

  case STOP:
    if ( TSK_StopPermanencyTimeHasElapsedM1() )
    {
      STM_NextState( &STM[M1], STOP_IDLE );
    }
    break;

  case STOP_IDLE:

#ifndef ABS_ENCODER_MODE
	
    STC_SetSpeedSensor( pSTC[M1],&VirtualSpeedSensorM1._Super );  	/*  sensor-less */
    VSS_Clear( &VirtualSpeedSensorM1 ); /* Reset measured speed in IDLE */
	
#endif
	
    /* USER CODE BEGIN MediumFrequencyTask M1 5 */

    /* USER CODE END MediumFrequencyTask M1 5 */
    STM_NextState( &STM[M1], IDLE );
    break;

  default:
    break;
  }

  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */
  
  Curr_Components Inull = {(int16_t)0, (int16_t)0};
  Volt_Components Vnull = {(int16_t)0, (int16_t)0};

  FOCVars[bMotor].Iab = Inull;
  FOCVars[bMotor].Ialphabeta = Inull;
  FOCVars[bMotor].Iqd = Inull;
  FOCVars[bMotor].Iqdref = Inull;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = Vnull;
  FOCVars[bMotor].Valphabeta = Vnull;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
  PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_InitAdditionalMethods(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_CalcCurrRef(uint8_t bMotor)
{
    
  /* USER CODE BEGIN FOC_CalcCurrRef 0 */
	
	//---Ken An add---//
	if(FOCVars[bMotor].bDriveInput == INTERNAL)
	{
	#if defined(POSITION_CONTROL)
		/* If in position torque mode, Iqref come from Angle PID*/
		if(Position_M1.Mode_Flag == P_TORQUE_MODE)
		{
			FOCVars[bMotor].hTeref = Position_CalcTorqueReferrence();		
			FOCVars[bMotor].Iqdref.qI_Component1 = FOCVars[bMotor].hTeref;
		}
		else
	#endif
		{
			FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
			FOCVars[bMotor].Iqdref.qI_Component1 = FOCVars[bMotor].hTeref;
		}

  }
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control 
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
uint8_t TSK_HighFrequencyTask(void)
{
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */
  
  uint8_t bMotorNbr = 0;
  uint16_t hFOCreturn;
	
	//---Ken An add---//
	SPI_Encoder_EIAngle();
	Abs_Encoder_M1._Super.hElAngle = SPI_EIAngle;
	//---End add---//	
 
#ifndef ABS_ENCODER_MODE	
  uint16_t hState;  /*  only if sensorless main*/
  Observer_Inputs_t STO_Inputs; /*  only if sensorless main*/

  STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* only if sensorless*/
  if ( STM[M1].bState == SWITCH_OVER )
  {
    if (!REMNG_RampCompleted(pREMNG[M1]))
    {
      FOCVars[M1].Iqdref.qI_Component1 = REMNG_Calc(pREMNG[M1]);
    }
  }
#endif 
	
	
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
  hFOCreturn = FOC_CurrController(M1);
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
  if(hFOCreturn == MC_FOC_DURATION)
  {
    STM_FaultProcessing(&STM[M1], MC_FOC_DURATION, 0);
  }
  else
  {
		
#ifndef  ABS_ENCODER_MODE
    bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1); 
    STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /*  only if sensorless*/
    STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(pBusSensorM1->_Super)); /*  only for sensorless*/
    STO_PLL_CalcElAngle (&STO_PLL_M1, &STO_Inputs);
    STO_PLL_CalcAvrgElSpeedDpp (&STO_PLL_M1); /*  Only in case of Sensor-less */
	 if (IsAccelerationStageReached == false)
    {
      STO_ResetPLL(&STO_PLL_M1);
    }  
    hState = STM_GetState(&STM[M1]);
    if((hState == START) || (hState == SWITCH_OVER) || (hState == START_RUN)) /*  only for sensor-less*/
    {
      int16_t hObsAngle = SPD_GetElAngle(&STO_PLL_M1._Super);      
      VSS_CalcElAngle(&VirtualSpeedSensorM1,&hObsAngle);  
    }
#endif 
		
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

   /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */  
  }
  /* USER CODE BEGIN HighFrequencyTask 1 */

#if 0
	
	//---Ken An add---//
	/* DAC output*/
	extern UI_Handle_t * pDAC;
	static int16_t temp1;
	static int16_t temp2;
	
//	temp1 = MC_GetLastRampFinalSpeedMotor1();
//	temp2 = MC_GetMecSpeedAverageMotor1();
	
	temp1 = FOCVars[M1].hElAngle;
	temp2 = Abs_Encoder_M1.Encoder_EIAngle;	
	
  DAC_SetUserChannelValue(pDAC, DAC_USER1, temp1);
	DAC_SetUserChannelValue(pDAC, DAC_USER2, temp2);
	//---End add---//	

#endif	

  /* USER CODE END HighFrequencyTask 1 */

  return bMotorNbr;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_FOC_DURATION otherwise
  */
inline uint16_t FOC_CurrController(uint8_t bMotor)
{
  Curr_Components Iab, Ialphabeta, Iqd;
  Volt_Components Valphabeta, Vqd;
  int16_t hElAngle;
  uint16_t hCodeError;
	
	LED_1_GPIO_Port->ODR &= ~LED_1_Pin;
	
  hElAngle = SPD_GetElAngle(STC_GetSpeedSensor(pSTC[bMotor]));
  PWMC_GetPhaseCurrents(pwmcHandle[bMotor], &Iab);
	
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.qV_Component1 = PI_Controller(pPIDIq[bMotor],
            (int32_t)(FOCVars[bMotor].Iqdref.qI_Component1) - Iqd.qI_Component1);

  Vqd.qV_Component2 = PI_Controller(pPIDId[bMotor],
            (int32_t)(FOCVars[bMotor].Iqdref.qI_Component2) - Iqd.qI_Component2);
	
//	Vqd.qV_Component1 = 6553;
//	Vqd.qV_Component2 = 0;
	
  FOCVars[bMotor].Vqd = Vqd;
	
  Vqd = Circle_Limitation(pCLM[bMotor], Vqd);
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[bMotor], Valphabeta);
  FOCVars[bMotor].Iab = Iab;
  FOCVars[bMotor].Ialphabeta = Ialphabeta;
  FOCVars[bMotor].Iqd = Iqd;
  FOCVars[bMotor].Valphabeta = Valphabeta;
  FOCVars[bMotor].hElAngle = hElAngle;
	
	LED_1_GPIO_Port->ODR |= LED_1_Pin;
	
  return(hCodeError);
	
	
	
}

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances. 
  *
  * Faults flags are updated here.
  */
void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (bMCBootCompleted == 1)
  {  
    TSK_SafetyTask_PWMOFF(M1);
    /* User conversion execution */
    RCM_ExecUserConv ();
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
}

/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  
  uint16_t CodeReturn = MC_NO_ERROR;
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};

  CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);                    
																																							/* check for fault. It return MC_BREAK_IN or MC_NO_FAULTS 
                                                                                 (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  if(bMotor == M1)
  {
    CodeReturn |=  errMask[bMotor] &RVBS_CalcAvVbus(pBusSensorM1);
  }

  STM_FaultProcessing(&STM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  switch (STM_GetState(&STM[bMotor])) /* Acts on PWM outputs in case of faults */
  {
  case FAULT_NOW:
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
    FOC_Clear(bMotor);
    MPM_Clear((MotorPowMeas_Handle_t*)pMPM[bMotor]);
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
    break;
  case FAULT_OVER:
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
	/* USER CODE BEGIN TSK_SafetyTask_PWMOFF 2 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 2 */
    break;
  default:
    break;
  }
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

/**
  * @brief  This function returns the reference of the MCInterface relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCI_Handle_t * Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
MCI_Handle_t * GetMCI(uint8_t bMotor)
{
  MCI_Handle_t * retVal = MC_NULL;
  if ((oMCInterface != MC_NULL) && (bMotor < NBR_OF_MOTORS))
  {
    retVal = oMCInterface[bMotor];
  }
  return retVal;
}

/**
  * @brief  This function returns the reference of the MCTuning relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCT_Handle_t motor control tuning handler for the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
MCT_Handle_t* GetMCT(uint8_t bMotor)
{
  MCT_Handle_t* retVal = MC_NULL;
  if (bMotor < NBR_OF_MOTORS)
  {
    retVal = &MCT[bMotor];
  }
  return retVal;
}

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected  
  * by the microcontroller and is used to put the system in safety condition.
  */
void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */
  
  R1VL1_SwitchOffPWM(pwmcHandle[M1]);
  STM_FaultProcessing(&STM[M1], MC_SW_ERROR, 0);
  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}
 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration 
  */
void mc_lock_pins (void)
{
LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
//LL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
//LL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
//LL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
LL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
LL_GPIO_LockPin(M1_CURR_AMPL_GPIO_Port, M1_CURR_AMPL_Pin);
//LL_GPIO_LockPin(M1_TEMPERATURE_GPIO_Port, M1_TEMPERATURE_Pin);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
