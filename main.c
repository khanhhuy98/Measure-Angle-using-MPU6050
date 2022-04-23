/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdlib.h>
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stm32g0xx_ll_usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

 //https://circuitdigest.com/microcontroller-projects/arduino-based-digital-protractor-using-mpu6050-gyroscope
 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCL_0         LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
#define SCL_1         LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
#define SDA_0         LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
#define SDA_1         LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
#define IICport       GPIOA
#define SCL           LL_GPIO_PIN_6
#define SDA           LL_GPIO_PIN_7

#define MPU6050_ADDR 							0xD0


#define SMPLRT_DIV_REG 						0x19
#define GYRO_CONFIG_REG 					0x1B
#define ACCEL_CONFIG_REG 					0x1C
#define ACCEL_XOUT_H_REG 					0x3B
#define ACCEL_YOUT_H_REG 					0x3D
#define ACCEL_ZOUT_H_REG 					0x3F
#define TEMP_OUT_H_REG 						0x41
#define GYRO_XOUT_H_REG 					0x43
#define PWR_MGMT_1_REG 						0x6B
#define WHO_AM_I_REG 							0x75



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static void  i2c_start(void);
static void  i2c_stop(void);
static void  wait_ack(void);
static void  i2c_writebyte(unsigned char a);
static unsigned char  i2c_readbyte(void);

static void SDAIN(void);
static void SDAOUT(void);
static void sendack(void);
static void sendnack(void);
static void i2c_writebit(unsigned char a);

void  i2c1_sw_acc_init(void);
void  i2c1_sw_acc_write(unsigned char sadd, unsigned char reg, unsigned char val);
uint8_t   i2c1_sw_acc_read(unsigned char sadd, unsigned char reg);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


static uint8_t accel_data[6];
static uint8_t gyro_data[6];
static uint8_t data[14];

static int16_t Accel_X_RAW = 0;
static int16_t Accel_Y_RAW = 0;
static int16_t Accel_Z_RAW = 0;

static int16_t temp_RAW = 0;

static int16_t Gyro_X_RAW = 0;
static int16_t Gyro_Y_RAW = 0;
static int16_t Gyro_Z_RAW = 0;

int16_t Gyro_X_CAL;
int16_t Gyro_Y_CAL;
int16_t Gyro_Z_CAL;

int16_t accel_total_vector;

int pos;

float Ax, Ay, Az, Gx, Gy, Gz, temp , angle_pit, angle_roll, accel_angle_pit, accel_angle_roll, angle_pit_output, angle_roll_output;

_Bool set_gyro_angles;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
* @brief send ack signal at i2c bus
*/
static void sendack(void)
{
    i2c_writebit(0);
}
/**
* @brief send nack signal at i2c bus
*/
static void sendnack(void)
{
    i2c_writebit(1);
}
/**
* @brief realese the SDA
*/
static void SDAIN(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;    
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
* @brief get the control of SDA
*/
static void SDAOUT(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void delay5us(void)
{
    unsigned short i;
    i=50;
    
    while(i--);
}

/**
* @brief send start signal at i2c bus
*/
static void i2c_start(void)
{
    SDAOUT();  
    SDA_1;
    delay5us();
    SCL_1;
    delay5us();
    SDA_0;
    delay5us();
    SCL_0;
    delay5us();
}

/**
* @brief send stop signal at i2c bus
*/
static void i2c_stop(void)
{
    SDAOUT();
    SDA_0;
    delay5us();
    SCL_1;
    delay5us();
    SDA_1;
    delay5us();
    SCL_0;
    delay5us();
}

/**
* @brief recieve ack signal at i2c bus
*/
static void  wait_ack(void)
{  
    int i;    

    SCL_0;
    delay5us();
    SDAIN();
    delay5us();
    SCL_1;
    delay5us();
    while (LL_GPIO_IsOutputPinSet(GPIOA, LL_GPIO_PIN_7)&&(i<0x2b0)) 
		{
        i++;
    }
    
    SCL_0;
    delay5us();
}

/**
* @brief write a byte at i2c bus
*
* @param the data to write
*/
static void  i2c_writebyte(unsigned char a)
{
    unsigned short i;
    
    SDAOUT();
    for (i=0; i<8; i++) {
        SCL_0;
        delay5us();
        if (a&0x80) {
            SDA_1;
        } else {
            SDA_0;
        }
        a = a<<1;
        delay5us();
        SCL_1;
        delay5us();
    }
}
/**
* @brief write a bit at i2c bus
*
* @param the data to write
*/
static void i2c_writebit(unsigned char a)
{
    SDAOUT();
    SCL_0;
    delay5us();
    if (a==0) {
        SDA_0;
    } else {
        SDA_1;
    }
    delay5us();
    SCL_1;
    delay5us();
    SCL_0;
    delay5us();
}


/**
* @brief read a byte at i2c bus
*
* @retval the data read
*/
static unsigned char i2c_readbyte()
{
    uint8_t i, temp;
    temp=0;
    SDAIN();
    SCL_0;      
    delay5us();
    for (i=0; i<8; i++) 
		{
        SCL_1;  
        delay5us();
        temp=(temp<<1)|LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_7);
        delay5us();
        SCL_0;  
        delay5us();
    }
    return temp;

}
/**
* @brief initial the i2c related gpios
*/
//void i2c1_sw_acc_init(void)
//{       
//    GPIO_InitTypeDef  GPIO_InitStructure;

//    __GPIOB_CLK_ENABLE();
//    GPIO_InitStructure.Pin = GPIO_PIN_6|GPIO_PIN_7;
//    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
//    GPIO_InitStructure.Pull = GPIO_NOPULL;
//    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH; //GPIO_SPEED_HIGH;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

//    SDA_1;
//    SCL_1;
//}

/**
* @brief write data at certain address on MPU6050
*
* @param add the in-chip address of the data
* @param Achar the data to write
*/
void  i2c1_sw_acc_write(unsigned char sadd, unsigned char reg, unsigned char val)
{
    i2c_start();
    i2c_writebyte(sadd);
    wait_ack();
    i2c_writebyte(reg);
    wait_ack();
    i2c_writebyte(val);
    wait_ack();
    i2c_stop();
}
/**
* @brief read data from certain address of MPU6050
*
* @param add the in-chip address of the data
* @retval the data read
*/
uint8_t  i2c1_sw_acc_read(unsigned char sadd, unsigned char reg)
{
    uint8_t temp;
    
    i2c_start();
    i2c_writebyte(sadd);
    wait_ack();
    delay5us();
    i2c_writebyte(reg);
    wait_ack();
    i2c_start();
    i2c_writebyte(sadd+1);
    wait_ack();
    temp = i2c_readbyte();
    sendnack();
    i2c_stop();
    
    return temp;
}
/**
* @brief read mutilple data from certain address of MPU6050
*
* @param add the in-chip address of the data
* @retval the data read
*/
void I2C_receive(uint8_t sadd, uint8_t reg, uint8_t data[], uint8_t size)
{

   
    i2c_start();
    i2c_writebyte(sadd);
    wait_ack();
    delay5us();
    i2c_writebyte(reg);
    wait_ack();
    i2c_start();
    i2c_writebyte(sadd+1);
    wait_ack();
		for (int i = 0;  i < size-1; i++)
		{
			data[i] = i2c_readbyte();
			sendack();
		}
		data[size-1] = i2c_readbyte();
    sendnack();
    i2c_stop();
    
}
// MPU6050 config register
void MPU6050_Config()
{
	uint8_t whoamI;
	whoamI = i2c1_sw_acc_read(MPU6050_ADDR,WHO_AM_I_REG);
	if(whoamI == 0x68)
	{
		i2c1_sw_acc_write(MPU6050_ADDR, PWR_MGMT_1_REG, 0x00);
		i2c1_sw_acc_write(MPU6050_ADDR, SMPLRT_DIV_REG, 0x07);
		i2c1_sw_acc_write(MPU6050_ADDR,ACCEL_CONFIG_REG,0x10);
		i2c1_sw_acc_write(MPU6050_ADDR,GYRO_CONFIG_REG,0x08);
	}
}
// Read data accelerometer
void MPU6050_Read_Accel (void)
{


	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	I2C_receive(MPU6050_ADDR, ACCEL_XOUT_H_REG, accel_data, 6);
		
		Accel_X_RAW = (int16_t)(accel_data[0] << 8 | accel_data [1]);
		Accel_Y_RAW = (int16_t)(accel_data[2] << 8 | accel_data [3]);
		Accel_Z_RAW = (int16_t)(accel_data[4] << 8 | accel_data [5]);
		
	
	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/
		Ax = Accel_X_RAW/16384.0;
		Ay = Accel_Y_RAW/16384.0;
		Az = Accel_Z_RAW/16384.0;
		printf("Ax = %f, Ay = %f, Az = %f \n\r", Ax,Ay,Az);

}
// Read data gyro
void MPU6050_Read_Gyro (void)
{
	

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	I2C_receive(MPU6050_ADDR, GYRO_XOUT_H_REG, gyro_data, 6);

	Gyro_X_RAW = (int16_t)(gyro_data[0] << 8 | gyro_data [1]);
	Gyro_Y_RAW = (int16_t)(gyro_data[2] << 8 | gyro_data [3]);
	Gyro_Z_RAW = (int16_t)(gyro_data[4] << 8 | gyro_data [5]);

	/*** convert the RAW values into dps (?/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
	printf("Gx = %f, Gy = %f, Gz = %f \n\r", Gx,Gy,Gz);
}
//Read data Accelerometer + temperature + gyro
void MPU6050_Read_All()
{
	I2C_receive(MPU6050_ADDR, ACCEL_XOUT_H_REG, data, 14);
		
		Accel_X_RAW = (int16_t)(data[0] << 8 | data [1]);
		Accel_Y_RAW = (int16_t)(data[2] << 8 | data [3]);
		Accel_Z_RAW = (int16_t)(data[4] << 8 | data [5]);
		
		temp_RAW = (int16_t)(data[6] << 8 | data [7]);
		
		Gyro_X_RAW = (int16_t)(data[8] << 8 | data [9]);
		Gyro_Y_RAW = (int16_t)(data[10] << 8 | data [11]);
		Gyro_Z_RAW = (int16_t)(data[12] << 8 | data [13]);
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int minVal=265;
	int maxVal=402;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	MPU6050_Config();
	
	LL_mDelay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		MPU6050_Read_All();
		// map those values from 265 to 402 as -90 to 90
		
	  int xAng = map(Accel_X_RAW,minVal,maxVal,-90,90);
    int yAng = map(Accel_Y_RAW,minVal,maxVal,-90,90);
    int zAng = map(Accel_Z_RAW,minVal,maxVal,-90,90);
		
		//Convert radians to degree
		Ax =  (atan2(-yAng, -zAng)+3.1415) * 180 / 3.1415;  
		Ay =  (atan2(-xAng, -zAng)+3.1415) * 180 / 3.1415;  
		
		Ax = map(Ax,0,180,0,180);
		Ay = map(Ay,0,180,0,180);
		printf("AngleX = %.1f \n\r", Ax);
		printf("AngleY = %.1f \n\r", Ay);
		LL_mDelay(1000);
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
  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(16000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(16000000);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB8   ------> I2C1_SCL
  PB9   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  /** I2C Initialization
  */
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00303D5B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);

  /**/
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  while(!LL_USART_IsActiveFlag_TXE(USART2)){;}
	/* Send byte through the UART peripheral */
	USART2->TDR = ch;
	return ch;
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

