#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "magnetometr.h"
#include "icm20948.h"
#include "gps.h"

#define LD2_PIN GPIO_PIN_5
#define LD2_GPIO_PORT GPIOA

#define RX_BUFFER_SIZE 12
volatile uint8_t rxBuffer[RX_BUFFER_SIZE];

GPIO_InitTypeDef GPIO_InitStruct;
UART_HandleTypeDef huart1, huart2, huart6;
I2C_HandleTypeDef hi2c1, hi2c2;
TIM_HandleTypeDef timer2, timer3, timer4;

DMA_HandleTypeDef hdma_usart1_rx;

float azs = 1.0f;
float accelScale = 0.0f;
const float accelRawScaling = 32767.5f;
const float G = 9.807f;

float gyroScale = 0.0f;
const float gyroRawScaling = 32767.5f; // =(2^16-1)/2 16 bit representation of gyro value to cover +/- range
const float _d2r = 3.14159265359f/180.0f; //degrees to radian/sec conversion

void init_core_clock(void);
void GPIO_Init(void);
void initMagnetometr(void);
void ak9916_magn_write_reg(uint8_t reg, uint8_t data);
void ak9916_magn_read_reg(uint8_t onset_reg, uint8_t len);

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART6_UART_Init(void);

static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);

volatile uint8_t rec[10];

volatile uint8_t magnReadFlag = 0;
volatile uint8_t accReadFlag = 0;
volatile uint8_t gyroReadFlag = 0;

volatile uint8_t gpsReceptionFlag = 0;
volatile uint8_t timerExpirationFlag = 0;

void DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdma_usart1_rx);

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);
}

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void USART1_IRQHandler(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_UART_IRQHandler(&huart1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
    	GPS_UART_CallBack();
    	gpsReceptionFlag = 1;
    }

    if(huart->Instance == USART2)
    {

    }
    if(huart->Instance == USART6)
    {

    }
}


void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&timer2);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    	timerExpirationFlag = 1;
    }
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&timer3);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&timer4);
}

int main(void)
{
	init_core_clock();

	HAL_Init();
	GPIO_Init();

	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();


	__USART2_CLK_ENABLE();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	__USART6_CLK_ENABLE();
    MX_USART6_UART_Init();

	MX_TIM2_Init();

	HAL_Delay(10);
	MX_I2C1_Init();
    HAL_Delay(10);

	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)ICM20948_ADDRESS << 1, 10, HAL_MAX_DELAY) != HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(500);
	}
	setUserBank(ub_0);
	uint8_t val = 0xc1;
	HAL_Delay(200);
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_PWR_MGMT_1, 1, 0xc1, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(200);
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_PWR_MGMT_1, 1, 0x01, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    HAL_Delay(200);
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_PWR_MGMT_2, 1, 0x00, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(100);
    setUserBank(ub_2);
    HAL_Delay(200);
    //output data rate start time alignment
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ODR_ALIGN_EN, 1, 0x01, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    HAL_Delay(200);
    //gyroscope configuration, sample rate divider = 0
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_GYRO_SMPLRT_DIV, 1, 0x00, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    HAL_Delay(200);


    const uint8_t degreesPerS250Lpf = 0b00000001;
    const uint8_t REG_B2_GYRO_CONFIG_1 = 0x01;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, REG_B2_GYRO_CONFIG_1, 1, degreesPerS250Lpf, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    gyroScale = 250.0f/gyroRawScaling * _d2r;

    HAL_Delay(200);
    //accelerometr configuration, sample rate divider = 0
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ACCEL_SMPLRT_DIV_1, 1, 0x00, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ACCEL_SMPLRT_DIV_2, 1, 0x00, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    const uint8_t accelConfig2gLpf = 0b00000001;
    const uint16_t REG_B2_ACCEL_CONFIG = 0x14;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, REG_B2_ACCEL_CONFIG, 1, accelConfig2gLpf, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    accelScale = G * 2.0f/accelRawScaling; // setting the accel scale to 2G

    HAL_Delay(300);
    //initMagnetometr();
    setUserBank(ub_0);
    initHMC5883L();

    HAL_Delay(200);

    setUserBank(ub_0);

	char cmd0[] = "AT\r\n";
	char cmd1[] = "AT+CWJAP=\"Nokia 8.3 5G\",\"a32448ed4674\"\r\n";
	char cmd2[] = "AT+CIPSTART=\"TCP\",\"192.168.7.18\",8081\r\n";

	GPS_Init();

	HAL_UART_Transmit(&huart6, (uint8_t*)cmd0, strlen(cmd0), HAL_MAX_DELAY);
	HAL_Delay(2000);
    HAL_UART_Transmit(&huart6, (uint8_t*)cmd1, strlen(cmd1), HAL_MAX_DELAY);
    HAL_Delay(6000);
    HAL_UART_Transmit(&huart6, (uint8_t*)cmd2, strlen(cmd2), HAL_MAX_DELAY);
    HAL_Delay(2000);

    while (1)
    {
    	if(timerExpirationFlag == 1)
    	{
			HAL_Delay(2);
			AccelData accelData = readAccData();
			HAL_Delay(2);
			GyroData gyroData = readGyroData();
			OrientationInSpace magnData = readRawDataFromMagnetometer();
			char measReadString[64];
			sprintf(measReadString, "%d_%d_%d_%d_%d_%d_%d_%d_%d_\r\n",
					accelData.xAcc, accelData.yAcc, accelData.zAcc,
					gyroData.xGyro, gyroData.yGyro, gyroData.zGyro,
					magnData.xAxis, magnData.yAxis, magnData.zAxis);


			if(HAL_UART_Transmit(&huart6, measReadString, strlen(measReadString), HAL_MAX_DELAY) != HAL_OK)
			{
				HAL_Delay(5000);
			}
			timerExpirationFlag = 0;
    	}

    	if(gpsReceptionFlag == 1)
    	{
    		char gpsReadString[64];
    		sprintf(gpsReadString, "%f_%f_%f_%f_%d\r\n",
    				gpsData.dec_longitude, gpsData.dec_latitude, gpsData.course_m, gpsData.course_t, gpsData.satelites);
    	   	if(HAL_UART_Transmit(&huart6, gpsReadString, strlen(gpsReadString), HAL_MAX_DELAY) != HAL_OK)
    	    {
    	    	 HAL_Delay(5000);
    	    }
    	}
    }
}

void init_core_clock(void)
{
  	SystemCoreClock = 16000000; //16MHz

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LD2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_PORT, &GPIO_InitStruct);
}

void ak9916_magn_write_reg(uint8_t reg, uint8_t data)
{
	setUserBank(ub_3);
	uint8_t tempData;
	tempData = AK09916_ADDRESS;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_ADDR, 1, AK09916_ADDRESS, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_REG, 1, reg, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_DO, 1, data, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    //enable single data write to the register
    tempData = 0x80|0x01;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_CTRL, 1, 0x80|0x01, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);
}

void ak9916_magn_read_reg(uint8_t onset_reg, uint8_t len)
{
	setUserBank(ub_3);
	uint8_t tempData;
	tempData = 0x80|AK09916_ADDRESS;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_ADDR, 1, (0x80|AK09916_ADDRESS) << 1, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_REG, 1, onset_reg, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    //enable single data write to the output register(s)
    tempData = 0x80|len;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_CTRL, 1, 0x80|len, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);
    setUserBank(ub_0);
}

void initMagnetometr(void)
{
	setUserBank(ub_0);
	uint8_t tempData;
	HAL_Delay(50);
    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDRESS << 1, B0_USER_CTRL, 1, &tempData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    //reset I2C master module
	HAL_Delay(50);
	tempData |= 0x02;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_USER_CTRL, 1, tempData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);
    //enable I2C master module
    tempData |= 0x20;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_USER_CTRL, 1, tempData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    //I2C master clock: 7 (400kHz)
    HAL_Delay(50);
    setUserBank(ub_3);

    tempData = 0x07;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_MST_CTRL, 1, tempData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);
    //LP CONFIG
    setUserBank(ub_0);
    tempData = 0x40;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_LP_CONFIG, 1, tempData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);

    setUserBank(ub_3);
    tempData = 0x03;
    //I2C_MST_ODR_CONFIG: 1.1kHz/(2^3) = 136Hz
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_MST_ODR_CONFIG, 1, tempData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);
    setUserBank(ub_0);

    //magnetometr reset
    ak9916_magn_write_reg(MAG_CNTL3, 0x01);
    HAL_Delay(100);
    //continouos mode 4: 100Hz
    ak9916_magn_write_reg(MAG_CNTL2, 0x08);
    setUserBank(ub_0);

}

void MX_USART1_UART_Init(void)
{


	GPIO_InitTypeDef gpio_uart1;
    gpio_uart1.Pin = GPIO_PIN_10;
	gpio_uart1.Mode = GPIO_MODE_AF_PP;
	gpio_uart1.Alternate = GPIO_AF7_USART1;
	gpio_uart1.Speed = GPIO_SPEED_HIGH;
	gpio_uart1.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio_uart1);

    __HAL_RCC_USART1_CLK_ENABLE();
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

    NVIC_SetPriority(USART1_IRQn, 0);
     NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(&huart1,  UART_IT_RXNE);
    HAL_UART_Receive_IT(&huart1, &rxBuffer, RX_BUFFER_SIZE);


}



void MX_USART2_UART_Init(void)
{
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //RX
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;

    HAL_UART_Init(&huart2);

}

void MX_USART6_UART_Init(void)
{
	//USART6_TX
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    //USART6_RX
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6);
}

static void MX_I2C1_Init(void)
{

	GPIO_InitTypeDef gpio_I2C1_SDA_SCL;
	gpio_I2C1_SDA_SCL.Pin = /*GPIO_PIN_4 |*/GPIO_PIN_8 | GPIO_PIN_9;
	gpio_I2C1_SDA_SCL.Mode = GPIO_MODE_AF_OD;
			// SCL, SDA
	gpio_I2C1_SDA_SCL.Pull = GPIO_PULLUP;
	gpio_I2C1_SDA_SCL.Alternate = GPIO_AF4_I2C1;
	gpio_I2C1_SDA_SCL.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &gpio_I2C1_SDA_SCL);

	__HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);

}

static void MX_I2C2_Init(void)
{

	GPIO_InitTypeDef gpio_I2C2_SDA_SCL;
	gpio_I2C2_SDA_SCL.Pin = GPIO_PIN_4;// |GPIO_PIN_10;// | GPIO_PIN_10;
	gpio_I2C2_SDA_SCL.Mode = GPIO_MODE_AF_OD;
			// SCL, SDA
	gpio_I2C2_SDA_SCL.Pull = GPIO_PULLUP;
	gpio_I2C2_SDA_SCL.Alternate = GPIO_AF4_I2C3;
	gpio_I2C2_SDA_SCL.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &gpio_I2C2_SDA_SCL);

	gpio_I2C2_SDA_SCL.Pin = /*GPIO_PIN_4 |*/GPIO_PIN_8;// | GPIO_PIN_10;
	HAL_GPIO_Init(GPIOA, &gpio_I2C2_SDA_SCL);

	__HAL_RCC_I2C3_CLK_ENABLE();

    hi2c1.Instance = I2C3;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);

}

static void MX_TIM2_Init(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();

	const uint16_t durationBetweenSendingTwoMeasurementsInMs = 50 * 10; // 50ms period

	timer2.Instance = TIM2;
	timer2.Init.Period = durationBetweenSendingTwoMeasurementsInMs - 1;
	timer2.Init.Prescaler = 1600 - 1;
	timer2.Init.ClockDivision = 0;
	timer2.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer2.Init.RepetitionCounter = 0;
	timer2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(&timer2);

	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_TIM_Base_Start_IT(&timer2);
}
