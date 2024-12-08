#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "usmart.h"
#include "myiic.h"
#include "PWM.h"
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"

#include <float.h> // 用于FLT_MIN

#define FPS2HZ  0x02
#define FPS4HZ  0x03
#define FPS8HZ  0x04
#define FPS16HZ 0x05
#define FPS32HZ 0x06

#define MLX90640_ADDR   0x33
#define RefreshRate     FPS4HZ 
#define TA_SHIFT    8 //Default shift for MLX90640 in open air

static uint16_t eeMLX90640[832];  
static float mlx90640To[768];
uint16_t frame[834];
float emissivity=0.95;
int status;
paramsMLX90640 mlx90640;

uint16_t ccrx = 950;
uint16_t ccry = 1340;
double kp = 1;
double ki = 0;
double kd = 0.1;
double output_x = 0.0;
double output_y = 0.0;
PIDController pid_x, pid_y;

#define ROWS 24
#define COLS 32
#define SUBMATRIX_SIZE 2

int get1DIndex(int row, int col) {
    return row * COLS + col;
}

void findMaxValueAndPosition(float array[], int size, float *maxValue, int *maxPosition) {
    if (size <= 0) {
        printf("Array is empty.\n");
        return;
    }

    *maxValue = array[0];
    *maxPosition = 0;

    for (int i = 1; i < size; ++i) {
        if (array[i] > *maxValue) {
            *maxValue = array[i];
            *maxPosition = i;
        }
    }
}

int main(void)
{
    
    HAL_Init();                             //初始化HAL库    
    Stm32_Clock_Init(RCC_PLL_MUL9);         //设置时钟,72M
    delay_init(72);                         //初始化延时函数
    uart_init(115200);                      //初始化串口
    usmart_dev.init(84);                    //初始化USMART    
    LED_Init();                             //初始化LED    
    KEY_Init();                             //初始化按键
    TIM3_PWM_Init(20000-1,72-1);            //初始化PWM
    MOVE_Init();
    PID_Init(&pid_x, kp, ki, kd);
    PID_Init(&pid_y, kp, ki, kd);
    
    
    MLX90640_I2CInit();
    delay_ms(1);
    MLX90640_SetRefreshRate(MLX90640_ADDR, RefreshRate);  //设置帧率
    delay_ms(1);
    MLX90640_SetChessMode(MLX90640_ADDR);                 //棋盘模式   
    
    delay_ms(1);
    status = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);  //读取像素校正参数 
    printf("The function MLX90640_DumpEE return status is %d \n",status);
    delay_ms(1);
    if (status != 0) printf("load system parameters error with code:%d\r\n",status);
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);  //解析校正参数
    printf("MLX90640_ExtractParameters status is %d \n",status);
    delay_ms(1);
    if (status != 0) printf("Parameter extraction failed with error code:%d\r\n",status);

    float maxValue = 0;
    float maxDertaTem = 0;
    int maxPosition = 0;
    int maxDertaTemPosition = 0;
    int max_x = 0;
    int max_y = 0;
    

    while(1)
    {
        status = MLX90640_GetFrameData(MLX90640_ADDR, frame);  //读取一帧原始数据
        if (status < 0)
        {
            printf("GetFrame Error: %d\r\n",status);
        }
        float vdd = MLX90640_GetVdd(frame, &mlx90640);  //计算 Vdd（这句可有可无）
        float Ta = MLX90640_GetTa(frame, &mlx90640);    //计算实时外壳温度
        
        float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
        MLX90640_CalculateTo(frame, &mlx90640, emissivity , tr, mlx90640To);            //计算像素点温度



        findMaxValueAndPosition(mlx90640To, 768, &maxValue, &maxPosition);
        
        printf("Maximum value: %.2f\n", maxValue);
        max_x = maxPosition / 32;
        max_y = maxPosition % 32;
        printf("Position of maximum is: (%d , %d)\n", max_x, max_y);

        for(int i = 0;i<768;i++)
        {
            mlx90640To[i] = maxValue - mlx90640To[i];
        }
        findMaxValueAndPosition(mlx90640To, 768, &maxDertaTem, &maxDertaTemPosition);
        
        if(maxDertaTem > 10)
        {
            ccrx = ccrx - 7*PID_Update(&pid_x, max_x, 12);
            ccry = ccry - 7*PID_Update(&pid_y, max_y, 16);
            MOVEX(ccrx);
            MOVEY(ccry);
        }
        

//        int j=0;
//        for(int i = 0; i < 768; i++){
//            if(i%32 == 0 && i != 768){
//                printf("\n");
//                printf("%d:",j);
//                j++;
//            }else if(i == 0){
//                printf("%d:",j);
//            }
//            printf("%2.2f ",mlx90640To[i]);
//        }
    }
    
}
