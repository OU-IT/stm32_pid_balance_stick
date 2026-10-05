#include "stm32f4xx.h"

float dt = 0.005f;
volatile uint32_t ms_tick = 0;
mpu6050_data raw_data;
kalman_filter_struct k_s;
volatile pid_struct p_s_position;
volatile pid_struct p_s_speed;

// ---- 中断与主循环通信的全局变量 ----
volatile uint8_t tim3_flag = 0;   // 定时器中断标志

void SysTick_Handler(void) { ms_tick++; }

void TIM3_INIT(void);
void my_nvic_init(void);
void TIM3_IRQHandler(void);

int main(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);   // 1ms 中断

    my_nvic_init();
    USART1_Init();   //pa9->rxd pa10->txd
    TIM3_INIT();
		PWM_Init();

    // -------初始化 MPU6050 、卡尔曼滤波器------
    my_mpu6050_init();
    raw_data = mpu6050_get_raw_data();
    float accel_angle_x = get_angle_x(get_accel_data(raw_data.accel_y), get_accel_data(raw_data.accel_z));
    k_s.dt = dt;
    k_s.Q = 0.8f;
    k_s.R = 1.9f;
    k_s.P_prev = 100.0f;
		uint32_t now = ms_tick;
		while(ms_tick - now < 1000){};  //等待放稳校准
    k_s.X_prev = accel_angle_x;
    k_s.gyro_bias = calibrate_gyro_bias(500); //获取陀螺仪漂移量
		//---------初始化pid------------
		//--------位置环----------
		p_s_position.out_max = 200.f;
		p_s_position.out_min = -200.0f;
		p_s_position.K_P = 0.0f;
		p_s_position.K_I = 0.0f;
		p_s_position.K_D = 0.0f;
		p_s_position.i = 0.0f;
		p_s_position.expect = 80.0f;
		//-----速度环---------
		p_s_speed.dt = dt;
		p_s_speed.K_P = 0.0f;
		p_s_speed.K_I = 0.0f;
		p_s_speed.K_D = 0.0f;
		p_s_speed.out_max = 100.0f;
		p_s_speed.out_min = -100.0f;
		p_s_speed.i = 0.0f;
		p_s_speed.expect = 0.0f;
		p_s_speed.prev_speed = 0.0f;
		
			
    USART_Printf(USART1, "System Ready\r\n");   // 测试
		//------------主循环------------
    while(1) {
        if(tim3_flag) {                 // 检测到定时标志
            tim3_flag = 0;              // 清除标志

            // ---- 读取数据并转换单位 ----
            raw_data = mpu6050_get_raw_data();
            float accel_angle_x = get_angle_x(get_accel_data(raw_data.accel_y),get_accel_data(raw_data.accel_z));
            float gyro_aps = gyro_raw_to_angle_p_s(raw_data.gyro_x);
            k_s.accel_data = accel_angle_x;
            k_s.gyro_data = gyro_aps;
						// ---- 数据滤波 ----
            float filtered = kalman_filter(&k_s);
            //USART_Printf(USART1, "%f,%f\r\n", accel_angle_x, filtered);
						// ---- 前馈 ---------
						float K_ff = 70.0f;
						// ---- pid算法处理 ----
						//-----位置环------------
						p_s_position.cur_position = filtered;
						p_s_position.cur_speed = gyro_aps - k_s.gyro_bias;
						int pid_out_position = (int)pid_position(&p_s_position);
						// ---- 速度环 ----
						p_s_speed.expect = pid_out_position;
						p_s_speed.cur_speed = gyro_aps - k_s.gyro_bias;
						int pid_out = pid_speed(&p_s_speed);
						int out = pid_out + K_ff * cosf(((filtered) /180.0f)*3.1415926f);
						if(filtered > 95){out = 0;}
						if(out > 100){out = 100;}
						if(out < -100){out = -100;}
						// ---- 电机输出 ----
						//USART_Printf(USART1,"%f,%d\n",filtered,out);
						motor_speed(out);
        }
    }
}

// ---- TIM3 初始化 ----
void TIM3_INIT(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 83;
    TIM_TimeBaseInitStruct.TIM_Period = 5000 - 1;  //5ms
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

// ---- NVIC 配置 ----
void my_nvic_init(void) {
    NVIC_SetPriorityGrouping(2);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStruct);
}

// ---- TIM3 中断服务函数 ----
void TIM3_IRQHandler(void) {
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
				TIM_ClearFlag(TIM3,TIM_IT_Update);  // 清除标志
        tim3_flag = 1;    // 置位主循环标志
    }
}