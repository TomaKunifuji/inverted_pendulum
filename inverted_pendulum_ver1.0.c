//------------------------------------//
// PB7(i2c_SDA)
// PB6(i2c_SCL)
// PA3(uart_read)
// PA2(uart_write)
//------------------------------------//
#include <stm32f10x.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>


#define	BNO055_SLAVEADDRESS_A 			0x28
#define	BNO055_SLAVEADDRESS_B 			0x29
#define	BNO055_ID 		 				0xA0
	
	// Power mode settings
#define	POWER_MODE_NORMAL   			0X00
#define	POWER_MODE_LOWPOWER 			0X01
#define	POWER_MODE_SUSPEND  			0X02
	
	// Operation mode settings
#define	OPERATION_MODE_CONFIG 			0X00
#define	OPERATION_MODE_ACCONLY 			0X01
#define	OPERATION_MODE_MAGONLY 			0X02
#define	OPERATION_MODE_GYRONLY 			0X03
#define	OPERATION_MODE_ACCMAG 			0X04
#define	OPERATION_MODE_ACCGYRO 			0X05
#define	OPERATION_MODE_MAGGYRO 			0X06
#define	OPERATION_MODE_AMG 				0X07
#define	OPERATION_MODE_IMUPLUS 			0X08
#define	OPERATION_MODE_COMPASS 			0X09
#define	OPERATION_MODE_M4G 				0X0A
#define	OPERATION_MODE_NDOF_FMC_OFF 	0X0B
#define	OPERATION_MODE_NDOF 			0X0C
	
	// Output vector type
#define	VECTOR_ACCELEROMETER 			0x08
#define	VECTOR_MAGNETOMETER  			0x0E
#define	VECTOR_GYROSCOPE     			0x14
#define	VECTOR_EULER         			0x1A
#define	VECTOR_LINEARACCEL   			0x28
#define	VECTOR_GRAVITY       			0x2E
	
	// REGISTER DEFINITION START
#define	BNO055_PAGE_ID_ADDR 			0X07
#define	BNO055_PAGE_0_ADDR	 			0X00
#define	BNO055_PAGE_1_ADDR	 			0X01

#define	BNO055_CHIP_ID_ADDR 			0x00
#define	BNO055_ACCEL_REV_ID_ADDR 		0x01
#define	BNO055_MAG_REV_ID_ADDR 			0x02
#define	BNO055_GYRO_REV_ID_ADDR 		0x03
#define	BNO055_SW_REV_ID_LSB_ADDR 		0x04
#define	BNO055_SW_REV_ID_MSB_ADDR 		0x05
#define	BNO055_BL_REV_ID_ADDR 			0X06
	
	// Accel data register 
#define	BNO055_ACCEL_DATA_X_LSB_ADDR 	0X08
#define	BNO055_ACCEL_DATA_X_MSB_ADDR 	0X09
#define	BNO055_ACCEL_DATA_Y_LSB_ADDR 	0X0A
#define	BNO055_ACCEL_DATA_Y_MSB_ADDR 	0X0B
#define	BNO055_ACCEL_DATA_Z_LSB_ADDR 	0X0C
#define	BNO055_ACCEL_DATA_Z_MSB_ADDR 	0X0D
	
	// Mag data register 
#define	BNO055_MAG_DATA_X_LSB_ADDR 		0X0E
#define	BNO055_MAG_DATA_X_MSB_ADDR 		0X0F
#define	BNO055_MAG_DATA_Y_LSB_ADDR 		0X10
#define	BNO055_MAG_DATA_Y_MSB_ADDR 		0X11
#define	BNO055_MAG_DATA_Z_LSB_ADDR 		0X12
#define	BNO055_MAG_DATA_Z_MSB_ADDR		0X13
	
	// Gyro data registers 
#define	BNO055_GYRO_DATA_X_LSB_ADDR 	0X14
#define	BNO055_GYRO_DATA_X_MSB_ADDR 	0X15
#define	BNO055_GYRO_DATA_Y_LSB_ADDR 	0X16
#define	BNO055_GYRO_DATA_Y_MSB_ADDR 	0X17
#define	BNO055_GYRO_DATA_Z_LSB_ADDR 	0X18
#define	BNO055_GYRO_DATA_Z_MSB_ADDR 	0X19
	
	// Euler data registers 
#define	BNO055_EULER_H_LSB_ADDR 		0X1A
#define	BNO055_EULER_H_MSB_ADDR 		0X1B
#define	BNO055_EULER_R_LSB_ADDR 		0X1C
#define	BNO055_EULER_R_MSB_ADDR 		0X1D
#define	BNO055_EULER_P_LSB_ADDR 		0X1E
#define	BNO055_EULER_P_MSB_ADDR 		0X1F
	
	// Quaternion data registers 
#define	BNO055_QUATERNION_DATA_W_LSB_ADDR 		0X20
#define	BNO055_QUATERNION_DATA_W_MSB_ADDR 		0X21
#define	BNO055_QUATERNION_DATA_X_LSB_ADDR 		0X22
#define	BNO055_QUATERNION_DATA_X_MSB_ADDR 		0X23
#define	BNO055_QUATERNION_DATA_Y_LSB_ADDR 		0X24
#define	BNO055_QUATERNION_DATA_Y_MSB_ADDR 		0X25
#define	BNO055_QUATERNION_DATA_Z_LSB_ADDR 		0X26
#define	BNO055_QUATERNION_DATA_Z_MSB_ADDR 		0X27
	
	// Linear acceleration data registers 
#define	BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR		0X28
#define	BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR		0X29
#define	BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR		0X2A
#define	BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR		0X2B
#define	BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR		0X2C
#define	BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR		0X2D
	
	// Gravity data registers 
#define	BNO055_GRAVITY_DATA_X_LSB_ADDR 	0X2E
#define	BNO055_GRAVITY_DATA_X_MSB_ADDR	0X2F
#define	BNO055_GRAVITY_DATA_Y_LSB_ADDR 	0X30
#define	BNO055_GRAVITY_DATA_Y_MSB_ADDR 	0X31
#define	BNO055_GRAVITY_DATA_Z_LSB_ADDR 	0X32
#define	BNO055_GRAVITY_DATA_Z_MSB_ADDR 	0X33
	
	// Temperature data register 
#define	BNO055_TEMP_ADDR 				0X34
	
	// Status registers 
#define	BNO055_CALIB_STAT_ADDR 			0X35
#define	BNO055_SELFTEST_RESULT_ADDR	 	0X36
#define	BNO055_INTR_STAT_ADDR 			0X37
	
#define	BNO055_SYS_CLK_STAT_ADDR 		0X38
#define	BNO055_SYS_STAT_ADDR 			0X39
#define	BNO055_SYS_ERR_ADDR 			0X3A

#define BNO055_CALIB_CHECK_ADDR			0XFF
	
	// Unit selection register 
#define	BNO055_UNIT_SEL_ADDR 			0X3B
#define	BNO055_DATA_SELECT_ADDR 		0X3C
	
	// Mode registers 
#define	BNO055_OPR_MODE_ADDR 			0X3D
#define	BNO055_PWR_MODE_ADDR 			0X3E
	
#define	BNO055_SYS_TRIGGER_ADDR 		0X3F
#define	BNO055_TEMP_SOURCE_ADDR 		0X40
	
	// Axis remap registers 
#define	BNO055_AXIS_MAP_CONFIG_ADDR 	0X41
#define	BNO055_AXIS_MAP_SIGN_ADDR 		0X42
	
	// SIC registers 
#define	BNO055_SIC_MATRIX_0_LSB_ADDR 	0X43
#define	BNO055_SIC_MATRIX_0_MSB_ADDR 	0X44
#define	BNO055_SIC_MATRIX_1_LSB_ADDR 	0X45
#define	BNO055_SIC_MATRIX_1_MSB_ADDR 	0X46
#define	BNO055_SIC_MATRIX_2_LSB_ADDR 	0X47
#define	BNO055_SIC_MATRIX_2_MSB_ADDR 	0X48
#define	BNO055_SIC_MATRIX_3_LSB_ADDR 	0X49
#define	BNO055_SIC_MATRIX_3_MSB_ADDR 	0X4A
#define	BNO055_SIC_MATRIX_4_LSB_ADDR 	0X4B
#define	BNO055_SIC_MATRIX_4_MSB_ADDR 	0X4C
#define	BNO055_SIC_MATRIX_5_LSB_ADDR 	0X4D
#define	BNO055_SIC_MATRIX_5_MSB_ADDR 	0X4E
#define	BNO055_SIC_MATRIX_6_LSB_ADDR 	0X4F
#define	BNO055_SIC_MATRIX_6_MSB_ADDR 	0X50
#define	BNO055_SIC_MATRIX_7_LSB_ADDR 	0X51
#define	BNO055_SIC_MATRIX_7_MSB_ADDR 	0X52
#define	BNO055_SIC_MATRIX_8_LSB_ADDR 	0X53
#define	BNO055_SIC_MATRIX_8_MSB_ADDR 	0X54
	
	// Accelerometer Offset registers
#define	ACCEL_OFFSET_X_LSB_ADDR 		0X55
#define	ACCEL_OFFSET_X_MSB_ADDR 		0X56
#define	ACCEL_OFFSET_Y_LSB_ADDR 		0X57
#define	ACCEL_OFFSET_Y_MSB_ADDR 		0X58
#define	ACCEL_OFFSET_Z_LSB_ADDR 		0X59
#define	ACCEL_OFFSET_Z_MSB_ADDR 		0X5A
	
	// Magnetometer Offset registers
#define	MAG_OFFSET_X_LSB_ADDR 			0X5B
#define	MAG_OFFSET_X_MSB_ADDR 			0X5C
#define	MAG_OFFSET_Y_LSB_ADDR 			0X5D
#define	MAG_OFFSET_Y_MSB_ADDR 			0X5E
#define	MAG_OFFSET_Z_LSB_ADDR 			0X5F
#define	MAG_OFFSET_Z_MSB_ADDR 			0X60
	
	// Gyroscope Offset registers
#define	GYRO_OFFSET_X_LSB_ADDR 			0X61
#define	GYRO_OFFSET_X_MSB_ADDR 			0X62
#define	GYRO_OFFSET_Y_LSB_ADDR 			0X63
#define	GYRO_OFFSET_Y_MSB_ADDR 			0X64
#define	GYRO_OFFSET_Z_LSB_ADDR 			0X65
#define	GYRO_OFFSET_Z_MSB_ADDR 			0X66
	
	// Radius registers
#define	ACCEL_RADIUS_LSB_ADDR 			0X67
#define	ACCEL_RADIUS_MSB_ADDR 			0X68
#define	MAG_RADIUS_LSB_ADDR 			0X69
#define	MAG_RADIUS_MSB_ADDR 			0X6A
	
	// REGISTER DEFINITION END


	// System settings
#define TIMEOUT 100000

	//moter setting

#define SW0 0x2000		//USER_swith(PC13)
#define Sensor0 0x13	//USE_PA0~1,PA4

#define Base_Speed 6000
#define Low_Base_Speed 5000
#define BeLow_Base_Speed 3000
#define Low_Speed 2500


#define Delay_time 10


//Function

	//init
void hard_init(void);
void usart2_init(void);
void i2c_init(void);
int BNO055_init(void);

	//i2c
int wait_for_flag(volatile uint16_t* reg, uint16_t flag);
uint8_t i2c_read_BNO055(uint16_t sl_adr, uint8_t radr);
int i2c_write_BNO055(uint16_t sl_adr, uint8_t wadr, uint8_t addr);

	//usart
void convert_roll_to_ascii(float roll_angle, char *ascii_str_float);
void usart2_tx(char* out_str);
void usart2_tx_char(char ascii);
void uint32_to_char_array(uint32_t num, char *str);

	//moter
void Tim3_Init(void);
unsigned int Get_SW(void);		// スイッチの値を取得する関数
void Right_Motor_Run(int);
void Right_BackMotor_Run(int);
void Left_Motor_Run(int);
void Left_BackMotor_Run(int);
void Back_Motor(int speed);
void Move_Motor(int speed);
void Motor_Reset(void);
void delay_ms(uint32_t ms);



	//Global　variable
int cnt;
int flg_1s;
int flg_2s;
int cnt2;
int flg_1s2;
int i2c_check;
int flg_timeout = 0;
bool error1 = false;


unsigned int Up = 0;   	// 表示タイミング及びスイッチ押下を示すフラグ変数
unsigned int Senser = 0;// 表示タイミング及びスイッチ押下を示すフラグ変数
unsigned int motor = 0;
volatile uint32_t tick = 0;

int SENSER = 0;
int CURRENT_STATE_TIME = 0;
int Stop_state = 1;
int Run_state = 2;
int Little_Right_state = 3;
int Right_state = 4;
int Little_Left_state = 5;
int Left_state = 6;
int Back_state = 7;
int Cross_state = 8;
int next_state = 0;
int a = 0;
int b = 0;


uint16_t data = 0x5555;
uint8_t data1;
uint8_t data0;
uint8_t chip_id;
uint8_t ACC_DATA_Z_MSB;
uint8_t ACC_DATA_Z_LSB;
uint8_t eul_data_p_msb;
uint8_t eul_data_p_lsb;
uint8_t eul_data_r_msb;
uint8_t eul_data_r_lsb;
uint8_t eul_data_h_msb;
uint8_t eul_data_h_lsb;
int16_t eul_data_r;
int eul_udata_r = 0x0;
int16_t eul_data_p;
int eul_udata_p = 0x0;
float pitch_angle = 0;
uint32_t duty_cycle = 0;

// 定数設定 ===============
float	KP = 0.025f;  // Pゲイン :誤差が大きければ大きいほど、比例して制御量が増加します。これにより、目標に向かう速度が早くなる
float	KD = 0.002f;   // Dゲイン :誤差の変化速度を減少させ、システムの安定性を向上させる
float	KI = 0.005;   // Iゲイン　:誤差が長時間続くと、修正が蓄積され、誤差をゼロに近づける
float	T = 0.1f; // 制御周期[秒]
float	R = 93.0f;

// 変数初期化 ===============
float integral_error = 0.0f;   // 積分の近似計算のための初期値設定(対象に応じて調整)
float prev_angle = 0.0f;      // 前回の誤差 = r - y; // 誤差を計算



char ascii_data[4];
char ascii_str1;
char ascii_float[1];



int main()
{
	hard_init();
	usart2_init();//USART1の初期設定
	i2c_init();
	BNO055_init();
	Tim3_Init();

	while(1){
		if(flg_1s == 1){             //0.1秒毎に
			flg_1s = 0;


			convert_roll_to_ascii(pitch_angle,ascii_float);			//ASCIIに変換
			//USART出力
			if(eul_udata_p == 0){									//符号入力
				usart2_tx("+");
			}else{
				usart2_tx("-");
			}
			usart2_tx(ascii_float);									//角度出力
			usart2_tx("--");
			uint32_to_char_array(duty_cycle,ascii_float);			//デューティサイクルをASCII変換
			usart2_tx(ascii_float);									//デューティサイクル値出力

			usart2_tx("\r");
			usart2_tx("\n");

		}

		//モーター始動
		if (Up & GPIO_IDR_IDR13) {
		Up &= ~GPIO_IDR_IDR13;

			if (motor == 0) {
				motor = 1;
			}else {					//モーター停止
				motor = 0;
				Motor_Reset();
			}
		}

		if(flg_2s == 1){             //0.1秒毎に
			flg_2s = 0;
			if (motor == 1){

				//一定角度範囲内は別挙動
				float absolute_angle = fabs(pitch_angle - R);  // Rからのズレの絶対値を計算
				if(absolute_angle < 30) {

					if (absolute_angle > 17) {
						duty_cycle = Base_Speed;
					} else if (absolute_angle > 7) {
						duty_cycle = Low_Base_Speed;
					} else if (absolute_angle > 4) {
						duty_cycle = BeLow_Base_Speed;
					} else if (absolute_angle > 1) {
						duty_cycle = Low_Speed;
					}

					if(pitch_angle > R){
						Back_Motor(0);
						if(a == 0){
							Move_Motor(duty_cycle);
							delay_ms(2);
							Move_Motor(0);
							delay_ms(10);

						}
						Move_Motor(duty_cycle);

					}else if(pitch_angle < R){
						Move_Motor(0);
						if(b == 0){
							Back_Motor(duty_cycle);
							delay_ms(2);
							Back_Motor(0);
							delay_ms(10);

						}
						Back_Motor(duty_cycle);
					}
				}else{

					while(flg_2s == 1){
						flg_2s = 0;

						// P, I, Dそれぞれの項を計算
						float p_term = KP * absolute_angle;
//			    		integral_error += absolute_angle * T;  // 積分項の計算
//			    		float i_term = KI * integral_error;
//			    		float d_term = KD * (prev_angle - absolute_angle) / T;  // 微分項の計算
						float control_signal = (p_term);
//						float control_signal = (p_term + i_term + d_term);

						// デューティサイクルの範囲を0〜100%に制限
						if (control_signal > 100) {
							control_signal = 100;
						} else if (control_signal < 0) {
							control_signal = 0;
						}
						duty_cycle = (uint32_t)(control_signal * 9000);
//						prev_angle = absolute_angle;	//今回の値を保存

						//モーター動作
						if(pitch_angle > R){
							Back_Motor(0);
							Move_Motor(duty_cycle);
						}else if(pitch_angle < R){
							Move_Motor(0);
							Back_Motor(duty_cycle);
						}
					}
				}
			}
		}
	}
}


//--------------moter--------------------//
void delay_ms(uint32_t ms) {
    tick = ms;
    while (tick != 0);
}

void Back_Motor(int speed){
	Right_Motor_Run(speed);
	Left_BackMotor_Run(speed);
}

void Move_Motor(int speed){
	Right_BackMotor_Run(speed);
	Left_Motor_Run(speed);
}

void Right_Motor_Run(int speed)
{
	TIM3->CCR3 =speed;
}

void Right_BackMotor_Run(int speed)
{
	TIM3->CCR4 =speed;
}

void Left_Motor_Run(int speed)
{
	TIM3->CCR1 =speed;
}

void Left_BackMotor_Run(int speed)
{
	TIM3->CCR2 =speed;
}

void Motor_Reset(void)
{
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
}

unsigned int Get_SW(void)
{
	unsigned int sw;

	sw = ~GPIOC->IDR;				// スイッチの値を（負論理なので）ビット反転して読み込む
	sw = sw &  GPIO_IDR_IDR13;;		// スイッチ以外のビットをマスクする

	return sw;
}


//--------------------i2c--------------------------//

//I2C読込
uint8_t i2c_read_BNO055(uint16_t sl_adr, uint8_t radr)
{
	uint32_t	status;
	uint8_t		rdata0;
	uint8_t		rdata1;
	uint8_t		rdata;

	//スレーブアドレス書き込み
	I2C1->CR1 |= I2C_CR1_ACK;								//ACK送信設定
	I2C1 -> CR1 |= I2C_CR1_START;							//スタートコンデション生成
	if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_SB)) return 1;		//スタートコンデションが生成されるまで待機（SR1_SBが1の時生成判定、続いてDRレジスタ書き込みによってクリア）

	I2C1 -> DR = (sl_adr<<1);								//スレーブアドレス書き込み
	if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_ADDR)) return 2;	//スレーブアドレス書き込みまで待機（SR1_ADDRが1の時書き込み完了判定）
    status = I2C1->SR2;										//レジスタクリア

    I2C1->DR = radr;										//読み込みたいレジスタのアドレス
    if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_BTF)) return 3;
    I2C1 -> CR1 |= I2C_CR1_STOP;

    //該当アドレスのデータ受信
	I2C1 -> CR1 |= I2C_CR1_START;							//スタートコンデション生成
	if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_SB)) return 4;		//スタートコンデションが生成されるまで待機（SR1_SBが1の時生成判定、続いてDRレジスタ書き込みによってクリア）

	I2C1 -> DR = (sl_adr<<1) | 0x1;							//スレーブアドレス読込
	if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_ADDR)) return 5;	//スレーブアドレス読込まで待機（SR1_ADDRが1の時書き込み完了判定）
    status = I2C1 -> SR2;									//レジスタクリア

    I2C1 -> CR1 &= ~I2C_CR1_ACK;							//次が最後の通信
    if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_RXNE)) return 6;	//データ受信待機
    rdata0 = I2C1 -> DR;									//データ読込

    I2C1 -> CR1 |= I2C_CR1_STOP;
    if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_RXNE)) return 7;
    rdata1 = I2C1 -> DR;

    rdata = rdata0;
    return rdata;
}

//I2C書込
int i2c_write_BNO055(uint16_t sl_adr, uint8_t wadr, uint8_t addr)
{
	uint32_t	status;

	I2C1 -> CR1 |= I2C_CR1_START;							//スタートコンデション生成
	if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_SB)) return 1;		//スタートコンデションが生成されるまで待機（SR1_SBが1の時生成判定、続いてDRレジスタ書き込みによってクリア）

	I2C1 -> DR = (sl_adr<<1);								//スレーブアドレス書き込み
	if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_ADDR)) return 2;	//スレーブアドレス書き込みまで待機（SR1_ADDRが1の時書き込み完了判定）
    status = I2C1->SR2;										//レジスタクリア

    I2C1->DR = wadr;										//書き込みたいレジスタのアドレス
    if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_TXE)) return 3;

	I2C1->DR = addr;										//書き込むアドレス
    if(wait_for_flag(&I2C1 ->SR1,I2C_SR1_TXE)) return 4;
    I2C1 -> CR1 |= I2C_CR1_STOP;

    return 0;
}

//アドレス送信待機関数
int wait_for_flag(volatile uint16_t* reg, uint16_t flag) {
    int timeout = 0;
    while(!(*reg & flag)) {
        if (timeout++ > TIMEOUT) {
            return 1; 						 // タイムアウトエラー
        }
    }
    return 0;  // 成功
}

//------------------Usart----------------------//
void convert_roll_to_ascii(float roll_angle, char *ascii_str_float)
{
    int16_t integer_part = (int16_t)roll_angle;  // 整数部分
    int16_t decimal_part = (int16_t)((roll_angle - integer_part) * 100);  // 小数部分（2桁）

    // 整数部と小数部を結合して文字列に変換
    sprintf(ascii_str_float, "%d.%d", integer_part, decimal_part);
}

void uint32_to_char_array(uint32_t num, char *str) {
    sprintf(str, "%u", (unsigned int)num);  // %luは32ビットの符号なし整数を文字列に変換
}

//文字送信
void usart2_tx(char* out_str)
{
	for(;*out_str != '\0';out_str++){//文字列の終わりでなければout_strをインクリメント
		while((USART2->SR & 0x80)==0x0);//送信完了待ち
		USART2->DR = *out_str;//1文字送信
	}

}

void usart2_tx_char(char ascii)
{
	while((USART2->SR & 0x80)==0x0);//送信完了待ち
	USART2->DR = ascii;//1文字送信
}


//-------------------Init------------------//
int BNO055_init(void)
{
	uint8_t status = 0;

	i2c_write_BNO055(BNO055_SLAVEADDRESS_A,BNO055_PAGE_ID_ADDR,BNO055_PAGE_0_ADDR);							//Register Map 0 Page
	status = i2c_read_BNO055(BNO055_SLAVEADDRESS_A,BNO055_CHIP_ID_ADDR);									//Chip ID Check
	i2c_write_BNO055(BNO055_SLAVEADDRESS_A,BNO055_PWR_MODE_ADDR,POWER_MODE_NORMAL);							//Power Mode Normal
	status = i2c_read_BNO055(BNO055_SLAVEADDRESS_A,BNO055_PWR_MODE_ADDR);									//Power Mode Check
	i2c_write_BNO055(BNO055_SLAVEADDRESS_A, BNO055_AXIS_MAP_CONFIG_ADDR, 0x21);
	status = i2c_read_BNO055(BNO055_SLAVEADDRESS_A,BNO055_AXIS_MAP_CONFIG_ADDR);
	i2c_write_BNO055(BNO055_SLAVEADDRESS_A, BNO055_AXIS_MAP_SIGN_ADDR, 0x02);
	status = i2c_read_BNO055(BNO055_SLAVEADDRESS_A,BNO055_AXIS_MAP_SIGN_ADDR);
//	i2c_write_BNO055(BNO055_SLAVEADDRESS_A,BNO055_OPR_MODE_ADDR,OPERATION_MODE_ACCGYRO);					//Operating Non-Fusion ACCGYRO Mode
	status = i2c_read_BNO055(BNO055_SLAVEADDRESS_A,BNO055_OPR_MODE_ADDR);									//Operating Mode Check
	i2c_write_BNO055(BNO055_SLAVEADDRESS_A,BNO055_OPR_MODE_ADDR,OPERATION_MODE_NDOF);						//Operating Fusion IMU Mode
	status = i2c_read_BNO055(BNO055_SLAVEADDRESS_A,BNO055_OPR_MODE_ADDR);									//Operating Mode Check
//	status = i2c_read_BNO055(BNO055_SLAVEADDRESS_A,BNO055_CALIB_STAT_ADDR);
	return status;

}


void hard_init(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency /1000);


	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //USRT2 イネーブル
	RCC->APB1ENR |=	RCC_APB1ENR_I2C1EN;	//I2C1 ENABLE (PB6,PB7はI2C1)
   	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;	//Timイネーブル

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN ; //ポートAをイネーブル
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	//portB ENABLE
   	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN |
   					RCC_APB2ENR_AFIOEN;

	//moter_REMAP(PC6,PC7,PC8,PC9)
	AFIO->MAPR |= 0x2000C00;
	AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_FULLREMAP;	       	//TIM3の信号をフルリマップ

	//i2c_port
	GPIOA->CRL = 0x44444944;	//PA3=AFIO_in(usrt2_rx) PA2=AFIO_out(uart2_tx)
	GPIOB->CRL = 0xdd444444;	//PB6,7 = 0xd(CNF = 11(オープンドレイン) MODE=01(出力モード:最大10Mhz))

	//moter_port

	GPIOC->CRL = 0xaa444444;	//PC8もオルタネート出力に設定
	GPIOC->CRH = 0x444444aa;
	GPIOC->ODR = 0;


	TIM3->PSC =22;//プリスケーラ23　CLKが24MHzなので1MHzでカウンタが動作
	TIM3->ARR =9999; //カウンタは10msで最大値9999

}


//USART1の初期設定
void usart2_init(void)
{
	USART2->BRR = 0xD06; // ボーレート設定 9600=32,000,000Hz/3334   3334->0xD06
	USART2->CR1 = 0x200c; //UE=1,TE=1,RE=1
}


//i2c初期設定
void i2c_init(void)
{
	I2C1->CR1 |= I2C_CR1_SWRST;  // ソフトウェアリセットを行う
	I2C1->CR1 &= ~I2C_CR1_SWRST; // リセット解除

	I2C1 -> CR2 &= ~I2C_CR2_FREQ;			//ペリフェラルクロックを初期化
	I2C1 -> CR2 |= 36;						//ペリフェラルクロックを36MHzに設定
	I2C1->CCR =  180;    					//SCL速度設定
	I2C1 -> TRISE = 37;						//速度制限
	I2C1 -> CR1 &= ~I2C_CR1_SMBTYPE;		//(SMBUSを無効化してI2C通信に切替)
	I2C1 -> CR1 |= I2C_CR1_ACK;				//(ACK ENABLE：　データ送信後にACK送信設定)
	I2C1 -> CR1 |= I2C_CR1_PE;				//有効化

}

void Tim3_Init(void) {

// 	TIM3->CCMR2 = 0x68; 	//OC3M=110(PWM),OC3PE=1,CSS4S=00(出力).
	TIM3->CCMR1 = 0x6868;	//OC1M-OC2M=110(PWM),OC1PE-OC2PE=1,CSS4S=00(出力)
	TIM3->CCMR2 = 0x6868;	//OC3M-OC4M=110(PWM),OC3PE-OC4PE=1,CSS4S=00(出力)
	TIM3->CCER =0x1111; 	//CC3E=1(CH1、3出力イネーブル/PC8/AIN1/右車輪)
	TIM3->CCR1 =1;			//比較値セット(PC6/BIN1/左車輪)
	TIM3->CCR2 =1;
	TIM3->CCR3 =1;
	TIM3->CCR4 =1;			//比較値セット(PC8/AIN1/右車輪)
	TIM3->CR1|=0x80;		//ARPE=1(自動プリロード)
	TIM3->CR1|=0x1;			//TIMスタート
}


//----------------------Handler------------------------//
void SysTick_Handler(void)      //1秒生成
{
	//chattering
	static unsigned int swc  = 0;	// 今回読み込んだスイッチの値
	static unsigned int swp1 = 0;	// 前回読み込んだスイッチの値
	static unsigned int swp2 = 0;	// 前々回に読み込んだスイッチの値

	static unsigned int sw_now  = 0;// 今回（現在）のスイッチの確定値
	static unsigned int sw_last = 0;// 前回のスイッチの確定値

	static int chat_count = 0;		// チャタリング除去用のカウンタ変数

    if (tick > 0) {					//delay
        tick--;
    }

	if (chat_count == 19) {	// 1000us を 10 回繰り返して 10ms を作り出す
		chat_count = 0;

		swp2 = swp1;
		swp1 = swc;
		swc  = Get_SW();

		if ((swp2 == swp1) && (swp1 == swc)) {	// 今回、前回、前々回の値が全て等しい
			sw_now = swc;						// 場合、今回の値を現在の確定値とする
		}

		if (sw_now != sw_last) {				// 前回のスイッチの値と異なる場合だけ以下の処理を行う
			if (sw_now & ~sw_last) {			// 立上り（スイッチの押下）を検出したら
				Up |= GPIO_IDR_IDR13;			// スイッチ押下のフラグを立てる
			}
			sw_last = sw_now;					// 現在のスイッチの値を sw_last に保存する
		}
	} else {
		chat_count++;
	}

	//usart
	if(cnt == 99){
      cnt = 0;
  	  flg_1s = 1;
  	  flg_2s = 1;

		chip_id = i2c_read_BNO055(BNO055_SLAVEADDRESS_A, BNO055_CHIP_ID_ADDR);  // レジスタ 0x00 はチップID
  		if (chip_id == 0xA0) {
  			data = 0x4949;					// 正常にBNO055が検出

			//角度データ読込
			eul_data_p_msb = i2c_read_BNO055(BNO055_SLAVEADDRESS_A, BNO055_EULER_P_MSB_ADDR);  // レジスタ 0x00 はチップID
			eul_data_p_lsb = i2c_read_BNO055(BNO055_SLAVEADDRESS_A, BNO055_EULER_P_LSB_ADDR);  // レジスタ 0x00 はチップID

			//角度データ格納
			eul_udata_p = eul_data_p_msb >> 7;						//最上位ビットを格納（0が正、1が負）
			eul_data_p_msb = eul_data_p_msb & 0x7F; 				//最上位ビットを0に
			eul_data_p = (eul_data_p_msb << 8) | eul_data_p_lsb;	//ビット結合

			pitch_angle = eul_data_p / 16.0f;						//データを16で割って格納
			if(pitch_angle > 181 ){
				pitch_angle = 0;
			}
			
  		} else {
  			i2c_init();
  			data = 0x4545; 					// BNO055が見つからない
  			data1 = (data >> 8) & 0xff;
  			data0 = data & 0xff;
  			usart2_tx_char(data1);
  			usart2_tx_char(data0);
  			flg_2s = 0;
  		}
	}else{
	  cnt++;
	}

	if(cnt2==19){
      cnt2 = 0;
  	  
  		if (chip_id != 0xA0) {
  			flg_2s = 0;
  		} else {
  			flg_2s = 1;
  		}
	}else{
	  cnt2++;
	}
}
