
#include "ps_intr.h"
#include "pid.h"
#include "fuzzy_pid.h"
#include "math.h"

u8 PL_BRAM_WRITE_FINISH=0;//PL侧读取、写入BRAM完毕后的标志位

//---------------------------------------------------------
//                    设置中断异常
//---------------------------------------------------------
void Setup_Intr_Exception(XScuGic * IntcInstancePtr)
{
	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler)XScuGic_InterruptHandler,
			(void *)IntcInstancePtr);
	Xil_ExceptionEnable();
}

//---------------------------------------------------------
//                    初始化中断系统
//---------------------------------------------------------
int Init_Intr_System(XScuGic * IntcInstancePtr)
{
	int Status;
	XScuGic_Config *IntcConfig;
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}
	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig, IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}
//---------------------------------------------------------
//                    中断初始化函数
//---------------------------------------------------------
int IntrInitFuntion(u16 DeviceId)
{
	XScuGic_Config *IntcConfig;
	int Status ;

//////////////////XScuGic相关函数////////////////////
	//check device id查找器件配置信息
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	//Initialization进行器件初始化
	Status = XScuGic_CfgInitialize(&INTCInst, IntcConfig, IntcConfig->CpuBaseAddress) ;
	//运行结果（可有可无）
	if (Status != XST_SUCCESS)
		return XST_FAILURE ;
	//设置中断优先级和触发类型
	XScuGic_SetPriorityTriggerType(&INTCInst, INTR_ID, 0xA0, 0x3);//应该是上升沿触发（1是高电平触发）
	//关联中断处理函数IntrHandler
	Status = XScuGic_Connect(&INTCInst, INTR_ID,(Xil_ExceptionHandler)IntrHandler,//这里绑定的中断处理程序 IntrHandler
			(void *)NULL) ;
	if (Status != XST_SUCCESS)
		return XST_FAILURE ;
	//为INTR_ID对应设备使能中断
	XScuGic_Enable(&INTCInst, INTR_ID) ;
////////////////////Xil相关函数////////////////////
	//给中断请求IRQ异常注册一个处理程序：将中断控制器GIC的中断处理程序与ARM处理器的硬件中断处理逻辑连接起来
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, &INTCInst);
	//使能IRQ异常
	Xil_ExceptionEnable();
	Setup_Intr_Exception(&INTCInst);//设置中断异常

	return XST_SUCCESS ;
}

//---------------------------------------------------------
//                 BRAM写入完毕的中断处理函数
//---------------------------------------------------------

//---------------------------------------------------------
//                 BRAM写入完毕的中断处理函数/频率跟踪的控制过程
//---------------------------------------------------------
// 因为每次中断都要重新进入中断函数，所以这里的变量要定义成全局变量，声明成静态变量应该也行。

void IntrHandler(void *CallbackRef)
{
	//变量定义区
	static int initial_down=0;
	//static u32 MW_center_freq = 0;//写入中心频率S1_FRE_TRACKING_MID_FTW 捷变频源中心频率控制字
	static int i=0;
	static float pre_Measure_Vaule = 0;
	static int state = 0; //
	static u32 MW_center_freq_reg1;
	static u32 MW_center_freq_reg2;
	//static u32 MW_center_freq_reg3;//调试
	//static u32 MW_center_freq_reg4;//调试
	static float B_ref;

	float u0 = 4*PI*0.01f; //这是和磁场的10^-5约了一下
	float PS_Cal_Current;
	//float MW_center_freq_real;
	//float MW_center_freq_init_real;
	//float MW_center_freq_reg1_real;
	//float MW_center_freq_reg2_real;

	static int Send_9_2_Current;
	static int Send_9_2_Current_cal;

	static int Send_9_2_Current_reg1 = 0;
	static int Send_9_2_Current_reg2 = 0;
	static int Send_9_2_Current_reg3 = 0;
	static int Send_9_2_Current_reg4 = 0;
	static int Send_9_2_Current_reg5 = 0;
	static int Send_9_2_Current_reg6 = 0;
	static int Send_9_2_Current_reg7 = 0;
	static int Send_9_2_Current_reg8 = 0;
	static int Send_9_2_Current_reg9 = 0;
	static int Send_9_2_Current_reg10 = 0;




	//读取频率跟踪控制需要的参数
	u8 S1_AD9914_MODE=XBram_ReadReg(XPAR_BRAM_0_BASEADDR,48); //0.读取S1_AD9914_MODE的地址（来自上位机）
	u8 S2_AD9914_MODE=XBram_ReadReg(XPAR_BRAM_0_BASEADDR,100);//S2_AD9914_MODE的地址,这里先屏蔽


	//跟踪峰1时不计算电流
	//if(S1_AD9914_MODE==2&&state==0)
	if(S1_AD9914_MODE==2){
		//变量最开始的初始化部分
		if(initial_down == 0){
			//PS端频率跟踪算法的中心频率计算完毕，将其写入PL侧AXI寄存器
			//MW_center_freq = MW_center_freq_init_1;
			//先给标定的起始微波中心频率
			MW_center_freq_reg1 = MW_center_freq_init_1;
			MW_center_freq_reg2 = MW_center_freq_init_2;
			NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_init_1) ;//写入中心频率S1_FRE_TRACKING_MID_FTW
			//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG40_OFFSET , 0x99111111) ;//写入中心频率S2_FRE_TRACKING_MID_FTW
			initial_down = 1;//初始化完毕
		}

		//频率跟踪控制
		else if(initial_down==1){
			NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg1) ;//写入中心频率S1_FRE_TRACKING_MID_FTW
			float delta_freq;//每次调节的delta Fre 单位MHz
			u32 demod_value_pl;//读取实时解调值,无符号24bit
			int demod_phase_pl;//读取实时相位值,有符号24bit
			float B_magnet;
			float B_delta;
			u32 Bram_8_11  = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,8);
			u32 Bram_12_15 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,12);
			u32 Bram_32_35 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,32);
			u32 Bram_36_39 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,36);
			u32 DDS_PINC = (((Bram_32_35 & 0xFFFF0000)>>16) | ((Bram_36_39 & 0x0000FFFF)<<16));
			float DDS_Freq = DDS_PINC*0.0000009313226f;//内参DDS kHz

			demod_phase_pl = ((Bram_12_15 & 0xFFFFFF00)>>8);//DC1解调相位(signed)扩展了2^23(-1~1)
			if(demod_phase_pl>=8388608){//符号位为1 是负数
				demod_phase_pl=demod_phase_pl-16777216;
			}
			float angle=(demod_phase_pl+(DDS_Freq*1470.65268f-1491.73171f))*0.00002145767f;//计算实时的相位差(单位角度，与Labview逻辑一致)

			demod_value_pl = ((Bram_8_11 & 0xFFFF0000)>>16) | ((Bram_12_15 & 0x000000FF)<<16); //DC1解调幅度(unsigned)扩展了2^24
			float amplitude = demod_value_pl*0.00078899f;//转化为幅度(单位mv，与Labview逻辑一致)


			if(amplitude>Value_model_swich) { //需要可调
				//当误差（解调幅值）大于设定的阈值时，先对误差进行一个非线性的处理
				delta_freq = amplitude/Slope_ctrl_reci; //delta_freq单位MHz
				if (delta_freq <=0 )
					delta_freq = -delta_freq;
				else delta_freq=delta_freq;

				if (delta_freq >=Limit_freq_shift ) //需要可控
					delta_freq = Limit_freq_shift;
				else delta_freq=delta_freq;
				//判断相位差
				if(Flag_phase_hop==0){
					if(angle>=0){// phase>0，正调
						MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);//将delta_freq(MHz)转换为delta频率控制字
					}
					else MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);// phase<0，反调
				}
				else {
					if(angle>=0){
						MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);
					}
					else MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);
				}
		    	//发送0值
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg2) ;//写入中心频率S1_FRE_TRACKING_MID_FTW
			}

			else if(amplitude < Limit_deadband) {//进入死区，不要动作，传输的电流值也不变
				MW_center_freq_reg1 = MW_center_freq_reg1; //微波源不调节 ，这里其实不写也
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg2) ;
			}

			else {
/*********************************Fuzzy PID+linear slope control*************************************/
				if(Tracking_MODE==1){
					//峰1中没有电流计算过程
					Fuzzytrans(0,amplitude,pre_Measure_Vaule); //(跟踪值，测量值，上一次的测量值)
					pre_Measure_Vaule=amplitude;
					delta_freq = fuzzypid_calc(amplitude);
					if (delta_freq <=0 )
						delta_freq = -delta_freq;
					else delta_freq=delta_freq;//变为绝对值

					if (delta_freq >=Limit_freq_shift ) //需要可控
						delta_freq = Limit_freq_shift;
					else delta_freq=delta_freq;

					if(Flag_phase_hop==0){
						if(angle>=0){// phase>0，正调
							MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);//将delta_freq(MHz)转换为delta频率控制字
						}
						else MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);// phase<0，反调
					}
					//理论这里需要加标志
					else {
						if(angle>=0){
							MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);
						}
						else MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);
					}
					//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg2) ;//写入中心频率S1_FRE_TRACKING_MID_FTW



					B_magnet = (delta_freq + (MW_center_freq_reg2-MW_center_freq_reg1)*0.00000111758709f)*1000.0f/(2*2.802f); //用差值算磁场值,和以前不一样。1000倍这里先扩大，后面上位机会消除。 2.802MHz/Gs为磁旋比 有符号数
					B_delta = B_magnet-B_ref; //计算外部电流引起的磁场偏移
					//B_magnet = (MW_center_freq_reg2-MW_center_freq_reg1)*1000.0f/(2*2.802f);
					PS_Cal_Current = (B_delta*2*PI*Distance)/u0;  //电流值估算   a为导线和NV色心位置固定时的相对距离,这里是Gs的单位，要换成T,在定义u0的时候就处理了。

					//Send_9_2_Current= (int)PS_Cal_Current; //只精确到0.001A

					//这里做滑动窗格的均值滤波
					Send_9_2_Current_reg1 = (int)PS_Cal_Current;
					Send_9_2_Current_reg2 = Send_9_2_Current_reg1;
					Send_9_2_Current_reg3 = Send_9_2_Current_reg2;
					Send_9_2_Current_reg4 = Send_9_2_Current_reg3;
					Send_9_2_Current_reg5 = Send_9_2_Current_reg4;
					Send_9_2_Current_reg6 = Send_9_2_Current_reg5;
					Send_9_2_Current_reg7 = Send_9_2_Current_reg6;
					Send_9_2_Current_reg8 = Send_9_2_Current_reg7;
					Send_9_2_Current_reg9 = Send_9_2_Current_reg8;
					Send_9_2_Current_reg10 = Send_9_2_Current_reg9;
					//Send_9_2_Current = Send_9_2_Current_reg1; //不滤波直接输出

					// /*
					//滤波处理
					if(i==0){Send_9_2_Current = Send_9_2_Current_reg1;}

					else if(i==1){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2) / 2;
					}
					else if(i==2){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3) / 3;
					}
					else if(i==3){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4) / 4;
					}
					else if(i==4){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5) / 5 ;
					}
					else if(i==5){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											   Send_9_2_Current_reg3+ Send_9_2_Current_reg4 +
												Send_9_2_Current_reg5+Send_9_2_Current_reg6) / 6;
						}
					else if(i==6){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4+
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6+
											Send_9_2_Current_reg7) / 7;
						}
					else if(i==7){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
											Send_9_2_Current_reg7 + Send_9_2_Current_reg8 ) / 8 ;
					}
					else if(i==8){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
											Send_9_2_Current_reg7 + Send_9_2_Current_reg8 +
											Send_9_2_Current_reg9) / 9 ;
					}
					else if(i>=9){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
											Send_9_2_Current_reg7 + Send_9_2_Current_reg8 +
											Send_9_2_Current_reg9 + Send_9_2_Current_reg10) / 10 ;
					//Send_9_2_Current=0; 只是看一下程序是不是执行的这里
					}

					i=i+1;






				}

/*********************************PID+linear slope control*************************************/
				else if(Tracking_MODE==0){
					PIDParament_Init(); //因为这里是实时更新，所以需要把初始化参数直接写到这里
					//上面的delta_freq是用来直接计算的，下面的delta_freq是用来做调节的
					delta_freq = pid_calc(amplitude); //通过PID调节来实现稳态误差的消除
					if (delta_freq <=0 )
						delta_freq = -delta_freq;
					else delta_freq=delta_freq;//变为绝对值

					if (delta_freq >=Limit_freq_shift ) //需要可控
						delta_freq = Limit_freq_shift;
					else delta_freq=delta_freq;

					if(Flag_phase_hop==0){
						if(angle>=0){// phase>0，正调
							MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);//将delta_freq(MHz)转换为delta频率控制字
						}
						else MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);// phase<0，反调
					}
					//理论这里需要加标志
					else {
						if(angle>=0){
							MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);
						}
						else MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);
					}



					B_magnet = (delta_freq + (MW_center_freq_reg2-MW_center_freq_reg1)*0.00000111758709f)*1000.0f/(2*2.802f); //用差值算磁场值,和以前不一样。1000倍这里先扩大，后面上位机会消除。 2.802MHz/Gs为磁旋比 有符号数
					B_delta = B_magnet-B_ref;
					//B_magnet = (MW_center_freq_reg2-MW_center_freq_reg1)*1000.0f/(2*2.802f);
					PS_Cal_Current = (B_delta*2*PI*Distance)/u0; //电流值估算   a为导线和NV色心位置固定时的相对距离,这里是Gs的单位，要换成T,在定义u0的时候就处理了。
					//Send_9_2_Current= (int)PS_Cal_Current; //只精确到0.001A

					//这里做滑动窗格的均值滤波
					Send_9_2_Current_reg1 = (int)PS_Cal_Current;
					Send_9_2_Current_reg2 = Send_9_2_Current_reg1;
					Send_9_2_Current_reg3 = Send_9_2_Current_reg2;
					Send_9_2_Current_reg4 = Send_9_2_Current_reg3;
					Send_9_2_Current_reg5 = Send_9_2_Current_reg4;
					Send_9_2_Current_reg6 = Send_9_2_Current_reg5;
					Send_9_2_Current_reg7 = Send_9_2_Current_reg6;
					Send_9_2_Current_reg8 = Send_9_2_Current_reg7;
					Send_9_2_Current_reg9 = Send_9_2_Current_reg8;
					Send_9_2_Current_reg10 = Send_9_2_Current_reg9;

					//Send_9_2_Current = Send_9_2_Current_reg1; //不滤波直接输出

					// /*
					//滤波处理
					if(i==0){Send_9_2_Current = Send_9_2_Current_reg1;}

					else if(i==1){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2) / 2;
					}
					else if(i==2){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3) / 3;
					}
					else if(i==3){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4) / 4;
					}
					else if(i==4){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5) / 5 ;
					}
					else if(i==5){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											   Send_9_2_Current_reg3+ Send_9_2_Current_reg4 +
												Send_9_2_Current_reg5+Send_9_2_Current_reg6) / 6;
						}
					else if(i==6){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4+
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6+
											Send_9_2_Current_reg7) / 7;
						}
					else if(i==7){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
											Send_9_2_Current_reg7 + Send_9_2_Current_reg8 ) / 8 ;
					}
					else if(i==8){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
											Send_9_2_Current_reg7 + Send_9_2_Current_reg8 +
											Send_9_2_Current_reg9) / 9 ;
					}
					else if(i>=9){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
										   Send_9_2_Current_reg7 + Send_9_2_Current_reg8 +
										  Send_9_2_Current_reg9 + Send_9_2_Current_reg10) / 10 ;

						//Send_9_2_Current = Send_9_2_Current_reg1; //只是看一下程序是不是执行的这里
					}


					i=i+1;







					//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq) ;//写入中心频率S1_FRE_TRACKING_MID_FTW
				}

				//切换到峰2
				//MW_center_freq_reg1 = MW_center_freq; //保存峰1跟踪完成的微波中心频率值
				//MW_center_freq = MW_center_freq_reg2; //开始切换到峰2
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg2) ;
			}
			//Send_9_2_Current = Send_9_2_Current;
		}
		//state = 1; //峰1跟踪完毕,拉高峰2跟踪的标志位

	}












	//进入第二个峰的跟踪，计算部分都在峰2进行计算
	//else if(S1_AD9914_MODE == 2 && state == 1)
	 else if(S2_AD9914_MODE == 2){
		if(initial_down == 0){
			//PS端频率跟踪算法的中心频率计算完毕，将其写入PL侧AXI寄存器
			//MW_center_freq = MW_center_freq_init_1;
			//先给标定的起始微波中心频率
			MW_center_freq_reg1 = MW_center_freq_init_1;
			MW_center_freq_reg2 = MW_center_freq_init_2;
			NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_init_2) ;//写入中心频率S1_FRE_TRACKING_MID_FTW
			//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG40_OFFSET , 0x99111111) ;//写入中心频率S2_FRE_TRACKING_MID_FTW
			initial_down = 1;//初始化完毕
		}
		else if(initial_down==1){
			NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg2) ;//写入中心频率S1_FRE_TRACKING_MID_FTW
			//在峰1跟踪结束的时候已经切换到峰2
			/**************************************************变量定义并读出新的解调信息***************************************************/
			float delta_freq;//每次调节的delta Fre 单位MHz
			float B_magnet;
			float B_delta;
			u32 demod_value_pl;//读取实时解调值,无符号24bit
			int demod_phase_pl;//读取实时相位值,有符号24bit
			u32 Bram_8_11  = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,8);
			u32 Bram_12_15 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,12);
			u32 Bram_32_35 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,32);
			u32 Bram_36_39 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,36);
			u32 DDS_PINC = (((Bram_32_35 & 0xFFFF0000)>>16) | ((Bram_36_39 & 0x0000FFFF)<<16));
			float DDS_Freq = DDS_PINC*0.0000009313226f;//内参DDS kHz
			demod_phase_pl = ((Bram_12_15 & 0xFFFFFF00)>>8);//DC1解调相位(signed)扩展了2^23(-1~1)
			if(demod_phase_pl>=8388608){//符号位为1 是负数
				demod_phase_pl=demod_phase_pl-16777216;
			}
			float angle=(demod_phase_pl+(DDS_Freq*1470.65268f-1491.73171f))*0.00002145767f;//计算实时的相位差(单位角度，与Labview逻辑一致)
			demod_value_pl = ((Bram_8_11 & 0xFFFF0000)>>16) | ((Bram_12_15 & 0x000000FF)<<16); //DC1解调幅度(unsigned)扩展了2^24
			float amplitude = demod_value_pl*0.00078899f;//转化为幅度(单位mv，与Labview逻辑一致)


			if(amplitude>Value_model_swich) { //大于阈值时按线性跟踪先处理
				//当误差（解调幅值）大于设定的阈值时，先对误差进行一个非线性的处理(还没做)
				delta_freq = amplitude/Slope_ctrl_reci; //delta_freq单位MHz
				if (delta_freq <=0 )
					delta_freq = -delta_freq;
				else delta_freq=delta_freq;
				if (delta_freq >=Limit_freq_shift ) //需要可控
					delta_freq = Limit_freq_shift;
				else delta_freq=delta_freq;
				//判断相位差
				if(Flag_phase_hop==0){
					if(angle>=0){// phase>0，正调
						MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);//将delta_freq(MHz)转换为delta频率控制字
					}
					else MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);// phase<0，反调
				}
				else {
					if(angle>=0){
						MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);
					}
					else MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);
				}
				//发送0值
				Send_9_2_Current = 1 * 0.0f; //9-2报文发送注意识别
				//这里9-2报文不能这么输出,为0就当成掉帧
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg1) ;//写入中心频率S1_FRE_TRACKING_MID_FTW
			}

			else if(amplitude < Limit_deadband) { //进入死区，不要动作，传输的电流值也不变
				MW_center_freq_reg2 = MW_center_freq_reg2; //微波源不调节 ，这里其实不写也
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg1) ;
				//下面注释掉的是如果Send_9_2_Current不想定义为static变量的方式
				//MW_center_freq_real = (3500.0f-MW_center_freq*0.00000055879354477);//变为MHz.MW_center_freq*0.0000011758709f
				//MW_center_freq_init_real = MW_center_freq_init*0.00000111758709f;//单位:MHz MW_center_freq_init*0.00000111758709f
				//B_magnet = (delta_freq + (MW_center_freq_real-MW_center_freq_init_real))*1000.0f/2.802f; //磁场值,1000倍这里先扩大，后面上位机会消除。 2.802MHz/Gs为磁旋比 有符号数
				//PS_Cal_Current = (B_magnet*2*PI*Distance)/u0; //电流值估算   a为导线和NV色心位置固定时的相对距离,这里是Gs的单位，要换成T,在定义u0的时候就处理了。
				//Send_9_2_Current = (int)PS_Cal_Current;
				Send_9_2_Current = Send_9_2_Current;
			}
			else {

				if(Tracking_MODE==1){
//					delta_freq = amplitude/Slope_ctrl_reci; //斜率绝对值
//					if (delta_freq <=0 )
//						delta_freq = -delta_freq;
//					else delta_freq=delta_freq; //变为绝对值，由相位来调整
//					if(Flag_phase_hop==0){
//						if(angle>=0){// phase>0，
//							delta_freq = delta_freq; //叠加进总体的偏差
//						}
//						else{ // phase<0，反调
//							delta_freq = -delta_freq;
//						}
//					}
//					else {
//						if(angle>=0){
//							delta_freq = -delta_freq;
//						}
//						else{
//							delta_freq = delta_freq;
//						}
//					}
					Fuzzytrans(0,amplitude,pre_Measure_Vaule); //(跟踪值，测量值，上一次的测量值)
					pre_Measure_Vaule=amplitude;
					delta_freq = fuzzypid_calc(amplitude);
					if (delta_freq <=0 )
						delta_freq = -delta_freq;
					else delta_freq=delta_freq;//变为绝对值

					if (delta_freq >=Limit_freq_shift ) //需要可控
						delta_freq = Limit_freq_shift;
					else delta_freq=delta_freq;

					if(Flag_phase_hop==0){
						if(angle>=0){// phase>0，正调
							MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);//将delta_freq(MHz)转换为delta频率控制字
						}
						else MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);// phase<0，反调
					}
					//理论这里需要加标志
					else {
						if(angle>=0){
							MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);
						}
						else MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);
					}

					//电流的计算部分,freq修正系数不一样是因为有一个扩展的是31位，有一个扩展的32位
					//MW_center_freq_reg2_real = (3500.0f-MW_center_freq_reg1*0.00000055879354477);//变为MHz.MW_center_freq*0.0000011758709f，实际就是reg2的实际频率
					//MW_center_freq_reg1_real = (3500.0f-MW_center_freq_reg1*0.00000055879354477);
					//MW_center_freq_init_real = MW_center_freq_init*0.00000111758709f;//单位:MHz MW_center_freq_init*0.00000111758709f
					B_magnet = (delta_freq + (MW_center_freq_reg2-MW_center_freq_reg1)*0.00000111758709f)*1000.0f/(2*2.802f); //用差值算磁场值,和以前不一样。1000倍这里先扩大，后面上位机会消除。 2.802MHz/Gs为磁旋比 有符号数
					B_delta = B_magnet-B_ref; //计算外部电流引起的磁场偏移
					//B_magnet = (MW_center_freq_reg2-MW_center_freq_reg1)*1000.0f/(2*2.802f);
					PS_Cal_Current = (B_delta*2*PI*Distance)/u0;  //电流值估算   a为导线和NV色心位置固定时的相对距离,这里是Gs的单位，要换成T,在定义u0的时候就处理了。

					//Send_9_2_Current= (int)PS_Cal_Current; //只精确到0.001A

					//这里做滑动窗格的均值滤波
					Send_9_2_Current_reg1 = (int)PS_Cal_Current;
					Send_9_2_Current_reg2 = Send_9_2_Current_reg1;
					Send_9_2_Current_reg3 = Send_9_2_Current_reg2;
					Send_9_2_Current_reg4 = Send_9_2_Current_reg3;
					Send_9_2_Current_reg5 = Send_9_2_Current_reg4;
					Send_9_2_Current_reg6 = Send_9_2_Current_reg5;
					Send_9_2_Current_reg7 = Send_9_2_Current_reg6;
					Send_9_2_Current_reg8 = Send_9_2_Current_reg7;
					Send_9_2_Current_reg9 = Send_9_2_Current_reg8;
					Send_9_2_Current_reg10 = Send_9_2_Current_reg9;
					//Send_9_2_Current = Send_9_2_Current_reg1; //不滤波直接输出

					// /*
					//滤波处理
					if(i==0){Send_9_2_Current = Send_9_2_Current_reg1;}

					else if(i==1){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2) / 2;
					}
					else if(i==2){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3) / 3;
					}
					else if(i==3){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4) / 4;
					}
					else if(i==4){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5) / 5 ;
					}
					else if(i==5){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											   Send_9_2_Current_reg3+ Send_9_2_Current_reg4 +
												Send_9_2_Current_reg5+Send_9_2_Current_reg6) / 6;
						}
					else if(i==6){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4+
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6+
											Send_9_2_Current_reg7) / 7;
						}
					else if(i==7){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
											Send_9_2_Current_reg7 + Send_9_2_Current_reg8 ) / 8 ;
					}
					else if(i==8){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
											Send_9_2_Current_reg7 + Send_9_2_Current_reg8 +
											Send_9_2_Current_reg9) / 9 ;
					}
					else if(i>=9){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
											Send_9_2_Current_reg7 + Send_9_2_Current_reg8 +
											Send_9_2_Current_reg9 + Send_9_2_Current_reg10) / 10 ;
					//Send_9_2_Current=0; 只是看一下程序是不是执行的这里
					}

//					//上面的delta_freq是用来直接计算的，下面的delta_freq是用来做调节的
//					Fuzzytrans(0,amplitude,pre_Measure_Vaule); //(跟踪值，测量值，上一次的测量值)
//					pre_Measure_Vaule=amplitude;
//					delta_freq = fuzzypid_calc(amplitude);
//					if (delta_freq <=0 )
//						delta_freq = -delta_freq;
//					else delta_freq=delta_freq;//变为绝对值
//
//					if (delta_freq >=Limit_freq_shift ) //需要可控
//						delta_freq = Limit_freq_shift;
//					else delta_freq=delta_freq;
//
//					if(Flag_phase_hop==0){
//						if(angle>=0){// phase>0，正调
//							MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);//将delta_freq(MHz)转换为delta频率控制字
//						}
//						else MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);// phase<0，反调
//					}
//					//理论这里需要加标志
//					else {
//						if(angle>=0){
//							MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);
//						}
//						else MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);
//					}
					//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq) ;//写入中心频率S1_FRE_TRACKING_MID_FTW
					i=i+1;
				}
	/***************************************************************PID控制模式****************************************/
				if(Tracking_MODE==0){
					PIDParament_Init(); //因为这里是实时更新，所以需要把初始化参数直接写到这里
					delta_freq = pid_calc(amplitude); //通过PID调节来实现稳态误差的消除
					if (delta_freq <=0 )
						delta_freq = -delta_freq;
					else delta_freq=delta_freq;//变为绝对值

					if (delta_freq >=Limit_freq_shift ) //需要可控
						delta_freq = Limit_freq_shift;
					else delta_freq=delta_freq;

					if(Flag_phase_hop==0){
						if(angle>=0){// phase>0，正调
							MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);//将delta_freq(MHz)转换为delta频率控制字
						}
						else MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);// phase<0，反调
					}
					//理论这里需要加标志
					else {
						if(angle>=0){
							MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);
						}
						else MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);
					}
					//这里先直接把电流值传出去
//					delta_freq = amplitude/Slope_ctrl_reci; //斜率绝对值
//					if (delta_freq <=0 )
//						delta_freq = -delta_freq;
//					else delta_freq=delta_freq; //变为绝对值，由相位来调整
//
//					//为0是，表示ODMR的左边相位为正，右边相位为负
//					if(Flag_phase_hop==0){
//						if(angle>=0){// phase>0，
//							delta_freq = delta_freq; //叠加进总体的偏差
//						}
//						else{ // phase<0，反调
//							delta_freq = -delta_freq;
//						}
//					}
//					//理论这里需要加标志
//					else {
//						if(angle>=0){
//							delta_freq = -delta_freq;
//						}
//						else{
//							delta_freq = delta_freq;
//						}
//					}

					B_magnet = (delta_freq + (MW_center_freq_reg2-MW_center_freq_reg1)*0.00000111758709f)*1000.0f/(2*2.802f); //用差值算磁场值,和以前不一样。1000倍这里先扩大，后面上位机会消除。 2.802MHz/Gs为磁旋比 有符号数
					B_delta = B_magnet-B_ref;
					//B_magnet = (MW_center_freq_reg2-MW_center_freq_reg1)*1000.0f/(2*2.802f);
					PS_Cal_Current = (B_delta*2*PI*Distance)/u0; //电流值估算   a为导线和NV色心位置固定时的相对距离,这里是Gs的单位，要换成T,在定义u0的时候就处理了。
					//Send_9_2_Current= (int)PS_Cal_Current; //只精确到0.001A

					//这里做滑动窗格的均值滤波
					Send_9_2_Current_reg1 = (int)PS_Cal_Current;
					Send_9_2_Current_reg2 = Send_9_2_Current_reg1;
					Send_9_2_Current_reg3 = Send_9_2_Current_reg2;
					Send_9_2_Current_reg4 = Send_9_2_Current_reg3;
					Send_9_2_Current_reg5 = Send_9_2_Current_reg4;
					Send_9_2_Current_reg6 = Send_9_2_Current_reg5;
					Send_9_2_Current_reg7 = Send_9_2_Current_reg6;
					Send_9_2_Current_reg8 = Send_9_2_Current_reg7;
					Send_9_2_Current_reg9 = Send_9_2_Current_reg8;
					Send_9_2_Current_reg10 = Send_9_2_Current_reg9;

					//Send_9_2_Current = Send_9_2_Current_reg1; //不滤波直接输出

					// /*
					//滤波处理
					if(i==0){Send_9_2_Current = Send_9_2_Current_reg1;}

					else if(i==1){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2) / 2;
					}
					else if(i==2){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3) / 3;
					}
					else if(i==3){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4) / 4;
					}
					else if(i==4){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5) / 5 ;
					}
					else if(i==5){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											   Send_9_2_Current_reg3+ Send_9_2_Current_reg4 +
												Send_9_2_Current_reg5+Send_9_2_Current_reg6) / 6;
						}
					else if(i==6){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4+
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6+
											Send_9_2_Current_reg7) / 7;
						}
					else if(i==7){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
											Send_9_2_Current_reg7 + Send_9_2_Current_reg8 ) / 8 ;
					}
					else if(i==8){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
											Send_9_2_Current_reg7 + Send_9_2_Current_reg8 +
											Send_9_2_Current_reg9) / 9 ;
					}
					else if(i>=9){
						Send_9_2_Current = (Send_9_2_Current_reg1 + Send_9_2_Current_reg2 +
											Send_9_2_Current_reg3 + Send_9_2_Current_reg4 +
											Send_9_2_Current_reg5 + Send_9_2_Current_reg6 +
										   Send_9_2_Current_reg7 + Send_9_2_Current_reg8 +
										  Send_9_2_Current_reg9 + Send_9_2_Current_reg10) / 10 ;

						//Send_9_2_Current = Send_9_2_Current_reg1; //只是看一下程序是不是执行的这里
					}
					 //

					//上面的delta_freq是用来直接计算的，下面的delta_freq是用来做调节的
//					delta_freq = pid_calc(amplitude); //通过PID调节来实现稳态误差的消除
//					if (delta_freq <=0 )
//						delta_freq = -delta_freq;
//					else delta_freq=delta_freq;//变为绝对值
//
//					if (delta_freq >=Limit_freq_shift ) //需要可控
//						delta_freq = Limit_freq_shift;
//					else delta_freq=delta_freq;
//
//					if(Flag_phase_hop==0){
//						if(angle>=0){// phase>0，正调
//							MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);//将delta_freq(MHz)转换为delta频率控制字
//						}
//						else MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);// phase<0，反调
//					}
//					//理论这里需要加标志
//					else {
//						if(angle>=0){
//							MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);
//						}
//						else MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);
//					}
					//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq) ;//写入中心频率S1_FRE_TRACKING_MID_FTW
					i=i+1;
				}

				//MW_center_freq_reg2 = MW_center_freq; //reg类型都是静态变量
				//MW_center_freq = MW_center_freq_reg1;
				//峰2的调节已经做完了，切换到峰1
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg1) ;//开始切换到峰1

			}
			//state = 0;
		}
	 }

	else if(S1_AD9914_MODE != 2&&S2_AD9914_MODE != 2){
		initial_down = 0;
		//MW_center_freq = MW_center_freq_init; //这是原来的程序
		MW_center_freq_reg1 = MW_center_freq_init_1;
		MW_center_freq_reg2 = MW_center_freq_init_2;
		B_ref = (MW_center_freq_init_2- MW_center_freq_init_1)*0.00000111758709f*1000.0f/(2*2.802f);
		//MW_center_freq_reg3 = MW_center_freq_init_1;//调试
		//MW_center_freq_reg4 = MW_center_freq_init_2;//调试
		FPIDParament_Init();//FUZZYPID初始化
	}

	//此处必须每个循环都执行  9-2报文模块需接收到PS_Fre_Cal_Already标志
	//use for judge the discard point
	//if(Send_9_2_Current>200000){
	//	//Send_9_2_Current = 0;
	//}

	NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG61_OFFSET , Send_9_2_Current) ;//发出计算电流值
    NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG60_OFFSET , 1) ;//拉高频率跟踪算法计算完毕标志 PS_Fre_Cal_Already REG60[0]
    NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG60_OFFSET , 0) ;//拉低频率跟踪算法计算完毕标志 PS_Fre_Cal_Already REG60[0]
	PL_BRAM_WRITE_FINISH=1;//PL侧BRAM读写完毕后引发的中断   将完毕标志拉高

}


