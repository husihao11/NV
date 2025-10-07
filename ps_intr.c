
#include "ps_intr.h"
#include "pid.h"
#include "fuzzy_pid.h"
#include "math.h"

u8 PL_BRAM_WRITE_FINISH=0;//PL���ȡ��д��BRAM��Ϻ�ı�־λ

//---------------------------------------------------------
//                    �����ж��쳣
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
//                    ��ʼ���ж�ϵͳ
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
//                    �жϳ�ʼ������
//---------------------------------------------------------
int IntrInitFuntion(u16 DeviceId)
{
	XScuGic_Config *IntcConfig;
	int Status ;

//////////////////XScuGic��غ���////////////////////
	//check device id��������������Ϣ
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	//Initialization����������ʼ��
	Status = XScuGic_CfgInitialize(&INTCInst, IntcConfig, IntcConfig->CpuBaseAddress) ;
	//���н�������п��ޣ�
	if (Status != XST_SUCCESS)
		return XST_FAILURE ;
	//�����ж����ȼ��ʹ�������
	XScuGic_SetPriorityTriggerType(&INTCInst, INTR_ID, 0xA0, 0x3);//Ӧ���������ش�����1�Ǹߵ�ƽ������
	//�����жϴ�����IntrHandler
	Status = XScuGic_Connect(&INTCInst, INTR_ID,(Xil_ExceptionHandler)IntrHandler,//����󶨵��жϴ������ IntrHandler
			(void *)NULL) ;
	if (Status != XST_SUCCESS)
		return XST_FAILURE ;
	//ΪINTR_ID��Ӧ�豸ʹ���ж�
	XScuGic_Enable(&INTCInst, INTR_ID) ;
////////////////////Xil��غ���////////////////////
	//���ж�����IRQ�쳣ע��һ��������򣺽��жϿ�����GIC���жϴ��������ARM��������Ӳ���жϴ����߼���������
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, &INTCInst);
	//ʹ��IRQ�쳣
	Xil_ExceptionEnable();
	Setup_Intr_Exception(&INTCInst);//�����ж��쳣

	return XST_SUCCESS ;
}

//---------------------------------------------------------
//                 BRAMд����ϵ��жϴ�����
//---------------------------------------------------------

//---------------------------------------------------------
//                 BRAMд����ϵ��жϴ�����/Ƶ�ʸ��ٵĿ��ƹ���
//---------------------------------------------------------
// ��Ϊÿ���ж϶�Ҫ���½����жϺ�������������ı���Ҫ�����ȫ�ֱ����������ɾ�̬����Ӧ��Ҳ�С�

void IntrHandler(void *CallbackRef)
{
	//����������
	static int initial_down=0;
	//static u32 MW_center_freq = 0;//д������Ƶ��S1_FRE_TRACKING_MID_FTW �ݱ�ƵԴ����Ƶ�ʿ�����
	static int i=0;
	static float pre_Measure_Vaule = 0;
	static int state = 0; //
	static u32 MW_center_freq_reg1;
	static u32 MW_center_freq_reg2;
	//static u32 MW_center_freq_reg3;//����
	//static u32 MW_center_freq_reg4;//����
	static float B_ref;

	float u0 = 4*PI*0.01f; //���Ǻʹų���10^-5Լ��һ��
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




	//��ȡƵ�ʸ��ٿ�����Ҫ�Ĳ���
	u8 S1_AD9914_MODE=XBram_ReadReg(XPAR_BRAM_0_BASEADDR,48); //0.��ȡS1_AD9914_MODE�ĵ�ַ��������λ����
	u8 S2_AD9914_MODE=XBram_ReadReg(XPAR_BRAM_0_BASEADDR,100);//S2_AD9914_MODE�ĵ�ַ,����������


	//���ٷ�1ʱ���������
	//if(S1_AD9914_MODE==2&&state==0)
	if(S1_AD9914_MODE==2){
		//�����ʼ�ĳ�ʼ������
		if(initial_down == 0){
			//PS��Ƶ�ʸ����㷨������Ƶ�ʼ�����ϣ�����д��PL��AXI�Ĵ���
			//MW_center_freq = MW_center_freq_init_1;
			//�ȸ��궨����ʼ΢������Ƶ��
			MW_center_freq_reg1 = MW_center_freq_init_1;
			MW_center_freq_reg2 = MW_center_freq_init_2;
			NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_init_1) ;//д������Ƶ��S1_FRE_TRACKING_MID_FTW
			//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG40_OFFSET , 0x99111111) ;//д������Ƶ��S2_FRE_TRACKING_MID_FTW
			initial_down = 1;//��ʼ�����
		}

		//Ƶ�ʸ��ٿ���
		else if(initial_down==1){
			NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg1) ;//д������Ƶ��S1_FRE_TRACKING_MID_FTW
			float delta_freq;//ÿ�ε��ڵ�delta Fre ��λMHz
			u32 demod_value_pl;//��ȡʵʱ���ֵ,�޷���24bit
			int demod_phase_pl;//��ȡʵʱ��λֵ,�з���24bit
			float B_magnet;
			float B_delta;
			u32 Bram_8_11  = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,8);
			u32 Bram_12_15 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,12);
			u32 Bram_32_35 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,32);
			u32 Bram_36_39 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,36);
			u32 DDS_PINC = (((Bram_32_35 & 0xFFFF0000)>>16) | ((Bram_36_39 & 0x0000FFFF)<<16));
			float DDS_Freq = DDS_PINC*0.0000009313226f;//�ڲ�DDS kHz

			demod_phase_pl = ((Bram_12_15 & 0xFFFFFF00)>>8);//DC1�����λ(signed)��չ��2^23(-1~1)
			if(demod_phase_pl>=8388608){//����λΪ1 �Ǹ���
				demod_phase_pl=demod_phase_pl-16777216;
			}
			float angle=(demod_phase_pl+(DDS_Freq*1470.65268f-1491.73171f))*0.00002145767f;//����ʵʱ����λ��(��λ�Ƕȣ���Labview�߼�һ��)

			demod_value_pl = ((Bram_8_11 & 0xFFFF0000)>>16) | ((Bram_12_15 & 0x000000FF)<<16); //DC1�������(unsigned)��չ��2^24
			float amplitude = demod_value_pl*0.00078899f;//ת��Ϊ����(��λmv����Labview�߼�һ��)


			if(amplitude>Value_model_swich) { //��Ҫ�ɵ�
				//���������ֵ�������趨����ֵʱ���ȶ�������һ�������ԵĴ���
				delta_freq = amplitude/Slope_ctrl_reci; //delta_freq��λMHz
				if (delta_freq <=0 )
					delta_freq = -delta_freq;
				else delta_freq=delta_freq;

				if (delta_freq >=Limit_freq_shift ) //��Ҫ�ɿ�
					delta_freq = Limit_freq_shift;
				else delta_freq=delta_freq;
				//�ж���λ��
				if(Flag_phase_hop==0){
					if(angle>=0){// phase>0������
						MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);//��delta_freq(MHz)ת��ΪdeltaƵ�ʿ�����
					}
					else MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);// phase<0������
				}
				else {
					if(angle>=0){
						MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);
					}
					else MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);
				}
		    	//����0ֵ
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg2) ;//д������Ƶ��S1_FRE_TRACKING_MID_FTW
			}

			else if(amplitude < Limit_deadband) {//������������Ҫ����������ĵ���ֵҲ����
				MW_center_freq_reg1 = MW_center_freq_reg1; //΢��Դ������ ��������ʵ��дҲ
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg2) ;
			}

			else {
/*********************************Fuzzy PID+linear slope control*************************************/
				if(Tracking_MODE==1){
					//��1��û�е����������
					Fuzzytrans(0,amplitude,pre_Measure_Vaule); //(����ֵ������ֵ����һ�εĲ���ֵ)
					pre_Measure_Vaule=amplitude;
					delta_freq = fuzzypid_calc(amplitude);
					if (delta_freq <=0 )
						delta_freq = -delta_freq;
					else delta_freq=delta_freq;//��Ϊ����ֵ

					if (delta_freq >=Limit_freq_shift ) //��Ҫ�ɿ�
						delta_freq = Limit_freq_shift;
					else delta_freq=delta_freq;

					if(Flag_phase_hop==0){
						if(angle>=0){// phase>0������
							MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);//��delta_freq(MHz)ת��ΪdeltaƵ�ʿ�����
						}
						else MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);// phase<0������
					}
					//����������Ҫ�ӱ�־
					else {
						if(angle>=0){
							MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);
						}
						else MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);
					}
					//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg2) ;//д������Ƶ��S1_FRE_TRACKING_MID_FTW



					B_magnet = (delta_freq + (MW_center_freq_reg2-MW_center_freq_reg1)*0.00000111758709f)*1000.0f/(2*2.802f); //�ò�ֵ��ų�ֵ,����ǰ��һ����1000�����������󣬺�����λ���������� 2.802MHz/GsΪ������ �з�����
					B_delta = B_magnet-B_ref; //�����ⲿ��������Ĵų�ƫ��
					//B_magnet = (MW_center_freq_reg2-MW_center_freq_reg1)*1000.0f/(2*2.802f);
					PS_Cal_Current = (B_delta*2*PI*Distance)/u0;  //����ֵ����   aΪ���ߺ�NVɫ��λ�ù̶�ʱ����Ծ���,������Gs�ĵ�λ��Ҫ����T,�ڶ���u0��ʱ��ʹ����ˡ�

					//Send_9_2_Current= (int)PS_Cal_Current; //ֻ��ȷ��0.001A

					//��������������ľ�ֵ�˲�
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
					//Send_9_2_Current = Send_9_2_Current_reg1; //���˲�ֱ�����

					// /*
					//�˲�����
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
					//Send_9_2_Current=0; ֻ�ǿ�һ�³����ǲ���ִ�е�����
					}

					i=i+1;






				}

/*********************************PID+linear slope control*************************************/
				else if(Tracking_MODE==0){
					PIDParament_Init(); //��Ϊ������ʵʱ���£�������Ҫ�ѳ�ʼ������ֱ��д������
					//�����delta_freq������ֱ�Ӽ���ģ������delta_freq�����������ڵ�
					delta_freq = pid_calc(amplitude); //ͨ��PID������ʵ����̬��������
					if (delta_freq <=0 )
						delta_freq = -delta_freq;
					else delta_freq=delta_freq;//��Ϊ����ֵ

					if (delta_freq >=Limit_freq_shift ) //��Ҫ�ɿ�
						delta_freq = Limit_freq_shift;
					else delta_freq=delta_freq;

					if(Flag_phase_hop==0){
						if(angle>=0){// phase>0������
							MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);//��delta_freq(MHz)ת��ΪdeltaƵ�ʿ�����
						}
						else MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);// phase<0������
					}
					//����������Ҫ�ӱ�־
					else {
						if(angle>=0){
							MW_center_freq_reg1 = MW_center_freq_reg1 - (int)(delta_freq*894784.8533333f);
						}
						else MW_center_freq_reg1 = MW_center_freq_reg1 + (int)(delta_freq*894784.8533333f);
					}



					B_magnet = (delta_freq + (MW_center_freq_reg2-MW_center_freq_reg1)*0.00000111758709f)*1000.0f/(2*2.802f); //�ò�ֵ��ų�ֵ,����ǰ��һ����1000�����������󣬺�����λ���������� 2.802MHz/GsΪ������ �з�����
					B_delta = B_magnet-B_ref;
					//B_magnet = (MW_center_freq_reg2-MW_center_freq_reg1)*1000.0f/(2*2.802f);
					PS_Cal_Current = (B_delta*2*PI*Distance)/u0; //����ֵ����   aΪ���ߺ�NVɫ��λ�ù̶�ʱ����Ծ���,������Gs�ĵ�λ��Ҫ����T,�ڶ���u0��ʱ��ʹ����ˡ�
					//Send_9_2_Current= (int)PS_Cal_Current; //ֻ��ȷ��0.001A

					//��������������ľ�ֵ�˲�
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

					//Send_9_2_Current = Send_9_2_Current_reg1; //���˲�ֱ�����

					// /*
					//�˲�����
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

						//Send_9_2_Current = Send_9_2_Current_reg1; //ֻ�ǿ�һ�³����ǲ���ִ�е�����
					}


					i=i+1;







					//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq) ;//д������Ƶ��S1_FRE_TRACKING_MID_FTW
				}

				//�л�����2
				//MW_center_freq_reg1 = MW_center_freq; //�����1������ɵ�΢������Ƶ��ֵ
				//MW_center_freq = MW_center_freq_reg2; //��ʼ�л�����2
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg2) ;
			}
			//Send_9_2_Current = Send_9_2_Current;
		}
		//state = 1; //��1�������,���߷�2���ٵı�־λ

	}












	//����ڶ�����ĸ��٣����㲿�ֶ��ڷ�2���м���
	//else if(S1_AD9914_MODE == 2 && state == 1)
	 else if(S2_AD9914_MODE == 2){
		if(initial_down == 0){
			//PS��Ƶ�ʸ����㷨������Ƶ�ʼ�����ϣ�����д��PL��AXI�Ĵ���
			//MW_center_freq = MW_center_freq_init_1;
			//�ȸ��궨����ʼ΢������Ƶ��
			MW_center_freq_reg1 = MW_center_freq_init_1;
			MW_center_freq_reg2 = MW_center_freq_init_2;
			NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_init_2) ;//д������Ƶ��S1_FRE_TRACKING_MID_FTW
			//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG40_OFFSET , 0x99111111) ;//д������Ƶ��S2_FRE_TRACKING_MID_FTW
			initial_down = 1;//��ʼ�����
		}
		else if(initial_down==1){
			NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg2) ;//д������Ƶ��S1_FRE_TRACKING_MID_FTW
			//�ڷ�1���ٽ�����ʱ���Ѿ��л�����2
			/**************************************************�������岢�����µĽ����Ϣ***************************************************/
			float delta_freq;//ÿ�ε��ڵ�delta Fre ��λMHz
			float B_magnet;
			float B_delta;
			u32 demod_value_pl;//��ȡʵʱ���ֵ,�޷���24bit
			int demod_phase_pl;//��ȡʵʱ��λֵ,�з���24bit
			u32 Bram_8_11  = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,8);
			u32 Bram_12_15 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,12);
			u32 Bram_32_35 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,32);
			u32 Bram_36_39 = XBram_ReadReg(XPAR_BRAM_0_BASEADDR,36);
			u32 DDS_PINC = (((Bram_32_35 & 0xFFFF0000)>>16) | ((Bram_36_39 & 0x0000FFFF)<<16));
			float DDS_Freq = DDS_PINC*0.0000009313226f;//�ڲ�DDS kHz
			demod_phase_pl = ((Bram_12_15 & 0xFFFFFF00)>>8);//DC1�����λ(signed)��չ��2^23(-1~1)
			if(demod_phase_pl>=8388608){//����λΪ1 �Ǹ���
				demod_phase_pl=demod_phase_pl-16777216;
			}
			float angle=(demod_phase_pl+(DDS_Freq*1470.65268f-1491.73171f))*0.00002145767f;//����ʵʱ����λ��(��λ�Ƕȣ���Labview�߼�һ��)
			demod_value_pl = ((Bram_8_11 & 0xFFFF0000)>>16) | ((Bram_12_15 & 0x000000FF)<<16); //DC1�������(unsigned)��չ��2^24
			float amplitude = demod_value_pl*0.00078899f;//ת��Ϊ����(��λmv����Labview�߼�һ��)


			if(amplitude>Value_model_swich) { //������ֵʱ�����Ը����ȴ���
				//���������ֵ�������趨����ֵʱ���ȶ�������һ�������ԵĴ���(��û��)
				delta_freq = amplitude/Slope_ctrl_reci; //delta_freq��λMHz
				if (delta_freq <=0 )
					delta_freq = -delta_freq;
				else delta_freq=delta_freq;
				if (delta_freq >=Limit_freq_shift ) //��Ҫ�ɿ�
					delta_freq = Limit_freq_shift;
				else delta_freq=delta_freq;
				//�ж���λ��
				if(Flag_phase_hop==0){
					if(angle>=0){// phase>0������
						MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);//��delta_freq(MHz)ת��ΪdeltaƵ�ʿ�����
					}
					else MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);// phase<0������
				}
				else {
					if(angle>=0){
						MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);
					}
					else MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);
				}
				//����0ֵ
				Send_9_2_Current = 1 * 0.0f; //9-2���ķ���ע��ʶ��
				//����9-2���Ĳ�����ô���,Ϊ0�͵��ɵ�֡
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg1) ;//д������Ƶ��S1_FRE_TRACKING_MID_FTW
			}

			else if(amplitude < Limit_deadband) { //������������Ҫ����������ĵ���ֵҲ����
				MW_center_freq_reg2 = MW_center_freq_reg2; //΢��Դ������ ��������ʵ��дҲ
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg1) ;
				//����ע�͵��������Send_9_2_Current���붨��Ϊstatic�����ķ�ʽ
				//MW_center_freq_real = (3500.0f-MW_center_freq*0.00000055879354477);//��ΪMHz.MW_center_freq*0.0000011758709f
				//MW_center_freq_init_real = MW_center_freq_init*0.00000111758709f;//��λ:MHz MW_center_freq_init*0.00000111758709f
				//B_magnet = (delta_freq + (MW_center_freq_real-MW_center_freq_init_real))*1000.0f/2.802f; //�ų�ֵ,1000�����������󣬺�����λ���������� 2.802MHz/GsΪ������ �з�����
				//PS_Cal_Current = (B_magnet*2*PI*Distance)/u0; //����ֵ����   aΪ���ߺ�NVɫ��λ�ù̶�ʱ����Ծ���,������Gs�ĵ�λ��Ҫ����T,�ڶ���u0��ʱ��ʹ����ˡ�
				//Send_9_2_Current = (int)PS_Cal_Current;
				Send_9_2_Current = Send_9_2_Current;
			}
			else {

				if(Tracking_MODE==1){
//					delta_freq = amplitude/Slope_ctrl_reci; //б�ʾ���ֵ
//					if (delta_freq <=0 )
//						delta_freq = -delta_freq;
//					else delta_freq=delta_freq; //��Ϊ����ֵ������λ������
//					if(Flag_phase_hop==0){
//						if(angle>=0){// phase>0��
//							delta_freq = delta_freq; //���ӽ������ƫ��
//						}
//						else{ // phase<0������
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
					Fuzzytrans(0,amplitude,pre_Measure_Vaule); //(����ֵ������ֵ����һ�εĲ���ֵ)
					pre_Measure_Vaule=amplitude;
					delta_freq = fuzzypid_calc(amplitude);
					if (delta_freq <=0 )
						delta_freq = -delta_freq;
					else delta_freq=delta_freq;//��Ϊ����ֵ

					if (delta_freq >=Limit_freq_shift ) //��Ҫ�ɿ�
						delta_freq = Limit_freq_shift;
					else delta_freq=delta_freq;

					if(Flag_phase_hop==0){
						if(angle>=0){// phase>0������
							MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);//��delta_freq(MHz)ת��ΪdeltaƵ�ʿ�����
						}
						else MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);// phase<0������
					}
					//����������Ҫ�ӱ�־
					else {
						if(angle>=0){
							MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);
						}
						else MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);
					}

					//�����ļ��㲿��,freq����ϵ����һ������Ϊ��һ����չ����31λ����һ����չ��32λ
					//MW_center_freq_reg2_real = (3500.0f-MW_center_freq_reg1*0.00000055879354477);//��ΪMHz.MW_center_freq*0.0000011758709f��ʵ�ʾ���reg2��ʵ��Ƶ��
					//MW_center_freq_reg1_real = (3500.0f-MW_center_freq_reg1*0.00000055879354477);
					//MW_center_freq_init_real = MW_center_freq_init*0.00000111758709f;//��λ:MHz MW_center_freq_init*0.00000111758709f
					B_magnet = (delta_freq + (MW_center_freq_reg2-MW_center_freq_reg1)*0.00000111758709f)*1000.0f/(2*2.802f); //�ò�ֵ��ų�ֵ,����ǰ��һ����1000�����������󣬺�����λ���������� 2.802MHz/GsΪ������ �з�����
					B_delta = B_magnet-B_ref; //�����ⲿ��������Ĵų�ƫ��
					//B_magnet = (MW_center_freq_reg2-MW_center_freq_reg1)*1000.0f/(2*2.802f);
					PS_Cal_Current = (B_delta*2*PI*Distance)/u0;  //����ֵ����   aΪ���ߺ�NVɫ��λ�ù̶�ʱ����Ծ���,������Gs�ĵ�λ��Ҫ����T,�ڶ���u0��ʱ��ʹ����ˡ�

					//Send_9_2_Current= (int)PS_Cal_Current; //ֻ��ȷ��0.001A

					//��������������ľ�ֵ�˲�
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
					//Send_9_2_Current = Send_9_2_Current_reg1; //���˲�ֱ�����

					// /*
					//�˲�����
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
					//Send_9_2_Current=0; ֻ�ǿ�һ�³����ǲ���ִ�е�����
					}

//					//�����delta_freq������ֱ�Ӽ���ģ������delta_freq�����������ڵ�
//					Fuzzytrans(0,amplitude,pre_Measure_Vaule); //(����ֵ������ֵ����һ�εĲ���ֵ)
//					pre_Measure_Vaule=amplitude;
//					delta_freq = fuzzypid_calc(amplitude);
//					if (delta_freq <=0 )
//						delta_freq = -delta_freq;
//					else delta_freq=delta_freq;//��Ϊ����ֵ
//
//					if (delta_freq >=Limit_freq_shift ) //��Ҫ�ɿ�
//						delta_freq = Limit_freq_shift;
//					else delta_freq=delta_freq;
//
//					if(Flag_phase_hop==0){
//						if(angle>=0){// phase>0������
//							MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);//��delta_freq(MHz)ת��ΪdeltaƵ�ʿ�����
//						}
//						else MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);// phase<0������
//					}
//					//����������Ҫ�ӱ�־
//					else {
//						if(angle>=0){
//							MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);
//						}
//						else MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);
//					}
					//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq) ;//д������Ƶ��S1_FRE_TRACKING_MID_FTW
					i=i+1;
				}
	/***************************************************************PID����ģʽ****************************************/
				if(Tracking_MODE==0){
					PIDParament_Init(); //��Ϊ������ʵʱ���£�������Ҫ�ѳ�ʼ������ֱ��д������
					delta_freq = pid_calc(amplitude); //ͨ��PID������ʵ����̬��������
					if (delta_freq <=0 )
						delta_freq = -delta_freq;
					else delta_freq=delta_freq;//��Ϊ����ֵ

					if (delta_freq >=Limit_freq_shift ) //��Ҫ�ɿ�
						delta_freq = Limit_freq_shift;
					else delta_freq=delta_freq;

					if(Flag_phase_hop==0){
						if(angle>=0){// phase>0������
							MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);//��delta_freq(MHz)ת��ΪdeltaƵ�ʿ�����
						}
						else MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);// phase<0������
					}
					//����������Ҫ�ӱ�־
					else {
						if(angle>=0){
							MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);
						}
						else MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);
					}
					//������ֱ�Ӱѵ���ֵ����ȥ
//					delta_freq = amplitude/Slope_ctrl_reci; //б�ʾ���ֵ
//					if (delta_freq <=0 )
//						delta_freq = -delta_freq;
//					else delta_freq=delta_freq; //��Ϊ����ֵ������λ������
//
//					//Ϊ0�ǣ���ʾODMR�������λΪ�����ұ���λΪ��
//					if(Flag_phase_hop==0){
//						if(angle>=0){// phase>0��
//							delta_freq = delta_freq; //���ӽ������ƫ��
//						}
//						else{ // phase<0������
//							delta_freq = -delta_freq;
//						}
//					}
//					//����������Ҫ�ӱ�־
//					else {
//						if(angle>=0){
//							delta_freq = -delta_freq;
//						}
//						else{
//							delta_freq = delta_freq;
//						}
//					}

					B_magnet = (delta_freq + (MW_center_freq_reg2-MW_center_freq_reg1)*0.00000111758709f)*1000.0f/(2*2.802f); //�ò�ֵ��ų�ֵ,����ǰ��һ����1000�����������󣬺�����λ���������� 2.802MHz/GsΪ������ �з�����
					B_delta = B_magnet-B_ref;
					//B_magnet = (MW_center_freq_reg2-MW_center_freq_reg1)*1000.0f/(2*2.802f);
					PS_Cal_Current = (B_delta*2*PI*Distance)/u0; //����ֵ����   aΪ���ߺ�NVɫ��λ�ù̶�ʱ����Ծ���,������Gs�ĵ�λ��Ҫ����T,�ڶ���u0��ʱ��ʹ����ˡ�
					//Send_9_2_Current= (int)PS_Cal_Current; //ֻ��ȷ��0.001A

					//��������������ľ�ֵ�˲�
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

					//Send_9_2_Current = Send_9_2_Current_reg1; //���˲�ֱ�����

					// /*
					//�˲�����
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

						//Send_9_2_Current = Send_9_2_Current_reg1; //ֻ�ǿ�һ�³����ǲ���ִ�е�����
					}
					 //

					//�����delta_freq������ֱ�Ӽ���ģ������delta_freq�����������ڵ�
//					delta_freq = pid_calc(amplitude); //ͨ��PID������ʵ����̬��������
//					if (delta_freq <=0 )
//						delta_freq = -delta_freq;
//					else delta_freq=delta_freq;//��Ϊ����ֵ
//
//					if (delta_freq >=Limit_freq_shift ) //��Ҫ�ɿ�
//						delta_freq = Limit_freq_shift;
//					else delta_freq=delta_freq;
//
//					if(Flag_phase_hop==0){
//						if(angle>=0){// phase>0������
//							MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);//��delta_freq(MHz)ת��ΪdeltaƵ�ʿ�����
//						}
//						else MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);// phase<0������
//					}
//					//����������Ҫ�ӱ�־
//					else {
//						if(angle>=0){
//							MW_center_freq_reg2 = MW_center_freq_reg2 - (int)(delta_freq*894784.8533333f);
//						}
//						else MW_center_freq_reg2 = MW_center_freq_reg2 + (int)(delta_freq*894784.8533333f);
//					}
					//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq) ;//д������Ƶ��S1_FRE_TRACKING_MID_FTW
					i=i+1;
				}

				//MW_center_freq_reg2 = MW_center_freq; //reg���Ͷ��Ǿ�̬����
				//MW_center_freq = MW_center_freq_reg1;
				//��2�ĵ����Ѿ������ˣ��л�����1
				//NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG25_OFFSET , MW_center_freq_reg1) ;//��ʼ�л�����1

			}
			//state = 0;
		}
	 }

	else if(S1_AD9914_MODE != 2&&S2_AD9914_MODE != 2){
		initial_down = 0;
		//MW_center_freq = MW_center_freq_init; //����ԭ���ĳ���
		MW_center_freq_reg1 = MW_center_freq_init_1;
		MW_center_freq_reg2 = MW_center_freq_init_2;
		B_ref = (MW_center_freq_init_2- MW_center_freq_init_1)*0.00000111758709f*1000.0f/(2*2.802f);
		//MW_center_freq_reg3 = MW_center_freq_init_1;//����
		//MW_center_freq_reg4 = MW_center_freq_init_2;//����
		FPIDParament_Init();//FUZZYPID��ʼ��
	}

	//�˴�����ÿ��ѭ����ִ��  9-2����ģ������յ�PS_Fre_Cal_Already��־
	//use for judge the discard point
	//if(Send_9_2_Current>200000){
	//	//Send_9_2_Current = 0;
	//}

	NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG61_OFFSET , Send_9_2_Current) ;//�����������ֵ
    NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG60_OFFSET , 1) ;//����Ƶ�ʸ����㷨������ϱ�־ PS_Fre_Cal_Already REG60[0]
    NV_FILTER_mWriteReg(PL_BRAM_BASE, NV_FILTER_S00_AXI_SLV_REG60_OFFSET , 0) ;//����Ƶ�ʸ����㷨������ϱ�־ PS_Fre_Cal_Already REG60[0]
	PL_BRAM_WRITE_FINISH=1;//PL��BRAM��д��Ϻ��������ж�   ����ϱ�־����

}


