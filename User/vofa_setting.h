#ifndef _VOFA_SETTING_H_
#define _VOFA_SETTING_H_
/*
修改VOFASendLen选择发送数据的长度
修改VOFAClearTime选择超时清空的时间（根据定时调用的时间更改）
修改VOFAClearLen选择超长丢弃的长度（一般不用改）
*/

extern float VofaData[16];//需要赋给控件的变量extern过来
extern float tempFloat[25];

#define VOFASendLen 16    //定义发送信号长度
#define VOFAClearTime 50 //定义超时清空时间  = VOFAOutTime*一次中断时间(0~255)
#define VOFAClearLen 20  //定义超长丢弃长度  (0~63)

/*需要的的滑块不需要注释，不需要用的要注释掉*/

#define UseVOFASlider1
#define UseVOFASlider2
#define UseVOFASlider3
#define UseVOFASlider4
#define UseVOFASlider5
#define UseVOFASlider6
//#define UseVOFASlider7

/*需要的的按键则取消注释，不需要用的则注释掉*/

// #define UseVOFAButton1
//#define UseVOFAButton2
//#define UseVOFAButton3
//#define UseVOFAButton4
//#define UseVOFAButton5
//#define UseVOFAButton6
//#define UseVOFAButton7
//#define UseVOFAButton8

/*需要的的按键则取消注释，不需要用的则注释掉*/

// #define UseVOFAKey1     //手自动切换
// #define UseVOFAKey2
//#define UseVOFAKey3
//#define UseVOFAKey4

/*需要的的遥控器则取消注释，不需要用的则注释掉*/

#define UseVOFABar1
// #define UseVOFABar2

//VOFA里要用到什么调参模块   把他的模块代码注释取消掉  然后在下方把模块传来的值赋到VofaData里

#ifdef UseVOFASlider1
#define Vofa_Slider1 VofaData[0]//滑块1的值存入VofaData[0]
#endif
#ifdef UseVOFASlider2
#define Vofa_Slider2 VofaData[1]
#endif
#ifdef UseVOFASlider3
#define Vofa_Slider3 VofaData[2]
#endif
#ifdef UseVOFASlider4
#define Vofa_Slider4 VofaData[3]
#endif
#ifdef UseVOFASlider5
#define Vofa_Slider5 VofaData[4]
#endif
#ifdef UseVOFASlider6
#define Vofa_Slider6 VofaData[5]
#endif
#ifdef UseVOFASlider7
#define Vofa_Slider7 VofaData[6]
#endif

#ifdef UseVOFAButton1
#define Vofa_Button1 VofaData[7]
#endif
#ifdef UseVOFAButton2
#define Vofa_Button2
#endif
#ifdef UseVOFAButton3
#define Vofa_Button3
#endif
#ifdef UseVOFAButton4
#define Vofa_Button4
#endif
#ifdef UseVOFAButton5
#define Vofa_Button5
#endif
#ifdef UseVOFAButton6
#define Vofa_Button6
#endif
#ifdef UseVOFAButton7
#define Vofa_Button7
#endif
#ifdef UseVOFAButton8
#define Vofa_Button8
#endif

#ifdef UseVOFAKey1
#define Vofa_Key1 VofaData[6]
#endif
#ifdef UseVOFAKey2
#define Vofa_Key2 VofaData[7]
#endif
#ifdef UseVOFAKey3
#define Vofa_Key3
#endif
#ifdef UseVOFAKey4
#define Vofa_Key4
#endif

#ifdef UseVOFABar1
#define Vofa_Bar_x1 VofaData[7]
#define Vofa_Bar_y1 VofaData[8]
#endif
#ifdef UseVOFABar2
#define Vofa_Bar_x2 VofaData[9]
#define Vofa_Bar_y2 VofaData[10]
#endif

#endif
