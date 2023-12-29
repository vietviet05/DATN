#include "display.h"
#include "ssd1306.h"
#include "MathExtern.h"
#include "main.h"
#include "UART.h" 
#include "RTC.h" 
#include "Button.h" 
#include "key.h"
//extern uint8_t onDisplay[2];
uint8_t onDisplay[2] ;
uint8_t times ;
void display() {
  if (display_se == 0) {
    valueShow();
  }

  if (display_se == 1) {
    menuShow();
  }

  if (display_se == 2 ) { 
    setPointShow(menu);
  }
  if (display_se == 3 ) {  
    readAndDisplayPPMArrayFromFlash();
  }
  SSD1306_UpdateScreen();
}

void valueShow() { 
	  if (gas_change == 0)
		{
			SSD1306_GotoXY (1, 50);  
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_Puts((char *)numberToString(hrtc1.date_time.hour), &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Puts (":", &Font_7x10, 1); 
		SSD1306_Puts((char *)numberToString(hrtc1.date_time.min), &Font_7x10, SSD1306_COLOR_WHITE); 
		SSD1306_Puts (":", &Font_7x10, 1); 
		SSD1306_Puts((char *)numberToString(hrtc1.date_time.second), &Font_7x10, SSD1306_COLOR_WHITE); 
		SSD1306_GotoXY (1,1);  
		SSD1306_Puts ("LPG:", &Font_7x10, 1);  
		SSD1306_Puts((char *)numberToString(ppm), &Font_11x18, SSD1306_COLOR_WHITE); 
    SSD1306_Puts ("ppm", &Font_7x10, 1);  
		SSD1306_Puts("             ", &Font_7x10, SSD1306_COLOR_WHITE);
}  
		if (gas_change == 1) {
		SSD1306_GotoXY (1, 50);  
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_Puts((char *)numberToString(hrtc1.date_time.hour), &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Puts (":", &Font_7x10, 1); 
		SSD1306_Puts((char *)numberToString(hrtc1.date_time.min), &Font_7x10, SSD1306_COLOR_WHITE); 
		SSD1306_Puts (":", &Font_7x10, 1); 
		SSD1306_Puts((char *)numberToString(hrtc1.date_time.second), &Font_7x10, SSD1306_COLOR_WHITE); 
		SSD1306_GotoXY (1,1);  
		SSD1306_Puts ("CH4:", &Font_7x10, 1);  
		SSD1306_Puts((char *)numberToString(ppm), &Font_11x18, SSD1306_COLOR_WHITE); 
    SSD1306_Puts ("ppm", &Font_7x10, 1);   
		SSD1306_Puts("             ", &Font_7x10, SSD1306_COLOR_WHITE);
		}
//  SSD1306_GotoXY(10,15);
//  SSD1306_Puts("Air: ", &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts((char *)numberToString(airHum), &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts(".", &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts((char *)numberToString((airHum - (uint16_t)airHum) * 10), &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts("%", &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts("             ", &Font_7x10, SSD1306_COLOR_WHITE);

//  SSD1306_GotoXY(10,30);
//  SSD1306_Puts("Humi: ", &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts((char *)numberToString(pHHum), &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts("%", &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts("             ", &Font_7x10, SSD1306_COLOR_WHITE);

//  SSD1306_GotoXY(10,45);
//  SSD1306_Puts("Light: ", &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts((char *)numberToString(lightHum), &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts("Lux", &Font_7x10, SSD1306_COLOR_WHITE);
//  SSD1306_Puts("             ", &Font_7x10, SSD1306_COLOR_WHITE);
}

void menuShow() {
     SSD1306_Fill(SSD1306_COLOR_BLACK);
		Ro = 5*RL/(adc1*3.3/4095)-RL;
		SSD1306_GotoXY (1,1); // goto 10, 10 
    SSD1306_Puts ("Calibration", &Font_7x10, 1); // print Hello  
    SSD1306_GotoXY (1,10); // goto 10, 10 	
		SSD1306_Puts ("completed", &Font_7x10, 1); // print Hello 	
    SSD1306_Puts((char *)numberToString(Ro), &Font_7x10, SSD1306_COLOR_WHITE);	
}

void setPointShow(uint8_t line) {  	     
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(1,1); 
  SSD1306_Puts("Review Value  ", &Font_7x10, line == 1 ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE);
	SSD1306_Puts("             ", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(1,15);
  SSD1306_Puts("Set Time   ", &Font_7x10, line == 2 ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE);
	SSD1306_Puts("             ", &Font_7x10, SSD1306_COLOR_WHITE);
//  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == 0 && HAL_GetTick() - times > 1000) 
//	{ 
//		menu ++; 
//	  times = HAL_GetTick();}	
//	btn_pressing_callback(Button_Typdef *ButtonX) {
//	}
	SSD1306_UpdateScreen();
//  float values[4] = {tempSP, airHumSP, pHHumSP, lightHumSP};

//  SSD1306_GotoXY(10,0);
//  SSD1306_Puts("  Set Point           ", &Font_7x10,SSD1306_COLOR_WHITE);

//  SSD1306_GotoXY(10,30);

//  if (offReceive == 1) {
//    SSD1306_Puts(" Wait to set                 ", &Font_7x10,SSD1306_COLOR_WHITE);
//  } else {
//    if (values[type] < 100) {
//      SSD1306_Puts("     ", &Font_7x10,SSD1306_COLOR_WHITE);
//      SSD1306_Puts((char *)numberToString(SP), &Font_7x10,SSD1306_COLOR_WHITE);
//    }
//    if (values[type] < 10000 && values[type] >= 100) {
//      SSD1306_Puts("    ", &Font_7x10,SSD1306_COLOR_WHITE);
//      SSD1306_Puts((char *)numberToString(SP), &Font_7x10,SSD1306_COLOR_WHITE);
//    }
//    SSD1306_Puts("                    ", &Font_7x10,SSD1306_COLOR_WHITE);
//  }


//  SSD1306_GotoXY(10,15);
//  SSD1306_Puts("                    ", &Font_7x10,SSD1306_COLOR_WHITE);

//  SSD1306_GotoXY(10,45);
//  SSD1306_Puts("                    ", &Font_7x10,SSD1306_COLOR_WHITE);
}