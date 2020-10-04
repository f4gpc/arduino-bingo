//********************************************************************************************
//-----------ECRAN TRAFIQUE------------------------------------------------------
//*********************************************************************************************

void ECRtrafique (){
  tft.fillScreen(BLACK);
  tft.setTextSize(4);
  tft.setCursor(0, 10);
  tft.setTextColor(GREEN,BLACK);
  tft.println("BINGO39 DUE BY F4GPC");
 // tft.println();
  if (PAS == 10)    tft.drawLine(362, (PosiFR + 80), 412, (PosiFR + 80),YELLOW );
  if (PAS == 100)   tft.drawLine(302, (PosiFR + 80), 350, (PosiFR + 80),YELLOW );
  if (PAS == 1000)  tft.drawLine(240, (PosiFR + 80), 290, (PosiFR + 80),YELLOW );
  //-----------------------------------------------------
  // S-METRE
  tft.drawLine(0, 319, 479, 319, RED);
  for (int i=0; i <= 7; i++){ 
    tft.drawLine((i*68), 299, (i*68), 319, RED);
    tft.drawLine(((i*68)+34), 306, ((i*68)+34), 319, RED);
  }

  tft.setTextSize(2);
  tft.setTextColor(WHITE,BLACK);
  tft.setCursor(0, 282);
  tft.println("S");
  tft.setCursor(64, 282);
  tft.println("3");
  tft.setCursor(132, 282);
  tft.println("5");
  tft.setCursor(200, 282);
  tft.println("7");
  tft.setCursor(268, 282);
  tft.println("9");
  tft.setTextColor(RED,BLACK);
  tft.setCursor(320, 282);
  tft.println("+20");
  tft.setCursor(388, 282);
  tft.println("+40");
  tft.setCursor(440, 282);
  tft.println("+60");
  
  
 //------------------------------------------------------ 
  if (SSb == 0)
  {
    Fr = (FiLSB - FRtx);
    tft.setTextColor(CYAN,BLACK);
    tft.setCursor(0, 50);
     tft.println("LSB");
  }
  else
  {
    Fr = (FRtx + FiUSB);
    tft.setTextColor(GREEN,BLACK);
    tft.setCursor(0, 50);
    tft.println("USB");
  }

  tft.setTextColor(GREEN,BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, (PosiFR - 20));
  tft.println("VFO A");
  tft.setTextColor(WHITE,BLACK);
  tft.setCursor(0, PosiVFO2);
  tft.println("VFO");
  tft.println(" B ");
  tft.setTextColor(WHITE,BLACK);
  afVFOA (WHITE);
  afVFOB (WHITE);
  sMetreSlo (0);
  sMetre (0);
  
}
