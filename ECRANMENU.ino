//**********************************************************************************************************************
//-------------- ECRAN MENU--------------------------------------------------------------------------------------------
//*********************************************************************************************************************
void ECRtmenu (){
  int customKey ;
int  val=1;
  tft.fillScreen(BLACK);
  tft.setTextSize(3);
  tft.setCursor(0, 10);
  tft.setTextColor(GREEN,BLACK);
  tft.println("1 > Smetre");
  tft.println("2 > m");
  tft.println("3 > ");
  tft.println("4 > ");
  tft.println("5 > ");
  tft.println("6 > ");
  tft.println("7 > ");
  tft.println("8 > ");
  tft.println("9 > ");
  tft.println("0 > HELP");
  tft.println("C > RETOUR");
  
  
  
  //---------------------------------------------------------------
  do{
  customKey = customKeypad.getKey();
  tft.setCursor(250, 10);
  if (siV == 0) tft.print("(rapide)");
  else tft.print("(lent)  ");

  
// clavié----------------------------------------------------------
   if (customKey){ 
    Serial.println(customKey);
     if (customKey == 67){ //C
      val=0;
     }
     if (customKey == 49){ // 49 POUR 1 COMENDE TIPE S-METRE
      if (siV == 0){
        siV =1;
        tft.drawLine(siM, 309, siM, 317,BLACK );
        tft.drawLine((siM + 1), 309, (siM + 1), 317,BLACK );
      }
      else{
        siV =0;
      }
     }
     if (customKey == 50){ //2
      dueFlashStorage.write(0, 254);
       Serial.print("teste : ");
       Serial.println(dueFlashStorage.read(0));
     }
   }
  } while (val);
  ECRtrafique ();
}
