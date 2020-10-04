/************************************************************************************************************************
 * --------------------------GESTION DE LA MEMOIRE EEPROM----------------------------------------------------------------
 ************************************************************************************************************************/
void EEPROMIniti(){
   uint8_t FirstTime = dueFlashStorage.read(0); // flash bytes will be 255 at first run
  Serial.print("code exécution pour la première fois: ");
  if (FirstTime) {
    Serial.println("oui");
    //EEPROMWriteInt(address, positionpoursauv);
      EEPROMWriteInt(1, FiLSBm);
      EEPROMWriteInt(5, FiUSBm);
      EEPROMWriteInt(9, trimFreqm);

    // écrire 0 à l'adresse 0 pour indiquer que ce n'est plus la première fois
    dueFlashStorage.write(0, 0);
  }
  else {
    Serial.println("non");
   /* FiLSB = EEPROMReadInt(1);
    FiUSB = EEPROMReadInt(5);
    trimFreq = EEPROMReadInt(9); */
  } 
 
}
void EEPROMWriteInt(int adresse, int val){
   Serial.println(val);
    //découpage de la variable val qui contient la valeur à sauvegarder en mémoire
    unsigned char faible = (val & 0x00FF); //récupère les 8 bits de droite (poids faible
    val=(val >> 8) ;
    unsigned char fort = (val & 0x00FF);  //décale puis récupère les 8 bits de gauche (poids fort)
    val=(val >> 8) ;
    unsigned char fort2 = (val & 0x00FF);
    val=(val >> 8) ;
    unsigned char fort3 = (val & 0x00FF);
    //puis on enregistre les deux variables obtenues en mémoire
    dueFlashStorage.write(adresse, fort3) ;
    dueFlashStorage.write(adresse+1, fort2) ;
    dueFlashStorage.write(adresse+2, fort) ; //on écrit les bits de poids fort en premier
    dueFlashStorage.write(adresse+3, faible) ; //puis on écrit les bits de poids faible à la case suivante
    
}

int EEPROMReadInt(int adresse){
    int val1 = 0 ; // int, vide qui va contenir le résultat de la lecture
    unsigned char fort3 = dueFlashStorage.read(adresse);
    unsigned char fort2 = dueFlashStorage.read(adresse+1);
    unsigned char fort = dueFlashStorage.read(adresse+2);     //récupère les 8 bits de gauche (poids fort)
    unsigned char faible = dueFlashStorage.read(adresse+3); //récupère les 8 bits de droite (poids faible)
    //assemblage des deux variable précédentes
    val1 = fort3 ;         
    val1 = (val1 << 8) ;
    val1 = (val1 | fort2) ;         
    val1 = (val1 << 8) ;
    val1 = (val1 | fort) ;         
    val1 = (val1 << 8) ;     
    val1 = (val1 | faible) ;
    
    return val1;
}
