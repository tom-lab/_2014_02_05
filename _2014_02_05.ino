/* Final Beta Version 0.5
 *
 */

#include <SoftwareSerial.h>
SoftwareSerial LEDSerial(20, 21);

//Einstellbare Werte
const long Waagezeit_Konstante = 120000; //Wartezeit bei fehlendem Statuswechsel bis Reset
const long RelativMaxWeg = 12000;   //Maximale Auslenkung Waagen (x2 weil 2 gestaffelte Waagen)
const long WegMinimum = 2000;       //Minimale Auslenkung Waagen
const int G_SchwellwertMin = 2;     //Gewichtsänderungen ab Wert * 10g werden registriert
const int Minimalgewicht = 10;      //wenn Gewicht < Minimalgewicht dann keine Lösung möglich  
const int G_Max = 300;            //Maximalgewicht (Wert*10g), für Skalierung Beschleunigung, V_Max und Fehlererkennung 
const int G_MaxChange = 100;      //Bei spontaner Gewichtsänderung größer Wert stoppen die Motoren (Fehlererkennung) 
const long FaktorGewicht = 200;   //Umrechnung Gewichtsdifferenz auf Weg (GetStatus) (Bei 500g Gewichtsdifferenz maximale Abstände zwischen Schalen)
const float BremsFaktor = 4.;     //Verhältnis Beschleunigung zu Bremse
const float a_Min = 1.;           //Minimalbeschleunigung in Hz/ms; war 2.
const float a_Max = 6.;           //Maximalbeschleunigung in Hz/ms; 
const int Schwellwert = 150;      //zulässige Differenz zur Zielposition


//const long BremsWegMax = 10000; //optional falls reale Geschwindigkeit einbezogen wird


//Pin Definition

  int chipDriver = 2;                     //RS485 Treiber für Gewichtstransmitter (HalbDuplex)
  int EndSchalterPin[5]={0,40,42,52,50};  //Induktive Endschalter
  int MagnetPin[5]={0,51,53,41,43};       //Zugmagnete
  int NotAusPin = 3;                      //NotTaste vom Kontrollraum aus

//Ende Pin Definition

int StartWert = 1;
int DoneFlag = 0;

boolean GewichtChange4 = false;

int G_Schwellwert = G_SchwellwertMin;      //Gewichtsänderungen ab Wert * 10g werden registriert
long Waagezeit_inaktiv;   


int Durchlauf[5];    //Für Pendelsimulation Anzahl der Durchläufe zur Zielposition

int MotorOben = 1;  //aktuell höchster Motor zur Übergabe der Fahrtcharakteristika an Motor4  

float a_Ref;
float a_Aktuell[5];
long b_Value[5];
long a_Value[5];

const long v_Max = 40000; //war 25000

int EndSchalter[5];

int KippStatus[5];

long Position[5]={0,0,0,0,0};
long PositionAlt[5];
long PositionSoll[5];

//-----------------Für Berechnung aktuelle Geschwindigkeit und Bremsweg
/*
long v_Ref;
long v_Aktuell[5];
const long v_Min = 2000;
long Bremsweg[5];
float Speed[5];
long TimeAlt[5];
long TimeNeu[5];
*/
//-----------------

int Richtung[5];

boolean M_Done[5]={true, true, true, true, true};

//-----

long Gewicht[5];
long GewichtAlt[5];

int GewichtStatus[5];    //Status 0: Normal, Status 1: ausgelesener Wert rubbish, Status 2: Gewicht zu hoch, Status 3: anything else 

int G_Change = 0;

int StatusNeu;
int StatusAlt; 
boolean StatusChange = false;
//-----

long drehwert = 0;

long kippPosition = -45000;
long kippPosUnten = -80000;

long ErrorDist = -30000;    //wenn Gewicht auf Schaale 4 dann fährt ein Motor um ErrorDistanz nach oben

long PosOben = -35000;
long PosMitteOben = -45000;
long PosMitte = -55000;
long PosMitteUnten = -65000;
long PosUnten = -75000;

long AnschlagOben = -10000;    //Schutzwert falls Waageschale zu hoch oder zu tief fährt
long AnschlagUnten = -93000;

long WegW1, WegW2;

long StartPosition[5]={0, PosMitteOben, PosMitteUnten, PosMitteUnten, PosMitte};


//------------------------------------------------------------------------------------
/*
 _______  _______ _________          _______ 
(  ____ \(  ____ \\__   __/|\     /|(  ____ )
| (    \/| (    \/   ) (   | )   ( || (    )|
| (_____ | (__       | |   | |   | || (____)|
(_____  )|  __)      | |   | |   | ||  _____)
      ) || (         | |   | |   | || (      
/\____) || (____/\   | |   | (___) || )      
\_______)(_______/   )_(   (_______)|/       
*/                                       

void setup()
{
  Serial.begin(115200); //Rechner
  Serial2.begin(115200); //Motoren
  Serial3.begin(115200); //Gewichtstransmitter
  LEDSerial.begin(38400);  //Softwareserial für LEDs und Musikshield

  
  pinMode(chipDriver,OUTPUT);
  digitalWrite(chipDriver, LOW);    //LOW setzt RS485 Chip auf Listen 

  for(int i=1; i<5; i++) {
    pinMode(MagnetPin[i], OUTPUT);  
    pinMode(EndSchalterPin[i], INPUT);
  }
  
  pinMode(NotAusPin, INPUT);
  
  LEDSerial.print("kerze;");       //"kerze" bedeutet flackern
  
  Waagezeit_inaktiv = millis();  
  
  Serial.println("setup finished");  
}





//----------------------------------------------------------------------------------------------------------------------------------
/*
 _        _______  _______  _______ 
( \      (  ___  )(  ___  )(  ____ )
| (      | (   ) || (   ) || (    )|
| |      | |   | || |   | || (____)|
| |      | |   | || |   | ||  _____)
| |      | |   | || |   | || (      
| (____/\| (___) || (___) || )      
(_______/(_______)(_______)|/  
*/

void loop()
{

StatusChange = false;  


//----------Check ob Neukalibirierung erforderlich (Berührung Endschalter oder lange Inaktiv)------------

            //Endschalter einlesen, wenn Kontakt dann Neukalibrierung  


  
            for (int i=1;i<=4;i++){                            
              EndSchalter[i] = digitalRead(EndSchalterPin[i]);
            }
              
            if ((EndSchalter[1] == 0)||(EndSchalter[2] == 0)||(EndSchalter[3] == 0)||(EndSchalter[4] == 0)) {  //wenn Endschalter Kontakt Neukalibrierung 
             StartWert = 1; 
            }
          
            if ((millis()-Waagezeit_inaktiv) > Waagezeit_Konstante){
             StartWert = 1; 
            }
  
//--------------------------------------------------------------------------------------------------------------
  
if (StartWert == 1){    
        calibration();
     }
      
    
else {
  
 //------------Gewicht einlesen und Gewichtsänderung feststellen-------------------------------------------------
 
 
  
  G_Change = 0;          //Gewichtswechsel-Flag auf NULL setzen
 
  for (int j=1;j<=4;j++){
    weightRead(j);//Gewicht einlesen
    delay(5);//Pause zum nächsten Transmitter
    if (Gewicht[j] < 0){Gewicht[j] = 0;}
    }
    
    
    
    for (int j=1;j<=4;j++){   
    
          if (abs(GewichtAlt[j]-Gewicht[j])>G_Schwellwert){
            G_Change = 1;
            GewichtAlt[j] = Gewicht[j];  

            if (abs(GewichtAlt[j]-Gewicht[j])>(G_MaxChange)){
      
                G_Change = 0;
                MotorHardStop(1);
                MotorHardStop(2);
                MotorHardStop(3);
                MotorHardStop(4);
                }            
            }     
    }
   
//-----------Ende Gewicht einlesen---------------------------------------------------
 
 
//-----------Begin Statusabfrage (neue SollPositionen für Motoren)------------------- 
 
  
      if ((G_Change == 1)&&(DoneFlag != 1)){      //Wenn Gewichtsänderung dann Motor-Werte aktualisieren 

          Waagezeit_inaktiv = millis();           //Falls StatusÄnderung Reset Waagezeit_inaktiv (für Restart nach bestimmter Inaktivzeit)     
          StatusNeu = GetStatus(Gewicht[1], Gewicht[2], Gewicht[3], Gewicht[4]);    //Alle aktuellen Gewichtswerte an Funktion "Getstatus" übergeben
                                                                                    //Funktion gibt Wert zwischen 1 und 15 zurück (SollPositionen)         
      
          if (StatusNeu == 15) StatusNeu = StatusAlt;                           //Wenn Minimalgewicht nicht erreicht Status belassen
       
          SetNewStatus(StatusNeu, Gewicht[4]);                                  //Aus Statusnummer jeweilige SollPositionen setzen
          StatusAlt = StatusNeu;
          StatusChange = true;
      
      
          for (int i=1; i<=4;i++){
          Durchlauf[i] = 0;        //Für Nachschwingdurchgänge beim Bewegen
          }

      }
    
   //-------------- Sonderfall Rätsel gelöst  
      
      else if (StatusNeu == 14){              //DONE!!!
	DoneFlag = 1;
        LEDSerial.print("blink;"); 
      }
      
      else if (StatusNeu == 13){
        Waagezeit_inaktiv = millis();           //Falls StatusÄnderung Reset Waagezeit_inaktiv (für Restart nach bestimmter Inaktivzeit)

      }
            
      
   //-------------- Ende Sonderfall
   
//------------------Ende StatusZuweisung--------------------------------------------------
  

//------------------Begin Motorfahrt / Fahrtkontrolle-------------------------------------

  if ((StatusChange == true) || (M_Done[1] == false) || (M_Done[2] == false) || (M_Done[3] == false) || (M_Done[4] == false)){
  MotorFahrt();
  }
  
  if ((DoneFlag == 1) && (M_Done[1] == true) && (M_Done[2] == true) && (M_Done[3] == true) && (M_Done[4] == true)){
    delay(3000);
    LEDSerial.print("kerze;");       
    DoneFlag = 0;
    calibration();
    delay(1000);
  }
  
  
  if (G_Change == 1) UpdateMotorCharacter();    //Bei Gewichtsänderung MotorWerte (Beschleunigung, MaximalSpeed, Bremsbeschleunigung aktualisieren)
  
  
  
}//----------------------Ende Normalschleife (falls kein Endschalter gedrückt) 
   
} //---------------------End of loop
















