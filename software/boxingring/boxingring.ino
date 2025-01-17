/*
 * Alternate controler : Boxing ring which acts as a gamepad :
 * You have to touch the strings to press buttons.
 * Board : "ESP32S3 Dev Module", able to manage touch + USB keyboard
 * Libraries : ESP32 3.0.7 
 * Authors : CC and Mila
 * Date : November 2024 ( during CTRL+ALT workshop)
 */

#include "USB.h"
#include "USBHIDKeyboard.h"
USBHIDKeyboard Keyboard;

#define DBGBUTTON 0
//Touchs PINS are  GPIO 1 -14 matching to TOUCH 1 14
// Pins 1  4 7  8 10  14 are chose because they are spaced enough
#define PIN_UP  8    //UP dont change
#define PIN_JMP 4    //JMP
#define PIN_LFT 13   //LFT dont change
#define PIN_DN  7    //DN
#define PIN_ALT 10   //ALT (mais non utilisé)
#define PIN_RGT 1    //RIGHT

//Sorties Leds de debug ; voisines des entrees touch testees
#define PINLED_UP  GPIO_NUM_2   //dont change
#define PINLED_JMP GPIO_NUM_5   //JMP
#define PINLED_LFT GPIO_NUM_15  //dont change
#define PINLED_DN  GPIO_NUM_3   //DN
#define PINLED_ALT GPIO_NUM_11  //ALT (mais non utilisé)
#define PINLED_RGT GPIO_NUM_14  //RIGHT

typedef struct {
  int      touchPin;
  gpio_num_t touchLED;
  int32_t oldTouch   ;
  int32_t curTouch   ;
  int32_t  deltaTouch ;
  int32_t  dynamicThreshold  ;
  int32_t  staticThreshold  ;
  //bool     touchPressed ;
  int      state ;
  int      stateOld ;
} capa;

char     sdbg[128] = {0};
//Variables utilisees pour les differents boutons

//Test avec vitrine STE 240129
capa c_up  = {PIN_UP , PINLED_UP , 0x7FFFFFFF, 0, 0, 4000, 30000, 0, 0};
capa c_dn  = {PIN_DN , PINLED_DN , 0x7FFFFFFF, 0, 0, 4000, 30000, 0, 0};
capa c_lft = {PIN_LFT, PINLED_LFT, 0x7FFFFFFF, 0, 0, 4000, 30000, 0, 0};
capa c_rgt = {PIN_RGT, PINLED_RGT, 0x7FFFFFFF, 0, 0, 4000, 30000, 0, 0};
capa c_jmp = {PIN_JMP, PINLED_JMP, 0x7FFFFFFF, 0, 0, 4000, 30000, 0, 0};
capa c_alt = {PIN_ALT, PINLED_ALT, 0x7FFFFFFF, 0, 0, 4000, 65000, 0, 0};


void setup()
{
  //Sorties LEDS DE DEBUG
  gpio_set_direction(PINLED_UP, GPIO_MODE_OUTPUT);
  gpio_set_direction(PINLED_DN, GPIO_MODE_OUTPUT);
  gpio_set_direction(PINLED_LFT, GPIO_MODE_OUTPUT);
  gpio_set_direction(PINLED_RGT, GPIO_MODE_OUTPUT);
  gpio_set_direction(PINLED_JMP, GPIO_MODE_OUTPUT);
  gpio_set_direction(PINLED_ALT, GPIO_MODE_OUTPUT);

  //masses virtuelles (quick & dirty)
  gpio_set_direction(GPIO_NUM_6, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_6, 0);

  gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_16, 0);

  gpio_set_direction(GPIO_NUM_46, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_46, 0);

  gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_12, 0);

  gpio_set_direction(GPIO_NUM_42, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_42, 0);

  //Serial.begin(115200);
  Serial.begin(1000000);
  Serial.println("Starting USB Keyboard ESP32-S3!");
  //Initialisation
  Keyboard.begin();
  USB.begin();
}

// Gestion d'un appui touche : on envoie juste les commandes pout indiquer
// qu'on a appuyé ou relaché une touche, pour ne pas encombrer la liaison USB ou Bluetooth
// Fonction générique pour toutes les touches
void manageKey(capa * cap, uint8_t KEY)
{
  //Test appuyé ou relaché
  cap->curTouch = touchRead(cap->touchPin);
  cap->deltaTouch = cap->curTouch - cap->oldTouch;

  sprintf(sdbg, "%06d,", cap->curTouch);   Serial.print(sdbg);
  sprintf(sdbg, "%06d,", cap->deltaTouch); Serial.print(sdbg);
  cap->oldTouch = cap->curTouch;

  //if ( ( cap->deltaTouch > cap->dynamicThreshold) && (cap->touchPressed == false) )
  if ( //( cap->deltaTouch > cap->dynamicThreshold)  ||
       ( cap->curTouch > cap->staticThreshold)
     )
  {
    /*Serial.print("PRESS!");
      sprintf(sdbg, "del= %d", cap->deltaTouch); Serial.print(sdbg);
      sprintf(sdbg, "cur= %d", cap->curTouch);   Serial.print(sdbg);
      Serial.println();*/
    cap->state = true;
  }
  else //if ( (cap->deltaTouch < -cap->dynamicThreshold)  )
  {
    /*Serial.print("RELEASE!");
      sprintf(sdbg, "del= %d", cap->deltaTouch); Serial.print(sdbg);
      sprintf(sdbg, "cur= %d", cap->curTouch);   Serial.print(sdbg);
      Serial.println();*/
    cap->state = false;
  }

  // cap->state = cap->touchPressed;

  if (  cap->stateOld != cap->state)
  {
#if DBGBUTTON
    Serial.print(cap->touchPin);
    Serial.print("chgto");
    Serial.println(cap->state);
#endif
    if (cap->state == HIGH)
    {
      Keyboard.press(KEY);
      gpio_set_level(cap->touchLED, 1);//
    }
    else
    {
      Keyboard.release(KEY);
      gpio_set_level(cap->touchLED, 0);//
    }
    cap->stateOld = cap->state;
  }
}

void loop()
{
  manageKey( &c_up , KEY_UP_ARROW );
  manageKey( &c_dn , KEY_DOWN_ARROW );
  manageKey( &c_lft, KEY_LEFT_ARROW );
  manageKey( &c_rgt, KEY_RIGHT_ARROW );
  manageKey( &c_jmp, ' ' );
 // manageKey( &c_alt, 'p' );

  Serial.println("0");
  delay(10);

}
