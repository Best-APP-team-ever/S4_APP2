#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>
#include "song.hpp"
#include "pcm_audio.hpp"

/*

Cher(e) stagiaire de H24,
Désolé, j'ai passé tout mon stage à prototyper le PCB et j'ai manqué de
temps pour m'occuper du logiciel. Tu devrais être pas loin de pouvoir
faire jouer quelque chose sur le haut-parleur; les librairies sont là,
mais je suis nul avec FreeRTOS... Bonne chance!

- Stagiaire de A23
- Certified Stinky boi

*/

using Sawtooth = Oscil<SAW2048_NUM_CELLS, SAMPLE_RATE>;
using SquareWv = Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, SAMPLE_RATE>;

#define PIN_SW1 2
#define PIN_SW2 3
#define PIN_RV1 A0
#define PIN_RV2 A1
#define PIN_RV3 A2
#define PIN_RV4 A3

SquareWv squarewv_;
Sawtooth sawtooth_;

float f = 1.0; //fréquence de coupure PIN_RV3
float q = 0.5; //fréquence de résonnance PIN_RV4
float fb = (q + (q/(1.0-f)));
int16_t b1 = f*f * 256;
int16_t a1 = (2-2*f+f*fb-f*f*fb) * 256;
int16_t a2 = -(1-2*f+f*fb+f*f-f*f*fb) * 256;
/* ******************* DEFINE ISR FUNCTION & VARIABLE *********************** */
// Task handle for the task to be notified
TaskHandle_t xButtonSW1TaskHandle = NULL;
TaskHandle_t xButtonSW2TaskHandle = NULL;

void toggleVarSW1();
void toggleVarSW2();

/* ******************* DEFINE TASK FUNCTION *********************** */
// Define a structure to hold the task parameters
typedef struct {
    uint8_t pin;
    uint32_t frequency; // Frequency in Hz
} TaskParams;

void TaskBlink( void *pvParameters );

void TaskBufferManip(void *pvParameters);

void ButtonSW1Task(void *pvParameters);
void ButtonSW2Task(void *pvParameters);

void TaskPlaySong(void *pvParameters);

/* ***************** END DEFINE TASK FUNCTION ********************* */

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

int8_t processVCF(int8_t input)
{
  static int8_t y1 = 0;
  static int8_t y2 = 0;

  int16_t y0 = b1 * input + a1 * y1 + a2 * y2;

  int8_t output = 0xFF & (y0 >> 8);

  y2 = y1;
  y1 = output;

  return output;
}

int8_t nextSample()
{
    // VCO
    int8_t vco = sawtooth_.next() + squarewv_.next();
    
    // VCF (activated)
    int8_t vcf = processVCF(vco);

    // VCA (disabled)   
    int8_t vca = vcf; //PIN_RV2

    int8_t output = vca;

    return output;
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT); //Push button SW1
    pinMode(PIN_SW2, INPUT); //Push button SW2

    pinMode(PIN_RV1, INPUT); //tempo
    pinMode(PIN_RV2, INPUT); //durée VCA
    pinMode(PIN_RV3, INPUT); //fréquence coupure
    pinMode(PIN_RV4, INPUT); //fréquence resonance
    
    Serial.begin(9600);

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);
    // setNoteHz(440.0); for test

    pcmSetup();

    Serial.println("Synth prototype ready");

/*------------------------- MUSIC BUFFER SETUP -------------------------*/
    xTaskCreate(
        TaskBufferManip
        ,  "Music buffer manipulation" 
        ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL//pas de param envoyé
        ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL );
    
/*------------------------- SONG SETUP -------------------------*/   
    xTaskCreate(
    TaskPlaySong
    ,  "Setting_Notes_To_Play"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

/*------------------------- BUTTON SETUP -------------------------*/
    xTaskCreate(
        ButtonSW1Task
        ,  "Gestion bouton SW1" 
        ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL//pas de param envoyé
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  &xButtonSW1TaskHandle );
    xTaskCreate(
        ButtonSW2Task
        ,  "Gestion bouton SW2" 
        ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL//pas de param envoyé
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  &xButtonSW2TaskHandle );
    // Attach the ISR to SW1
    attachInterrupt(digitalPinToInterrupt(PIN_SW1), toggleVarSW1, RISING);
    // Attach the ISR to SW2
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), toggleVarSW2, FALLING);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}

void loop()
{   
    
}

/*--------------------------------------------------*/
/*----------------------- ISR ----------------------*/
/*--------------------------------------------------*/

void toggleVarSW1()
{
    //need to deactivate all ISR?
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the task that the button was pressed
    vTaskNotifyGiveFromISR(xButtonSW1TaskHandle, &xHigherPriorityTaskWoken);
    //need to activate all ISR?
}

void toggleVarSW2()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the task that the button was pressed
    vTaskNotifyGiveFromISR(xButtonSW2TaskHandle, &xHigherPriorityTaskWoken);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

// 
void TaskPlaySong(void *pvParameters)
{
  //Find array size
  int arraySize = sizeof(song) / sizeof(song[0]);
  Serial.println(arraySize);

  while(1){
    // Incrément en fonction de la grandeur du array
    for(int i=0; i<arraySize ;i++)
    {
      Serial.print("Iteration: ");
      Serial.print(i);
      Serial.print("Setting note: ");
      Serial.println(song[i].freq);
      setNoteHz(song[i].freq);
      //Delais avant prochaine note
      vTaskDelay( (song[i].duration * TEMPO_16T_MS) / portTICK_PERIOD_MS ); 

    }
  }
  

}

void TaskBlink(void *pvParameters) {
    // Cast the argument to the correct type
    TaskParams *params = (TaskParams *)pvParameters;

    pinMode(params->pin, OUTPUT);
    uint32_t delayTime = 1000 / (params->frequency * 2); // Calculate delay time in milliseconds

    while (1) {
        digitalWrite(params->pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(delayTime));
        digitalWrite(params->pin, LOW);
        vTaskDelay(pdMS_TO_TICKS(delayTime));
    }
}

void TaskBufferManip(void *pvParameters) {
    // Cast the argument to the correct type
    //TaskParams *params = (TaskParams *)pvParameters;    
    while (1) 
    {
        if(!pcmBufferFull())
        {
            pcmAddSample(nextSample());
            //Serial.println("Filling up Buffer!");
        }
        else if(pcmBufferFull())
        {
            vTaskDelay(32/portTICK_PERIOD_MS);
            //Serial.println("Buffer is full!");
        }
        else if(pcmBufferEmpty())
        {
            Serial.println("Buffer is empty!"); //gestion à faire plus tard
        }
    }
}

void ButtonSW1Task(void *pvParameters)
{
    for (;;)
    {
        // Wait for the notification (button press)
        setNoteHz(0.0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while(digitalRead(PIN_SW1) == HIGH)
        {
            setNoteHz(440.0);
            //Add debounce delay
            vTaskDelay(16/portTICK_PERIOD_MS);
        }

        //Serial.println("SW1 has been pressed!");
    }
}

void ButtonSW2Task(void *pvParameters)
{
    for (;;)
    {
        // Wait for the notification (button press)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Handle the button press (e.g., toggle an LED)
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        // Optional: Add debounce delay
        vTaskDelay(pdMS_TO_TICKS(50));
        Serial.println("SW2 has been pressed!");
    }
}
