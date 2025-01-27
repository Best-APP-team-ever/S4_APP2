#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>
#include "song.hpp"
#include <semphr.h>
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

#define PIN_RV1 A0  //Tempo
#define PIN_RV2 A1  //durée VCA 0 à 3 sec
#define PIN_RV3 A2  //fréquence de coupure 0 à pi
#define PIN_RV4 A3  //fréquence de resonance 0 à 1

SquareWv squarewv_;
Sawtooth sawtooth_;

float f = 1.0; //fréquence de coupure PIN_RV3
float q = 0.5; //fréquence de résonnance PIN_RV4
float vcaPeriod = 1.0; // 0 à 3
float fb = (q + (q/(1.0-f)));
int16_t b1 = f*f * 256;
int16_t a1 = (2-2*f+f*fb-f*f*fb) * 256;
int16_t a2 = -(1-2*f+f*fb+f*f-f*f*fb) * 256;

//Variable d'état de chanson

/* ******************* DEFINE ISR FUNCTION & VARIABLE *********************** */
// Task handle for the task to be notified
TaskHandle_t xButtonSW1TaskHandle = NULL;
TaskHandle_t xButtonSW2TaskHandle = NULL;
TaskHandle_t xPlaySongTaskHandle = NULL;

void toggleVarSW1();
void toggleVarSW2();

/* ******************* DEFINE TASK FUNCTIONS *********************** */
// Define a structure to hold the task parameters
typedef struct {
    uint8_t pin;
    uint32_t frequency; // Frequency in Hz
} TaskParams;

// Task pour le laboratoire & unused
void TaskBlink( void *pvParameters );
void NoButtonPressedTask(void *pvParameters);

// Task pour le remplissage du buffer
void TaskBufferManip(void *pvParameters);

// Task de gestion des boutons 
void ButtonSW1Task(void *pvParameters);
void ButtonSW2Task(void *pvParameters);
void potentiometerTask(void *pvParameters);

// Task qui set les notes de la chanson
void PlaySongTask(void *pvParameters); 

// Mutex handle
SemaphoreHandle_t MutexPotentiometer;
bool VCA_active = false;     // Tracks whether VCA is active

/* ***************** END DEFINE TASK FUNCTION ********************* */

// Fonction pour set une note
void setNoteHz(float note)
{
    //Serial.print("Note frequency playing is: ");
    //Serial.println(note);
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

// Fonction du VCF
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

// Fonction du VCA
float processVCA(float vcaPeriod)
{
    static uint32_t startTime = 0;  // Tracks when the VCA envelope starts
    float scaleFactor = 1.0;        // Amplitude scaling factor

    // Check if the envelope should start
    if (!VCA_active)
    {
        startTime = millis(); // Start tracking time
    }
    
    if(VCA_active)
    {// Calculate elapsed time since the envelope started
        uint32_t elapsedTime = millis() - startTime;

        // Determine if we're still within the VCA period
        if (elapsedTime < (uint32_t)(vcaPeriod * 1000)) // vcaPeriod in seconds, convert to ms
        {
            // Compute the scaling factor based on elapsed time
            scaleFactor = 1.0 - ((float)elapsedTime / (vcaPeriod * 1000));
        }
        else
        {
            // Envelope is complete; stop the sound
            scaleFactor = 0.0;
            setNoteHz(0.0);
            VCA_active = false; // Reset for the next sound event
        }
    }
    else
    {
        scaleFactor = 1.0;
    }

    return scaleFactor;
}

int8_t nextSample()
{
    // VCO
    int8_t vco = sawtooth_.next() + squarewv_.next();
    
    // VCF (activated)
    int8_t vcf = processVCF(vco);

    // VCA (activated)   
    
    float scaleFactor = processVCA(vcaPeriod);
    //Serial.print("Processing VCA... ");
    //Serial.println(scaleFactor);

    // Scale the VCF output using the computed amplitude scaling factor
    int16_t scaledVCA = vcf * scaleFactor;
    // Clamp the output to int8_t range (-128 to 127)
    int8_t vca = (scaledVCA > 127) ? 127 : (scaledVCA < -128 ? -128 : scaledVCA);


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

    pcmSetup();
    setNoteHz(0);
    Serial.println("Synth prototype ready");
    
/*------------------MUTEX-----------------------*/
    // Create the mutex
    MutexPotentiometer = xSemaphoreCreateMutex();
    if (MutexPotentiometer == NULL) {
        Serial.println("Failed to create mutex!");
        //while (1); // Stay here if mutex creation failed
    }

/*------------------------- MUSIC BUFFER SETUP -------------------------*/
    xTaskCreate(
        TaskBufferManip
        ,  "Music buffer manipulation" 
        ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL//pas de param envoyé
        ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL );
    
/*------------------------- SONG SETUP -------------------------*/   
    xTaskCreate(
    PlaySongTask
    ,  "Setting_Notes_To_Play"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &xPlaySongTaskHandle // Notification from SW2
    );

/*------------------------- BUTTON SETUP -------------------------*/
    xTaskCreate(
        ButtonSW1Task
        ,  "Gestion bouton SW1" 
        ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL//pas de param envoyé
        ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  &xButtonSW1TaskHandle );
    xTaskCreate(
        ButtonSW2Task
        ,  "Gestion bouton SW2" 
        ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL//pas de param envoyé
        ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  &xButtonSW2TaskHandle );
    // Attach the ISR to SW1
    attachInterrupt(digitalPinToInterrupt(PIN_SW1), toggleVarSW1, RISING);
    // Attach the ISR to SW2
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), toggleVarSW2, RISING); 


/*------------------------- POTENTIOMETER SETUP -------------------------*/
    xTaskCreate(
        potentiometerTask
        ,  "Gestion potentiomètres et un peu de math" 
        ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL//pas de param envoyé
        ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL );
    
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
void PlaySongTask(void *pvParameters)
{
    //Find array size
    int arraySize = sizeof(song) / sizeof(song[0]);
    //Serial.println(arraySize);

    for(;;)
    {
        //Wait until notified
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while(digitalRead(PIN_SW2) == HIGH){
            // Incrément en fonction de la grandeur du array
            for(int i=0; i<arraySize; i++)
            {
                if (digitalRead(PIN_SW2) == LOW){
                    break;
                }

                // Serial.print("Iteration: ");
                // Serial.print(i);
                // Serial.print(" Setting note: ");
                // Serial.println(song[i].freq);
                setNoteHz(song[i].freq);
                //Delais avant prochaine note
                // Serial.print("Tempo is: ");
                // Serial.println(tempo);
                // Serial.println((song[i].duration * TEMPO_16T_MS(tempo)) / portTICK_PERIOD_MS);
                vTaskDelay((song[i].duration * TEMPO_16T_MS(tempo)) / portTICK_PERIOD_MS ); 

            }
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
        if(pcmBufferEmpty())
        {
            Serial.println("Buffer is empty!"); //gestion à faire plus tard
        }
        while(!pcmBufferFull())
        {
            if (xSemaphoreTake(MutexPotentiometer, portMAX_DELAY) == pdTRUE)
            {
                pcmAddSample(nextSample());
                xSemaphoreGive(MutexPotentiometer);
            }
            //Serial.println("Filling up Buffer!");
        }
        if(pcmBufferFull())
        {
            vTaskDelay(16/portTICK_PERIOD_MS);
            //Serial.println("Buffer is full!");
        }
        
    }
}

void NoButtonPressedTask(void *pvParameters){
    for(;;){
        //Check si aucune action si oui on joue pas de musique
        if ( (digitalRead(PIN_SW1) == LOW) & (digitalRead(PIN_SW2) == LOW) )
        {
            setNoteHz(0);
        }
        vTaskDelay(64/portTICK_PERIOD_MS);
    }

}

void ButtonSW1Task(void *pvParameters)
{
    for (;;)
    {
        // Wait for the notification (button press)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while(digitalRead(PIN_SW1) == HIGH)
        {
            setNoteHz(750.0);
            //Serial.println("Presently 'SW1' is pressed, setting note to '440.0'Hz. Delay of '16'ms...");
            //Add debounce delay
            taskYIELD();
            vTaskDelay(16/portTICK_PERIOD_MS);
        }
        //falling edge here
        VCA_active = true;

        //Serial.println("SW1 has been pressed!");
    }
}

void ButtonSW2Task(void *pvParameters)
{
    for (;;)
    {
        // Wait for the notification (button press)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //Play music as the button is pressed
        while(digitalRead(PIN_SW2) == HIGH)
        {
            //Notify the play song Task
            //Serial.println("Presently 'SW2' is pressed, playing song. Delay of '16'ms...");
            xTaskNotifyGive(xPlaySongTaskHandle);
            taskYIELD();
            vTaskDelay(16/portTICK_PERIOD_MS);
        }  
        //Stop song
        //falling edge here
        VCA_active = true;

        // Debounce delay
        vTaskDelay(16/portTICK_PERIOD_MS);
    }
}


void potentiometerTask(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100 ms period aka 10Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize the reference time

    for (;;)
    {
        // Request the mutex
        if (xSemaphoreTake(MutexPotentiometer, portMAX_DELAY) == pdTRUE)
        {
            //Serial.println("Presently reading all potentiometers. Delay of'100'ms...");
            //data aquisition
            f = (analogRead(PIN_RV3)/1024.0)*3.1;  //0 à presque pi
            //Serial.print("f: ");
            //Serial.println(f);
            q = (analogRead(PIN_RV4)/1024.0)*0.9;   //0 à presque 1
            //Serial.print("q: ");
            //Serial.println(q);
            vcaPeriod = (analogRead(PIN_RV2)/1024.0)*3.0; //0 à 3
            //Serial.print("vca: ");
            //Serial.println(vcaPeriod);
            tempo = ((analogRead(PIN_RV1)/1024.0)*180.0)+60.0;

            // a little bit of math
            fb = (q + (q/(1.0-f)));
            b1 = f*f * 256;
            a1 = (2-2*f+f*fb-f*f*fb) * 256;
            a2 = -(1-2*f+f*fb+f*f-f*f*fb) * 256;
            taskYIELD();
            xSemaphoreGive(MutexPotentiometer);
            // Wait for the next cycle in 100ms
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }
}