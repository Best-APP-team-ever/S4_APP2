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

Parfait je te fait ca 0 probleme 
- Certified Stinky boi

*/

using Sawtooth = Oscil<SAW2048_NUM_CELLS, SAMPLE_RATE>;
using SquareWv = Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, SAMPLE_RATE>;

#define PIN_SW1 2   // Boutton
#define PIN_SW2 3   // Boutton
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

// Task pour le remplissage du buffer
void TaskBufferManip(void *pvParameters);

// Task de gestion des boutons 
void ButtonSW1Task(void *pvParameters);
void ButtonSW2Task(void *pvParameters);
void potentiometerTask(void *pvParameters);

// Task qui set les notes de la chanson
void PlaySongTask(void *pvParameters); 

/* Autres fonctions*/

// Fonction pour set une note
void setNoteHz(float note);
// Fonction du VCF
int8_t processVCF(int8_t input);
// Fonction du VCA
float processVCA(float vcaPeriod);
// gestion Sample
int8_t nextSample();

// Mutex handle
SemaphoreHandle_t MutexPotentiometer;
bool VCA_active = false;     // Tracks whether VCA is active

/* ***************** END DEFINE TASK FUNCTION ********************* */




void setup()
{
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

    // Setup a 0
    pcmSetup();
    setNoteHz(0);
    Serial.println("Synth prototype ready"); // Indice pour dire que le code roule 
    
/*------------------MUTEX-----------------------*/
    // Create the mutex
    MutexPotentiometer = xSemaphoreCreateMutex();
    if (MutexPotentiometer == NULL) {
        Serial.println("Failed to create mutex!"); // Code d'erreur 
        //while (1); // Stay here if mutex creation failed
    }

/*------------------------- MUSIC BUFFER SETUP -------------------------*/
    xTaskCreate(
        TaskBufferManip
        ,  "Music buffer manipulation" 
        ,  128  // Stack Size
        ,  NULL // pas de param envoyé
        ,  3  // Priorité de 3, we want buffer to fill up as the highest priority for quality of sound 
        ,  NULL );
    
/*------------------------- SONG SETUP -------------------------*/   
    xTaskCreate(
    PlaySongTask
    ,  "Setting_Notes_To_Play"   // A name just for humans
    ,  128  // Stack Size
    ,  NULL // pas de param envoyé
    ,  2  // Priorité de 2, la fonction principale c'est jouer de la musique donc il faut les bonnes notes mais moins prioritaire que le remplissage du buffer
    ,  &xPlaySongTaskHandle // Notification from SW2
    );

/*------------------------- BUTTON SETUP -------------------------*/
    xTaskCreate(
        ButtonSW1Task
        ,  "Gestion bouton SW1" 
        ,  128  // Stack Size
        ,  NULL //pas de param envoyé
        ,  1  // Tache plus haut niveau, c'est elle qui set une note spécifique
        ,  &xButtonSW1TaskHandle );
    xTaskCreate(
        ButtonSW2Task
        ,  "Gestion bouton SW2" 
        ,  128  // Stack Size
        ,  NULL //pas de param envoyé
        ,  1  // Tache plus haut niveau, c'est elle qui appel playSong, donc elle doit être moins prioritere que cette tache la 
        ,  &xButtonSW2TaskHandle );
    /* Interrupt pour les boutons pour register l'appuie des boutons*/
    // Attach the ISR to SW1
    attachInterrupt(digitalPinToInterrupt(PIN_SW1), toggleVarSW1, RISING);
    // Attach the ISR to SW2
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), toggleVarSW2, RISING); 

/*------------------------- POTENTIOMETER SETUP -------------------------*/
    xTaskCreate(
        potentiometerTask
        ,  "Gestion potentiomètres et un peu de math" 
        ,  1024  // Stack Size
        ,  NULL // pas de param envoyé
        ,  0  // Priorié la plus basse, Puisqu'on lit seulement a 10Hz et que c'est des potentiometre qui vont rester a leurs valeurs après modification on a pas besoin de vitesse extra
              // De plus, il y des math dans cette tâche et on ne veut pas quelle occupe toute la ressource 
        ,  NULL );
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}





void loop()
{   
    // Fait a-r djien (FreeRTOS gere tinquiete)
}






/*--------------------------------------------------*/
/*----------------------- ISR ----------------------*/
/*--------------------------------------------------*/
// les deux ISR suivant doivent executer le moins possible d'action pour reprendre le plus rapidement possible 
void toggleVarSW1()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Notify the task that the button was pressed
    vTaskNotifyGiveFromISR(xButtonSW1TaskHandle, &xHigherPriorityTaskWoken); // Notify la tache associer au bouton 
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

// Fonction qui gere le set note pour faire rouler des chansons avec un tempo demander
void PlaySongTask(void *pvParameters)
{
    //Find array size 
    int arraySize = sizeof(song) / sizeof(song[0]);

    for(;;)
    {
        //Wait until notified
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while(digitalRead(PIN_SW2) == HIGH){
            // Incrément en fonction de la grandeur du array pour 
            for(int i=0; i<arraySize; i++)
            {
                if (digitalRead(PIN_SW2) == LOW){
                    break;
                }
                setNoteHz(song[i].freq);
                vTaskDelay((song[i].duration * TEMPO_16T_MS(tempo)) / portTICK_PERIOD_MS );
            }
        }
    }
}


void TaskBufferManip(void *pvParameters) {
    // Cast the argument to the correct type
    //TaskParams *params = (TaskParams *)pvParameters;    
    while (1) 
    {
        if(pcmBufferEmpty())
        {
            // Serial.println("Buffer is empty!"); // Pour debug, print rajoute des delais inutiles alors mis en commentaire
        }
        while(!pcmBufferFull())
        {
            if (xSemaphoreTake(MutexPotentiometer, portMAX_DELAY) == pdTRUE)
            {
                pcmAddSample(nextSample());
                xSemaphoreGive(MutexPotentiometer);
            }
        }
        if(pcmBufferFull())
        {
            taskYIELD(); //Si le buffer est plein on donne la priorité aux autres task
            //vTaskDelay(16/portTICK_PERIOD_MS); //Si le buffer est plein met un temps d'attente pour laisser les autres tâches rouler
            //Serial.println("Buffer is full!"); // pour debug
        }
        
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
            
            //Debounce delay
            vTaskDelay(16/portTICK_PERIOD_MS); // une fois la note set il donne sa priorité au autre tout en gérant le debouce
        }
        //falling edge here
        VCA_active = true;

        // Debounce delay after button release
        vTaskDelay(16/portTICK_PERIOD_MS);
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
            //Serial.println("Presently 'SW2' is pressed, playing song.);
            xTaskNotifyGive(xPlaySongTaskHandle); 
            // Debounce delay during button press
            vTaskDelay(16/portTICK_PERIOD_MS);
        }  
        // Stop song (falling edge detected here)
        VCA_active = true;

        // Debounce delay after button release
        vTaskDelay(16/portTICK_PERIOD_MS);
    }
}


void potentiometerTask(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100 ms period aka 10Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize the reference time

    for (;;)
    {
        // Request the mutex for writing values
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