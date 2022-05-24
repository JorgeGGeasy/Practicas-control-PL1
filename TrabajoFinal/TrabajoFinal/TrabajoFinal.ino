// DECLARACIONES //////////////////////////////////////////////////////////////////////////

// ACTIVACION DE CODIGO

#define NOMBRE_PRAC "P1-A"
#define VERSION_SW "1.2"

#define ACTIVA_P1A
//#define DEBUG_P1A
#define ACTIVA_P1B1
#define ACTIVA_P1B2
#define ACTIVA_P1B3
#define ACTIVA_P1C
#define DEBUG_P1C
//#define ACTIVA_P1C_MED_ANG
//#define ACTIVA_P1D2
#define ACTIVA_P1D3

// Display OLED ///////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// Parametros cola de la interrupcion del encoder ///////////////////////////////////////
#define TAM_COLA_I 1024 /*num mensajes*/
#define TAM_MSG_I 1 /*num caracteres por mensaje*/

// TIEMPOS
#define BLOQUEO_TAREA_LOOPCONTR_MS 10 
#define BLOQUEO_TAREA_MEDIDA_MS 10

// Configuración PWM  ////////////////////////////////////////////////////////////////////
uint32_t pwmfreq = 1000; // 1KHz
const uint8_t pwmChannel = 0;
const uint8_t pwmresolution = 8;
const int PWM_Max = pow(2,pwmresolution)-1; //

// Pines driver motor ////////////////////////////////////////////////////////////////////
const uint8_t PWM_Pin = 32; // Entrada EN 
const uint8_t PWM_f = 16; // Entrada PWM1 
const uint8_t PWM_r = 17; // Entrada PWM2 

// Voltaje maximo motor ////////////////////////////////////////////////////////////////////
float SupplyVolt = 12;

// Pines encoder ////////////////////////////////////////////////////////////////////
const uint8_t A_enc_pin = 35;
const uint8_t B_enc_pin = 34;

// Conversión a angulo y velocidad del Pololu 3072
//const float conv_rad = ; 
//const float conv_rev = ;
//const float conv_rad_grados = ; 
const float pi = 3.1415926535897932384626433832795;
const int pasos_vuelta = 1200;

// Declarar funciones ////////////////////////////////////////////////////////////////////
void config_sp(); // Configuracion puerto serie
void config_oled(); // Configuracion OLED
void config_enc(); // Configuracion del encoder
void config_PWM(); // Configuracion PWM
void excita_motor(float v_motor); // Excitacion motor con PWM
float interpola_vel_vol_lut(float x); // Interpolacion velocidad/voltios LUT

// TABLA VELOCIDAD-VOLTAJE P1D
#ifdef ACTIVA_P1D2
#define LONG_LUT 12
//Vector de tensiones
const float Vol_LUT[LONG_LUT] = {0, 1.55, 1.6, 2, 3, 4, 5, 6, 7, 8, 9, 100};
// Vector de velocidades
const float Vel_LUT[LONG_LUT] = {0, 0, 0.32, 1.32,3.56,5.54,6.7,7.5,8.1,8.9,9.2};
#endif

// Variables globales ////////////////////////////////////////////////////////////////////
int32_t ang_cnt = 0;
int32_t ultimoEstado = 0;
float pwm_volt = 0;
int32_t pwm_motor = 0;
int32_t sign_v_ant = 0;
float v_medida = 0;    // Valor medido de angulo o velocidad -----------------
float ref_val = 0;     // Valor de referencia de angulo o velocidad
int8_t start_stop = 0; //1 -> en funcionamiento | 0 -> parado 

// parte proporcional
float K_p = 27;
// parte integradora
float Ti = 27;
float K_i = K_p / Ti;
// parte derivativa
float Td = 0.0025;
float K_d = K_p * Td; 
// Declaracion objetos  ////////////////////////////////////////////////////////////////////

xQueueHandle cola_enc; // Cola encoder

/*
 RUTINAS ATENCION INTERRUPCIONES ########################################################################
*/

/* 
 Rutina de atención a interrupción ISC_enc --------------------------------------------
*/

void IRAM_ATTR ISR_enc() {
  // Lee las salidas del Encoder    
  int valA = digitalRead(A_enc_pin);
  int valB = digitalRead(B_enc_pin);
  
 
  // Procesa los datos
  uint8_t valor = 2*valA + valB;
  // Enviar los bytes a la cola 
  if (xQueueSendFromISR(cola_enc, &valor ,NULL) != pdTRUE)
  {
    printf("Error de escritura en la cola cola_enc \n");
  };
}

/*
 TAREAS #############################################################################
*/

/*
 Tarea task_enc #####################################################################
*/
#ifdef ACTIVA_P1A
void task_enc(void* arg) {
  // Declaracion de variables locales
  uint8_t valor_recibido = 0;
  int valor_anterior = 0;
  while(1){
  
    // Espera a leer los datos de la cola
    if (xQueueReceive( cola_enc, &valor_recibido ,(TickType_t) portMAX_DELAY) == pdTRUE){
      // Codificar la fase del encoder
      if(valor_recibido == 2){
        valor_recibido = 3;
      }else if(valor_recibido == 3){
        valor_recibido = 2;
      }
      // Calcular incremento/decremento y actualizar valor 
      if(valor_anterior==0 && valor_recibido == 3){
        ang_cnt--;
      }else if(valor_anterior==3 && valor_recibido == 0){
        ang_cnt++;
      }else if(valor_recibido < valor_anterior){
        ang_cnt--; // retrocedemos
      }
      else if(valor_recibido > valor_anterior){
        ang_cnt++; // avanzamos
      }

      valor_anterior = valor_recibido;
      #ifdef DEBUG_P1A
        // Enviar al monitor serie
        Serial.println(ang_cnt);

      #endif
    } else {
      printf("Error de lectura de la cola cola_enc \n");
    }
  }
}
#endif


/* 
Tarea de configuración de parámetros  #####################################################################
*/
void task_config(void *pvParameter) {
  char ini_char = 'V';
  char S = 'S';
  char R = 'R';
  char K = 'K';
  char P = 'P';
  char I = 'I';
  char D = 'D';
  while(1) { 
  char variable;
  variable = Serial.read();

  if(variable == ini_char){
    pwm_volt = Serial.parseFloat();
  }

   if(variable == R){
    ref_val = Serial.parseFloat();
  }
  
  if(variable == S){
    if(start_stop == 1){
      start_stop = 0;
      Serial.print("--STOP--");
      }
    else if(start_stop == 0){
      start_stop = 1;
      Serial.print("--START--");
      }
    }

    if(variable == K){
      variable = Serial.read();
      
      if(variable == D){
        K_d = Serial.parseFloat();
      }
      else if(variable == I){
        K_i = Serial.parseFloat();
      }
      
      else if(variable == P){
        K_p = Serial.parseFloat();
      }
     }
    // Activacion de la tarea cada 0.1s
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/* 
Tarea del lazo principal del controlador  #####################################################################
*/
#ifdef ACTIVA_P1B3
void task_loopcontr(void* arg) {
  float radianes = 0;
  float v_media_anterior = 0; // voltios anteriores, para el calculo de la velocidad
  float errorRps = 0;// E[n]
  float error_anterior = 0; // E[n-1]
  float vi_anterior = 0; // Vi[n-1]
  while(1) {
    if(start_stop == 1){ 
      // Excitacion del motor con PWM
      
      #ifdef ACTIVA_P1C_MED_ANG // Medida de angulo
        v_medida = (2*pi*ang_cnt)/pasos_vuelta;
      #else // Medida de velocidad
        v_medida = (2*pi*ang_cnt)/pasos_vuelta;
        float diferencia_angulos = v_medida - v_media_anterior; // rad
        float Tm = 0.01;//BLOQUEO_TAREA_LOOPCONTR_MS / 1000;
        float velocidad = ((diferencia_angulos)/(Tm*2*pi)); // vuelta/s (rps)
        
        v_media_anterior = v_medida;
        v_medida = velocidad;
        
    
        #ifdef ACTIVA_P1D3
          errorRps = ref_val - velocidad;

          float vp = K_p*errorRps;//Vp[n] = Kp*E[n]
          float vi = K_i*Tm*errorRps+vi_anterior; // Vi[n] = Kp*Tm*E[n]+Vi[n-1]
          float vd = (K_d/Tm)*(errorRps-error_anterior); // Vd[n] = Kp/Tm*(E[n]-E[n-1])
          float v = vp+vi+vd;
          pwm_volt = v;
          //Serial.println(pwm_volt);
          /*if(errorRps > 0){
            pwm_volt -= v;
          }
          else if(errorRps < 0){
            pwm_volt += v;
          }*/
          
          vi_anterior = vi;
          error_anterior = errorRps;
        #endif
    
        #ifdef ACTIVA_P1D2
          pwm_volt = interpola_vel_vol_lut(ref_val);
        #endif
      
      
      //Serial.print("RPS: ");
      //Serial.println(velocidad);
    #endif
      excita_motor(pwm_volt);
    }
    else{
      ang_cnt = 0;
      pwm_motor = 0;
      v_medida = 0;
      excita_motor(0);
    }
// ang en Tm / Tm
    
    // Activacion de la tarea cada 0.01s
      vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
  }
}
#endif

/* 
Tarea del lazo principal del controlador  #####################################################################
*/
#ifdef DEBUG_P1C
void task_medidas(void* arg) 
{
  while(1) { 
    if(start_stop == 1){
      // Mostrar medidas de angulo y velocidad del motor
      #ifdef ACTIVA_P1C_MED_ANG // Medida de angulo
        float v_angulo = v_medida *((360)/(2*pi));
        Serial.print("Med: ");
        Serial.print(v_angulo);
        Serial.print(", Ref: ");
        Serial.print(ref_val);
        Serial.print(", KP:  ");
        Serial.print(K_p);
        Serial.print(", KI:  ");
        Serial.print(K_i);
        Serial.print(", KD:  ");
        Serial.print(K_d);
        Serial.println("   ");
      #else // Medida de velocidad
        Serial.print("Med: ");
        Serial.print(v_medida);
        Serial.print(", Ref: ");
        Serial.print(ref_val);
        Serial.print(", KP:  ");
        Serial.print(K_p);
        Serial.print(", KI:  ");
        Serial.print(K_i);
        Serial.print(", KD:  ");
        Serial.print(K_d);
        Serial.println("   ");
      #endif
    }
    // Activacion de la tarea cada 1s
     vTaskDelay(BLOQUEO_TAREA_MEDIDA_MS / portTICK_PERIOD_MS);
  }
}
#endif

/*
SET UP -----------------------------------------------------------------------------------
*/
void setup() {
  // Configuracion puerto serie
  config_sp();
  
  // Configuracion OLED
  config_oled();

  // Configuracion PWM
  config_PWM();

  // Crear cola_enc
  cola_enc = xQueueCreate(TAM_COLA_I, TAM_MSG_I);
  if(cola_enc == NULL){
  Serial.println("Error en creacion de cola_enc");
  exit(-1);
  };

  // Crear la tarea task_enc
  if(xTaskCreate(&task_enc, "task_enc", 2048, NULL, 1, NULL) != pdPASS){
  Serial.println("Error en creacion tarea task_enc");
  exit(-1);
  }

  // Crear la tarea task_config
  if(xTaskCreate(&task_config, "task_config", 2048, NULL, 1, NULL) != pdPASS){
  Serial.println("Error en creacion tarea task_config");
  exit(-1);
  }

  // Crear la tarea task_loopcontr
  if(xTaskCreate(&task_loopcontr, "task_loopcontr", 2048, NULL, 1, NULL) != pdPASS){
  Serial.println("Error en creacion tarea task_loopcontr");
  exit(-1);
  }

  #ifdef DEBUG_P1C
    // Crear la tarea task_medidas
      if(xTaskCreate(&task_medidas, "task_medidas", 2048, NULL, 1, NULL) != pdPASS){
      Serial.println("Error en creacion tarea task_medidas");
      exit(-1);
      }
  #endif

  // Configuracion del encoder
  config_enc();
}

/*
LOOP ---- NO USAR ------------------------------------------------------------------- 
*/
void loop() {}

// FUNCIONES ////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del encoder
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1A
void config_enc(){
  // Configuracion de pines del encoder
  pinMode(A_enc_pin, INPUT);
  pinMode(B_enc_pin, INPUT);
  
  // Configuracion interrupcion
  attachInterrupt(digitalPinToInterrupt(A_enc_pin),ISR_enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(B_enc_pin),ISR_enc, CHANGE);
} 
#endif
////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del PWM
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1B2
void config_PWM(){
  // Configuracion de pines de control PWM
  pinMode(PWM_f, OUTPUT);
  pinMode(PWM_r, OUTPUT);
  // Configuracion LED PWM 
  ledcSetup(pwmChannel,pwmfreq,pwmresolution);
  // Asignar el controlador PWM al GPIO
 ledcAttachPin(PWM_Pin, pwmChannel);
}  
#endif

////////////////////////////////////////////////////////////////////////////////////
// Funcion excitacion del motor con PWM
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1B3
void excita_motor(float v_motor){
  if(v_motor > 12){
    v_motor = 12;
  }
  if(v_motor < -12){
    v_motor = -12;
  }
  // Sentido de giro del motor
  if(v_motor > 0){
    if(pwm_motor < 0){
    digitalWrite(PWM_f, LOW);
    digitalWrite(PWM_r, LOW);
    }
    digitalWrite(PWM_f, HIGH);
    digitalWrite(PWM_r, LOW);
  }
  else if(v_motor < 0){
    if(pwm_motor > 0){
    digitalWrite(PWM_f, LOW);
    digitalWrite(PWM_r, LOW);
    }
    digitalWrite(PWM_f, LOW);
    digitalWrite(PWM_r, HIGH);
  } 

  // Calcula y limita el valor de configuración del PWM
    pwm_motor = (v_motor*255)/12;
  // El valor de excitación debe estar entro 0 y PWM_Max
  //Serial.println(pwm_motor);
  // Excitacion del motor con PWM
 // PWM DEBE DE SER POSITVA
 if(pwm_motor < 0){
    ledcWrite(pwmChannel,pwm_motor*-1);
 }else{
  ledcWrite(pwmChannel,pwm_motor);
 }
}  
#endif

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del puerto serie
////////////////////////////////////////////////////////////////////////////////////
void config_sp(){
  Serial.begin(115200);
  Serial.println("  ");
  Serial.println("--------------------------------------------");
  Serial.println("PRACTICA CONTROLADOR MOTOR " NOMBRE_PRAC);
  Serial.println("--------------------------------------------");
}  

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del OLED
////////////////////////////////////////////////////////////////////////////////////
void config_oled(){
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C) ;
  display.clearDisplay();
  display.setTextColor(WHITE);        // 
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("CONTR. MOTOR " NOMBRE_PRAC));
  display.display();
  delay(1000);
  display.setTextColor(BLACK,WHITE);        // 
  display.setCursor(0,20);             // Start at top-left corner
  display.println(F(" SW v." VERSION_SW));
  display.display();
  delay(1000);
}  


////////////////////////////////////////////////////////////////////////////////////
// Funcion de interpolacion LUT de Velocidad-Voltaje
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1D2
float interpola_vel_vol_lut(float x) {
  // Buscar el valor superior más pequeño del array
  int8_t i = 0;
  if ( x >= Vel_LUT[LONG_LUT - 2] ) {i = LONG_LUT - 2;}
  else {while ( x > Vel_LUT[i+1] ) i++;}

  // Guardar valor superior e inferior
  float xL = Vel_LUT[i];
  float yL = Vol_LUT[i];
  float xR = Vel_LUT[i+1];
  float yR = Vol_LUT[i+1];

  // Interpolar
  float dydx = ( yR - yL ) / ( xR - xL );

  return yL + dydx * ( x - xL );
}
#endif
