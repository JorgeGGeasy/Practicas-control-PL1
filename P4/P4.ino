/* 
PRACTICA 1: IMPLEMENTACION DE UN CONTROLADOR PARA UN MOTOR DE DC

*/

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
#define ACTIVA_P1D2
//#define ACTIVA_P1D3

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
const float Vol_LUT[LONG_LUT] = {0, 1, 1.5 , 2, 3, 4, 5, 6, 7, 8, 9, 100};
// Vector de velocidades
const float Vel_LUT[LONG_LUT] = {0, 0, 0, 0, 2.8, 4.8, 6, 6.8, 7.6, 8.2, 8.6, 9};
#endif

// Variables globales ////////////////////////////////////////////////////////////////////
int32_t ang_cnt = 0;
int32_t ultimoEstado = 0;
float pwm_volt = 0;
int32_t pwm_motor = 0;
//int32_t sign_v_ant = 0;
float v_medida = 0;    // Valor medido de angulo o velocidad -----------------
float ref_val = 0;     // Valor de referencia de angulo o velocidad
int8_t start_stop = 0; //1 -> en funcionamiento | 0 -> parado 
//float K_p = ;

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
  uint8_t Buffer;
  uint32_t ang = 0;
	while(1){
		// Espera a leer los datos de la cola
		if (xQueueReceive( cola_enc, &Buffer ,(TickType_t) portMAX_DELAY) == pdTRUE){
			// Codificar la fase del encoder
       switch (Buffer){
      case 0:
        ang = 1;
        break;
      case 1:
        ang = 2;
        break;
      case 2:
        ang = 4;
        break;
      case 3:
        ang = 3;
        break;
      default:
      break;
      }
      }
			// Calcular incremento/decremento y actualizar valor 
      if(ultimoEstado == 0){
        ultimoEstado = ang;
      }

      if(ang == 1 && ultimoEstado == 4){
        ang_cnt++;
      }else if(ang == 4 && ultimoEstado == 1){
        ang_cnt--;
      }else if(ang > ultimoEstado){
        ang_cnt++;
      }else if(ang < ultimoEstado){
        ang_cnt--;
      }
      ultimoEstado = ang;
			#ifdef DEBUG_P1A
		 // Enviar al monitor serie
     //Serial.print("Valor ang");
     //Serial.println(ang);
     //Serial.print("Valor ultimoEstado");
     //Serial.println(ultimoEstado);
     Serial.print("Ang_cnt: ");
     Serial.println(ang_cnt);
			#endif
		// } else {
		// 	printf("Error de lectura de la cola cola_enc \n");
		// }
	
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
	while(1) { 
  char variable;
  variable = Serial.read();

  if(variable == ini_char){
    pwm_volt = Serial.parseFloat();
    Serial.print("Voltaje motor = ");
    Serial.println(pwm_volt);
  }

   if(variable == R){
    ref_val = Serial.parseFloat();
    Serial.print("Valor de referencia = ");
    Serial.print(ref_val);
    Serial.print(" º o ");
    Serial.print(ref_val);
    Serial.println(" rps");
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
  float valorAnterior = 0;
  float errorRps = 0;
	while(1) {
    if(start_stop == 1){ 
		// Excitacion del motor con PWM
		excita_motor(pwm_volt);
    #ifdef ACTIVA_P1C_MED_ANG // Medida de angulo
    v_medida = (2*3.14*ang_cnt)/1200;
    #else // Medida de velocidad
    radianes = (2*3.14*ang_cnt)/1200;
    float rps = (radianes - valorAnterior )/(0.01*2*3.14);
    valorAnterior = radianes;
    v_medida = rps;

    #ifdef ACTIVA_P1D3
    errorRps = rps-ref_val;
    if(errorRps > 0){
      pwm_volt -= 0.05;
    }
    else if(errorRps < 0){
      pwm_volt += 0.05;
    }
    #endif

    #ifdef ACTIVA_P1D2
    pwm_volt = interpola_vel_vol_lut(ref_val);
    #endif
    
    
    Serial.print("RPS: ");
    Serial.println(rps);
    #endif
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
    float v_angulo = v_medida *((360)/(2*3.14));
    Serial.print("Med: ");
    Serial.print(v_angulo);
    Serial.print(", Ref: ");
    Serial.println(ref_val);
		#else // Medida de velocidad
    Serial.print("Med: ");
    Serial.println(v_medida);
    Serial.print(", Ref: ");
    Serial.println(ref_val);
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
