#include <PID_v1.h>

Arduino_h
#define MEDIA_VELOCIDAD_SONIDO 0.017175 // Mitad de la velocidad del sonido a 20 °C expresada en cm/µs
#define PIN_TRIGGER 7 //VERDE 
#define PIN_ECHO 8   /// AMARILLO
#define ESPERA_ENTRE_LECTURAS 1000 // tiempo entre lecturas consecutivas en milisegundos 1000
#define TIMEOUT_PULSO 25000 // la espera máxima de es 30 ms o 30000 µs

#define D_a_Fondo 21.0
#define Area 33.333 //33.1830
#define PWM_pin 3  
//toggle button

int LEDState=0;
int LEDPin=9;//9;
int buttonPin=12;//12;
int buttonNew;
int buttonOld=1;
int dt=100; 

float distancia;
unsigned long tiempo;
unsigned long cronometro;
unsigned long reloj=0;

int duty_cycle = 178.5; // between 0 and 255, 150 = 3 VOLTS

int ledstate = LOW;
int buttonstate;
int buttonold = HIGH;
int t=100;



// PID
// Constantes del controlador
double Kp=375, Ki=Kp*(0.008), Kd=0; // Kp=150, Ki=Kp*(1/10), Kd=Kp*(1/30)
                                    //Kp=150, Ki=Kp*(1/16)


// variables externas del controlador
double Input, Output, Setpoint;
int SampleTime = 1000; //1 sec
PID pidController(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



//Fin



void setup()
{
  Serial.begin(9600);
  pinMode(PWM_pin,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  pinMode(PIN_TRIGGER,OUTPUT);
  digitalWrite(PIN_TRIGGER,LOW); // Para «limpiar» el pulso del pin trigger del módulo
  delayMicroseconds(2);
  pinMode(LEDPin, OUTPUT);
  pinMode(buttonPin, INPUT); 

////////////PID
   Input = D_a_Fondo-distancia;
   Setpoint = 4;
   
   pidController.SetMode(AUTOMATIC);     // encender el PID
   pidController.SetSampleTime(SampleTime);

  
}

void loop()
{



estado();

if(LEDState==1){

  cronometro=millis()-reloj;
  if(cronometro>ESPERA_ENTRE_LECTURAS)
  {
    digitalWrite(PIN_TRIGGER,HIGH); // Un pulso a nivel alto…
    delayMicroseconds(10); // …durante 10 µs y
    digitalWrite(PIN_TRIGGER,LOW); // …volver al nivel bajo
    tiempo=pulseIn(PIN_ECHO,HIGH,TIMEOUT_PULSO); // Medir el tiempo que tarda en llegar un pulso
    distancia=MEDIA_VELOCIDAD_SONIDO*tiempo;
    Serial.print(D_a_Fondo-distancia);
    Serial.print(",");
   
   
     Input = D_a_Fondo-distancia;
     pidController.Compute();         // actualizar el PID
      reloj=millis();
     Serial.print(Output);
      Serial.print(",");
      Serial.println();
     analogWrite(PWM_pin,Output);
     
  
  }

}
else{
analogWrite(PWM_pin,0 );
  }
}



//////////////////////////
void estado()
{
buttonNew=digitalRead(buttonPin);
if(buttonOld==0 && buttonNew==1){
  if (LEDState==0){
   // analogWrite( PWM_pin, duty_cycle );
    digitalWrite(LEDPin,HIGH);
    LEDState=1;
  }
  else{
  //  analogWrite( PWM_pin, 0 );
    digitalWrite(LEDPin, LOW);
    LEDState=0;
  }
}
buttonOld=buttonNew;
delay(t);
}

 
