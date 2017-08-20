// Este programa está basado en el Control PID de Brett Beauregard: http://brettbeauregard.com/blog/wp-content/uploads/2012/07/Gu%C3%ADa-de-uso-PID-para-Arduino.pdf

// **********************************************  Patillaje ******************************************************************************************************
const byte    encA = 2;                  // Entrada de la señal A del encoder.
const byte    encB = 3;                  // Entrada de la señal B del encoder.
const byte    PWMA = 5;                  // Salida PWM a la primera patilla del motor a través de un puente en H.
const byte    PWMB = 6;                  // Salida PWM a la segunda patilla del motor a través de un puente en H.

// *********************************************** Variables Globales PID *****************************************************************************************
unsigned long lastTime = 0,   SampleTime = 0;                    // Variables de tiempo discreto.
double        Input    = 0.0, Setpoint   = 0.0;                  //     "     de posición del motor y posición a la que queremos llevar el motor (posición designada).
double        ITerm    = 0.0, dInput     = 0.0, lastInput = 0.0; //     "     de error integral, error derivativo y posición anterior del motor
double        kp       = 0.0, ki         = 0.0, kd        = 0.0; // Constantes: proprocional, integral y derivativa.
double        outMin   = 0.0, outMax     = 0.0;                  // Límites para no sobrepasar la resolución del PWM.
double        error    = 0.0, kpid       = 0.0;                  // Diferencia entre la posición real del motor y la posición designada.

// **************************************************** Otras Variables *******************************************************************************************
volatile long contador =  0;             // En esta variable se guardará los pulsos del encoder y que interpreremos como distancia (o ángulo si ese fuese el caso).
byte          ant      =  0,    act = 0; // Sólo se utiliza los dos primeros bits de estas variables y servirán para decodificar el encoder. (ant=anterior, act=actual.)
byte          cmd      =  0;             // Un byte que utilizamos para la comunicación serie. (cmd=comando.)
unsigned int  tmp      =  0;             // Variable que utilizaremos para poner el tiempo de muestreo.
const byte    ledok    = 13;             // El pin 13 de los Arduinos tienen un led que utilizo para mostrar que el motor ya ha llegado a la posición designada.
// ****************************************************************************************************************************************************************

void setup()                          // Inicializamos todo las variables que sean necesarias, configuramos los pines de entrada/salida y configuramos el terminal serie.
{
  Serial.begin(115200);               // Configura la velocidad en baudios del terminal serie. Hoy en día "115200" es soportada por la gran mayoría de computadores.
  
  pinMode(PWMA, OUTPUT);              // Declara las dos salidas PWM para el control del motor (pin 5).
  pinMode(PWMB, OUTPUT);              //           "               "               "           (pin 6).
  digitalWrite(PWMA, LOW);            // Y ambas salidas las inicializa a cero.
  digitalWrite(PWMB, LOW);
  
  TCCR0B = TCCR0B & 0b11111000 | 0x1; // https://playground.arduino.cc/Code/PwmFrequency  &  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  TCCR1B = TCCR1B & 0b11111000 | 0x1; // Aquí podemos variar la frecuencia del PWM con un número de 1 (32KHz) hasta 7 (32Hz). El número que pongamos es un divisor de frecuencia. Min.=7, Max.=1.
  
  attachInterrupt(digitalPinToInterrupt(encA), encoder, CHANGE); // En cualquier flanco ascendente o descendente
  attachInterrupt(digitalPinToInterrupt(encB), encoder, CHANGE); // de los pines 2 y 3, actúa la interrupción.

  // Límites máximo y mínimo; corresponde a Max.: 0=0V hasta 255=5V (PWMA), y Min.: 0=0V hasta -255=5V (PWMB). Ambos PWM se convertirán a la salida en valores absolutos, nunca negativos.
  outMax =  255.0;                    // Límite máximo del controlador PID.
  outMin = -outMax;                   // Límite mínimo del controlador PID.
  
  tmp = 25;
  SetSampleTime(tmp);                 // Tiempo de muestreo.
  
  kp = 1.0;                           // Constantes PID iniciales. Los valores son los adecuados para un encoder de 334 ppr,
  ki = 0.05;                          // pero como el lector de encoder está diseñado como x4 equivale a uno de 1336 ppr. (ppr = pulsos por revolución.)                  
  kd = 25.0;
  
  SetTunings(kp, ki, kd);             // Llama a la función de sintonización y le envía los valores que hemos cargado anteriormente.
  
  Setpoint=(double)contador;          // Para evitar que haga cosas extrañas al ponerse en marcha o después de resetear, igualamos los dos valores para que comience estando quieto el motor.
  
  imprimir(0);                         // Muestra los datos de sintonización y el tiempo de muestreo por el terminal serie.
}

void loop()
{
  double Out = Compute();             // Llama a la función "Computer()" para calcular el error y darle una equivalencia de PWM y se carga en la variable 'Out'.
  
  // *********************************************** Control del Motor *************************************************
  if (abs(error) < kp)                // Cuando está muy cerca del punto designado, parar el motor.
  { 
    digitalWrite(PWMA, 0);            // Pone a 0 los dos pines del puente en H.
    digitalWrite(PWMB, 0);
    digitalWrite(ledok, HIGH);        // Se enciende el led (pin 13) porque ya está en la posición designada.
  }
  else                                // En caso contrario hemos de ver si el motor ha de ir hacia delante o hacia atrás.
  {
    if (Out > 0.0)                    // Mueve el motor hacia delante con el PWM correspondiente a su posición.
    {
      digitalWrite(PWMB, 0);          // Pone a 0 el segundo pin del puente en H.
      analogWrite(PWMA, abs(Out));    // Por el primer pin sale la señal PWM.
    }
    if (Out < 0.0)                    // Mueve el motor hacia  atrás   con el PWM correspondiente a su posición.
    {
      digitalWrite(PWMA, 0);          // Pone a 0 el primer pin del puente en H.
      analogWrite(PWMB, abs(Out));    // Por el segundo pin sale la señal PWM.
    }
  }
  
  // Recepción de datos para mover el motor o modificar las constantes PID o el tiempo de muestreo.
  if (Serial.available() > 0)         // Comprueba si ha recibido algún dato por el terminal serie.
  {
    cmd = Serial.read();              // "cmd" guarda el byte recibido.
    if (cmd >  'Z') cmd -= 32;        // Si una letra entra minúscula la covierte en mayúscula.
    if (cmd == 'W') Setpoint += 5.0;  // Si es la letra 'W' mueve 5 pasos hacia delante. Estos son movimientos relativos (por incrementos).
    if (cmd == 'Q') Setpoint -= 5.0;  // Aquí son esos 5 pasos pero hacia atrás, si se pulsa la letra 'Q'.
    if (cmd == 'S') Setpoint += 400.0;// Se repite lo mismo en el resto de las teclas que le hayamos dado alguna función.
    if (cmd == 'A') Setpoint -= 400.0;  
    if (cmd == 'X') Setpoint += 5000.0;
    if (cmd == 'Z') Setpoint -= 5000.0;
    if (cmd == 'M') Setpoint += 12000.0;
    if (cmd == 'N') Setpoint -= 12000.0;
    // Decodificador para modificar las constantes PID.
    switch(cmd)                                                                        // Si ponemos en el terminal, por ejemplo "p2.5 i0.5 d40" y pulsas enter  tomará esos valores y los cargará en kp, ki y kd.
    {                                                                                  // También se puede poner individualmente, por ejemplo "p5.5", sólo cambiará el parámetro kp, los mismo si son de dos en dos.
      case 'P': kp  = Serial.parseFloat(); SetTunings(kp, ki, kd); imprimir(0); break; // Carga las constantes y presenta en el terminal serie los valores de las variables que hayan sido modificadas.
      case 'I': ki  = Serial.parseFloat(); SetTunings(kp, ki, kd); imprimir(0); break;
      case 'D': kd  = Serial.parseFloat(); SetTunings(kp, ki, kd); imprimir(0); break;
      case 'T': tmp = Serial.parseInt();   SetSampleTime(tmp);     imprimir(0); break;
      case 'M': Setpoint = Serial.parseFloat();                    imprimir(1); break; // Esta línea permite introducir una posición absoluta. Ex: m13360 (y luego enter) e irá a esa posición absoluta.
      case 'K': imprimir(0); break; 
      default:  imprimir(1); break;
    }
    digitalWrite(ledok, LOW);         // Cuando entra una nueva posición se apaga el led y no se volverá a encender hasta que el motor llegue a la posición que le hayamos designado.
  }
}

double Compute(void)
{
   unsigned long now = millis();                  // Toma el número total de "ticks" que hay en ese instante. (1 tick = 1 milisengundo.)
   unsigned long timeChange = (now - lastTime);   // Resta el tiempo actual con el último tiempo que se guardó (esto último se hace al final de esta función).
   
   if(timeChange >= SampleTime)                   // Si se cumple el tiempo de muestreo entonces calcula la salida.
   {
     Input  = (double)contador;                   // Lectura del encoder óptico. El valor del contador se incrementa/decrementa a través de las interrupciones extrenas (pines 2 y 3).
     
     error  = (Setpoint - Input)  * kp;           // Calcula el error proporcional.
          
     dInput = (Input - lastInput) * kd;           // Calcula el error derivativo.

     double kn = kp;                              // Hago que KP se carge en otra variable (KN) de esta manera puedo manipular ese valor sin afectar al valor original.
     
     if (error==0.0) kn *= 100.0;                 // Esto nos sirve para "empequeñecer" ITerm en la línea de abajo. (... else ITerm -= dInput / (kn * kd); ) Ayuda a "clavar" el motor en el lugar designado y elimina parte del "sonido" mientras intenta "clavarse".
     
     if (dInput == 0.0) ITerm += error * ki; else ITerm -= dInput / (kn * kd);       // Esta línea permite dos cosas. 1) Suaviza la llegada a la posición designada. 2) Hace que el error integral no sea tan dependiente del error proporcional, se consigue que el error integral sólo actúe cuando está muy muy cerca de la posición designada.
                                                                                     // Para una mayor suavidad al acercarse al punto designado puedes cambiar el "else" por este otro: else ITerm -= (dInput / (kn * ki * kd));
     if (ITerm > outMax) ITerm = outMax; else if (ITerm < outMin) ITerm = outMin;    // Delimita el error integral para eliminar el "efecto windup".
     
     double out = error + ITerm - dInput;          // Suma todos los errores, es la salida del control PID.
     if (out > outMax) out = outMax; else if (out < outMin) out = outMin; // Delimita la salida para que pueda estar el PWM entre un valor de 0 y 5 voltios.
     
     lastInput = Input;                            // Se guarda la posición actual para convertirla en pasado.
     lastTime  = now;                              // Se guarda el tiempo   actual para convertirlo en pasado.
     return out;                                   // Devuelve el valor de salida PID. Este 'out' es diferente del 'Out' del programa principal, sin embargo éste se transfiere al otro.
   }
}

void SetTunings(double kp, double ki, double kd)   // A las constantes KI y KD se le incluye el tiempo de muestreo. Así sólo se ha de calcular esto una sola vez.
{
   double SampleTimeInSec = ((double)SampleTime) / 1000.0;
   kp = kp;
   ki = ki * SampleTimeInSec;
   kd = kd / SampleTimeInSec;
}

void SetSampleTime(int NewSampleTime)              // Función que se llama cuando queremos poner un tiempo de muestreo dado.
{                                                  // Evita que el tiempo de muestreo sea 0 o negativo.
   if (NewSampleTime > 0) SampleTime = (unsigned long)NewSampleTime;
}

void imprimir(byte flag) // Si "flag" es 0, imprime en el terminal serie los datos de las contantes PID, tiempo de muestreo y posición. En los demás casos sólo imprime la posición del motor.
{
  if (flag==0)
  {
    Serial.print("KP=");     Serial.print(kp);
    Serial.print(" KI=");    Serial.print(ki);
    Serial.print(" KD=");    Serial.print(kd);
    Serial.print(" Time=");  Serial.println(SampleTime);
  }
  Serial.print("Posicion:"); Serial.println((long) Setpoint);
}

void encoder(void)                    // Encoder x4. Cuando se produzca cualquier cambio en el encoder esta parte hará que incremente o decremente el contador.
{
    ant=act;                          // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.
    act=PIND & 12;                    // Guardamos en 'act' el valor que hay en ese instante en el encoder y hacemos un
                                      // enmascaramiento para aislar los dos únicos bits que utilizamos para esta finalidad.
    if(ant==12 && act==4)  contador++;// Incrementa el contador si el encoder se mueve hacia delante.
    if(ant==4  && act==0)  contador++;
    if(ant==0  && act==8)  contador++;
    if(ant==8  && act==12) contador++;
    
    if(ant==4  && act==12) contador--;// Decrementa el contador si el encoder se mueve hacia atrás.
    if(ant==0  && act==4)  contador--;
    if(ant==8  && act==0)  contador--;
    if(ant==12 && act==8)  contador--;
}
