// Este programa es el algoritmo PID "refinedPID.ino" simplificado.
// Para motores de muy poca resolución, por ejemplo de 4 ppr, eleva el tiempo de muestreo a 1000 y luego ves bajando este valor hasta que encuentres una velocidad
// de posicionamiento óptimo. En principio no hace falta tocar nada más.
// Más información: https://sites.google.com/site/proyectosroboticos/control-de-motores/control-pid-mejorado

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
double        error    = 0.0;                                    // Desviación o error entre la posición real del motor y la posición designada.

// **************************************************** Otras Variables *******************************************************************************************
volatile long contador =  0;             // En esta variable se guardará los pulsos del encoder y que interpreremos como distancia (o ángulo si ese fuese el caso).
byte          ant      =  0,    act = 0; // Sólo se utiliza los dos primeros bits de estas variables y servirán para decodificar el encoder. (ant=anterior, act=actual.)
byte          cmd      =  0;             // Un byte que utilizamos para la comunicación serie. (cmd=comando.)
byte          pwm      =  0;             // Es el PWM, se transformará en voltaje real en las bobinas de los motores.
const byte    ledok    = 13;             // El pin 13 de los Arduinos tienen un led que utilizo para mostrar que el motor ya ha llegado a la posición designada.
// ****************************************************************************************************************************************************************

void setup(void)                        // Inicializamos todo las variables que sean necesarias, configuramos los pines de entrada/salida y el terminal serie.
{
  Serial.begin(115200);                 // Configura la velocidad en baudios del terminal serie. Hoy en día "115200" es soportada por la gran mayoría de computadores.
  
  pinMode(PWMA, OUTPUT);                // Declara las dos salidas PWM para el control del motor (pin 5).
  pinMode(PWMB, OUTPUT);                //           "               "               "           (pin 6).
  digitalWrite(PWMA, LOW);              // Y ambas salidas se inicializan a cero.
  digitalWrite(PWMB, LOW);
  
  TCCR0B = TCCR0B & B11111000 | 1;   // Configuración de la frecuencia del PWM para los pines 5 y 6. https://arduino-info.wikispaces.com/Arduino-PWM-Frequency
                                     // Podemos variar la frecuencia del PWM con un número de 1 (32KHz) hasta 7 (32Hz). El número que pongamos es un divisor de frecuencia. Min.=7, Max.=1. Está a la máxima frecuencia y es como mejor resultado me ha dado y además es silencioso.
  attachInterrupt(digitalPinToInterrupt(encA), encoder, CHANGE); // En cualquier flanco ascendente o descendente
  attachInterrupt(digitalPinToInterrupt(encB), encoder, CHANGE); // en los pines 2 y 3 actúa la interrupción.
  
  // Acotación máxima y mínima; corresponde a Max.: 0=0V hasta 255=5V (PWMA), y Min.: 0=0V hasta -255=5V (PWMB). El PWM se convertirá a la salida en un valor absoluto, nunca negativo.
  outMax =  255.0;                      // Límite máximo del controlador PID.
  outMin = -outMax;                     // Límite mínimo del controlador PID.
  
  SampleTime = 50;                      // Se le asigna el tiempo de muestreo en milisegundos.
  
  kp = 1.0;                             // Constantes PID iniciales. Los valores son los adecuados para un encoder de 334 ppr (con un motor de 12V),
  ki = 0.05;                            // pero como el lector de encoder está diseñado como x4, entonces equivale a uno de 1336 ppr. (ppr = pulsos por revolución.)
  kd = 23.0;
  
  Setpoint = (double)contador;          // Para evitar que haga cosas extrañas al ponerse en marcha o después de resetear, igualamos los dos valores para que comience estando quieto el motor.
  
  imprimir(3);                          // Muestra las constantes de sintonización, el tiempo de muestreo y la posición por el terminal serie.
}

void loop(void)
{
  double Out = Compute();               // Llama a la función "Compute()" para calcular la desviación y el resultado lo carga en la variable 'Out'.
  
  // *********************************************** Control del Motor *************************************************
  if (error == 0.0)                     // Cuando está en el punto designado, parar el motor.
  {
    digitalWrite(PWMA, LOW);            // Pone a 0 los dos pines del puente en H.
    digitalWrite(PWMB, LOW);
    digitalWrite(ledok, HIGH);          // Se enciende el led (pin 13) porque ya está en la posición designada.
  }
  else                                  // De no ser igual, significa que el motor ha de girar en un sentido o al contrario; esto lo determina el signo que contiene "Out".
  {
    pwm = abs(Out);                     // Transfiere a la variable pwm el valor absoluto de Out.
    // if (pwm < 50) pwm = 50;          // Línea experimental. Se trata de hacer que el motor tenga una voltaje mínimo para comenzar a girar, aunque esto no es necesario.
    
    if (Out > 0.0)                      // Gira el motor en un sentido con el PWM correspondiente a su posición.
    {
      digitalWrite(PWMB, LOW);          // Pone a 0 el segundo pin del puente en H.
      analogWrite(PWMA, pwm);           // Por el primer pin sale la señal PWM.
    }
    else                                // Gira el motor en sentido contrario con el PWM correspondiente a su posición.
    {
      digitalWrite(PWMA, LOW);          // Pone a 0 el primer pin del puente en H.
      analogWrite(PWMB, pwm);           // Por el segundo pin sale la señal PWM.
    }
  }
  
  // Recepción de datos para posicionar el motor, o modificar las constantes PID, o el tiempo de muestreo. Admite posiciones relativas y absolutas.
  if (Serial.available() > 0)           // Comprueba si ha recibido algún dato por el terminal serie.
  {
    cmd = 0;                            // Por seguridad "limpiamos" cmd.
    cmd = Serial.read();                // "cmd" guarda el byte recibido.
    if (cmd > 31)
    {
      byte flags = 0;                                      // Borramos la bandera que decide lo que hay que imprimir.
      if (cmd >  'Z') cmd -= 32;                           // Si una letra entra en minúscula la covierte en mayúscula.
      if (cmd == 'W') { Setpoint += 5.0;     flags = 2; }  // Si (por ejemplo) es la letra 'W' mueve 5 pasos hacia delante. Estos son movimientos relativos.
      if (cmd == 'Q') { Setpoint -= 5.0;     flags = 2; }  // Aquí son esos 5 pasos pero hacia atrás si se pulsa la letra 'Q'.
      if (cmd == 'S') { Setpoint += 400.0;   flags = 2; }  // Se repite lo mismo en el resto de las teclas.
      if (cmd == 'A') { Setpoint -= 400.0;   flags = 2; }
      if (cmd == 'X') { Setpoint += 5000.0;  flags = 2; }
      if (cmd == 'Z') { Setpoint -= 5000.0;  flags = 2; }
      if (cmd == '2') { Setpoint += 12000.0; flags = 2; }
      if (cmd == '1') { Setpoint -= 12000.0; flags = 2; }
      if (cmd == '0') { Setpoint = 0.0;      flags = 2; }  // Ir a Inicio.
      
      // Decodificador para modificar las constantes PID.
      switch(cmd)                                          // Si ponemos en el terminal serie, por ejemplo "P2.5 I0.5 D40" y pulsas enter  tomará esos valores y los cargará en kp, ki y kd.
      {                                                    // También se puede poner individualmente, por ejemplo "P5.5", sólo cambiará el parámetro kp, los mismo si son de dos en dos.
        case 'P': kp  = Serial.parseFloat();        flags = 1; break; // Carga las constantes y presenta en el terminal serie los valores de las variables que hayan sido modificadas.
        case 'I': ki  = Serial.parseFloat();        flags = 1; break;
        case 'D': kd  = Serial.parseFloat();        flags = 1; break;
        case 'T': SampleTime = Serial.parseInt();   flags = 1; break;
        case 'G': Setpoint   = Serial.parseFloat(); flags = 2; break; // Esta línea permite introducir una posición absoluta. Ex: G23000 (y luego enter) e irá a esa posición.
        case 'K':                                   flags = 3; break;
      }
      if (flags == 2) digitalWrite(ledok, LOW); // Cuando entra una posición nueva se apaga el led y no se volverá a encender hasta que el motor llegue a la posición que le hayamos designado.
      
      imprimir(flags);
    }
  }
}

// Cálculo PID.
double Compute(void)
{
   unsigned long now = millis();                  // Toma el número total de milisegundos que hay en ese instante.
   unsigned long timeChange = (now - lastTime);   // Resta el tiempo actual con el último tiempo que se guardó (esto último se hace al final de esta función).
   
   if(timeChange >= SampleTime)                   // Si se cumple el tiempo de muestreo entonces calcula la salida.
   {
     Input  = (double)contador;                   // Lee el valor del encoder óptico. El valor del contador se incrementa/decrementa a través de las interrupciones externas (pines 2 y 3).
     
     error  = (Setpoint - Input)  * kp;           // Calcula el error proporcional.
     dInput = (Input - lastInput) * kd;           // Calcula el error derivativo.
     
     // Esta línea permite dos cosas: 1) Suaviza la llegada a la meta. 2) El error integral se auto-ajusta a las circunstancias del motor.
     if (dInput == 0.0)  ITerm += (error * ki); else ITerm -= (dInput * ki);
     // Acota el error integral para eliminar el "efecto windup".
     if (ITerm > outMax) ITerm = outMax; else if (ITerm < outMin) ITerm = outMin;
     
     double Output = error + ITerm - dInput;      // Suma todos los errores, es la salida del control PID.
     if (Output > outMax) Output = outMax; else if (Output < outMin) Output = outMin; // Acota la salida para que el PWM pueda estar entre outMin y outMax.
     
     lastInput = Input;                           // Se guarda la posición para convertirla en pasado.
     lastTime  = now;                             // Se guarda el tiempo   para convertirlo en pasado.
     
     return Output;                               // Devuelve el valor de salida PID.
   }
}

// Encoder x4. Cuando se produzca cualquier cambio en el encoder esta parte hará que incremente o decremente el contador.
void encoder(void)
{
    ant=act;                            // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.
    act=PIND & 12;                      // Guardamos en 'act' el valor que hay en ese instante en el encoder y hacemos un
                                        // enmascaramiento para aislar los dos únicos bits que utilizamos para esta finalidad.
    if(ant==12 && act==4)  contador++;  // Incrementa el contador si el encoder se mueve hacia delante.
    if(ant==4  && act==0)  contador++;
    if(ant==0  && act==8)  contador++;
    if(ant==8  && act==12) contador++;
    
    if(ant==4  && act==12) contador--;  // Decrementa el contador si el encoder se mueve hacia atrás.
    if(ant==0  && act==4)  contador--;
    if(ant==8  && act==0)  contador--;
    if(ant==12 && act==8)  contador--;
}

// Función para imprimir los datos en el terminal serie.
void imprimir(byte flag) // Imprime en el terminal serie los datos de las contantes PID, tiempo de muestreo y posición. Según las necesidades se muestran algunos datos o todos ellos.
{
  if ((flag == 1) || (flag == 3))
  {
    Serial.print("KP=");     Serial.print(kp);
    Serial.print(" KI=");    Serial.print(ki);
    Serial.print(" KD=");    Serial.print(kd);
    Serial.print(" Time=");  Serial.println(SampleTime);
  }
  if ((flag == 2) || (flag == 3))
  {
    Serial.print("Posicion:");
    Serial.println((long)Setpoint);
  }
}
