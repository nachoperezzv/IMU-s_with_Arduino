/*
 * Programa para calibración de la IMU MPU-6050.
 * Código realizado por:
 *    Pérez Vilaplana, Ignacio
 * 
 * Actividad 2:
 */

 //Se incluyen las librerías del sensor MPU-6050 ("MPU6050.h")
 //así como "I2Cdev.h" y "Wire.h" para la manipulación del 
 //sensor de manera más eficiente. Aunque en un principio no se
 //necesita la libreria I2C puesto que no se va a haber otros 
 //dispositivos esclavos
#include "MPU6050.h"
#include "I2Cdev.h"
#include "Wire.h"

//Declaración de funciones empleadas
void calibracion();

//Declaración del objeto de la clase MPU6050 para manipular
//el sensor. El modo de uso va a ser el predeterminado (0x68),
//pero se podría emplear otro (0x69). Los rangos por defecto
//de este sensor son:
//    · acelerómetro -> -2g / 2g
//    · giroscopio   -> -250ºseg / 250ºseg
MPU6050 imu;

//Declaración para los valores RAW del acelerómetro y giroscopio
int ax, ay, az;
int gx, gy, gz;

//Declaración variables filtros pasa bajos
long int f_ax, f_ay, f_az;  int p_ax, p_ay, p_az;
long int f_gx, f_gy, f_gz;  int p_gx, p_gy, p_gz;

//Declaración variables para offSets
int ax_o, ay_o, az_o;
int gx_o, gy_o, gz_o;

//Declaración cálculo angulo con aceleraciones
float accel_ang_x;
float accel_ang_y;

//Declaración cálculo angulo con giroscopio
long int tiempo_prev, dt;
float girosc_ang_x;  float girosc_ang_x_prev;
float girosc_ang_y;  float girosc_ang_y_prev;

//Declaración cálculo angulo con Filtro Complementario
float ang_x;         float ang_x_prev;
float ang_y;         float ang_y_prev;

//Declaración de otras variables
int counter = 0;
int medidas = 0;
int t_p;
int t_o;

void setup() {
  // Configuración del puerto serie
  Serial.begin(115200);

  // Configuración del I2C
  Wire.begin();

  // Configuración de la IMU MPU-6050
  imu.initialize();

  // Testeo de la conexión. Si se ha iniciado correctamente
  // se deben leer los offset de los tres ejes para el ace. y giro.
  if(imu.testConnection()){
    Serial.println("IMU iniciada");
    ax_o = imu.getXAccelOffset();   gx_o = imu.getXGyroOffset();
    ay_o = imu.getYAccelOffset();   gy_o = imu.getYGyroOffset();
    az_o = imu.getZAccelOffset();   gz_o = imu.getZGyroOffset();
    Serial.println("Presione tecla para comenzar calibración");
    while(true){if (Serial.available()) break;} 
    
    //Inicio de la calibracion
    Serial.println("Calibrando...");
    calibracion();
    Serial.println("Calibracion finalizada");
    
    //Espera a que usuario decida comenzar las medidas
    while(Serial.available()){Serial.read();} //Vaciar buffer
    Serial.println("Presione tecla para comenzar medición");
    while(true){if (Serial.available()) break;}
    delay(1000);
    tiempo_prev = millis();
    t_p = millis();
    t_o = millis();

    //Impresión primera línea para archivo CSV
    Serial.println("T,RollA,PithA,RollG,PithG,RollFC,PitchFC");
  }
  else{
    Serial.println("Fallo incio IMU");
  }
}

void loop() {

  // Toma de medidas cada 50ms hasta tomar 200 medidas
  if(millis()-t_p>=50 and medidas<200){
      // Calculo de los angulos de rotación a partir de la 
      // variación de la aceleración en los ejes X e Y
      calc_ang_accel();
  
      // Cálculo de los ángulos de rotación a partir de la
      // variación de la velocidad angular en los ejes X e Y
      calc_ang_giros();

      // Cálculo de los ángulos tras aplicar el filtro 
      // complementario (acelerómetro + giroscopio)
      ang_x = 0.97*girosc_ang_x + 0.03*accel_ang_x;
      ang_y = 0.97*girosc_ang_y + 0.03*accel_ang_y;

      // Copiamos los datos en estas variables para que el
      // cálculo de los nuevos ángulos del giroscopio se haga
      // con valores filtrados y actualizados.
      ang_x_prev = ang_x;
      ang_y_prev = ang_y;

      // Impresión de los datos en formato CSV
      Serial.print(t_p-t_o);      Serial.print(",");
      Serial.print(accel_ang_x);  Serial.print(",");
      Serial.print(accel_ang_y);  Serial.print(",");
      Serial.print(girosc_ang_x); Serial.print(",");
      Serial.print(girosc_ang_y); Serial.print(",");
      Serial.print(ang_x);        Serial.print(",");
      Serial.println(ang_y);    


      Serial.println(dt);

      // Incremento del número de medidas tomadas
      medidas++;

      // Nuevo tiempo para medida
      t_p = millis();
  }
}


void calibracion(){
    //Se va a realizar mientras que el offset no tengas unos valores
    //de error aceptables. Se ha escogido el 3 como error máximo. 
    //La aceleración en el eje Z debe de ser de un 1G debido a la 
    //gravedad terrestre. 

    //Lectura de las aceleraciones y velocidades angulares
    imu.getAcceleration(&ax,&ay,&az);
    imu.getAcceleration(&gx,&gy,&gz);
    
    while(abs(ax)>10 or abs(ay)>10 or az/16384<0.99 or
          abs(gx)>10 or abs(gy)>10 or abs(gz)>10){
      
      // Leer las aceleraciones y velocidades angulares
        imu.getAcceleration(&ax, &ay, &az);
        imu.getRotation(&gx, &gy, &gz);
      
        // Filtrar las lecturas
        f_ax = f_ax-(f_ax>>5)+ax;
        p_ax = f_ax>>5;
      
        f_ay = f_ay-(f_ay>>5)+ay;
        p_ay = f_ay>>5;
      
        f_az = f_az-(f_az>>5)+az;
        p_az = f_az>>5;
      
        f_gx = f_gx-(f_gx>>3)+gx;
        p_gx = f_gx>>3;
      
        f_gy = f_gy-(f_gy>>3)+gy;
        p_gy = f_gy>>3;
      
        f_gz = f_gz-(f_gz>>3)+gz;
        p_gz = f_gz>>3;
      
        //Cada 100 lecturas corregir el offset
        if (counter==100){      
              
          //Calibrar el acelerometro a 1g en el eje z (ajustar el offset)
          if (p_ax>0) ax_o--;
          else {ax_o++;}
          if (p_ay>0) ay_o--;
          else {ay_o++;}
          if (p_az-16384>0) az_o--;
          else {az_o++;}
          
          imu.setXAccelOffset(ax_o);
          imu.setYAccelOffset(ay_o);
          imu.setZAccelOffset(az_o);
      
          //Calibrar el giroscopio a 0º/s en todos los ejes (ajustar el offset)
          if (p_gx>0) gx_o--;
          else {gx_o++;}
          if (p_gy>0) gy_o--;
          else {gy_o++;}
          if (p_gz>0) gz_o--;
          else {gz_o++;}
          
          imu.setXGyroOffset(gx_o);
          imu.setYGyroOffset(gy_o);
          imu.setZGyroOffset(gz_o);    
      
          counter=0;
        }
        counter++;        
    }
}

void calc_ang_accel(){
  imu.getAcceleration(&ax,&ay,&az);
  accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
}

void calc_ang_giros(){
  imu.getRotation(&gx,&gy,&gz);

  // Tiempo transcurrido desde la última medida para la integración
  // de la variación de velocidad y cálculo de nueva posición
  dt = millis() - tiempo_prev;
  tiempo_prev = millis();

  // Cálculo de la posición
  const float gyroScale = 250.0/32768.0;
  girosc_ang_x = (float)gx*gyroScale*dt/1000.0 + ang_x_prev;
  girosc_ang_y = (float)gy*gyroScale*dt/1000.0 + ang_y_prev;

  // Se guarda la última posición para recalcular la variación en la 
  // próxima medida
  //girosc_ang_x_prev=girosc_ang_x;
  //girosc_ang_y_prev=girosc_ang_y;
}
