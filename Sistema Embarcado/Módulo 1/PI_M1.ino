/*---    Firmware para Microcontrolador ESP32 instaladoo no módulo M1
Projeto de Esteira Eletrônica com sensoriamento para reabilitação FisioGama   ---*/

//-------------------------------------Bibliotecas---------------------------------------------
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heaertRate.h"
#include "spo2_algorithm.h"

//-------------------------------------MAX30105
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
int32_t bufferLength; 
int32_t spo2;
int8_t validSPO2; 
int32_t heartRate;
int8_t validHeartRate; 

//-------------------------------------Piezo
int iPiezo = 0;
//-------------------------------Monitor de carga
#define R2 100
#define R3 10
#define VOLTAGE_MAX 4200
#define VOLTAGE_MIN 3300
#define ADC_reference 1100
int adc = 18, adc_value;

//-------------------------------------MQTT
//Um topico para cada variavel a ser enviada
#define TOPICO_BPM
#define TOPICO_SPO2
#define TOPICO_Piezo
#define TOPICO_Bateria 
#define ID_MQTT  "esp32_mqtt"     //id mqtt (para identificação de sessão)

const char* SSID = " "; // Nome da rede para se conectar
const char* PASSWORD = " "; //Senhar da rede para se conectar
const char* BROKER_MQTT = ""; //URL do Broker
int BROKER_PORT = 1883;

WiFiClient espClient;
PubSubClient MQTT(espClient);

//-------------------------------------Funções 
void initWiFi(void); //Inicializa e conecta-se ao Wi-Fi
void initMQTT(void); //Inicializa parametros de conexão MQTT
void mqtt_callback(char* topic, byte* payload, unsigned int length); //Chamada toda vez que uma informação chega
void reconnectMQTT(void); //Reconecta-se ao broker caso a conexão caia
void reconnectWiFi(void); //Reconecta-se ao Wi-Fi caso a conexão caia
void VerificaConexoesWiFIEMQTT(void); // Verifica o estaddo das conexões

void initWiFi(void){
  delay(10);
  Serial.println("------Conexao WI-FI------");
  Serial.print("Conectando-se na rede: ");
  Serial.println(SSID);
  Serial.println("Aguarde");
  reconnectWiFi();
}

void initMQTT(void){
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);   //informa qual broker e porta deve ser conectado
  MQTT.setCallback(mqtt_callback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}

void mqtt_callback(char* topic, byte* payload, unsigned int length){
  String msg;
    for(int i = 0; i < length; i++){
      char c = (char)payload[i];
      msg += c;
    }
  iPWM1 = msg.toInt(); //Armazena valor de PWM 
}

void reconnectMQTT(void){
  while(!MQTT.connected()){
    Serial.print("* Tentando se conectar ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);
    if (MQTT.connect(ID_MQTT)){
      Serial.println("Conectado com sucesso ao broker MQTT!");
      MQTT.subscribe(TOPICO_SUBSCRIBE_LED); 
    } 
    else{
      Serial.println("Falha ao reconectar no broker.");
      Serial.println("Havera nova tentatica de conexao em 2s");
      delay(2000);
    }
  }
}

void VerificaConexoesWiFIEMQTT(void){
  if(!MQTT.connected()) 
    reconnectMQTT();
    reconnectWiFi();
}

void reconnectWiFi(void) {
  if(WiFi.status() == WL_CONNECTED)
    return;
    
  WiFi.begin(SSID, PASSWORD);  
  while(WiFi.status() != WL_CONNECTED){
    delay(100);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Conectado com sucesso na rede ");
  Serial.print(SSID);
  Serial.println("IP obtido: ");
  Serial.println(WiFi.localIP());
}

Pulse_event(){
  pulsos++; //Incrementa uma volta no contador de pulsos
  portYIELD_FROM_ISR();
}

//--------------------------------SETUP--------------------------------------------------------
void setup() {
  //Inicia comunicações
  Serial.begin(115200);
  initWifi();
  initMQTT();
  
  if(!particleSensor.begin(Wire, I2C_SPEED_FAST)){
    Serial.println("MAX30105 não foi encontrado");
    while (1);
  }

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }


  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  
  pinMode(adc, INPUT);
  // Transformando valores de tensao maxima e minima da bateria 
  //que passa pelo divisor de tensao para o equivalente de um valor retornado pelo ADC
  battery_max_adc = (VOLTAGE_MAX*R3/(R2+R3)) * ADC_reference/4096;
  battery_min_adc = (VOLTAGE_MIN*R3/(R2+R3))  * ADC_reference/4096;

}

//--------------------------------LOOP---------------------------------------------------------
void loop() {
  VerificaConexoesWiFIEMQTT();

  //Realiza leitura dos sensores
  //SPO2
  bufferLength = 100; 

  //Lê 100 primeiras amostras e determian o alcance do sinal
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false)
      particleSensor.check(); //Verifica se o sensor possui novas informações

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  //Calcula o a freq e o Sp02 dps de 100 primeiras amostras
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Obtem mais amostras e a cada 1 segundo calcula freq e Sp02
  while(1){
    //limpa as primeiras 25 amostras e desloca as demais 75
    for(byte i = 25; i < 100; i++){
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //obtem 25 amostras antes de calcula a freq
    for(byte i = 75; i < 100; i++){
      while(particleSensor.available() == false) 
        particleSensor.check();

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();

    }
    //Apos obter 25 novass amostras calcula Sp02 e a freq
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }

  //Realiza leitura analogica do Piezo
  iPiezo = analogRead(39);
  //Calculo da porcentagem de bateria
  adc_value = analogRead(adc); //Tensao da bateria
  int battery_percentage = 100 * (adc - BATTERY_MIN_ADC) / (BATTERY_MAX_ADC - BATTERY_MIN_ADC);

  if (battery_percentage < 0)
      battery_percentage = 0;
  if (battery_percentage > 100)
      battery_percentage = 100;
 
  //Publica valores dos sensores para o Broker
  MQTT.publish(TOPICO_BPM, heartRate);
  MQTT.publish(TOPICO_SPO2, spo2);
  MQTT.publish(TOPICO_Piezo, iPiezo);
  
  MQTT.loop();
}
