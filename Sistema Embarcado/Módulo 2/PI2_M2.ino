/*---    Firmware para Microcontrolador ESP32 instaladoo no módulo M2
Projeto de Esteira Eletrônica com sensoriamento para reabilitação FisioGama---*/

//-------------------------------------Bibliotecas---------------------------------------------
#include <WiFi.h>
#include <PubSubClient.h>

//-------------------------------------SG - Sensores de força
int iSG1, iSG2, iSG3, iSG4 = 0;

//-------------------------------------S1 - Encoder
long int pulsos = 0;
unsigned long timeps = 0;
float vel = 0;
float diametro = 0;

//-------------------------------------MQTT
//Um topico para cada variavel a ser enviada
#define TOPICO_SG1
#define TOPICO_SG2
#define TOPICO_SG3
#define TOPICO_SG4
#define TOPICO_S1

//Um topico para cada variavel a ser lida
#define TOPICO_SUBSCRIBE_PWM1
 
#define ID_MQTT  "esp32_mqtt"     //id mqtt (para identificação de sessão)

const char* SSID = " "; // Nome da rede para se conectar
const char* PASSWORD = " "; //Senhar da rede para se conectar
const char* BROKER_MQTT = ""; //URL do Broker
int BROKER_PORT = 1883;

WiFiClient espClient;
PubSubClient MQTT(espClient);
//-------------------------------------Motor
// E0 e E1 para definição de sentido de rotação do motor
#define E0 22
#define E1 21
#define PWM1 33
#define S1 32
int iPWM1 = 0;

//Configuração de variaveis de PWM
const int freq = 30000;
const int canal = 0;
const int resolution = 8;
//-------------------------------------Sensores
// Entradas das células de força
#define SG1 36
#define SG2 39
#define SG3 34
#define SG4 35
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
  pulsos++; //incrementa uma volta no contador de pulsos
  portYIELD_FROM_ISR();
}

Pulse_event2(){
  ledcWrite(0, 0);  //desliga o motor no instante que o sensor recebe o sinal da presilha e desativa o moto por 1 minuto, tempo suficiente para desligamento da maquina
  delay(60000);
  portYIELD_FROM_ISR();
}

//--------------------------------SETUP--------------------------------------------------------
void setup() {
  pinMode(E0, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  ledcSetup(canal, freq, resolution);
  ledcAttachPin(PWM, canal);
  
  //Configuração inicial do motor como parado e a direção
  digitalWrite(E0, LOW);
  digitalWrite(E1, LOW);
  ledcWrite(0, 0);

  //Inicia comunicações
  Serial.begin(115200);
  initWifi();
  initMQTT();

  //Ativa interrupt
  attachInterrupt(digitalPinToInterrupt(S1),Pulse_Event, RISING);
  attachInterrupt(digitalPinToInterrupt(S2),Pulse_Event2, FALLING);
}

//--------------------------------SETUP--------------------------------------------------------
void loop() {
  timeps = millis();  //Inicia contagem de tempo
  
  VerificaConexoesWiFIEMQTT();

  //Ajusta PWM do motor de acordo com o valor do broker
  ledcWrite(0, iPWM1);
  
  if(timeps == 1000){ //Se passado 1 segundo, realiza calculo de velocidade
    detachInterrupt(S1);
    vel = (pulsos / 600) * diametro*3.6; //Calculo da velocidade em km/s
    timeps = 0;
    pulsos = 0;
  }

  //Realiza leitura dos sensores de força
  iSG1 = analogRead(SG1);
  iSG2 = analogRead(SG2);
  iSG3 = analogRead(SG3);
  iSG4 = analogRead(SG4);

  //Publica valores dos sensores para o Broker
  MQTT.publish(TOPICO_SG1, iSG1);
  MQTT.publish(TOPICO_SG2, iSG2);
  MQTT.publish(TOPICO_SG3, iSG3);
  MQTT.publish(TOPICO_SG4, iSG4);
  MQTT.publish(TOPICO_S1, vel);
  
  MQTT.loop();
}
