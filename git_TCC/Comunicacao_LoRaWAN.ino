/************************************************
* Código adaptado de:
* RoboCore - IoT DevKit - LoRaWAN   
*
* Conecta a rede LoRaWAN e envia a leitura do
* módulo GPS a cada 5 minutos ou quando o botão
* for pressionado.
***********************************************/


//verifica se o hardware conectado é um ESP23
#if !defined(ARDUINO_ESP32_DEV)
#error Use this example with the ESP32
#endif

// --------------------------------------------------
// Bibliotecas

// Incluimos todas as bibliotecas que usaremos neste codigo
#include "RoboCore_SMW_SX1276M0.h" //Permite a comunicação com o módulo LoRaWAN Bee
#include "ArduinoJson.h" //Permite a manipulação de JSON, como sua serialização
#include "HardwareSerial.h" //Permite que qualquer GPIO seja utilizado para comunicação serial
#include <TinyGPS++.h> // Bibliteca com as funções de GPS
      
// --------------------------------------------------
// Variáveis

// Informamos ao ESP32 com quais pinos vamos nos conectar ao módulo GPS
HardwareSerial SerialGPS(1);
#define RXD1 32
#define TXD1 33

//objeto GPS
TinyGPSPlus gps;

// Informamos ao ESP32 com quais pinos vamos nos conectar ao módulo LoRaWAN Bee
HardwareSerial LoRaSerial(2);
#define RXD2 16
#define TXD2 17
      
// Informamos ao ESP32 em quais pinos nossos sensores, botão e LEDs estão ligados
const int pinoBotao = 4;
const int statusLED = 13;
const int sendLED = 2;

// Inicializamos a variável que fará a leitura do botão
int estadoBotao = 0;

//Inicializar variáveis de latitude e longitude
int lat, lng;

// Criamos a instância do LoRaWAN, que será usada no decorrer do código
SMW_SX1276M0 lorawan(LoRaSerial);

CommandResponse response;

// Inserimos as chaves APPEUI e APPKEY, conforme configurado na plataforma ProIoT
const char APPEUI[] = "70d505d24e120513";
const char APPKEY[] = "d7aae5b015acf79be8a5ffe490e56f3c";

// Criamos uma variável que determinará de quanto em quanto tempo a informacao via LoRaWAN será enviada
const unsigned long PAUSE_TIME = 300000; // [ms] (5 min)
unsigned long timeout;

// --------------------------------------------------
// Prototypes

void event_handler(Event);

// --------------------------------------------------
// --------------------------------------------------

// início da função setup()
void setup() {
      
  // Inicializamos a comunicacao serial entre ESP32 e computador
  Serial.begin(115200);
  Serial.println(F("--- SMW_SX1276M0 Join (OTAA) ---"));

  // Inicializamos a comunicacao serial entre ESP32 e modulo GPS
  SerialGPS.begin(9600, SERIAL_8N1, RXD1, TXD1);
  
  // Inicializamos a comunicacao serial entre ESP32 e modulo LoRaWAN Bee
  LoRaSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Criamos uma variável que irá "escutar" se vier algo do modulo LoRaWAN Bee
  lorawan.event_listener = &event_handler;
  Serial.println(F("Handler set"));
  
  // Resetamos o modulo LoRaWAN Bee para configurá-lo
  delay(1000);
  lorawan.setPinReset(5);
  lorawan.reset();
  
  // Lemos o DEVEUI do modulo LoRaWAN Bee e mostramos no Monitor Serial
  char deveui[16];
  response = lorawan.get_DevEUI(deveui);
  if(response == CommandResponse::OK){
    Serial.print(F("DevEUI: "));
    Serial.write((uint8_t *)deveui, 16);
    Serial.println();
  } else {
    Serial.println(F("Error getting the Device EUI"));
  }

  // Configuramos a chave APPEUI no módulo e mostramos no Monitor Serial
  response = lorawan.set_AppEUI(APPEUI);
  if(response == CommandResponse::OK){
    Serial.print(F("Application EUI set ("));
    Serial.write((uint8_t *)APPEUI, 16);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Application EUI"));
  }

  // Configuramos a chave APPKEY no módulo e mostramos no Monitor Serial
  response = lorawan.set_AppKey(APPKEY);
  if(response == CommandResponse::OK){
    Serial.print(F("Application Key set ("));
    Serial.write((uint8_t *)APPKEY, 32);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Application Key"));
  }

  // Configuramos o modo de operação do LoRaWAN Bee para OTAA
  response = lorawan.set_JoinMode(SMW_SX1276M0_JOIN_MODE_OTAA);
  if(response == CommandResponse::OK){
    Serial.println(F("Mode set to OTAA"));
  } else {
    Serial.println(F("Error setting the join mode"));
  }

  // Começamos as tentativas para conexão na rede LoRaWAN da ATC
  Serial.println(F("Joining the network"));
  lorawan.join();
  
  // Configuramos os pinos como entrada ou saida, conforme sua funcao
  pinMode(pinoBotao, INPUT);
  pinMode(statusLED, OUTPUT);
  pinMode(sendLED, OUTPUT);
  
}

// --------------------------------------------------
// Criamos uma função para fazer a leitura dos dados, seja quando o código envia
// automaticamente o payload para a plataforma ou quando pressionamos o botão
      
void enviaDados(){

  // Chama a função que obtêm os dados de latitude e longitude
  funcaoGPS();
  
  // Acendemos o LED azul do ESP32 quando iniciamos a leitura e envio dos dados
  digitalWrite(sendLED, HIGH);

  // Criamos uma variável JSON que irá conter a quantidade (2) e os valores das variáveis
  DynamicJsonDocument json(JSON_OBJECT_SIZE(2));
  
  // Configuramos a variável JSON com os ALIAS criados na plataforma ProIoT
  // e salvamos em cada componente o respectivo valor da variável lida
  json["LAT"] = lat;
  json["LNG"] = lng;

  // Criamos uma String chamada payload que irá conter todas as informações do JSON
  String payload = "";
      
  // Serialização da String para estar no padrão de envio à rede LoRaWAN
  serializeJson(json, payload);
      
  // Mostra no Monitor Serial os valores enviado para debugging
  Serial.print("Valores enviados: ");
  Serial.println(payload);

  // Enviamos todas as informações via LoRaWAN
  // sendT é para enviar Texto (sendText)
  lorawan.sendT(1, payload);

  // Apagamos o LED azul do ESP32 quando terminamos de enviar
  digitalWrite(sendLED, LOW);
  
}

// --------------------------------------------------

// --------------------------------------------------
// Criamos uma função para fazer a leitura das informações do GPS

void funcaoGPS(){
  
  // Só entra na rotina caso haja dados vindo da serial
  while (SerialGPS.available() > 0 ){
        
    // Canaliza os caracteres do formato NMEA para valores em graus 
    if(gps.encode(SerialGPS.read())){
          
      // Se a localização for válida, obtêm os valores de latitude e longitude    
      if (gps.location.isValid()) {
        lat=gps.location.lat()*1000000;
        lng=gps.location.lng()*1000000;
        Serial.print("LAT = "); 
        Serial.println(lat );
        Serial.print("LONG = ");
        Serial.println(lng);
        return;
        } 
    } 
  }
}

// --------------------------------------------------



void loop() {
  // "Escutamos" se algo vem do módulo LoRaWAN Bee
  lorawan.listen();

  // Se esta conectado a rede entra nesta rotina
  if(lorawan.isConnected()){
  
    // A cada PAUSE_TIME milisegundos, acessamos a função de envio de dados
    if(timeout < millis()){

      enviaDados();
      
      timeout = millis() + PAUSE_TIME;
        
    // Faz a leitura do botão e acessa a função de envio de dados
    estadoBotao = digitalRead(pinoBotao);
    if(estadoBotao == LOW){
      delay(30);
      estadoBotao = digitalRead(pinoBotao);
      enviaDados();
      while(digitalRead(pinoBotao) == LOW){
        // Espera soltar o botao
      }
    }
    

    }
  } else {
    // Se nao conseguir se conectar a rede LoRaWAN, imprime no 
    // Monitor Serial "." a cada 5 segundos
    if(timeout < millis()){
      Serial.println('.');
      timeout = millis() + 5000; // 5 s
    }
  }
}

// --------------------------------------------------
// --------------------------------------------------

// Verifica o que foi recebido do módulo LoRaWAN Bee e, se o que foi recebido
// for um evento do tipo JOINED, informa que se conectou no Monitor Serial e 
// acende o LED do IoT DevKit
void event_handler(Event type){
  if(type == Event::JOINED){
    Serial.println(F("Joined"));
    digitalWrite(statusLED, HIGH);
  }
}
