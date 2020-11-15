#include <WiFiEspClient.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include "SoftwareSerial.h"
#include <ThingsBoard.h>

#define WIFI_AP "Familia_RV_2G" //nome da rede wifi
#define WIFI_PASSWORD "50731802"   //senha da rede

#define TOKEN "OFOiUMHZCG4ORlWgoqwf" //token - definido pelo componente adicionado no thingsboard OFOiUMHZCG4ORlWgoqwf 

#define svdc A0  
#define sidc0 A1
#define sidc1 A2


char thingsboardServer[] = "demo.thingsboard.io"; //servidor mqtt - no caso é o do thingsboard demo.thingsboard.io

// Objetos 
WiFiEspClient espClient;
ThingsBoard tb(espClient);

// Definição do RX TX por SW.
SoftwareSerial soft(2, 3); // RX, TX

int status = WL_IDLE_STATUS;
unsigned long lastSend;

void setup() {
    Serial.begin(9600);
    InitWiFi();
    lastSend = 0;
    pinMode(svdc, INPUT);
    pinMode(sidc0, INPUT);
    pinMode(sidc1, INPUT);
//    pinMode(svac, INPUT);
}

void loop() {
    // Checa se está conectado ao wi-fi
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
        while ( status != WL_CONNECTED) {
        Serial.print("Tentando se conectar à rede: ");
        Serial.println(WIFI_AP);
        // Tentar conexão
        status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
        delay(500);
        }
        Serial.println("Conectado!");
    }

    // Se o Thingsboard estiver desconectado
    if ( !tb.connected() ) {
        reconnect();
    }

    // Comparo o tempo desde que rodei o prog. princ. pela primeira vez até o momento atual
    // só vou ler os sensores se o tempo entre o primeiro envio for >1s
    if ( millis() - lastSend > 1000 ) { // Update and send only after 1 seconds
        ler_sensores();
        lastSend = millis();
    }

    tb.loop();
}

float v_dc(int pino, float correcao){
  // Correção em volts - atentar que é usado mV (0.xxxx) 
  // Sensor DC - correcao = 0.053737 
  float total;  // Valor médio da leitura
  int it;       // Iterações para média - quantas leituras serão feitas
  it = 10000; 
  total = 0;
  
  // Média da leitura dos sensor
  for (int i=0; i<it; i++){
    total = total + analogRead(pino);
  } 
  total = total/it;
  
  return (((total*25.0)/1023.0) + correcao);
}

float i_dc(int pino, int imax ,float correcao){
  /*  Função para retornar o valor de corrente lido do sensor DC
   *    - Pino - pino ANALÓGICO do sensor
   *    - imax - corrente máx do sensor
   *    - correcao - valor númerico - precisa ser calibrado, varia de acordo com o sensor - aprox. 511
   */
  int it;                       // iterações para adquirir média de leitura
  float res = (5.0/1023.0), s;  // resolução (Padrão para o arduino UNO), sensibilidade
  unsigned long int total;      // total medido dos sensores
  
  total=0;
  it=1000;
  
  // Seleciono a sensibilidade de acordo com o sensor usado - valores em mV/A (Retirado do datasheet)
  switch (imax){
    case 5:   // Sensor 5A 
      s = 185.0 * 0.001;
      break;
    case 20:  // Sensor 20A
      s = 100.0 * 0.001;
      break;
    case 30:  // Sensor 30A
      s = 66.0 * 0.001;
      break;
  }

  // Somo o total de leitura realizada
  for (int i=0;i<it;i++){
    total = total + analogRead(pino); // Total lido (numero puro da leitura - 0 a 1023)
  }
  
  total = total/it; // Retiro a média
 
  return ((res/s)*(total-correcao));
}

void ler_sensores()
{
  float vac, idc1, perc, pot;
  
  float vdc = v_dc(svdc, 0.053737);
  Serial.println("Leitura VCC-DC: " + String(vdc));
 
  
  float idc0 = i_dc(sidc0, 5, 511.0);
  Serial.println("Leitura Ip-DC_0: " + String(idc0));

  perc = ((vdc-7.5)*100)/4;
  pot = (idc0 * vdc);
  Serial.print("perBAT: ");
  Serial.println(perc);
  Serial.print("Potencia: ");
  Serial.println(pot);
  if ( isnan(vdc)) {
    Serial.println("Erro na leitura - vdc");
    return;
  }

  if ( isnan(idc0)) {
    Serial.println("Erro na leitura - idc0"); // trocar para representar qual corrente (placa ou bateria)
    return;
  }

  if ( isnan(idc1)) {
    Serial.println("Erro na leitura - idc1"); // trocar para representar qual corrente (placa ou bateria)
    return;
  }
  Serial.println("Enviando dados...");

  tb.sendTelemetryFloat("svdc", vdc);
  tb.sendTelemetryFloat("sidc0", idc0);
  tb.sendTelemetryFloat("perc_bat", perc);
  tb.sendTelemetryFloat("pot", pot);
}

void InitWiFi()
{
    // initialize serial for ESP module
    soft.begin(9600);
    // initialize ESP module
    WiFi.init(&soft);
    // check for the presence of the shield
    if (WiFi.status() == WL_NO_SHIELD) {
        Serial.println("Módulo Wi-Fi não detectado");
        // don't continue
        while (true);
    }

    Serial.println("Conectando ...");
        // attempt to connect to WiFi network
        while ( status != WL_CONNECTED) {
        Serial.print("Tentando se conectar à rede: ");
        Serial.println(WIFI_AP);
        // Connect to WPA/WPA2 network
        status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
        delay(500);
    }
    Serial.println("Connected to AP");
}

void reconnect() {
  // Tento conectar enquanto não haver sucesso
  while (!tb.connected()) {
    Serial.print("Tentando se conectar com o Thingsboard ...");
    // Tentativa de conexão ao TB.
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println( "[CONECTADO]" );
    } else {
      Serial.print( "[ERRO]" );
      Serial.println( " : Tentando novamente em 5s" );
      delay( 5000 );
    }
  }
}
