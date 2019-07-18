<a href="http://mcm.ipg.pt"><img src="http://www.ipg.pt/website/imgs/logotipo_ipg.jpg" title="IPG(MCM)" alt="IPG MCM 2018/19"></a>


<img src="https://user-images.githubusercontent.com/2634610/61477078-62371680-a986-11e9-8614-d7b2d84b5f3b.gif" width="400"><img src="https://user-images.githubusercontent.com/2634610/60996390-693da380-a34c-11e9-9300-e9928b95cf10.png" width="350">
# LoRa-TTN-SensorBox SmartFarm

Os dados ficam muito mais bonitos quando os pode visualizar. Este projeto torna fácil enviar dados de sensores de nodes LoRaWAN conectados à rede Things. 

Neste repositório encontra um step by step para registar um Node (ESP32 LoRa) na plataform TTN e a forma de enviar para esta os valores dos sensores: Luminosidade, Humidade, Temperatura, eCo2, TVOc (Qualidade do ar), Som dB, Pressão Atmosférica e valor de carga da bateria utilizando LoRa. Isto numa caixa enegeticamente autonoma que pode ficar a vários kilometros de ditância entre o node e o gateway. 

Pode assim realizar leitura de sensores em locais remotos sem necessidade de acesso à internet nesses locais.

Para visualizar os dados num mapa e na forma de um gráfico, pode utilizar este repositório:
https://github.com/daeynasvistas/LoRa-TTN-map


![emb](https://user-images.githubusercontent.com/2634610/61481326-f2c62480-a98f-11e9-9f43-5ac02e99809f.png)


## Alterar em cada Node (Main.cpp)
 ```` C++
static const PROGMEM u1_t NWKSKEY[16] = { 0x82, 0x60 };  //COLOCAR AQUI "Network Session Key" (ver screenshot)
static const u1_t PROGMEM APPSKEY[16] = { 0x59, 0x76 };  //COLOCAR AQUI "App Session Key"
static const u4_t DEVADDR = 0x26011874;                  //COLOCAR AQUI "0xDevice Address"
 ````

## Alterar em cada Node (Main.cpp)
indicar de forma implícita a freq. que configurou no dragino, neste caso "868300000"
 ````C++
    #if defined(CFG_eu868)
    //LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    //LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
 ````
 
## Payload enviado com 16 Bytes
 ````C++
  dataTX[0] = dtempC;
  dataTX[1] = dtempC >> 8;
  dataTX[2] = (dhumidity);
  dataTX[3] = (dhumidity) >> 8;
  dataTX[4] = dpressure;
  dataTX[5] = dpressure >> 8;
  dataTX[6] = daltitude;
  dataTX[7] = daltitude >> 8;  
  dataTX[8]  = (dco2);
  dataTX[9]  = (dco2) >> 8;  
  dataTX[10] = (dTVOC);
  dataTX[11] = (dTVOC) >> 8;  
  dataTX[12] = LuxSensorValue;
  dataTX[13] = (LuxSensorValue) >> 8;
  dataTX[14] = dSoundValue; 
  dataTX[15] = (dBattery) ;
 ````
 
## Deep Sleep  
  ````C++
#define TIME_TO_SLEEP  3600    /* ESP32 vai para deep sleep (in seconds) 3600 = 1hora */
  ````
  
## Calibrar medidor de nível da Bateria (3000mA <-> 3.7V)
  ````C++
// MONITOR BAT LEVEL calibração
#include <Pangodream_18650_CL.h> // Medidor BAT
#define ADC_PIN 34
#define CONV_FACTOR 1.7
#define READS 10
  ````
  
## Equipamento utilizado:

| Equipamento           | Descrição                                         |
| ----------------------|:-------------------------------------------------:|
| ESP32 LoRA TTGO       | Lora, Wifi, BLE                                   |
| Placa Solar           | 5V 1W                                             |
| BME280 Digital Sensor | Temperature Humidity Atmospheric Pressure         |
| CJMCU-811V1           | NTC CO2 eCO2 TVOC Air Mass                        |
| GL5528                | Light Sensor Module Light Photosensitive Sensor   |
| GY-BMP280             | High Precision Atmospheric Pressure sensor        |
| FC-28                 | Soil Hygrometer Humidity Detection                |
| OKY3131               | High sensitive voice sensor module                |
| KY-026                | Flame sensor module                               |
| 6DOF MPU-6050         | Axis Gyro With Accelerometer                      |

### Dragino Gateway

![1_lEjTfNTFnHXEfNBk9L4ENA](https://user-images.githubusercontent.com/2634610/61480043-fad09500-a98c-11e9-92f8-39651ff9bb0f.jpeg)

ver aqui:
https://pplware.sapo.pt/tutoriais/dragino-lora-gateway-ligar-rede-wi-fi/
  
# Como instalar

## 1. Clonar repositório 
(Foi utilizado VisualStudio Code + PlatformIO, mas pode editar Main.cpp com Arduino IDE)

 ````
 $ git clone https://github.com/daeynasvistas/LoRa-TTN-Node
 ````
 
 Librarias utilizadas
  ````
lib_deps =
     # Using library Name
     Adafruit CCS811 Library
     Adafruit BME280 Library
     LMIC-Arduino
     ESP8266_SSD1306
     https://github.com/enen/db_meter_library
     https://github.com/pangodream/18650CL         
               
  ````
  
  
 
## 2. Configurar Gateway Dragino:

![2](https://user-images.githubusercontent.com/2634610/60979177-28349780-a32a-11e9-8a4e-4e61da2ec065.PNG)
![3](https://user-images.githubusercontent.com/2634610/60979176-28349780-a32a-11e9-8e82-f1761ff73820.PNG)

Utilizar MAC do Gateway com ID:
'''
a840411bc834ffff  <- adicionar f até 16bit
'''

## 3. Na Consola TTN
### 3.1 Registar uma nova GATEWAY

![4](https://user-images.githubusercontent.com/2634610/60979935-7302df00-a32b-11e9-8283-677316e516a7.png)
![5](https://user-images.githubusercontent.com/2634610/60980019-9b8ad900-a32b-11e9-85bd-c030760e7c3f.PNG)
![6](https://user-images.githubusercontent.com/2634610/60980101-c2490f80-a32b-11e9-9398-1541cc9e1632.PNG)

### 3.2 Adicionar uma nova aplicação

![7](https://user-images.githubusercontent.com/2634610/60980247-05a37e00-a32c-11e9-9ac9-ce3cfaa22ae8.PNG)
![8](https://user-images.githubusercontent.com/2634610/60980437-6337ca80-a32c-11e9-9ac8-792edc661a5c.PNG)

#### Obtemos Assim a chave da applicação (App ID)
![9](https://user-images.githubusercontent.com/2634610/60980508-84002000-a32c-11e9-8a67-de1345c74486.PNG)

## 4. Registar um novo Node (na aplicação anteriormente criada)
![10](https://user-images.githubusercontent.com/2634610/60980723-e3f6c680-a32c-11e9-96d6-f858a3a0b249.PNG)

### 4.1 Obtemos as chaves necessárias aqui
![11](https://user-images.githubusercontent.com/2634610/60980966-61223b80-a32d-11e9-87c5-328d7a43f843.png)

## 5. Foi implementado um Payload "custom"
![12](https://user-images.githubusercontent.com/2634610/60981751-d2162300-a32e-11e9-9ac4-ad43772cf6a8.PNG)

### 5.1 Payload Custom

  ````JS
function Decoder(bytes, port) {
  var temp = parseInt(bytes[0] + (bytes[1] << 8 )) / 100;
  var humidity = parseInt(bytes[2] + (bytes[3] << 8 )) / 100;
  var pressure = parseInt(bytes[4] + (bytes[5] << 8 )) / 10;
  var altitude = parseInt(bytes[6] + (bytes[7] << 8 ) - 100) / 10;
  var eCo2 = parseInt(bytes[8] + (bytes[9] << 8 ) ) / 10; 
  var TVOC = parseInt(bytes[10] + (bytes[11] << 8 )) / 10; 
  var LUX = parseInt(bytes[12] + (bytes[13] << 8 )) ; 
  var SOUND = parseInt(bytes[14]);  
  var Battery = parseInt(bytes[15]);  
    
  return { temp: temp,
         humidity: humidity,
         pressure: pressure, 
         altitude: altitude,
         eCo2:eCo2,
         TVOC:TVOC,
         LUX:LUX,
         SOUND:SOUND,
         Battery:Battery
  }
}
  ````


## Resultado
<img src="https://user-images.githubusercontent.com/2634610/61478098-bfcc6280-a988-11e9-84fb-2ab843553e6c.png" width="400"><img src="https://user-images.githubusercontent.com/2634610/61477861-2604b580-a988-11e9-952f-e13b586248d8.png" width="400">

![IMG_20190715_090358](https://user-images.githubusercontent.com/2634610/61478252-10dc5680-a989-11e9-8bc8-b252453047d1.jpg)
![Capturar](https://user-images.githubusercontent.com/2634610/61478576-ad9ef400-a989-11e9-8cfc-eb5198ba3966.PNG)



