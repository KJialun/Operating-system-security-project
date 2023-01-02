#include <WiFi.h>
#include <PubSubClient.h>
#include "mbedtls/aes.h"  //this is a header file we use to encrypt the data. 
#include <SimpleDHT.h>
#include "mbedtls/md.h" //header used to authenticate the data using HMAC



/*=========================== 用户自定义全局变量&宏定义 =============================*/
#define Sensor 26
#define button 27
#define LED 32
// #define LED2 33
#define BUFFER_SIZE 16  // AES 128 bit => 16 byte 

/*=========================== 全局变量及宏定义 =============================*/
//Sensor
int sample_error;
uint8_t temperature = 0;
uint8_t humidity = 0;
int Received_temperature = 0;
int Received_humidity = 0;

unsigned char *encrypted,*decrypted,*unpadding_decrypted;;  // why we use unsigned char *?  1. because the  output of  mbedtls_aes_crypt_ecb() is unsigned char  
//more reasons check:  https://stackoverflow.com/questions/14769917/when-to-use-unsigned-char-pointer#:~:text=The%20unsinged%20char%20type%20is%20usually%20used%20as,the%20binary%20data%20buffer%20%28or%20its%201st%20element%29.
uint8_t padding_plainText_length_global; 
uint8_t padding_len ;
char * key = "9728908140000000";
char * authkey = "hardcoded"; //we could use DH-algo to decide on a key, but the assistants told us this would suffice, as to simulate the key being e.g. hardcoded in
const int HMAC_result_length = 32;


char plainText[100] ;  
char Received[100] ; 
char Receive_encrypted[100];
char Receive_authen[100];
char Receive_integrity[100];





SimpleDHT11 dht11(Sensor);
WiFiClient espClient;                                                                         // WiFiClient : 创建wifi 客户端对象,用于接入WIFI
PubSubClient mqttClient(espClient);                                                           // PubSubClient 创建mqtt 客户端对象,用于接入MQTT
PubSubClient mClient(espClient);                                                              // PubSubClient 创建mqtt 客户端对象,用于接入MQTT

const char* ssid = "jialun";
const char* password = "66666666";
const char* mqtt_server = "192.168.1.39";
const char *clientID = "ESP32_A";

/*定义线程句柄*/
TaskHandle_t ThreadWiFi;    //WIFI线程
TaskHandle_t ThreadMQTT;    //MQTT线程
TaskHandle_t ThreadSensor;  //Sensor线程
/*定义定时器句柄*/
TimerHandle_t TimeAlarmLED;

/*定义信号量*/
SemaphoreHandle_t sem_WIFI;   //信号量用于WIFI线程
SemaphoreHandle_t sem_MQTT;   //信号量用于MQTT
SemaphoreHandle_t sem_Sensor; //信号量传感器用于上传数据

/*线程入口声明*/
void ThreadWiFiEntry(void *pvParameters);    //WIFI线程入口
void ThreadMqttEntry(void *pvParameters);    //MQTT线程入口
void ThreadSensorEntry(void *pvParameters);  //Sensor线程入口

/*定时器入口声明*/
void TIMAlarmLEDEntry(TimerHandle_t xTimer); //LED警告定时器入口

/*函数声明*/
void TaskAPPStart();                                                       // 应用任务启动函数
void wifi_connect();                                                       // WiFi连接
void mqtt_connect();                                                       // MQTT私有云平台连接
void callback(char *topics, byte *payload, unsigned int length);           // MQTT统一监听订阅主题回调函数
// void SensorInit();                                                         // 传感器初始化


/*====================================== 初始化 =========================================*/
/*
 * @brief:初始化 
 * @param:none
 * @retval:none
*/
void setup()
{
  Serial.begin(115200); //设置串口波特率
  pinMode(LED, OUTPUT); //电源、WIFI连接状态指示灯
  /*====================用户初始化=======================*/
  /**如 IO口模式初始化**/

  /*====================用户初始化=======================*/
  // SensorInit();   //传感器初始化
  TaskAPPStart(); //任务启动
}
void loop()
{
  /**
   * 不放程序,所有代码放在线程任务中进行 
   */
}

/*======================================= TaskAPPStart ====================================*/
/*
 * @brief: 启动创建任务
 * @param:none
 * @retval:none
*/
void TaskAPPStart()
{
    sem_WIFI = xSemaphoreCreateCounting(1, 0);
    if (sem_WIFI == NULL)
    {
      Serial.println("WIFI Semaphore Create Failed ......");
    }
    else
    {
      Serial.println("WIFI Semaphore Create Success......");
    }
    sem_MQTT = xSemaphoreCreateCounting(1, 0);
    if (sem_MQTT == NULL)
    {
      Serial.println("MQTT Semaphore Create Failed......");
    }
    else
    {
      Serial.println("MQTT Semaphore Create Success......");
    }
    sem_Sensor = xSemaphoreCreateCounting(1, 0);
    if (sem_Sensor == NULL)
    {
      Serial.println("Sensor Semaphore Create Failed......");
    }
    else
    {
      Serial.println("Sensor Semaphore Create Success......");
    }
    
    /*创建WiFi线程*/
    BaseType_t status;
    status = xTaskCreatePinnedToCore(
        (TaskFunction_t)ThreadWiFiEntry,   //WIFI线程入口
        (const char *const) "Thread_WIFI", //线程名称
        (const uint32_t)2048 * 2,          //线程栈
        (void *const)NULL,                 //WIFI线程入口参数
        (UBaseType_t)15,                   //线程优先级 0-24 数值越大优先级越高
        (TaskHandle_t *)&ThreadWiFi,       //线程句柄
        (const BaseType_t)APP_CPU_NUM);    //指定内核1
    if (status == pdPASS)
    {
      Serial.println("WiFi Task Create Success...");
    }
    else
    {
      Serial.println("WiFi Task Create Failed...");
    }
    /*创建MQTT线程*/
    status = xTaskCreatePinnedToCore(
        (TaskFunction_t)ThreadMqttEntry,   //MQTT线程入口
        (const char *const) "Thread_MQTT", //线程名称
        (const uint32_t)8192,              //线程栈
        (void *const)NULL,                 //MQTT线程入口参数
        (UBaseType_t)13,                   //线程优先级 0-24 数值越大优先级越高
        (TaskHandle_t *)&ThreadMQTT,       //线程句柄
        (const BaseType_t)APP_CPU_NUM);    //指定内核1
    if (status == pdPASS)
    {
      Serial.println("MQTT Task Create Success...");
    }
    else
    {
      Serial.println("MQTT Task Create Failed...");
    }
    /*创建Sensor线程*/
    status = xTaskCreatePinnedToCore(
        (TaskFunction_t)ThreadSensorEntry,   //Sensor线程入口
        (const char *const) "Thread_Sensor", //线程名称
        (const uint32_t)8192,                //线程栈
        (void *const)NULL,                   //Sensor线程入口参数
        (UBaseType_t)10,                     //线程优先级 0-24 数值越大优先级越高
        (TaskHandle_t *)&ThreadSensor,       //线程句柄
        (const BaseType_t)PRO_CPU_NUM);      //指定内核1
    if (status == pdPASS)
    {
      Serial.println("Sensor Task Create Success...");
    }
    else
    {
      Serial.println("Sensor Task Create Failed...");
    }
    /*连接网络提示定时器*/
    TimeAlarmLED = xTimerCreate(
        "TIM_Alarm_LED",    //定时器名称
        pdMS_TO_TICKS(100), //定时器定时时间
        pdTRUE,             //pdTRUE = 周期性 , pdFALSE = 一次性
        0,                  //定时器ID
        TIMAlarmLEDEntry);  //定时器回调函数

    xTimerStart(TimeAlarmLED, 0);
    xSemaphoreGive(sem_WIFI);
    delay(1000);
}

/*======================================= 线程 定时器 入口 ====================================*/
/*
 * @brief:WIFI线程入口
 * @param:none
 * @retval:none
*/
void ThreadWiFiEntry(void *pvParameters)
{
  /*无限等待WIFI信号量*/
  xSemaphoreTake(sem_WIFI, portMAX_DELAY);
  /*删除主线程*/
  //vTaskDelete(ThreadMain);
  Serial.println("===WiFi Connection start===");
  wifi_connect(); //wifi连接
  while (1)
  {
    /* code */
  if (!WiFi.isConnected())
    {
      /*检验MQTT和WIFI是否断连,断连芯片重启重新配网*/
      Serial.println("WIFI connection failed.....");
      ESP.restart();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/*
 * @brief:MQTT线程入口
 * @param:none
 * @retval:none
*/
void ThreadMqttEntry(void *pvParameters)
{
  /*无限等待MQTT信号量*/
  xSemaphoreTake(sem_MQTT, portMAX_DELAY);
  /*挂起WIFI线程*/
  Serial.println("===MQTT Connection start===");

  mqtt_connect(); //mqtt连接

  while (1)
  {
    if (!mqttClient.connected())
    {
      /*检验MQTT和WIFI是否断连,断连芯片重启重新配网*/
      Serial.println("MQTT connection failed");
      ESP.restart();
    }
    mqttClient.loop(); //MQTT客户端保活
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/*
 * @brief:Sensor线程入口
 * @param:none
 * @retval:none
*/
void ThreadSensorEntry(void *pvParameters)
{
  /*等待获取传感器信号量*/
  xSemaphoreTake(sem_Sensor, portMAX_DELAY);

  while (1)
  {

    sample_error = SimpleDHTErrSuccess;
    // check if ESP32 successfuly received data from DHT11 , if not received yet ,delay 1s
    if ((sample_error = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
      Serial.print("Read DHT11 failed, err="); 
      Serial.println(sample_error); 
      delay(1000);
    }else{
    Serial.print("DHT11 Sample OK: ");
    // package temperature and humidity data into plaintext
    sprintf(plainText, "%d%d", (int)temperature,(int)humidity);
    Serial.println(plainText);                                          //串口查看
    // encryption
    encrypt(plainText, key, &encrypted,&padding_plainText_length_global);                
  
    Serial.println("\nCiphered Hex :");
    for (int i = 0; i < padding_plainText_length_global; i++) {
  
      Serial.printf("%02x",encrypted[i]);
      Serial.printf(" "); 
    }
    Serial.println("");
      
    //authentication
    unsigned char hmacResult[HMAC_result_length]; 
    HMAC((char*)encrypted, authkey, hmacResult, HMAC_result_length, false );

    //integrity
    unsigned char hashResult[HMAC_result_length];
    hash((char*)encrypted, hashResult, HMAC_result_length, false);

    //now, these three bytestrings should be concatenated and sent through the internet
    int totallength = padding_plainText_length_global + HMAC_result_length + HMAC_result_length + 1;
    unsigned char total_byte_stream[totallength];
    for (int i = 0; i < padding_plainText_length_global; i++){
      total_byte_stream[i] = encrypted[i];
    }
    free(encrypted);  

    for (int i = padding_plainText_length_global; i < padding_plainText_length_global + HMAC_result_length; i++){
      total_byte_stream[i] = hmacResult[i-padding_plainText_length_global];
    }
    for (int i = padding_plainText_length_global + HMAC_result_length; i < totallength-1; i++){
      total_byte_stream[i] = hashResult[i- padding_plainText_length_global - HMAC_result_length];
    }

      char _hex_sub[3];
      sprintf(_hex_sub, "%02x", padding_plainText_length_global);
      char _hex = (char)strtol(_hex_sub, NULL, 16);  // convert byte to Hex   example :  26  => 1A  

      total_byte_stream[totallength-1] =(unsigned char) _hex;


      Serial.println("\n\nTotal sent  Hex:");

      for (int i = 0; i < totallength; i++) {
        Serial.printf("%02x",total_byte_stream[i]);
        Serial.printf(" "); 
      }

    mqttClient.publish("dataESP1", (char *)total_byte_stream,(unsigned int)totallength);
    vTaskDelay(pdMS_TO_TICKS(3000));      
    }  

  }
}

/*
 * @brief:空闲线程入口
 * @param:none
 * @retval:none
*/
void TIMAlarmLEDEntry(TimerHandle_t xTimer)
{
  digitalWrite(LED, !digitalRead(LED));
}
/*======================================== Encryption & Decreption Function =========================================*/

void PKCS5_padding(char* input,char** output, uint8_t* len) {

  uint8_t input_length = strlen(input);

    if(input_length%BUFFER_SIZE!=0){
        uint8_t padding_length = BUFFER_SIZE - (input_length%BUFFER_SIZE);
        *len = input_length + padding_length;       
    }else{
        *len = input_length;
    }


    *output = (char*)calloc(*len, sizeof(char*)); // using calloc() means we allocate some memory in the heap so that the encrypted data is stored there
    // https://www.youtube.com/watch?v=xDVC3wKjS64   explaination of Dynamic memory allocation in C - malloc calloc realloc free

    if(input_length%BUFFER_SIZE != 0){
    
        char _hex_array[*len];
        for(uint8_t i=0;i<input_length;++i){
            _hex_array[i] = input[i];
        }
        char _hex_sub[3];
        sprintf(_hex_sub, "%02x", *len-input_length);
        char _hex = (char)strtol(_hex_sub, NULL, 16);  // convert byte to Hex   example :  26  => 1A  
        for(uint8_t i=input_length;i<*len;++i){
            _hex_array[i] = _hex;
        }


            uint8_t _iterations = (uint8_t)(*len)/BUFFER_SIZE;
            uint8_t _current_position = 0;
            for(uint8_t i=0;i<_iterations;++i){
                memcpy(*output+_current_position, _hex_array+_current_position, sizeof(char)*BUFFER_SIZE);
                _current_position+=16;
            }


    }else{

        char _hex_array[input_length];
        for(uint8_t i=0;i<input_length;++i){
            _hex_array[i] = input[i];
        }  
            uint8_t _iterations = (uint8_t)(*len)/BUFFER_SIZE;
            uint8_t _current_position = 0;
            for(uint8_t i=0;i<_iterations;++i){
                memcpy(*output+_current_position, _hex_array+_current_position, sizeof(char)*BUFFER_SIZE);
                _current_position+=16;
            }         
    }

}

// PKCS5 padding Tutorial :  https://www.di-mgt.com.au/cryptopad.html  

// what above piece of code does is create a temporary array and make it the size we need which is a multiple of 16.
// then we fill the first part by the data that need encrypting. the rest of the array is filled by the hex values.
// the hex values are determined by something like this.
// assume that the data we have is 10 bytes. so we need 6 bytes to make it 16bytes.
// so the first part of the temporary array is filled by the data we need  encrypting. the remaingin 6bytes are filled by,

// 0x06
// so if the data size was 3bytes then we need 13bytes as padding so we fill the rest of the array by

// 0x0D


void PKCS5_unpadding(unsigned char* input,unsigned char** output, uint8_t* padding_len) {

  uint8_t input_length = strlen((char *)input);


    *output = (unsigned char*)calloc(*padding_len, sizeof(unsigned char*)); // using calloc() means we allocate some memory in the heap so that the encrypted data is stored there
    // https://www.youtube.com/watch?v=xDVC3wKjS64   explaination of Dynamic memory allocation in C - malloc calloc realloc free

    if(*padding_len < 16){
    
        char _hex_array[input_length-*padding_len];
        for(uint8_t i=0;i<input_length-*padding_len;++i){
            _hex_array[i] = input[i];
        }

        memcpy(*output, _hex_array, sizeof(char)*(input_length-*padding_len));

    }else{
        char _hex_array[input_length];
        memcpy(*output, _hex_array, sizeof(char)*(input_length));
    }

}

void encrypt(char * plainText, char * key,  unsigned char** encrypted, uint8_t* padding_plainText_length_global){
 
  mbedtls_aes_context context;
 
  mbedtls_aes_init( &context );
  mbedtls_aes_setkey_enc( &context, (const unsigned char*) key, strlen(key) * 8 );

// padding  
  uint8_t padding_plainText_length;
  char *padding_plainText;
  PKCS5_padding(plainText, &padding_plainText, &padding_plainText_length);

  // Serial.println(padding_plainText_length);
  *padding_plainText_length_global=padding_plainText_length;

  *encrypted = (unsigned char*)calloc(padding_plainText_length, sizeof(unsigned char*));

//encryption 

  uint8_t _iterations = (uint8_t)(padding_plainText_length)/BUFFER_SIZE;
  char* _buffer = (char*)calloc(BUFFER_SIZE, sizeof(char*));
  unsigned char* _encrypted_buffer = (unsigned char*)calloc(BUFFER_SIZE, sizeof(unsigned char*));
  uint8_t _current_position = 0;

  for(uint8_t i=0; i<_iterations; ++i){
      memcpy(_buffer,padding_plainText+_current_position,sizeof(char)*BUFFER_SIZE);
      mbedtls_aes_crypt_ecb(&context, MBEDTLS_AES_ENCRYPT, (const unsigned char*)_buffer, _encrypted_buffer);
      memcpy(*encrypted+_current_position, _encrypted_buffer,sizeof(unsigned char)*BUFFER_SIZE);
      _current_position+=16;
  }
  free(padding_plainText);
  free(_encrypted_buffer);
  free(_buffer);

  mbedtls_aes_free( &context );
}

//most of the butter is from https://techtutorialsx.com/2018/01/25/esp32-arduino-applying-the-hmac-sha-256-mechanism/
void HMAC(char * plainText, char * key,  unsigned char * hmacresult, const int len, bool debug){
  mbedtls_md_context_t context; //struct for all the TLS functions we will call (HMAC from the TLS library)
  mbedtls_md_type_t SHA_type = MBEDTLS_MD_SHA256; //we shouldn't go lower than 256 as these codes are no longer that safe. SHA512, however, is military-grade thus overkill
  
  mbedtls_md_init(&context); // initialize context
  mbedtls_md_setup(&context, mbedtls_md_info_from_type(SHA_type),1); //setup. The 1 indicated that it will use HMAC (thus, a key)
  mbedtls_md_hmac_starts(&context, (const unsigned char *) key, strlen(key));
  mbedtls_md_hmac_update(&context, (const unsigned char *) plainText, strlen(plainText));
  mbedtls_md_hmac_finish(&context, hmacresult);
  mbedtls_md_free(&context); //Free and clear: https://os.mbed.com/teams/Arcola/code/mbedtls/docs/tip/md_8h.html

  if(debug){
    Serial.print("AUTHHash: ");
    for(int i= 0; i< len; i++){
      Serial.printf("%02x",hmacresult[i]);
      Serial.printf(" "); 
    }
    Serial.println("");
  }

}

//most of the butter is from https://techtutorialsx.com/2018/01/25/esp32-arduino-applying-the-hmac-sha-256-mechanism/
void hash(char * plainText,  unsigned char * hashresult, const int len, bool debug){
  mbedtls_md_context_t context; //struct for all the TLS functions we will call (HMAC from the TLS library)
  mbedtls_md_type_t SHA_type = MBEDTLS_MD_SHA256; //we shouldn't go lower than 256 as these codes are no longer that safe. SHA512, however, is military-grade thus overkill
  
  mbedtls_md_init(&context); // initialize context
  mbedtls_md_setup(&context, mbedtls_md_info_from_type(SHA_type),0); //setup. The 1 indicated that it will NOT use HMAC (thus, NO key)
  mbedtls_md_starts(&context);
  mbedtls_md_update(&context, (const unsigned char *) plainText, strlen(plainText));
  mbedtls_md_finish(&context, hashresult);
  mbedtls_md_free(&context); //Free and clear: https://os.mbed.com/teams/Arcola/code/mbedtls/docs/tip/md_8h.html

  if(debug){
    Serial.print("INTEHash: ");
    for(int i= 0; i< len; i++){
      Serial.printf("%02x",hashresult[i]);
      Serial.printf(" "); 
    }
    Serial.println("");
  }

}

void decrypt(unsigned char * encrypted, char * key, unsigned char** decrypted, uint8_t* padding_plainText_length_global){
 
  mbedtls_aes_context context;
 
  mbedtls_aes_init( &context );
  mbedtls_aes_setkey_dec( &context, (const unsigned char*) key, strlen(key) * 8 );

  uint8_t padding_plainText_length=*padding_plainText_length_global;

  *decrypted = (unsigned char*)calloc(padding_plainText_length, sizeof(unsigned char*));
  
  uint8_t _iterations = (uint8_t)(padding_plainText_length)/BUFFER_SIZE;
  unsigned char* _buffer = (unsigned char*)calloc(BUFFER_SIZE, sizeof(unsigned char*));
  unsigned char* _decrypted_buffer = (unsigned char*)calloc(BUFFER_SIZE, sizeof(unsigned char*));
  uint8_t _current_position = 0;

  for(uint8_t i=0; i<_iterations; ++i){
      memcpy(_buffer,encrypted + _current_position,sizeof(char)*BUFFER_SIZE);
      mbedtls_aes_crypt_ecb(&context, MBEDTLS_AES_DECRYPT, (const unsigned char*)_buffer, _decrypted_buffer);
      memcpy(*decrypted+_current_position, _decrypted_buffer,sizeof(unsigned char)*BUFFER_SIZE);
      _current_position+=16;
  }

  free(_decrypted_buffer);
  free(_buffer);
  mbedtls_aes_free( &context );
}
//   the explanation of encryption and decryption code check :  https://techtutorialsx.com/2018/05/10/esp32-arduino-decrypt-aes-128-in-ecb-mode/
/*======================================== WIFI and MQTT Function =========================================*/

/*
 * @brief:连接WiFi 
 * @param:none
 * @retval:none
*/
void wifi_connect()
{
  /*WIFI连接*/
  WiFi.begin(ssid, password);
  //判断连接状态
  int count = 0;
  while (WiFi.status() != WL_CONNECTED)
  // while(!Blinker.connected())
  {
    //vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("WIFI is connecting ...");
    if (count >= 5)
    {
      /*超时5s 结束程序*/
      break;
    }
    delay(1000);
    count++;
  }

  if (WiFi.isConnected())
  {
    Serial.println("WIFI connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP().toString());
    delay(1500);
    /*发送信号量MQTT,触发MQTT线程*/
    xSemaphoreGive(sem_MQTT);
  }
  // else
  // {
  //   Serial.println("WIFI connection failed.....");
  // }
}


/*
 * @brief:MQTT连接
 * @param:none
 * @retval:none
*/
void mqtt_connect()
{
  /*设置mqtt服务器地址和端口*/
  mqttClient.setServer(mqtt_server, 1883);
  /*设置mqtt保持活跃时间 ms*/
  mqttClient.setKeepAlive(60);
  /*设置消息回调函数*/
  mqttClient.setCallback(callback);
  /**
   * 连接服务器 
   * 
   * mqttClient.connect(ClientID,Name,Password)
   * 
   * ClientID : 接入服务器的MQTT客户端ID
   * Name ：接入服务器的用户
   * Password ：接入服务器的密码
   * 
   */

  int count = 0;
  while (!mqttClient.connect(clientID))
  {
    Serial.println("MQTT is connecting ...");
    if (count >= 5)
    {
      break;
    }
    count++;
    delay(1000);
  }
  if (mqttClient.connected())
  {
    /*连接成功 关闭AlarmLED定时器*/
    xTimerStop(TimeAlarmLED, 0);
    /*LED常亮示意正常工作*/
    digitalWrite(LED, HIGH);
    Serial.println("MQTT is connected");

    /**
    * 订阅主题
    * client.subscribe(topic,qos)
    * topic ： 订阅的主题
    * qos : 服务质量,Broker 与 Client 之间消息通信关系. 0 or 1 or 2
    * 最多一次（0）只发不管收没收到
    * 最少一次（1）至少接受到一次
    * 只一次  （2）确保只接受一次,慢,安全最高  
    */

    mqttClient.subscribe("dataESP1", 0);
    // mqttClient.subscribe("/LED_OFF", 0);
    // mqttClient.subscribe("/test", 0);
    /**
    * 发出消息
    * 
    * client.publish(topic,payload,retained)
    * 
    * topic ： 发布的主题
    * payload ： 消息体
    * retained ：保留消息 true or false
    */
    // mqttClient.publish("/test", "... this is a test of mqtt ...", false);
    /* WIFI,MQTT 均连上之后 发送信号量 */
    xSemaphoreGive(sem_Sensor); //触发Sensor 传感器线程
  }
}

/*
 * @brief:MQTT回调函数,用于获取订阅的消息
 * @param:topics 订阅的主题
 * @param:payload 订阅的主题的消息 注：byte 只能一个字节一个字节读
 * @param:length 消息大小
 * @retval:none
*/
void callback(char* topic, byte* message, unsigned int length)
{
  Serial.printf(" "); 
  Serial.println("Message arrived on topic: ");
  Serial.println(topic);

      for (uint8_t i = 0; i < length ; i++) {
        Received[i]= ( char)message[i];
      }; 

    uint8_t encrypt_len = (uint8_t)Received[length-1];
      for (uint8_t i = 0; i < encrypt_len ; i++) {
        Receive_encrypted[i]= (char)Received[i];
      }; 
    
      for (uint8_t i = encrypt_len; i < encrypt_len + 32; i++) {
        Receive_authen[i-encrypt_len]= (unsigned char)Received[i];
      };             
      for (uint8_t i = encrypt_len + 32; i < encrypt_len + 64; i++) {
        Receive_integrity[i-encrypt_len - 32]= (unsigned char)Received[i];
      }; 

      Serial.print("\n\ Received data length :");Serial.print(strlen(Received));
      Serial.print("\n\ Received Encryption data length :");Serial.print(strlen(Receive_encrypted));
      Serial.print("\n\ Received Authentication data length :");Serial.print(strlen(Receive_authen));
      Serial.print("\n\ Received integrity data length :");Serial.print(strlen(Receive_integrity));

           //authentication calc
              unsigned char hmacResult_check[32];
              HMAC(Receive_encrypted, authkey, hmacResult_check, HMAC_result_length, false );
            //   Serial.println("\n authentication check: ");
            //   for(int i= 0; i< 32; i++){
            //     Serial.printf("%02x",hmacResult_check[i]);
            //     Serial.printf(" "); 
            //   }      
            //  Serial.println("\n Received authen : ");
            //   for(int i= 0; i< 32; i++){
            //     Serial.printf("%02x",Receive_authen[i]);
            //     Serial.printf(" "); 
            //   }             

            //integrity   calc 
              unsigned char hashResult_check[32];
              hash((char*)Receive_encrypted, hashResult_check, HMAC_result_length, false);
              // Serial.println("\n integrity check: ");
              // for(int i= 0; i< 32; i++){
              //   Serial.printf("%02x",hashResult_check[i]);
              //   Serial.printf(" "); 
              // }  

            // compare received data with calculation data  
              int check_authentication;
              int check_integrity;
            check_authentication = memcmp(Receive_authen, hmacResult_check, 32);
            check_integrity = memcmp(Receive_integrity, hashResult_check, 32);

          if (check_authentication == 0){

            Serial.println("\n\n Authentication check success");

                      if (check_integrity == 0){
                      Serial.println("\n\n integrity check success");
                        //decryption
                        Serial.println("\n\nDeciphered Hex:");
                        decrypt((unsigned char*)Receive_encrypted, key, &decrypted,&encrypt_len);
                          for (int i = 0; i < encrypt_len; i++) {
                            Serial.printf("%02x",decrypted[i]);
                            Serial.printf(" "); 
                          }
                        // unpadding 
                           uint8_t padding_len = (uint8_t)decrypted[encrypt_len-1];

                          PKCS5_unpadding( decrypted ,&unpadding_decrypted, &padding_len);

                          Serial.println("\n\n unpadding_decrypted HEX :");

                          
                          for (int i = 0; i < encrypt_len-padding_len; i++) {    
       
                            Serial.printf("%x",unpadding_decrypted[i]);
                            Serial.printf(" "); 
                          }

                          Serial.println("\n\n unpadding_decrypted TEXT :");


                          Received_temperature = (atoi((char*)unpadding_decrypted))/100;
                          Received_humidity   = (atoi((char*)unpadding_decrypted))%100 ;
 
                          Serial.println(Received_temperature);  // temperature and  humidity 
                          Serial.println(Received_humidity);  

                          free(decrypted);
                          free(unpadding_decrypted);       

                      }else{
                      Serial.println("\n\n integrity check failed");
                      }

          }else{
              Serial.println("\n\n Authentication check failed");
          }

}






