#include <Arduino.h>
#include <avr/wdt.h>
#include <Ethernet.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>


//Таймайт
#define TIMEOUT_SHUT 10000
//Максимальная длина команды
#define COM_LEN 32
//Таумаут на прием команды
#define TIMEOUT_COM 15000

#define FAILS_TIMES 3

#define I2C_ADDRESS 0b1000

#define TIMEOUT_CURTAIN 8000
/*
Команды

init_с

Контроллер производит проверку работоспособности компонентов кассеты (датчиков, двигателей) без включения приводов.

test_с

Контроллер производит проверку работоспособности компонентов кассеты(датчиков, двигателей) включая привод кассеты, кассета открывается и закрывается.

open_с

Контролер открывает шторку. В случае неудачи после T секунд и X попыток присылает код ошибки.

close_с

Контролер закрывает шторку кассеты. В случае неудачи после T секунд и X попыток присылает код ошибки.

status_c

Контроллер опрашивает основные датчики и выдает код ошибки выполнения команды и статус состояния датчиков.




Параметры:

N, где N –номер кассеты. (пример: “init_c:2\n”)

Возвращаемое значение:

Результат выполнения инициализации: “A:X\n”, A,X - десятичные числа в ASCII представлении.
A-код ответа;
X-UID (Уникальный идентификационный номер кассеты) – 32 бита, старший байт – тип кассеты, остальные уникальный номер.



init_с_all

Контроллер производит проверку работоспособности компонентов всех кассет (датчиков, двигателей) без включения приводов.

Параметры:

нет

Возвращаемое значение:

Результат выполнения инициализации: “A:N:An:Xn:A(n+1):X(n+1):A(n+2):X(n+2)…\n”, A,N,An,Xn - десятичные числа в ASCII представлении.
A-код ответа от кластера кассет;
N-Количество кассет в кластере;
An-код ответа от каждой кассеты;
Xn-UID (Уникальный идентификационный номер кассеты) – 32 бита, старший байт – тип кассеты, остальные уникальный номер.


test_c_all

Контроллер производит проверку работоспособности компонентов всех кассет (датчиков, двигателей) включая привод кассеты, кассета открывается и закрывается.

open_c_all

Контролер открывает шторки всех кассет в кластере. В случае неудачи после T секунд и X попыток присылает код ошибки.

close_с_all

Контролер закрывает шторки всех кассет кластера. В случае неудачи после T секунд и X попыток присылает код ошибки.

status_c_all

Контроллер опрашивает основные датчики каждой кассеты в кластере и выдает код ошибки выполнения команды и статус состояния датчиков каждой кассеты.

Возвращаемое значение:
Результат выполнения команды: “A:N:An:Bn:A(n+1):B(n+1)\n”, A,N,An:Bn - десятичные числа в ASCII представлении.
A-код ответа от кластера кассет;
N-количество кассет в кластере;
An-код ответа для каждой кассеты;
и т.п.
Bn- байт состояния датчиков в десятичном счислении в ASCII представлении.
После перевода в двоичную систему счисления:
Самый младший бит – датчик открытия,
Второй справа бит датчик закрытия,
Третий справа бит – дополнительный датчик №1,

Если бит равен 1, датчик в сработавшем режиме, если 0, то нет.

mac_c

Служебная команда для получения mac адреса контроллера кластера кассет

port_c

Служебная команда для изменения порта

reboot_c

Служебная команда для перезапуска контроллера

is_с_closed, is_с_opened, is_с_success

Опрос контроллера, находится ли кассета в указанном состоянии.
is_c_closed – закрыта ли кассета?
is_c_opened – открыта ли кассета?
is_c_success – готова ли кассета к работе?


*/



const byte nack=0x15;
//Список команд
const char commands[16][16]={"init_c","init_c_all","test_c","test_c_all","open_c","open_c_all","close_c","close_c_all","status_c","status_c_all","mac_c","port_c","reboot_c","is_c_closed","is_c_opened","is_c_sucsess"};
//char gen_com[10][16];
//Массив строк остальных аргументов
//Максимум 10 аргументов
char second_arg[16][16];

//Глобальая переменная для мак-адресса, считываемого из EEPROM
byte mac[6] = {};
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
//};
//assign an IP address for the controller:

//Глобальая переменная для IP адресса, считываемого из EEPROM, для подключения в случае проблем с DHCP
IPAddress ip(192, 168, 1, 130);
//Глобальая переменная для шлюза, считываемого из EEPROM, для подключения в случае проблем с DHCP
IPAddress gateway(192, 168, 1, 1);
//Глобальая переменная для маски подсети, считываемого из EEPROM, для подключения в случае проблем с DHCP
IPAddress subnet(255, 255, 255, 0);

//Временный символ
char temp_char;

//Глобальая переменная для порта, считываемого из EEPROM
word port_number=30002;

//Инициализируем сервер на заданном порту
EthernetServer server = EthernetServer(port_number);

//const int in_put=8;
//const int out_put=7;

//Время последнего чтения
long lastReadingTime = 0;
//int toggle_switch=0;
//Буфер для принятой команды
//Не более 32 символов
char com_recv[COM_LEN]={};
//Номер байта принятой команды
uint8_t char_number=0;

//Переменная для хранения числа кассет в кластере, считывается из EEPROM
uint8_t number_of_cassete=0;

uint32_t answer_cassete[16]={};

uint8_t global_answer=255;



//Функция чтения конфигурации сети адреса из EEPROM
IPAddress read_network_config_eeprom(byte con){
  byte con_num[4];
  for (int i=0;i<4;i++){
    con_num[i]=EEPROM.read(i+con*4);
  }
//Функция чтения всей конфигурации сети адреса из EEPROM
IPAddress net(con_num[0],con_num[1],con_num[2],con_num[3]);
return net;
}

//Функция програмного перезапуска
void software_Reboot()
{
  wdt_enable(WDTO_15MS);
  while(1)
  {
  }
}

//Функция чтения мак адреса из EEPROM
void read_mac(){
  for (int i=0;i<6;i++){
      mac[i]=EEPROM.read(i+12); //3*4
  }
}
//Функция чтения порта адреса из EEPROM
void read_port(){
    port_number=word(EEPROM.read(6+12),EEPROM.read(6+13));
}

//Функция записи порта адреса в EEPROM
void write_port(word port_num){
  EEPROM.write((6+12),highByte(port_num));
  EEPROM.write((6+13),lowByte(port_num));
}

void read_number_of_cassete(){
  number_of_cassete=EEPROM.read(6+14);
  }


//Функция посылки команды от мастера слэйву по I2C (адрес слэйва, команда)
void send_command(int i2c_address, uint8_t i2c_command){
    Wire.beginTransmission(i2c_address);
    Wire.write(i2c_command);
    Wire.endTransmission();
    delay(100);
}

//Функция приема команды от слэйва (адрес слэйва, команда которую посылали)
uint32_t recieve_answer(int i2c_address, uint8_t i2c_command){

//Переменная, которую вернем. 4 байта
    uint32_t answer=0;
//Читаем от слэйва 5 байт. 4 байта сам результат, пятый для проверки выполнения и т.п.
    Wire.requestFrom(i2c_address, 5);
    //байт для приема байта от слэйва
    uint8_t c=0;
    //счетчик принятых байт
    uint8_t b=0;
    //WatchDog
    wdt_enable(WDTO_8S);
    //Пока есть данные считываем
    while (Wire.available()){
    c = Wire.read();
    //Проверяем 5-й байт ответа

    //Если команда принятна,и обработана, возвращаем полученый ответ
     if ((b==4)&&(c==(i2c_command-32))){
        return answer;
    }
    //Если команда принятна,но не обработана, возвращаем единицы
     if ((b==4)&&(c==i2c_command)){
        return 0xFFFFFFFF;
    //Если пятый бит ничему не равен, возвращаем нули
    if ((b==4)&&(c!=(i2c_command-32))){
        return 0x00000000;
    }

    }
// Отладочная отправка
//    Serial.println(c);
//Формируем ответ
    answer=answer<<8;
    answer=answer+c;
    b++;
    }
    wdt_disable();
    return answer;
}



//Функция чтения состояния датчиков
int read_sensors(){
  byte sensors=0;


  return sensors;
}

void listenForEthernetClients(void);
int parscommand(void);
int do_command(int);
void comm_reboot_c(void);
void comm_ip_c(void);
void comm_mac_c(void);
void comm_init_c(void);
void comm_init_c_all(void);
void comm_open_c(void);
void comm_open_c_all(void);
void comm_close_c(void);
void comm_close_c_all(void);
void comm_status_c(void);
void comm_status_c_all(void);
void comm_test_c(void);
void comm_test_c_all(void);
void comm_port_c(void);

void comm_is_c_closed(void);
void comm_is_c_opened(void);
void comm_is_c_sucsess(void);



void setup() {
//Инициализация портов ввода/вывода

//Подключение к шине I2C в режиме мастера
Wire.begin();


// Читаем конфигурацию сети из EEPROM
 ip=read_network_config_eeprom(0);
 gateway=read_network_config_eeprom(1);
 subnet=read_network_config_eeprom(2);
 read_mac();
 read_port();
 read_number_of_cassete();

//Инициализируем TCP/IP сервер на порту считанном из EEPROM
server = EthernetServer(port_number);

//Инициализируем UART для отладочного вывода
  Serial.begin(9600);
//Пытаемся подключиться к сети по DHCP
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
//Если не получилось по DHCP
//Подключаемся к сети с данными считанными из EEPROM
    Ethernet.begin(mac, ip, gateway, subnet);
  }
  // give the Ethernet shield a second to initialize:
  delay(1000);
  Serial.println("connecting...");

//Запуск сервера
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  // initalize the  data ready and chip select pins:
  // give the sensor and Ethernet shield time to set up:
  delay(1000);
}

//Основной цикл
void loop() {
  // check for a reading no more than once a second.
  //Раз в секунду производим некие действия, если нет подключенных клиентов
  if ((millis() - lastReadingTime) > 1000) {

//Печать в ком-порт отладочной инфы
    Serial.println("We are alive ");

  Serial.println(read_sensors());

//    Serial.println(digitalRead(in_put));
//    if (digitalRead(in_put) != toggle_switch) {
//      toggle_switch=digitalRead(in_put);
      // timestamp the last time you got a reading:

//    }
    lastReadingTime = millis();
  }


  // listen for incoming Ethernet connections:
//Вызов функция обработки входящего соединения
  listenForEthernetClients();
}

//Функция обработки входящего соединения
void listenForEthernetClients(){
//Переменная отметки времени
  long time_stamper=0;
//Слушаем клиентов
  EthernetClient client = server.available();
//Если есть подключенный клиент ждем от него данные
  if (client) {
    Serial.println("Got a client");
//Пока есть соединение с клиентом
    while (client.connected()) {
//Если от клиента есть данные, обрабатываем их посимвольно, пока они не закончатся
      if (client.available()) {
//Отмечаем время для обрыва соединения по таймауту приема конца команды
        time_stamper=millis();
//Читаем символ сообщения
        char c = client.read();
//Если это символ конца строки обрабатываем команду
        if (c == '\n') {
//Отладочная инфа в ком порт
          Serial.println("Starting sending");
          Serial.println(com_recv);
//        server.println(com_recv);
//        delay(3000);
//Выполняем отпарсеную команду
          do_command(parscommand());
//Очищаем буфер, наверное глупым способом
          for(int i=0;i<=char_number;i++){
          com_recv[i]=0;
          }
          char_number=0;
          break;
        }
//Если символ не \r, добавляем в буфер
        else if (c != '\r') {
          // you've gotten a character on the current line
//          currentLineIsBlank = false;
          com_recv[char_number]=c;
          char_number++;
//Если количество символов превысило допустимое, то посылаем NACK и разрываем соединение
          if (char_number>COM_LEN){
            server.write(nack);
            server.print('\n');
            client.stop();
            break;
          }
//          Serial.println(com_recv);
        }
        if ((millis()-time_stamper)>TIMEOUT_COM){
        server.write(nack);
        server.print('\n');
        client.stop();
        break;
      }
      }
//Если время приема символов превысило таймаут посылаем NACK и разрываем соединение

    }
  }
}

//Парсинг команды
//Скорее всего требует доработки

int parscommand(){
  for (int j=0;j<16;j++){
  for (int i=0;i<16;i++){
  second_arg[j][i]=0;
  }
  }
  for (int j=0;j<16;j++){
//  for (int i=0;i<=(char_number-1);i++){
  for (int i=0;i<=(char_number);i++){
    if ((commands[j][i]!=com_recv[i])&&(com_recv[i]!=':')){
      Serial.print(commands[j][i]);
      Serial.print(' ');
      Serial.println(com_recv[i]);
    break;
    }
//    if (i==(char_number-1)){
    if (i==(char_number)){
      Serial.println("OK");
      return j;
      }
    if (com_recv[i]==':'){
//      for (int f=i+1;f<=(char_number-1);f++){
        int k=0;
        int h=1;
        for (int f=i+1;f<=(char_number);f++){
        if (com_recv[f]==':'){
          k++;
          h++;
        }
        second_arg[k][f-(i+h)]=com_recv[f];
      }
      return j;
    }
  }
  }
  return 0xFF;
}

//{"init_c","init_c_all","test_c","test_c_all","open_c","open_c_all","close_c","close_c_all","status_c","status_c_all","mac_c","port_c","reboot_c","is_c_closed","is_c_opened","is_c_sucsess"};

/*
void comm_reboot_c(void);
void comm_ip_c(void);
void comm_mac_c(void);
void comm_init_c(void);
void comm_init_c_all(void);
void comm_open_c(void);
void comm_open_c_all(void);
void comm_close_c(void);
void comm_close_c_all(void);
void comm_status_c(void);
void comm_status_c_all(void);
void comm_test_c(void);
void comm_test_c_all(void);
void comm_port_c(void);

void comm_is_closed_c(void);
void comm_is_opened_c(void);
void comm_is_sucsess_c(void);

*/
//Выполнение отпарсеной команды
//Ответ ACK закомментирован
int do_command(int comm){
switch (comm){
  case 0:
//    server.print(0x6);
//    server.print('\n');
    comm_init_c();
    break;
  case 1:
    comm_init_c_all();
    break;
  case 2:
    void comm_test_c();
    break;
  case 3:
    comm_test_c_all();
    break;
  case 4:
    comm_open_c();
    break;
  case 5:
    comm_open_c_all();
    break;
  case 6:
    comm_close_c();
    break;
  case 7:
    comm_close_c_all();
    break;
  case 8:
    comm_status_c();
    break;
  case 9:
      comm_status_c_all();
    break;
  case 10:
      comm_mac_c();
    break;
  case 11:
      comm_port_c();
    break;
  case 12:
      comm_reboot_c();
    break;
  case 13:
      comm_is_c_closed();
    break;
  case 14:
      comm_is_c_opened();
    break;
  case 15:
      comm_is_c_sucsess();
    break;

//Если команда не распознана посылаем NACK
  default:
    server.write(nack);
    server.print('\n');
    Serial.println("Unknown command");
}
}

//Обработка команды reboot
void comm_reboot_c(){
  server.print("0\n");
  delay(500);
  software_Reboot();
}

//Обработка команды mac
void comm_mac_c(){
//    server.print("MAC ");
    server.print(0);
    server.print(':');
    for (int i=0;i<6;i++){
    server.print(mac[i],DEC);
    }
    server.print('\n');
    Serial.println("mac");
}

//Функция обработки второго операнда - адреса слота
uint8_t get_slot_number(){
    //Цель получить адрес слота, это переменная для его возврата
    uint8_t i2c_adr=0;

    int i=0;
    //Обрабатываем массив остальных аргументов, пока значения лежат в зоне символов цифр
    while((second_arg[0][i]>=0x30)&&(second_arg[0][i]<=0x39)){
            //Прибавляем десяток, т.к. у нас изначально 0 на первый символ это не действует
  i2c_adr*=10;
  //Прибавляем символ-смещение
  i2c_adr+=byte(second_arg[0][i])-0x30;
  i++;
  }
  //Возвращаем номер слота, но без прибавки адреса  I2C
  return i2c_adr;
}

// команда init, основная цель вернуть номер кассеты, вставленной в слот, что во-первых позволяет обнаружить присутствие кассеты в слоте
// во-вторых понять, что за кассета установлена в слот
void comm_init_c(){
    //Переменная ответа
    uint32_t answer_long=0;
    //Переменная адреса I2C контроллеа кассеты
    uint8_t i2c_adr=0;
    //Полусаем номер слота
    i2c_adr=get_slot_number();
    //Посылаем команду на номер слота+смещение для валидного I2C адреса
  send_command(i2c_adr+I2C_ADDRESS,'i');
  //команда должна пройти быстро, поэтому ждем немного
  delay(100);
  //вызываем функцию получения ответа от кассеты с проверкой
  answer_long=recieve_answer(i2c_adr+I2C_ADDRESS,'i');
  //если ответ лежит в допустимом диапазоне, возвращаем 0 и через : сам ответ, который представляет UID
  if ((answer_long!=0)&&(answer_long!=0xFFFFFFFF)){
    server.print(0);
    server.print(':');
    server.print(answer_long);
    server.print('\n');
  }
  //если ответ 0, то возвращаем код ошибки 1 (потом надо подробнее посмотреть)
  if (answer_long==0){
    server.print(1);
    server.print(':');
    server.print(answer_long);
    server.print('\n');
  }

  //если ответ FFFFFFFF, то возвращаем код ошибки 1 (потом надо подробнее посмотреть)
  if (answer_long==0xFFFFFFFF){
    server.print(2);
    server.print(':');
    server.print(answer_long);
    server.print('\n');
  }
}

// команда init_c_all, основная цель вернуть количество кассет в кластере и номер каждой кассеты, вставленной в слот, что во-первых позволяет обнаружить присутствие кассеты в слоте
// во-вторых понять, что за кассета установлена в слот
void comm_init_c_all(){
    global_answer=0;
    for (int i=0;i<number_of_cassete;i++){
        send_command(i+I2C_ADDRESS,'i');
        delay(100);
    }
    for (int i=0;i<number_of_cassete;i++){
        answer_cassete[i]=recieve_answer(i+I2C_ADDRESS,'i');
        if ((answer_cassete[i]!=0)&&(answer_cassete[i]!=0xFFFFFFFF)&&(global_answer==0)){
            global_answer=0;
        }
        if (answer_cassete[i]==0){
            global_answer=1;
        }
        if (answer_cassete[i]==0xFFFFFFFF){
            global_answer=1;
        }
        delay(100);
    }
  server.print(global_answer);
  server.print(':');
  server.print(number_of_cassete);

  for (int i=0;i<number_of_cassete;i++){
    if ((answer_cassete[i]!=0)&&(answer_cassete[i]!=0xFFFFFFFF)){
            server.print(':');
            server.print(0);
            server.print(':');
            server.print(answer_cassete[i]);
    }
    if (answer_cassete[i]==0){
            server.print(':');
            server.print(1);
            server.print(':');
            server.print(answer_cassete[i]);
    }
    if (answer_cassete[i]==0xFFFFFFFF){
             server.print(':');
            server.print(2);
            server.print(':');
            server.print(answer_cassete[i]);
        }
  }
  server.print('\n');
}

//Обработка команды open_c
void comm_open_c(){
    uint32_t answer_long=0;
    uint8_t i2c_adr=0;
    i2c_adr=get_slot_number();
    send_command(i2c_adr+I2C_ADDRESS,'o');
    delay(TIMEOUT_CURTAIN);
    answer_long=recieve_answer(i2c_adr+I2C_ADDRESS,'o');
        if ((answer_long==0x01000000)){
            server.print(0);
            server.print('\n');
        }
        if (answer_long==0){
            server.print(1);
            server.print('\n');
        }
        if (answer_long==0xFFFFFFFF){
            server.print(2);
            server.print('\n');
        }
        if (answer_long==0x02000000){
            server.print(10);
            server.print('\n');
        }

}

void comm_open_c_all(){
    uint32_t answer_long=0;
    global_answer=0;
    for (int i=0;i<number_of_cassete;i++){
        send_command(i+I2C_ADDRESS,'o');
        delay(100);
    }
    delay(TIMEOUT_CURTAIN);
    for (int i=0;i<number_of_cassete;i++){
        answer_cassete[i]=recieve_answer(i+I2C_ADDRESS,'o');
        if ((global_answer==0)&&(answer_cassete[i]==0x01000000)){
            global_answer=0;
        }
        if (answer_cassete[i]==0){
            global_answer=1;
        }
        if (answer_cassete[i]==0xFFFFFFFF){
            global_answer=1;
        }
        if (answer_cassete[i]==0x02000000){
            global_answer=10;
        }
        delay(100);
    }
  server.print(global_answer);
  server.print(':');
  server.print(number_of_cassete);

  for (int i=0;i<number_of_cassete;i++){
    if (answer_cassete[i]==0x01000000){
            server.print(':');
            server.print(0);

    }
    if (answer_cassete[i]==0){
            server.print(':');
            server.print(1);

    }
    if (answer_cassete[i]==0xFFFFFFFF){
            server.print(':');
            server.print(2);

        }
    if (answer_cassete[i]==0x02000000){
            server.print(':');
            server.print(10);

        }
  }
  server.print('\n');
}


//Обработка команды close
void comm_close_c(){
    uint32_t answer_long=0;
    uint8_t i2c_adr=0;
    i2c_adr=get_slot_number();
    send_command(i2c_adr+I2C_ADDRESS,'c');
    delay(TIMEOUT_CURTAIN);
    answer_long=recieve_answer(i2c_adr+I2C_ADDRESS,'c');
        if (answer_long==0x01000000){
            server.print(0);
            server.print('\n');
        }
        if (answer_long==0){
            server.print(1);
            server.print('\n');
        }
        if (answer_long==0xFFFFFFFF){
            server.print(2);
            server.print('\n');
        }
        if (answer_long==0x02000000){
            server.print(10);
            server.print('\n');
        }
}

void comm_close_c_all(){
    uint32_t answer_long=0;

    global_answer=0;
    for (int i=0;i<number_of_cassete;i++){
        send_command(i+I2C_ADDRESS,'c');
        delay(100);
    }
    delay(TIMEOUT_CURTAIN);
    for (int i=0;i<number_of_cassete;i++){
    answer_cassete[i]=recieve_answer(i+I2C_ADDRESS,'c');
    if ((global_answer==0)&&(answer_cassete[i]==0x01000000)){
            global_answer=0;
        }
        if (answer_cassete[i]==0){
            global_answer=1;
        }
        if (answer_cassete[i]==0xFFFFFFFF){
            global_answer=1;
        }
        if (answer_cassete[i]==0x02000000){
            global_answer=10;
        }
        delay(100);
    }
  server.print(global_answer);
  server.print(':');
  server.print(number_of_cassete);

  for (int i=0;i<number_of_cassete;i++){
    if (answer_cassete[i]==0x01000000){
            server.print(':');
            server.print(0);

    }
    if (answer_cassete[i]==0){
            server.print(':');
            server.print(1);

    }
    if (answer_cassete[i]==0xFFFFFFFF){
            server.print(':');
            server.print(2);

        }
    if (answer_cassete[i]==0x02000000){
            server.print(':');
            server.print(10);

        }
  }
  server.print('\n');
}

//Обработка команды status
void comm_status_c(){
    uint32_t answer_long=0;
    uint8_t i2c_adr=0;
    uint32_t sensor_status=0xFFFFFFFF;
    uint8_t sensor_state=0xFF;
    i2c_adr=get_slot_number();
    send_command(i2c_adr+I2C_ADDRESS,'s');
    delay(100);
    answer_long=recieve_answer(i2c_adr+I2C_ADDRESS,'s');
        if ((answer_long!=0)&&(answer_long!=0xFFFFFFFF)){
            server.print(0);
            server.print(':');
            sensor_status=answer_long;
            sensor_status>>24;
            sensor_state=sensor_status;
            sensor_state<<1;
            sensor_status=answer_long;
            sensor_status<<8;
            sensor_status>>24;
            sensor_state+=sensor_status;
            server.print(sensor_state);
            server.print('\n');
        }
        if (answer_long==0){
            server.print(1);
            server.print(':');
            server.print(3);
            server.print('\n');
        }
        if (answer_long==0xFFFFFFFF){
            server.print(2);
            server.print(':');
            server.print(3);
            server.print('\n');
        }


}

void comm_status_c_all(){
    uint32_t answer_long=0;
    uint8_t i2c_adr=0;
    uint32_t sensor_status=0xFFFFFFFF;
    uint8_t sensor_state=0xFF;
    global_answer=0;
    for (int i=0;i<number_of_cassete;i++){
        send_command(i+I2C_ADDRESS,'s');
        delay(100);
    }
    delay(100);
    for (int i=0;i<number_of_cassete;i++){
        answer_cassete[i]=recieve_answer(i+I2C_ADDRESS,'s');
        if ((answer_cassete[i]!=0)&&(answer_cassete[i]!=0xFFFFFFFF)&&(global_answer==0)){
            global_answer=0;
        }
        if (answer_cassete[i]==0){
            global_answer=1;
        }
        if (answer_cassete[i]==0xFFFFFFFF){
            global_answer=1;
        }
        delay(100);
    }
  server.print(global_answer);
  server.print(':');
  server.print(number_of_cassete);

  for (int i=0;i<number_of_cassete;i++){
    if ((answer_cassete[i]!=0)&&(answer_cassete[i]!=0xFFFFFFFF)){
            server.print(':');
            server.print(0);
            server.print(':');
            sensor_status=answer_cassete[i];
            sensor_status>>24;
            sensor_state=sensor_status;
            sensor_state<<1;
            sensor_status=answer_cassete[i];
            sensor_status<<8;
            sensor_status>>24;
            sensor_state+=sensor_status;
            server.print(sensor_state);
            }
    if (answer_cassete[i]==0){
            server.print(':');
            server.print(1);
            server.print(':');
            server.print(1);
    }
    if (answer_cassete[i]==0xFFFFFFFF){
            server.print(':');
            server.print(2);
            server.print(':');
            server.print(2);
        }


  }
  server.print('\n');
}

//Обработка команды test
//Пока полузаглушка
void comm_test_c(){
  uint32_t answer_long=0;
    uint8_t i2c_adr=0;
    i2c_adr=get_slot_number();
    send_command(i2c_adr+I2C_ADDRESS,'t');
    delay(TIMEOUT_CURTAIN);
    delay(TIMEOUT_CURTAIN);
    answer_long=recieve_answer(i2c_adr+I2C_ADDRESS,'t');
        if ((answer_long!=0)&&(answer_long!=0xFFFFFFFF)){
            server.print(0);
            server.print('\n');
        }
        if (answer_long==0){
            server.print(1);
            server.print('\n');
        }
        if (answer_long==0xFFFFFFFF){
            server.print(2);
            server.print('\n');
        }
}

void comm_test_c_all(){
    server.print(0);
    server.print(':');
    server.print(3);
    server.print('\n');
}

//Обработка команды port
void comm_port_c(){
  int i=0;
  port_number=0;
  while((second_arg[0][i]>=0x30)&&(second_arg[0][i]<=0x39)){
  port_number*=10;
  port_number+=byte(second_arg[0][i])-0x30;
  i++;
  }
  if (port_number!=0){
    write_port(port_number);
    server.print(0,DEC);
    server.print('\n');
  }
  else{
    server.print(1,DEC);
    server.print('\n');
  }
}


void comm_is_c_closed(){
    uint32_t answer_long=0;
    uint8_t i2c_adr=0;
    i2c_adr=get_slot_number();
    send_command(i2c_adr+I2C_ADDRESS,'s');
    delay(100);
    answer_long=recieve_answer(i2c_adr+I2C_ADDRESS,'s');
    if (answer_long==0x01000000){
            server.print(0);
            server.print('\n');
            return;
        }
    if (answer_long==0){
            server.print(1);
            server.print('\n');
            return;
        }
    if (answer_long==0xFFFFFFFF){
            server.print(2);
            server.print('\n');
            return;
        }
    server.print(3);
    server.print('\n');

}

void comm_is_c_opened(){
    uint32_t answer_long=0;
    uint8_t i2c_adr=0;
    i2c_adr=get_slot_number();
    send_command(i2c_adr+I2C_ADDRESS,'s');
    delay(100);
    answer_long=recieve_answer(i2c_adr+I2C_ADDRESS,'s');
    if (answer_long==0x00010000){
            server.print(0);
            server.print('\n');
            return;
        }
    if (answer_long==0){
            server.print(1);
            server.print('\n');
            return;
        }
    if (answer_long==0xFFFFFFFF){
            server.print(2);
            server.print('\n');
            return;
        }
    server.print(3);
    server.print('\n');
}


void comm_is_c_sucsess(){
uint32_t answer_long=0;
    uint8_t i2c_adr=0;
    i2c_adr=get_slot_number();
    send_command(i2c_adr+I2C_ADDRESS,'s');
    delay(100);
    answer_long=recieve_answer(i2c_adr+I2C_ADDRESS,'s');
    if ((answer_long==0x01000000)||(answer_long==0x00010000)){
            server.print(0);
            server.print('\n');
            return;
        }
    if (answer_long==0){
            server.print(1);
            server.print('\n');
            return;
        }
    if (answer_long==0xFFFFFFFF){
            server.print(2);
            server.print('\n');
            return;
        }
    server.print(3);
    server.print('\n');

}


