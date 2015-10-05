#include <Arduino.h>
#include <avr/wdt.h>
#include <Ethernet.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>


//�������
#define TIMEOUT_SHUT 10000
//������������ ����� �������
#define COM_LEN 32
//������� �� ����� �������
#define TIMEOUT_COM 15000

#define FAILS_TIMES 3

#define I2C_ADDRESS 0b1000

#define TIMEOUT_CURTAIN 8000
/*
�������

init_�

���������� ���������� �������� ����������������� ����������� ������� (��������, ����������) ��� ��������� ��������.

test_�

���������� ���������� �������� ����������������� ����������� �������(��������, ����������) ������� ������ �������, ������� ����������� � �����������.

open_�

��������� ��������� ������. � ������ ������� ����� T ������ � X ������� ��������� ��� ������.

close_�

��������� ��������� ������ �������. � ������ ������� ����� T ������ � X ������� ��������� ��� ������.

status_c

���������� ���������� �������� ������� � ������ ��� ������ ���������� ������� � ������ ��������� ��������.




���������:

N, ��� N ������ �������. (������: �init_c:2\n�)

������������ ��������:

��������� ���������� �������������: �A:X\n�, A,X - ���������� ����� � ASCII �������������.
A-��� ������;
X-UID (���������� ����������������� ����� �������) � 32 ����, ������� ���� � ��� �������, ��������� ���������� �����.



init_�_all

���������� ���������� �������� ����������������� ����������� ���� ������ (��������, ����������) ��� ��������� ��������.

���������:

���

������������ ��������:

��������� ���������� �������������: �A:N:An:Xn:A(n+1):X(n+1):A(n+2):X(n+2)�\n�, A,N,An,Xn - ���������� ����� � ASCII �������������.
A-��� ������ �� �������� ������;
N-���������� ������ � ��������;
An-��� ������ �� ������ �������;
Xn-UID (���������� ����������������� ����� �������) � 32 ����, ������� ���� � ��� �������, ��������� ���������� �����.


test_c_all

���������� ���������� �������� ����������������� ����������� ���� ������ (��������, ����������) ������� ������ �������, ������� ����������� � �����������.

open_c_all

��������� ��������� ������ ���� ������ � ��������. � ������ ������� ����� T ������ � X ������� ��������� ��� ������.

close_�_all

��������� ��������� ������ ���� ������ ��������. � ������ ������� ����� T ������ � X ������� ��������� ��� ������.

status_c_all

���������� ���������� �������� ������� ������ ������� � �������� � ������ ��� ������ ���������� ������� � ������ ��������� �������� ������ �������.

������������ ��������:
��������� ���������� �������: �A:N:An:Bn:A(n+1):B(n+1)\n�, A,N,An:Bn - ���������� ����� � ASCII �������������.
A-��� ������ �� �������� ������;
N-���������� ������ � ��������;
An-��� ������ ��� ������ �������;
� �.�.
Bn- ���� ��������� �������� � ���������� ��������� � ASCII �������������.
����� �������� � �������� ������� ���������:
����� ������� ��� � ������ ��������,
������ ������ ��� ������ ��������,
������ ������ ��� � �������������� ������ �1,

���� ��� ����� 1, ������ � ����������� ������, ���� 0, �� ���.

mac_c

��������� ������� ��� ��������� mac ������ ����������� �������� ������

port_c

��������� ������� ��� ��������� �����

reboot_c

��������� ������� ��� ����������� �����������

is_�_closed, is_�_opened, is_�_success

����� �����������, ��������� �� ������� � ��������� ���������.
is_c_closed � ������� �� �������?
is_c_opened � ������� �� �������?
is_c_success � ������ �� ������� � ������?


*/



const byte nack=0x15;
//������ ������
const char commands[16][16]={"init_c","init_c_all","test_c","test_c_all","open_c","open_c_all","close_c","close_c_all","status_c","status_c_all","mac_c","port_c","reboot_c","is_c_closed","is_c_opened","is_c_sucsess"};
//char gen_com[10][16];
//������ ����� ��������� ����������
//�������� 10 ����������
char second_arg[16][16];

//��������� ���������� ��� ���-�������, ������������ �� EEPROM
byte mac[6] = {};
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
//};
//assign an IP address for the controller:

//��������� ���������� ��� IP �������, ������������ �� EEPROM, ��� ����������� � ������ ������� � DHCP
IPAddress ip(192, 168, 1, 130);
//��������� ���������� ��� �����, ������������ �� EEPROM, ��� ����������� � ������ ������� � DHCP
IPAddress gateway(192, 168, 1, 1);
//��������� ���������� ��� ����� �������, ������������ �� EEPROM, ��� ����������� � ������ ������� � DHCP
IPAddress subnet(255, 255, 255, 0);

//��������� ������
char temp_char;

//��������� ���������� ��� �����, ������������ �� EEPROM
word port_number=30002;

//�������������� ������ �� �������� �����
EthernetServer server = EthernetServer(port_number);

//const int in_put=8;
//const int out_put=7;

//����� ���������� ������
long lastReadingTime = 0;
//int toggle_switch=0;
//����� ��� �������� �������
//�� ����� 32 ��������
char com_recv[COM_LEN]={};
//����� ����� �������� �������
uint8_t char_number=0;

//���������� ��� �������� ����� ������ � ��������, ����������� �� EEPROM
uint8_t number_of_cassete=0;

uint32_t answer_cassete[16]={};

uint8_t global_answer=255;



//������� ������ ������������ ���� ������ �� EEPROM
IPAddress read_network_config_eeprom(byte con){
  byte con_num[4];
  for (int i=0;i<4;i++){
    con_num[i]=EEPROM.read(i+con*4);
  }
//������� ������ ���� ������������ ���� ������ �� EEPROM
IPAddress net(con_num[0],con_num[1],con_num[2],con_num[3]);
return net;
}

//������� ����������� �����������
void software_Reboot()
{
  wdt_enable(WDTO_15MS);
  while(1)
  {
  }
}

//������� ������ ��� ������ �� EEPROM
void read_mac(){
  for (int i=0;i<6;i++){
      mac[i]=EEPROM.read(i+12); //3*4
  }
}
//������� ������ ����� ������ �� EEPROM
void read_port(){
    port_number=word(EEPROM.read(6+12),EEPROM.read(6+13));
}

//������� ������ ����� ������ � EEPROM
void write_port(word port_num){
  EEPROM.write((6+12),highByte(port_num));
  EEPROM.write((6+13),lowByte(port_num));
}

void read_number_of_cassete(){
  number_of_cassete=EEPROM.read(6+14);
  }


//������� ������� ������� �� ������� ������ �� I2C (����� ������, �������)
void send_command(int i2c_address, uint8_t i2c_command){
    Wire.beginTransmission(i2c_address);
    Wire.write(i2c_command);
    Wire.endTransmission();
    delay(100);
}

//������� ������ ������� �� ������ (����� ������, ������� ������� ��������)
uint32_t recieve_answer(int i2c_address, uint8_t i2c_command){

//����������, ������� ������. 4 �����
    uint32_t answer=0;
//������ �� ������ 5 ����. 4 ����� ��� ���������, ����� ��� �������� ���������� � �.�.
    Wire.requestFrom(i2c_address, 5);
    //���� ��� ������ ����� �� ������
    uint8_t c=0;
    //������� �������� ����
    uint8_t b=0;
    //WatchDog
    wdt_enable(WDTO_8S);
    //���� ���� ������ ���������
    while (Wire.available()){
    c = Wire.read();
    //��������� 5-� ���� ������

    //���� ������� ��������,� ����������, ���������� ��������� �����
     if ((b==4)&&(c==(i2c_command-32))){
        return answer;
    }
    //���� ������� ��������,�� �� ����������, ���������� �������
     if ((b==4)&&(c==i2c_command)){
        return 0xFFFFFFFF;
    //���� ����� ��� ������ �� �����, ���������� ����
    if ((b==4)&&(c!=(i2c_command-32))){
        return 0x00000000;
    }

    }
// ���������� ��������
//    Serial.println(c);
//��������� �����
    answer=answer<<8;
    answer=answer+c;
    b++;
    }
    wdt_disable();
    return answer;
}



//������� ������ ��������� ��������
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
//������������� ������ �����/������

//����������� � ���� I2C � ������ �������
Wire.begin();


// ������ ������������ ���� �� EEPROM
 ip=read_network_config_eeprom(0);
 gateway=read_network_config_eeprom(1);
 subnet=read_network_config_eeprom(2);
 read_mac();
 read_port();
 read_number_of_cassete();

//�������������� TCP/IP ������ �� ����� ��������� �� EEPROM
server = EthernetServer(port_number);

//�������������� UART ��� ����������� ������
  Serial.begin(9600);
//�������� ������������ � ���� �� DHCP
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
//���� �� ���������� �� DHCP
//������������ � ���� � ������� ���������� �� EEPROM
    Ethernet.begin(mac, ip, gateway, subnet);
  }
  // give the Ethernet shield a second to initialize:
  delay(1000);
  Serial.println("connecting...");

//������ �������
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  // initalize the  data ready and chip select pins:
  // give the sensor and Ethernet shield time to set up:
  delay(1000);
}

//�������� ����
void loop() {
  // check for a reading no more than once a second.
  //��� � ������� ���������� ����� ��������, ���� ��� ������������ ��������
  if ((millis() - lastReadingTime) > 1000) {

//������ � ���-���� ���������� ����
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
//����� ������� ��������� ��������� ����������
  listenForEthernetClients();
}

//������� ��������� ��������� ����������
void listenForEthernetClients(){
//���������� ������� �������
  long time_stamper=0;
//������� ��������
  EthernetClient client = server.available();
//���� ���� ������������ ������ ���� �� ���� ������
  if (client) {
    Serial.println("Got a client");
//���� ���� ���������� � ��������
    while (client.connected()) {
//���� �� ������� ���� ������, ������������ �� �����������, ���� ��� �� ����������
      if (client.available()) {
//�������� ����� ��� ������ ���������� �� �������� ������ ����� �������
        time_stamper=millis();
//������ ������ ���������
        char c = client.read();
//���� ��� ������ ����� ������ ������������ �������
        if (c == '\n') {
//���������� ���� � ��� ����
          Serial.println("Starting sending");
          Serial.println(com_recv);
//        server.println(com_recv);
//        delay(3000);
//��������� ���������� �������
          do_command(parscommand());
//������� �����, �������� ������ ��������
          for(int i=0;i<=char_number;i++){
          com_recv[i]=0;
          }
          char_number=0;
          break;
        }
//���� ������ �� \r, ��������� � �����
        else if (c != '\r') {
          // you've gotten a character on the current line
//          currentLineIsBlank = false;
          com_recv[char_number]=c;
          char_number++;
//���� ���������� �������� ��������� ����������, �� �������� NACK � ��������� ����������
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
//���� ����� ������ �������� ��������� ������� �������� NACK � ��������� ����������

    }
  }
}

//������� �������
//������ ����� ������� ���������

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
//���������� ���������� �������
//����� ACK ���������������
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

//���� ������� �� ���������� �������� NACK
  default:
    server.write(nack);
    server.print('\n');
    Serial.println("Unknown command");
}
}

//��������� ������� reboot
void comm_reboot_c(){
  server.print("0\n");
  delay(500);
  software_Reboot();
}

//��������� ������� mac
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

//������� ��������� ������� �������� - ������ �����
uint8_t get_slot_number(){
    //���� �������� ����� �����, ��� ���������� ��� ��� ��������
    uint8_t i2c_adr=0;

    int i=0;
    //������������ ������ ��������� ����������, ���� �������� ����� � ���� �������� ����
    while((second_arg[0][i]>=0x30)&&(second_arg[0][i]<=0x39)){
            //���������� �������, �.�. � ��� ���������� 0 �� ������ ������ ��� �� ���������
  i2c_adr*=10;
  //���������� ������-��������
  i2c_adr+=byte(second_arg[0][i])-0x30;
  i++;
  }
  //���������� ����� �����, �� ��� �������� ������  I2C
  return i2c_adr;
}

// ������� init, �������� ���� ������� ����� �������, ����������� � ����, ��� ��-������ ��������� ���������� ����������� ������� � �����
// ��-������ ������, ��� �� ������� ����������� � ����
void comm_init_c(){
    //���������� ������
    uint32_t answer_long=0;
    //���������� ������ I2C ���������� �������
    uint8_t i2c_adr=0;
    //�������� ����� �����
    i2c_adr=get_slot_number();
    //�������� ������� �� ����� �����+�������� ��� ��������� I2C ������
  send_command(i2c_adr+I2C_ADDRESS,'i');
  //������� ������ ������ ������, ������� ���� �������
  delay(100);
  //�������� ������� ��������� ������ �� ������� � ���������
  answer_long=recieve_answer(i2c_adr+I2C_ADDRESS,'i');
  //���� ����� ����� � ���������� ���������, ���������� 0 � ����� : ��� �����, ������� ������������ UID
  if ((answer_long!=0)&&(answer_long!=0xFFFFFFFF)){
    server.print(0);
    server.print(':');
    server.print(answer_long);
    server.print('\n');
  }
  //���� ����� 0, �� ���������� ��� ������ 1 (����� ���� ��������� ����������)
  if (answer_long==0){
    server.print(1);
    server.print(':');
    server.print(answer_long);
    server.print('\n');
  }

  //���� ����� FFFFFFFF, �� ���������� ��� ������ 1 (����� ���� ��������� ����������)
  if (answer_long==0xFFFFFFFF){
    server.print(2);
    server.print(':');
    server.print(answer_long);
    server.print('\n');
  }
}

// ������� init_c_all, �������� ���� ������� ���������� ������ � �������� � ����� ������ �������, ����������� � ����, ��� ��-������ ��������� ���������� ����������� ������� � �����
// ��-������ ������, ��� �� ������� ����������� � ����
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

//��������� ������� open_c
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


//��������� ������� close
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

//��������� ������� status
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

//��������� ������� test
//���� ������������
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

//��������� ������� port
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


