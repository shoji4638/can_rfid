//03:2023/08/15 とりあえずCS3で認識 Cmd0x3A ステータス読み出し途中で止まる。
//04:2023/08/16 CS3で正常に認識,ID返信NG
//05:2023/08/16 正常動作（IDを取得できず、強制入力）、コンタクトも？
//06:コンタクト関係テスト
//06-02:CanReadをSwitch化　06-03:CanReadをSwitch化 動作OK
//LINK_RFID_10:LINK_10よりコピー　RFIDを追加OK?
//11:Tag_IDの取得をポインタとする
//12:RFID Debug Mode追加 シリアル出力をTextから、バイナリへ
//13:Tag⇒CanBusへ送信テスト、Link88の基本情報などをヘッダーファイル化に分けた。⇒git can_rfid 初版
//Git can_rfid RFIDデバック確認モード追加,SERIAL出力をASCIIとバイナリを選択式に
/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Arduino_CAN.h>
#include "marklin_can.h"

/**************************************************************************************
 * define
 **************************************************************************************/
//#define Active_Cmd  0x16
#define Out01 D6
#define Out02 D7
#define LED_R D6
#define LED_G D7
#define button01 16
#define button02 17
#define button03 18
#define button04 19

#define rfid_debug  //rfidをデバックする時にコメントを外す。
#define id_ascii    //rfidの出力をASCIIにする。コメントアウト時は、バイナリー出力

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/
//        cmd=0x16 DLC=6   [Address 4byte  No2][向き][OnOff]
//uint8_t const msg_data[] = {0x00,0x00,0x30,0x02,0x00,0x01,0x00,0x00};
//uint8_t const uid[] = { 0x53, 0x38, 0x3B, 0x99 };
//unsigned int const hash_ar88 = 0x4721;
//uint8_t msg_data[8];
uint8_t id;
unsigned int hash_cs = 0x0000;
unsigned int mode = 0, CanWrite = 0;
uint32_t CAN_ID;

uint8_t on_off[16] = {
  false, false, false, false, false, false, false, false,
  false, false, false, false, false, false, false, false
};
//RFID用
struct Tag_Lok {
  int lok;
  unsigned char id[5];
};

Tag_Lok TagLok_Data[10];

uint8_t inByte = 0;
int cnt = 0, cnt_temp = 0, timeup = 0, crc;
//uint8_t recive_data[50], tag_id[5];
unsigned char tag_id[5], recive_data[50];

int rfid_read(unsigned char *p_tag, int Debug_mode) {
  /*******************************
    Sub RFID Reader
  *******************************/
  int ret = 0;  //1<:TagID取得　-1>:エラー

  cnt = Serial1.available();
  if (cnt == 30) {
    int id_index = 4;

    digitalWrite(LED_G, HIGH);

    for (int ii = 0; ii < 30; ii++) {
      inByte = Serial1.read();
      if (inByte != -1) {
#ifdef rfid_debug
        Serial.print(" ");
        Serial.print(inByte, HEX);
#endif
        recive_data[ii] = inByte;

        switch (ii) {
          case 0:
            if (recive_data[ii] == 0x02) {
#ifdef rfid_debug
              Serial.print(" Start Code OK [");
#endif
            } else {
#ifdef rfid_debug
              Serial.println(" Start Code Not:02 Error!!!");
#endif
              digitalWrite(LED_R, HIGH);
              ret = -1;
            }
            break;
          case 1:
          case 3:
          case 5:
          case 7:
          case 9:
            if (recive_data[ii] >= 0x41) {  //0-9 or A-F
                                            //              tag_id[id_index] = recive_data[ii] - 'A' + 10;
              *(p_tag + id_index) = recive_data[ii] - 'A' + 10;
            } else {
              //              tag_id[id_index] = recive_data[ii] - '0';
              *(p_tag + id_index) = recive_data[ii] - '0';
            }
            break;

          case 2:
          case 4:
          case 6:
          case 8:
            if (recive_data[ii] >= 0x41) {  //0-9 or A-F
              *(p_tag + id_index) = (recive_data[ii] - 'A' + 10) * 0x10 + *(p_tag + id_index);
            } else {
              *(p_tag + id_index) = (recive_data[ii] - '0') * 0x10 + *(p_tag + id_index);
            }
            id_index--;
            break;
          case 10:
            if (recive_data[ii] >= 0x41) {  //0-9 or A-F
              *(p_tag + id_index) = (recive_data[ii] - 'A' + 10) * 0x10 + *(p_tag + id_index);
            } else {
              *(p_tag + id_index) = (recive_data[ii] - '0') * 0x10 + *(p_tag + id_index);
            }
#ifdef rfid_debug
            Serial.print("]");
#endif
            break;
          case 27:
            crc = recive_data[ii];
            break;
          case 28:
            if (crc + recive_data[ii] == 0xFF) {
#ifdef rfid_debug
              Serial.print(" CRC OK");
#endif
            } else {
              digitalWrite(LED_R, HIGH);  //受信異常
#ifdef rfid_debug
              Serial.println(" CRC ERROR!!:");
              Serial.print(crc + recive_data[ii]);
#endif
              ret = -2;
            }
            break;
          case 29:
            if (recive_data[ii] == 0x03) {
#ifdef rfid_debug
              Serial.println(" End Code OK");
              //取得Tag表示
              Serial.print("Tag:");
              for (int ii = 0; ii < 10; ii++) {
                Serial.print(" ");
                Serial.print(tag_id[ii], HEX);
              }
              Serial.println("");
#endif
              digitalWrite(LED_R, LOW);  //受信正常
              ret = 1;
            } else {
              digitalWrite(LED_R, HIGH);  //受信異常
#ifdef rfid_debug
              Serial.println(" End Code Not:03 Error!!!");
#endif
              ret = -3;
            }
            break;
        }  //switch(ii)のEnd
        if (ret < 0) break;

      } else {  //読込終了？エラー これはないはず。
#ifdef rfid_debug
        Serial.println(" Read Error!");
#endif
        ret = -99;
        break;
      }

    }                          //for(int ii = 0; ii < 30; ii++)のEnd
    digitalWrite(LED_G, LOW);  //受信完了

  } else {  //cnt != 30
    if (cnt != 0) {
      digitalWrite(LED_G, HIGH);
      if (cnt == cnt_temp) {
        if (timeup++ > 10000) {
          digitalWrite(LED_G, HIGH);
          while (Serial1.read() != -1) {}
          timeup = 0;
#ifdef rfid_debug
          Serial.print("cnt:");
          Serial.print(cnt);
          Serial.println(" Timeup");
#endif
        }
      } else {
        cnt_temp = cnt;
        timeup = 0;
      }
    } else {  // cnt = 0
      digitalWrite(LED_G, LOW);
    }
  }  //cnt != 30 End
  return ret;
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial1.begin(9600, SERIAL_8N1);  //RFID読み込み
  Serial.println("Start UART Receive:D0<-RFID");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Out01, OUTPUT);
  pinMode(Out02, OUTPUT);
  pinMode(button01, INPUT_PULLUP);
  pinMode(button02, INPUT_PULLUP);
  pinMode(button03, INPUT_PULLUP);
  pinMode(button04, INPUT_PULLUP);

  if (!CAN.begin(CanBitRate::BR_250k)) {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }
  mode = 0;
  Serial.println("Mode:0");
  digitalWrite(LED_G, HIGH);

  for (int i = 0; i < 8; i++) {
    Serial.print(L88_DATA[0][0][i], HEX);
    Serial.print(",");
  }
  Serial.println("");
}

void loop() {

  /*******************************
     CAN Read
  *******************************/
  if (CAN.available()) {
    int hash, cmd;
    CanMsg const msg = CAN.read();
    hash = msg.id & 0xFFFF;  //HASH
    cmd = msg.id >> 16;      //COMMAND
                             //    ret = memcmp(msg.data, msg_data, msg.data_length);  //Data比較
                             /*    Serial.print("Cmd:");
    Serial.print(cmd);
    Serial.print(" Hash:");
    Serial.print(hash);
    Serial.print(" Msg:");
    Serial.print(msg);
    Serial.print(" Ret:");
    Serial.println(ret);
*/
    //Bootloader（機器接続）受信⇒CSのHASHを取得
    switch (mode) {
      case 0:
        if (cmd == 0x36 && msg.data_length == 0) {
          hash_cs = hash;
          mode = 1;
          CanWrite = 1;
          Serial.print("Mode 0->1: Cmd 0x36(R)");
          Serial.print("HASH_CS:");
          Serial.println(hash_cs, HEX);
        }
        break;

      case 2:
        //認識後初めてのPing受信
        if (cmd == 0x30 && msg.data_length == 0) {
          mode = 3;
          CanWrite = 1;
          Serial.println("Mode 2->3: Cmd 0x30(R) 1st Ping Receive:");
        }
        break;

      case 4:
        //？動かない⇒間違い
        if (cmd == 0x00 && msg.data[4] == 0x0C && msg.data_length == 7 && msg.data[0] == uid[0] && msg.data[1] == uid[1] && msg.data[2] == uid[2] && msg.data[3] == uid[3]) {
          mode = 5;
          CanWrite = 1;
          id = msg.data[5] * 0x100 + msg.data[6];
          Serial.print("Mode 4->5: Cmd 0x00(R)Sub 0x0C Receive ID: ");
          Serial.println(id);
        }
        break;

      case 10:
        //通常時（mode=10
        //Ping応答
        if (cmd == 0x30 && msg.data_length == 0) {
          mode = 11;
          CanWrite = 1;
          digitalWrite(LED_R, HIGH);
          Serial.println("Mode 10->11: Cmd 0x30(R) Ping Receive:");
        }
        //Cmd 0x3A（ステータス要求）受信
        if (cmd == 0x3A && msg.data_length == 5 && uid[0] == msg.data[0] && uid[1] == msg.data[1] && uid[2] == msg.data[2] && uid[3] == msg.data[3]) {
          mode = 1000 + msg.data[4] * 10;  //mode = 1000 + index*10
          CanWrite = 1;
          Serial.print("Cmd=3A:Status Request mode:");
          Serial.print(mode);
          Serial.print(" UID:");
          Serial.print(msg.data[0], HEX);
          Serial.print(msg.data[1], HEX);
          Serial.print(msg.data[2], HEX);
          Serial.print(msg.data[3], HEX);
          Serial.print(" Index:");
          Serial.println(msg.data[4], HEX);
        }
        /*      //Cmd 0x00 SubCmd0x0b 受信
        if (cmd == 0x00 && msg.data[4] == 0x0B && uid[0] == msg.data[0] && uid[1] == msg.data[1] && uid[2] == msg.data[2] && uid[3] == msg.data[3]) {
          mode = 12;
          id = msg.data[5];
          Serial.print("Mode 10->12: SubCmd 0x0B(R):data[5]:Set Pr");
          Serial.print(id,HEX);
          Serial.print(" CanMsg:");
          Serial.println(msg);
        }
  */
        //Cmd 0x00 SubCmd0x0B 受信
        if (cmd == 0x00 && msg.data[4] == 0x0B && uid[0] == msg.data[0] && uid[1] == msg.data[1] && uid[2] == msg.data[2] && uid[3] == msg.data[3]) {
          mode = 13;
          CanWrite = 1;
          id = msg.data[6];
          Serial.print("Mode 10->12: SubCmd 0x0C(R):data[5]:Set Pr");
          Serial.print(id, HEX);
          Serial.print(" CanMsg:");
          Serial.println(msg);
        }

        if (cmd == 0x23) {
          Serial.print("Cmd0x23:");
          Serial.print(msg);
        }
        break;
    }  //switch (mode)　END
  }    //if (CAN.available())　END


  /*********************************************
    RFID Read
     rfid_read(*p_tag,Debug_mode)  p_tagに5byteのIDを入れる。 
  *********************************************/
#ifdef rfid_debug
  if (CanWrite != 1) {
#else
  if (mode == 10 && CanWrite != 1) {
#endif

    unsigned long rfid_ret = rfid_read(&tag_id[0], -1);

    if (rfid_ret > 0) {
      //Tag所得
      Serial.print("Tag:");
      for (int ii = 0; ii < 5; ii++) {
#ifdef id_ascii
        Serial.print(tag_id[ii], HEX);  //HEXでASCII出力
        Serial.write(" ");
#else
        Serial.write(tag_id[ii]);  //バイナリーデータ送信
#endif
      }
      Serial.println("");
#ifndef rfid_debug
      mode = 100;
      CanWrite = 1;
#endif
    }
    if (rfid_ret < 0) {
      //Tag取得エラー
      Serial.print("RFID READ ERROR!!!:Code:");
      Serial.println(rfid_ret);
    }
  }

  /******************************
  // SW 状況確認
  ******************************/
  if (mode == 10 && CanWrite != 1) {
    for (int n = 0; n < 4; n++) {
      if (digitalRead(16 + n) != on_off[n]) {
        CAN_ID = 0x00230000 + hash_cs;
        //Cmd 0x23             {[ID:2Byte][CH:2Byte][New][Old][Time:2Byte]}
        uint8_t onoff_data[8] = { 0x00, 0x03, 0x00, n + 1, 0x00, 0x00, 0x00, 0x46 };
        //        onoff_data[3] = n + 1;  //CHを登録
        if (on_off[n] == false) {
          onoff_data[4] = 0x01;  //onoff_data[5] = 0x00;
        } else {
          onoff_data[5] = 0x01;  //onoff_data[4] = 0x00;
        }
        //        if (on_off[n] == false) onoff_data[4] = 0x01; else onoff_data[5] = 0x01;         //onoff_data[4] = 0x00;
        CanMsg send_msg(CAN_ID, sizeof(onoff_data), onoff_data);

        if (int const rc = CAN.write(send_msg); rc < 0) {
          Serial.print("Cmd0x23:CAN.write(...) failed with error code ");
          Serial.println(rc);
        } else {
          Serial.print("Cmd:0x23(W):");
          Serial.print(send_msg);
          Serial.println(on_off[n]);
          on_off[n] = digitalRead(16 + n);
        }
      }
    }
  }

  /*************************
  // Can Write(Send)
  *************************/

  while (CanWrite == 1) {
    //Cmd0x37 Bootloader応答
    if (mode == 1) {
      CAN_ID = 0x00370000 + hash_ar88;
      //BootLoader返信　　（UID 4byte,バージョン:1.1,機種:0040(Link88)）
      uint8_t msg_data[] = { uid[0], uid[1], uid[2], uid[3], 0x01, 0x01, 0x00, 0x40 };
      CanMsg send_msg(CAN_ID, sizeof(msg_data), msg_data);

      if (int const rc = CAN.write(send_msg); rc < 0) {
        Serial.print("Mode = 1->0: Cmd:0x37(W):CAN.write(...) failed with error code ");
        Serial.print(rc);
        Serial.print("  HASH_CS:");
        Serial.println(hash_cs, HEX);
      } else {
        Serial.println("Mode 1->2: Cmd:0x37(W):Bootloader Anser!");
        mode = 2;
        CanWrite = 0;
      }
    }

    //Cmd0x31:初めてのPing応答
    if (mode == 3) {
      CAN_ID = 0x00310000 + hash_ar88;
      //Ping返信（UID 4byte,SWバージョン:0100,機種:0040(Link88)）
      uint8_t msg_data[] = { uid[0], uid[1], uid[2], uid[3], 0x01, 0x01, 0x00, 0x40 };
      CanMsg send_msg(CAN_ID, sizeof(msg_data), msg_data);

      if (int const rc = CAN.write(send_msg); rc < 0) {
        Serial.print("Mode = 3->3: Cmd:0x31(W):1st Ping :CAN.write(...) failed with error code ");
        Serial.print(rc);
        Serial.print("  HASH_CS:");
        Serial.println(hash_cs, HEX);
      } else {
        Serial.print("Mode 3->4: Cmd:0x31(W):1st Ping Anser! Send_Msg:");
        Serial.println(send_msg);
        mode = 5;
        CanWrite = 1;  //注意 続けてIDを送信
      }
    }

    if (mode == 5) {
      //Cmd:0x00 SubCmd:0x0C ID返信
      CAN_ID = 0x00010000 + hash_cs;
      //ID返信            （UID 4byte,SubCmd 0x0C,0x00,確認ID）
      //uint8_t msg_data[] = {uid[0],uid[1],uid[2],uid[3],0x0C,0x00,uid[3]}; CS2?
      uint8_t msg_data[] = { uid[0], uid[1], uid[2], uid[3], 0x0C, 0x00, 0x03 };
      CanMsg send_msg(CAN_ID, sizeof(msg_data), msg_data);

      if (int const rc = CAN.write(send_msg); rc < 0) {
        Serial.print("Mode 5->5: Cmd:0x00(W) SubCmd:0x0c:CAN.write(...) failed with error code ");
        Serial.print(rc);
        Serial.print("  HASH_CS:");
        Serial.println(hash_cs, HEX);
      } else {
        Serial.print("Mode 5->10: Cmd:0x00(W) SubCmd:0x0c:確認ID:");
        Serial.print(msg_data[7]);
        Serial.print(" Send_Msg:");
        Serial.println(send_msg);
        mode = 10;
        CanWrite = 0;
      }
    }

    if (mode == 11) {
      //Cmd 0x31 Ping返信
      CAN_ID = 0x00310000 + hash_ar88;
      //Ping返信（UID 4byte,SWバージョン:0100,機種:0040(Link88)）
      uint8_t msg_data[] = { uid[0], uid[1], uid[2], uid[3], 0x01, 0x01, 0x00, 0x40 };  //BootLoader返信（UID 4byte,SWバージョン:0100,機種:0040(Link88)）
      CanMsg send_msg(CAN_ID, sizeof(msg_data), msg_data);

      if (int const rc = CAN.write(send_msg); rc < 0) {
        Serial.print("Mode 11->10 Cmd:0x31(W):CAN.write(...) failed with error code ");
        Serial.print(rc);
        Serial.print("  HASH_CS:");
        Serial.println(hash_cs, HEX);
        mode = 10;
        CanWrite = 0;  //ちょっとぐらいPing応答できなくてもOKか？
      } else {
        Serial.print("Mode 11->10: Cmd:0x31(W):Ping Rep! Send_Msg:");
        Serial.println(send_msg);
        mode = 10;
        CanWrite = 0;
        digitalWrite(LED_R, LOW);
      }
    }

    if (mode == 13) {
      //
      CAN_ID = 0x00010000 + hash_cs;
      //uint8_t const msg_data[] = {0x53,0x38,0x39,0x99,0x01,0x00,0x00,0x40}; //BootLoader返信（UID 4byte,SWバージョン:0100,機種:0040(Link88)）
      uint8_t msg_data[] = { uid[0], uid[1], uid[2], uid[3], 0x0B, 0x03, 0x01 };
      CanMsg send_msg(CAN_ID, sizeof(msg_data), msg_data);

      if (int const rc = CAN.write(send_msg); rc < 0) {
        Serial.print("Mode 13 Cmd0x00 SubCmd0x0B:CAN.write(...) failed with error code ");
        Serial.print(rc);
        Serial.print("  ID:");
        Serial.println(id);
      } else {
        Serial.print("Mode 13->10: SubCmd:0x0B(W):ID Anser! ID:");
        Serial.print(id);
        Serial.print(" RepMsg:");
        Serial.println(send_msg);
        mode = 10;
        CanWrite = 0;
      }
    }

    if (mode == 100) {

      CAN_ID = 0x00770000 + hash_ar88;
      uint8_t msg_data[] = { tag_id[0], tag_id[1], tag_id[2], tag_id[3], tag_id[4] };
      CanMsg send_msg(CAN_ID, sizeof(msg_data), msg_data);
      if (int const rc = CAN.write(send_msg); rc < 0) {
        Serial.print("Cmd:0x77_[END] CAN.write(...) failed with error code ");
        Serial.print(rc);
        Serial.print("  MODE:");
        Serial.println(mode);
        //            for (;;) { }
      } else {
        Serial.print("Cmd:0x77 MODE:");
        Serial.print(mode);
        mode = 10;
        CanWrite = 0;
      }
    }

    if (mode >= 1000) {
      int index = int((mode - 1000) / 10);
      int paket_no = mode % 10;
      while (L88_DATA[index][paket_no][2] != uid[2] or L88_DATA[index][paket_no][3] != uid[3]) {
        //msg=0x003B0301+paket_no, dlc=8, data=L88_DATA[index][paket_no][0:8])
        CAN_ID = 0x003B0301 + paket_no;
        uint8_t msg_data[] = { L88_DATA[index][paket_no][0], L88_DATA[index][paket_no][1],
                               L88_DATA[index][paket_no][2], L88_DATA[index][paket_no][3],
                               L88_DATA[index][paket_no][4], L88_DATA[index][paket_no][5],
                               L88_DATA[index][paket_no][6], L88_DATA[index][paket_no][7] };
        //memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
        CanMsg send_msg(CAN_ID, sizeof(msg_data), msg_data);
        if (int const rc = CAN.write(send_msg); rc < 0) {
          Serial.print(" Cmd:0x3B CAN.write(...) failed with error code ");
          Serial.print(rc);
          Serial.print("  MODE:");
          Serial.println(mode);
        } else {

          Serial.print(" Cmd=3B:mode:");
          Serial.print(mode);
          Serial.print(" index:");
          Serial.print(index);
          Serial.print(" paket_no");
          Serial.print(paket_no);
          Serial.print(" Data:");
          Serial.print(L88_DATA[index][paket_no][0]);
          Serial.print(L88_DATA[index][paket_no][1]);
          Serial.print(L88_DATA[index][paket_no][2]);
          Serial.print(L88_DATA[index][paket_no][3]);
          Serial.print(L88_DATA[index][paket_no][4]);
          Serial.print(L88_DATA[index][paket_no][5]);
          Serial.print(L88_DATA[index][paket_no][6]);
          Serial.println(L88_DATA[index][paket_no][7]);

          mode = mode + 1;
          index = int((mode - 1000) / 10);
          paket_no = mode % 10;
        }
      }
      CAN_ID = 0x003B0000 + hash_ar88;
      //  uint8_t const msg_data[] = {0x53,0x38,0x39,0x99,0x01,0x00,0x00,0x40}; //BootLoader返信（UID 4byte,SWバージョン:0100,機種:0040(Link88)）
      uint8_t msg_data[] = { L88_DATA[index][paket_no][0], L88_DATA[index][paket_no][1],
                             L88_DATA[index][paket_no][2], L88_DATA[index][paket_no][3],
                             L88_DATA[index][paket_no][4], L88_DATA[index][paket_no][5] };
      //          memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
      CanMsg send_msg(CAN_ID, sizeof(msg_data), msg_data);
      if (int const rc = CAN.write(send_msg); rc < 0) {
        Serial.print("Cmd:0x3B_[END] CAN.write(...) failed with error code ");
        Serial.print(rc);
        Serial.print("  MODE:");
        Serial.println(mode);
        //            for (;;) { }
      } else {
        Serial.print("Cmd:0x3B END MODE:");
        Serial.print(mode);
        Serial.print(index);
        Serial.print(paket_no);
        Serial.print(L88_DATA[index][paket_no][0]);
        Serial.print(L88_DATA[index][paket_no][1]);
        Serial.print(L88_DATA[index][paket_no][2]);
        Serial.print(L88_DATA[index][paket_no][3]);
        Serial.print(L88_DATA[index][paket_no][4]);
        Serial.println(L88_DATA[index][paket_no][5]);
        Serial.println(" Cmd:0x3B_[END] Send Done!");
        mode = 10;
        CanWrite = 0;
      }
    }
  }  //while (CanWrite == 1) END

}  // Main Loop