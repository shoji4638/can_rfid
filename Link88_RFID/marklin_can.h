#ifndef MARKLIN_CAN_H
#define MARKLIN_CAN_H
//------------------------------------------------

//------------------------------------------------
//  マクロ定義(Macro definition)
//------------------------------------------------

//------------------------------------------------
//  型定義(Type definition)
//------------------------------------------------
uint8_t const uid[] = { 0x53, 0x38, 0x3B, 0x99 };
unsigned int const hash_ar88 = 0x4721;

//                    [index][row][]
uint8_t const L88_DATA[0x0D][8][8] = {                              //
  {                                                                 //index:00 04
                                                                    //   [index数:2byte]           [UID3-0x38][UID4]
                                                                    //		{0x00,0x0C,0x00,0x00, 0x00,0x00,0x9A,0x6B}, //301 000C0000  0000 UID3-0x38 UID4
    { 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, uid[2] - 0x38, uid[3] },  //301 00020000  0000 UID3-0x38 UID4
                                                                    //      [製造番号8byte]
                                                                    //		{0x36,0x30,0x38,0x38, 0x33,0x2D,0x31,0x00}, //302 36303838  33000000 6088 3-1
                                                                    //		{0x36,0x30,0x38,0x38, 0x33,0x00,0x00,0x00}, //302 36303838  33000000 6088 3
    { 0x53, 0x68, 0x6F, 0x6A, 0x69, 0x40, 0x00, 0x00 },             //302 36303838  33000000 Shoji@
                                                                    //   [製品名]
    { 0x4c, 0x69, 0x6e, 0x6b, 0x20, 0x53, 0x38, 0x38 },             //303 4C696E6B  20533838 Link S88
                                                                    //		{0x53,0x68,0x6f,0x6a, 0x69,0x20,0x38,0x38}, //303 53696E6B  20533838 Shoji 88
                                                                    //		{0x48,0x52,0x53,0x20, 0x57,0x69,0x46,0x69}, //303 53696E6B  20533838 HRS WiFi
    { 0x20, 0x38, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00 },             //304 20383800  00000000  88
                                                                    //		{0x53,0x38,0xF0,0x6B, 0x00,0x04,0x00,0x00}, //HASH UID      0004
    { uid[0], uid[1], uid[2], uid[3], 0x00, 0x04, 0x00, 0x00 },     //HASH UID  index:00 row:04 0x00,0x00
    {},                                                             //306
    {},                                                             //307
    {} },
  {
    //index:01 07
    { 0x01, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02 },          //301 01010200 00000000 L1-1:Einzel? 01:Tastatur
    { 0x41, 0x75, 0x73, 0x77, 0x65, 0x72, 0x74, 0x75 },          //302 41757377 65727475 Ausw ertu
    { 0x6E, 0x67, 0x20, 0x31, 0x20, 0x2D, 0x20, 0x31 },          //303 6e672031 202d2031 ng 1  - 1
    { 0x36, 0x00, 0x45, 0x69, 0x6E, 0x7A, 0x65, 0x6C },          //304 36004569 6e7a656c 6 Ei nzel
    { 0x6E, 0x00, 0x54, 0x61, 0x73, 0x74, 0x61, 0x74 },          //305 6e005461 73746174 n Ta stat
    { 0x75, 0x72, 0x6D, 0x61, 0x74, 0x72, 0x69, 0x78 },          //306 75726d61 74726978 urma trix
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },          //307 00000000 00000000
                                                                 //		{0x53,0x38,0xF0,0x6B, 0x01,0x07,0x00,0x00}, //HASH UID     0107
    { uid[0], uid[1], uid[2], uid[3], 0x01, 0x07, 0x00, 0x00 },  //HASH UID  index:01 low:07 0x00,0x00

  },
  {                                                              //index:02 05
    { 0x02, 0x02, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x05 },          //301 02020000 001f0000 L2-2 Min:0 Max:31 Set:0
    { 0x4C, 0xC3, 0xA4, 0x6E, 0x67, 0x65, 0x20, 0x42 },          //302 4cc3a46e 67652042 La:n ge B
    { 0x75, 0x73, 0x20, 0x32, 0x20, 0x28, 0x52, 0x4A },          //303 75732031 2028524a us 1  (RJ
    { 0x34, 0x35, 0x2D, 0x31, 0x29, 0x00, 0x30, 0x00 },          //304 34352d31 29003000 45-1 ) 0
    { 0x33, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },          //305 33310000 00000000 31
                                                                 //		{0x53,0x38,0xF0,0x6B, 0x02,0x05,0x00,0x00}, //HASH UID     0205
    { uid[0], uid[1], uid[2], uid[3], 0x02, 0x05, 0x00, 0x00 },  //HASH UID  index:02 low:05 0x00,0x00
    {},                                                          //307
    {} },
  {                                                              //index:03 05
    { 0x03, 0x02, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00 },          //301 03020000 001f0000 L3-2 Min:0 Max:31 Set:0
    { 0x4C, 0xC3, 0xA4, 0x6E, 0x67, 0x65, 0x20, 0x42 },          //302 4cc3a46e 67652042 La:n ge B
    { 0x75, 0x73, 0x20, 0x32, 0x20, 0x28, 0x52, 0x4A },          //303 75732032 2028524a us 2  (RJ
    { 0x34, 0x35, 0x2D, 0x32, 0x29, 0x00, 0x30, 0x00 },          //304 34352d32 29003000 45-1 ) 0
    { 0x33, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },          //305 33310000 00000000 31
                                                                 //		{0x53,0x38,0xF0,0x6B, 0x03,0x05,0x00,0x00}, //HASH UID     0305
    { uid[0], uid[1], uid[2], uid[3], 0x03, 0x05, 0x00, 0x00 },  //HASH UID  index:03 low:05 0x00,0x00
    {},                                                          //307
    {} },
  {                                                              //index:04 05
    { 0x04, 0x02, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00 },          //301 04020000 001f0000 L4-2 Min:0 Max:31 Set:0
    { 0x4C, 0xC3, 0xA4, 0x6E, 0x67, 0x65, 0x20, 0x42 },          //302 4cc3a46e 67652042 La:n ge B
    { 0x75, 0x73, 0x20, 0x33, 0x20, 0x28, 0x36, 0x2D },          //303 75732033 2028362d us 3  (6-
    { 0x50, 0x6F, 0x6C, 0x69, 0x67, 0x29, 0x00, 0x30 },          //304 506f6c69 67290030 Poli g) 0
    { 0x00, 0x33, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00 },          //305 00333100 00000000  31
                                                                 //		{0x53,0x38,0xF0,0x6B, 0x04,0x05,0x00,0x00}, //HASH UID     0405
    { uid[0], uid[1], uid[2], uid[3], 0x04, 0x05, 0x00, 0x00 },  //HASH UID  index:04 low:05 0x00,0x00
    {},                                                          //307
    {} },
  {                                                              //index:05 06
    { 0x05, 0x02, 0x00, 0x0A, 0x03, 0xE8, 0x00, 0x64 },          //301 0502000a 03e80064 L5-2 Min:10 Max:1000 Set:100
    { 0x5A, 0x79, 0x6B, 0x6C, 0x75, 0x73, 0x7A, 0x65 },          //302 5a796b6c 75737a65 Zykl usze
    { 0x69, 0x74, 0x20, 0x42, 0x75, 0x73, 0x20, 0x31 },          //303 69742042 75732031 it B us 1
    { 0x20, 0x28, 0x52, 0x4A, 0x34, 0x35, 0x2D, 0x31 },          //304 2028524a 34352d31  (RJ 45-1
    { 0x29, 0x00, 0x31, 0x30, 0x00, 0x31, 0x30, 0x30 },          //305 29003130 00313030 ) 10  100
    { 0x30, 0x00, 0x6D, 0x73, 0x00, 0x00, 0x00, 0x00 },          //306 30006d73 00000000 0 ms
                                                                 //		{0x53,0x38,0xF0,0x6B, 0x05,0x06,0x00,0x00}, //HASH UID     0506
    { uid[0], uid[1], uid[2], uid[3], 0x05, 0x06, 0x00, 0x00 },  //HASH UID  index:00 low:04 0x00,0x00
    {} },
  {                                                              //index:06 06
    { 0x06, 0x0C, 0x00, 0x0A, 0x03, 0xE8, 0x00, 0x64 },          //301 0602000a 03e80064
    { 0x5A, 0x79, 0x6B, 0x6C, 0x75, 0x73, 0x7A, 0x65 },          //302 5a796b6c 75737a65
    { 0x69, 0x74, 0x20, 0x42, 0x75, 0x73, 0x20, 0x32 },          //303 69
    { 0x20, 0x28, 0x52, 0x4A, 0x34, 0x35, 0x2D, 0x32 },          //304
    { 0x29, 0x00, 0x31, 0x30, 0x00, 0x31, 0x30, 0x30 },          //305
    { 0x30, 0x00, 0x6D, 0x73, 0x00, 0x00, 0x00, 0x00 },          //306
                                                                 //		{0x53,0x38,0xF0,0x6B, 0x06,0x06,0x00,0x00}, //END
    { uid[0], uid[1], uid[2], uid[3], 0x06, 0x06, 0x00, 0x00 },  //HASH UID  index:00 low:04 0x00,0x00
    {} },
  {                                                              //index:07 06
    { 0x07, 0x02, 0x00, 0x0A, 0x03, 0xE8, 0x00, 0x64 },          //301 L7-2 min:10 Max:1000 Set:100
    { 0x5A, 0x79, 0x6B, 0x6C, 0x75, 0x73, 0x7A, 0x65 },          //302 Zyklusze
    { 0x69, 0x74, 0x20, 0x42, 0x75, 0x73, 0x20, 0x33 },          //303 it_Bus_3
    { 0x20, 0x28, 0x36, 0x2D, 0x50, 0x6F, 0x6C, 0x69 },          //304 _(6-Poli
    { 0x67, 0x29, 0x00, 0x31, 0x30, 0x00, 0x31, 0x30 },          //305 'g) 10 10
    { 0x30, 0x30, 0x00, 0x6D, 0x73, 0x00, 0x00, 0x00 },          //306 00 ms
                                                                 //		{0x53,0x38,0xF0,0x6B, 0x07,0x06,0x00,0x00}, //END
    { uid[0], uid[1], uid[2], uid[3], 0x07, 0x06, 0x00, 0x00 },  //HASH UID  index:00 low:04 0x00,0x00
    {} },
  {                                                              //index:08 05
    { 0x08, 0x02, 0x00, 0x64, 0x03, 0xE8, 0x00, 0xA7 },          //301 L8-2 min:100 max:1000 set:167
    { 0x42, 0x69, 0x74, 0x7A, 0x65, 0x69, 0x74, 0x20 },          //302 Bitzeit
    { 0x53, 0x38, 0x38, 0x00, 0x31, 0x30, 0x30, 0x00 },          //303 S88 100
    { 0x31, 0x30, 0x30, 0x30, 0x00, 0xC2, 0xB5, 0x73 },          //304 1000 ??s
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },          //305
                                                                 //		{0x53,0x38,0xF0,0x6B, 0x08,0x05,0x00,0x00},//END
    { uid[0], uid[1], uid[2], uid[3], 0x08, 0x05, 0x00, 0x00 },  //HASH UID  index:00 low:04 0x00,0x00
    {},                                                          //307
    {} },
  {                                                              //index:09 05
    { 0x09, 0x02, 0x00, 0x0A, 0x03, 0xE8, 0x00, 0x64 },          //301
    { 0x5A, 0x79, 0x6B, 0x6C, 0x75, 0x73, 0x7A, 0x65 },          //302
    { 0x69, 0x74, 0x20, 0x31, 0x20, 0x2D, 0x20, 0x31 },          //303
    { 0x36, 0x00, 0x31, 0x30, 0x00, 0x31, 0x30, 0x30 },          //304
    { 0x30, 0x00, 0x6D, 0x73, 0x00, 0x00, 0x00, 0x00 },          //305
                                                                 //		{0x53,0x38,0xF0,0x6B, 0x09,0x05,0x00,0x00}, //END
    { uid[0], uid[1], uid[2], uid[3], 0x09, 0x05, 0x00, 0x00 },  //HASH UID  index:00 low:04 0x00,0x00
    {},                                                          //307
    {} },
  {                                                              //index:0a 05
    { 0x0A, 0x02, 0x00, 0x0A, 0x00, 0x64, 0x00, 0x25 },          //301
    { 0x5A, 0x79, 0x6B, 0x6C, 0x75, 0x73, 0x7A, 0x65 },          //302
    { 0x69, 0x74, 0x20, 0x54, 0x61, 0x73, 0x74, 0x61 },          //303
    { 0x74, 0x75, 0x72, 0x00, 0x31, 0x30, 0x00, 0x31 },          //304
    { 0x30, 0x30, 0x00, 0x6D, 0x73, 0x00, 0x00, 0x00 },          //305
                                                                 //		{0x53,0x38,0xF0,0x6B, 0x0a,0x05,0x00,0x00}, //END
    { uid[0], uid[1], uid[2], uid[3], 0x0a, 0x05, 0x00, 0x00 },  //HASH UID  index:0a low:05 0x00,0x00
    {},                                                          //307
    {} },
  {                                                              //index:0b 04
    { 0x0B, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00 },          //301
    { 0x53, 0x70, 0x61, 0x6C, 0x74, 0x65, 0x6E, 0x20 },          //302
    { 0x54, 0x61, 0x73, 0x74, 0x61, 0x74, 0x75, 0x72 },          //303
    { 0x00, 0x30, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00 },          //304
    { uid[0], uid[1], uid[2], uid[3], 0x0b, 0x04, 0x00, 0x00 },  //HASH UID  index:0b low:04 0x00,0x00
    {},                                                          //306
    {},                                                          //307
    {} },
  {                                                              //index:0c 04
    { 0x0C, 0x02, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00 },          //301 LC-2 Min:0 Max:15 Set:0
    { 0x5A, 0x65, 0x69, 0x6C, 0x65, 0x6E, 0x20, 0x54 },          //302 Zeilen T
    { 0x61, 0x73, 0x74, 0x61, 0x74, 0x75, 0x72, 0x00 },          //303 astatur
    { 0x30, 0x00, 0x31, 0x35, 0x00, 0x00, 0x00, 0x00 },          //304 0_15____
    { uid[0], uid[1], uid[2], uid[3], 0x0c, 0x04, 0x00, 0x00 },  //HASH UID  index:0c low:04 0x00,0x00
    {},                                                          //306
    {},                                                          //307
    {} }
};

//------------------------------------------------
//  プロトタイプ宣言(Prototype declaration)
//------------------------------------------------


//------------------------------------------------
#endif
