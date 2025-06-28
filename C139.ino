// address 00-07 connected on port F (analog pins 00-07)
// address 08-15 connected on port K (analog pins 08-15)

// data 00-07 connected on port A (digital pins 22-29)
// data 08-15 connected on port C (digital pins 30-37)
const byte DATA_PULLUPS = 0x00;

// control lines on port L (digital pins 42-49)
/*
  0x01  CS-   >
  0x02  R+W-  >
  0x04  RES-  >
  0x08  IRQ-  <
  0x10  DTACK-  <
  0x20  DT-   >
  0x40  12MHz >
  0x80  16MHz >
*/
const byte CTRL_CS = 0x01;  // active low
const byte CTRL_RW = 0x02;
const byte CTRL_RES = 0x04;    // active low
const byte CTRL_IRQ = 0x08;    // active low
const byte CTRL_DTACK = 0x10;  // active low
const byte CTRL_DT = 0x20;     // active low
const byte CTRL_12MHz = 0x40;
const byte CTRL_16MHz = 0x80;

// aux on port H (digital pins 17,16,-,6,7,8,9,-)
/*
  --    = 0x01 (17)
  --    = 0x02 (16)
  --    = 0x04 (--)
  RINGI = 0x08 (6)
  RINGO = 0x10 (7)
  --    = 0x20 (8)
  --    = 0x40 (9)
  --    = 0x80 (--)
*/
const byte AUX_RX = 0x08;
const byte AUX_TX = 0x10;

const byte OSC_12MHZ_DELAY = 4;
const byte OSC_16MHZ_DELAY = 3;

bool OSC_12MHZ = true;
bool OSC_16MHZ = true;

byte OSC_12MHZ_TICKS = OSC_12MHZ_DELAY;
byte OSC_16MHZ_TICKS = OSC_16MHZ_DELAY;

short COUNT_16MHZ = 0;
short COUNT_12MHZ = 0;

byte INTCOUNT = 0x00;

byte CTRL = 0xff;

char buffer0[0x80];

bool DEBUG_LOG = true;

// serial command stuff
const byte command_len = 32;
const char command_end = '\n';
char command_buf[command_len] = { 0 };
byte command_cnt = 0;
String command_prt[8];

// tx detector
byte txclock = CTRL_16MHz;
byte txstate = AUX_TX;
byte txactive = 0;
byte txhigh = 0;
byte txcount = 0;
byte txdata[0x10] = { 0 };
byte txindex = 0;

byte TX_SIZE = 32;

byte rxdata[0x10] = { 0 };


// setup logic
void setup() {
  setupPins();
  setupSerial();

  DEBUG_LOG = false;
  util_reset();
  util_idle();
}


void setupPins() {
  // address 00-07 on port F (analog pins 00-07)
  DDRF = 0xff;
  PORTF = 0x00;

  // address 08-15 on port K (analog pins 08-15)
  DDRK = 0xff;
  PORTK = 0x00;

  // data 00-07 on port A (digital pins 22-29)
  DDRA = 0x00;
  PORTA = DATA_PULLUPS;

  // data 08-15 on port C (digital pins 30-37)
  DDRC = 0x00;
  PORTC = DATA_PULLUPS;

  // control lines on port L (digital pins 42-49)
  DDRL = 0xe7;
  PORTL = 0xff;

  // aux on port H (digital pins 17,16,-,6,7,8,9,-)
  // tx input, rx output (default high)
  DDRH = 0x08;
  PORTH = 0x08;

  digitalWrite(13, LOW);
}

void setupSerial() {
  // Start the serial port
  Serial.begin(250000);
  delay(100);
  Serial.println("--");
}

void loop() {
  size_t result = Serial.readBytesUntil(command_end, command_buf, command_len);
  if (result > 0) {
    split();

    //Serial.print("This just in ... ");
    //Serial.println(command_prt[0]);

    if (command_prt[0] == "idle")
      util_idle();
    else if (command_prt[0] == "reset")
      util_reset();
    else if (command_prt[0] == "sync16")
      c139_sync16();
    else if (command_prt[0] == "sync12")
      c139_sync12();
    else if ((command_prt[0] == "tick") && (command_cnt == 1))
      c139_tick();
    else if ((command_prt[0] == "tick") && (command_cnt == 2))
      c139_tick_x(fromHex(command_prt[1]));
    else if (command_prt[0] == "test")
      c139_test();
    else if (command_prt[0] == "debug") {
      DEBUG_LOG = !DEBUG_LOG;
      Serial.print("Debug Log is now ");
      Serial.println(DEBUG_LOG ? "ENABLED" : "DISABLED");
    } else if ((command_prt[0] == "memr") && (command_cnt == 2))
      c139_memr(fromHex(command_prt[1]));
    else if ((command_prt[0] == "memw") && (command_cnt == 3))
      c139_memw(fromHex(command_prt[1]), fromHex(command_prt[2]));
    else if ((command_prt[0] == "regr") && (command_cnt == 2))
      c139_regr(fromHex(command_prt[1]));
    else if ((command_prt[0] == "regw") && (command_cnt == 3))
      c139_regw(fromHex(command_prt[1]), fromHex(command_prt[2]));
    else if (command_prt[0] == "memclear")
      c139_memclear();
    else if (command_prt[0] == "memtest")
      util_memtest();
    else if (command_prt[0] == "byte")
      TX_SIZE = 32;
    else if (command_prt[0] == "word")
      TX_SIZE = 16;
    else if (command_prt[0] == "rdump")
      util_rdump();
    else if ((command_prt[0] == "dump") && (command_cnt == 2))
      util_dump(fromHex(command_prt[1]));
    else if ((command_prt[0] == "recv") && (command_cnt == 3))
      util_recv(fromHex(command_prt[1]), fromHex(command_prt[2]));
    else if ((command_prt[0] == "send_84") && (command_cnt == 5))
      util_send_84(fromHex(command_prt[1]), fromHex(command_prt[2]), fromHex(command_prt[3]), fromHex(command_prt[4]));
    else if ((command_prt[0] == "send_d") && (command_cnt == 2))
      util_send_d(fromHex(command_prt[1]));
    else if ((command_prt[0] == "send_d2") && (command_cnt == 3))
      util_send_d2(fromHex(command_prt[1]), fromHex(command_prt[2]));
    else if ((command_prt[0] == "testmode") && (command_cnt == 4))
      util_testmode(fromHex(command_prt[1]), fromHex(command_prt[2]), fromHex(command_prt[3]));
    else if (command_prt[0] == "finallap")
      util_finallap();
    else if (command_prt[0] == "suzuka8h")
      util_suzuka8h();
    else if (command_prt[0] == "ridgera2")
      util_ridgera2();
    else {
      Serial.print("unknown command '");
      Serial.print(command_prt[0]);
      Serial.print("' (");
      Serial.print(command_cnt);
      Serial.println(")");
    }
    memset(command_buf, 0, sizeof command_buf);
  }
}

void split() {
  String command_str = command_buf;
  for (byte i = 0; i < 8; i++) {
    command_prt[i] = "";
  }
  command_cnt = 0;
  while (command_str.length() > 0) {
    int index = command_str.indexOf(' ');
    if (index == -1)  // No space found
    {
      command_prt[command_cnt++] = command_str;
      break;
    } else {
      command_prt[command_cnt++] = command_str.substring(0, index);
      command_str = command_str.substring(index + 1);
    }
  }
}

short fromHex(String str) {
  char bytes[command_len];
  str.toCharArray(bytes, str.length() + 1);
  return strtol(bytes, 0, 16);
}

void util_reset() {
  c139_reset();
  c139_tick();
  c139_sync12();
  Serial.println("C139 in RESET");
}

void util_idle() {
  c139_idle();
  c139_tick();
  c139_sync12();
  Serial.println("C139 now IDLE");
}

void util_memtest() {
  bool old_debug = DEBUG_LOG;
  DEBUG_LOG = false;

  short data_1 = 0;
  short data_2 = 0;
  short data_r = 0;

  c139_idle();
  c139_tick_x(16);

  Serial.println("-- memory bit test...");
  // bit test memory
  data_1 = 0x00aa;
  data_2 = 0x0155;
  for (short addr = 0; addr < 0x2000; addr++) {
    c139_memw(addr, data_1);
    data_r = c139_memr(addr);
    if (data_r != data_1) {
      sprintf(buffer0, "failed at %04X | should be %04x, is %04x", addr, data_1, data_r);
      Serial.println(buffer0);
      DEBUG_LOG = old_debug;
      return;
    }

    c139_memw(addr, data_2);
    data_r = c139_memr(addr);
    if (data_r != data_2) {
      sprintf(buffer0, "failed at %04X | should be %04x, is %04x", addr, data_2, data_r);
      Serial.println(buffer0);
      DEBUG_LOG = old_debug;
      return;
    }
  }

  Serial.println("-- memory fill test...");
  // fill memory
  data_1 = 0x01ff;
  for (short addr = 0; addr < 0x2000; addr++) {
    c139_memw(addr, data_1);
  }
  for (short addr = 0; addr < 0x2000; addr++) {
    data_r = c139_memr(addr);
    if (data_r != data_1) {
      sprintf(buffer0, "failed at %04X | should be %04x, is %04x", addr, data_2, data_r);
      Serial.println(buffer0);
      DEBUG_LOG = old_debug;
      return;
    }
  }

  Serial.println("-- memory clear test...");
  // clear memory
  data_1 = 0x0000;
  for (short addr = 0; addr < 0x2000; addr++) {
    c139_memw(addr, data_1);
  }
  for (short addr = 0; addr < 0x2000; addr++) {
    data_r = c139_memr(addr);
    if (data_r != data_1) {
      sprintf(buffer0, "failed at %04X | should be %04x, is %04x", addr, data_2, data_r);
      Serial.println(buffer0);
      DEBUG_LOG = old_debug;
      return;
    }
  }

  Serial.println("memtest ok.");
}

void util_dump(short offs) {
  bool old_debug = DEBUG_LOG;
  DEBUG_LOG = false;

  short data[8];
  for (short addr = 0; addr < 0x80; addr += 8) {
    for (short x = 0; x < 8; x++) {
      data[x] = c139_memr(offs + addr + x);
    }
    sprintf(buffer0, "%04X | %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x", offs + addr, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    Serial.println(buffer0);
  }

  DEBUG_LOG = old_debug;
  Serial.println("done.");
}

void util_rdump() {
  bool old_debug = DEBUG_LOG;
  DEBUG_LOG = false;

  short data[8];
  for (short x = 0; x < 8; x++) {
    data[x] = c139_regr(x);
  }
  sprintf(buffer0, "%04X | %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x", 0, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  Serial.println(buffer0);

  DEBUG_LOG = old_debug;
  Serial.println("done.");
}


void util_recv(short data, short data2) {
  bool old_debug = DEBUG_LOG;
  DEBUG_LOG = false;

  rxdata[0] = 0;  // START BIT
  rxdata[1] = (data >> 8) & 1;
  rxdata[2] = (data >> 7) & 1;
  rxdata[3] = (data >> 6) & 1;
  rxdata[4] = (data >> 5) & 1;
  rxdata[5] = (data >> 4) & 1;
  rxdata[6] = (data >> 3) & 1;
  rxdata[7] = (data >> 2) & 1;
  rxdata[8] = (data >> 1) & 1;
  rxdata[9] = (data >> 0) & 1;
  rxdata[10] = 1;  // STOP BIT

  c139_sync16();

  for (byte idx = 0; idx < 11; idx++) {
    // each "bit"
    PORTH = rxdata[idx] * AUX_RX;

    for (byte cnt = 0; cnt < TX_SIZE; cnt++) {
      c139_tick_x(OSC_16MHZ_DELAY);
    }
  }

  rxdata[0] = 0;  // START BIT
  rxdata[1] = (data2 >> 8) & 1;
  rxdata[2] = (data2 >> 7) & 1;
  rxdata[3] = (data2 >> 6) & 1;
  rxdata[4] = (data2 >> 5) & 1;
  rxdata[5] = (data2 >> 4) & 1;
  rxdata[6] = (data2 >> 3) & 1;
  rxdata[7] = (data2 >> 2) & 1;
  rxdata[8] = (data2 >> 1) & 1;
  rxdata[9] = (data2 >> 0) & 1;
  rxdata[10] = 1;  // STOP BIT

  for (byte idx = 0; idx < 11; idx++) {
    // each "bit"
    PORTH = rxdata[idx] * AUX_RX;
    for (byte cnt = 0; cnt < TX_SIZE; cnt++) {
      c139_tick_x(OSC_16MHZ_DELAY);
    }
  }

  DEBUG_LOG = old_debug;
  Serial.println("done.");
  util_dump(0x1000);
}

void util_send_84(short data, short data2, short data3, short data4) {
  DEBUG_LOG = false;
  c139_memclear();

  // c139 init - regs @ 0x8f34
  c139_regw(0x00, 0);
  c139_regw(0x01, 0x0f);
  c139_regw(0x02, 0);
  c139_regw(0x03, 0);
  c139_regw(0x04, 0);
  c139_regw(0x05, 4);
  c139_regw(0x06, 0);
  c139_regw(0x07, 0);

  c139_memw(0x00, data);
  c139_memw(0x01, data2);
  c139_memw(0x02, data3);
  c139_memw(0x03, data4);

  c139_regw(0x01, 8);
  c139_tick_x(0x7fff);

  util_dump(0x1000);
}

void util_send_d(short data) {
  DEBUG_LOG = false;
  c139_memclear();

  // c139 init - regs @ 0x8f34
  c139_regw(0x00, 0);
  c139_regw(0x01, 0x0f);
  c139_regw(0x02, 0);
  c139_regw(0x03, 0);
  c139_regw(0x04, 0);
  c139_regw(0x05, 0);
  c139_regw(0x06, 0);
  c139_regw(0x07, 0);


  c139_regw(0x01, 0x0d);

  c139_memw(0x00, data);
  c139_tick();

  c139_regw(0x05, 1);
  c139_tick_x(0x7fff);

  util_dump(0x1000);
}

void util_send_d2(short data, short data2) {
  DEBUG_LOG = false;
  c139_memclear();

  // c139 init - regs @ 0x8f34
  c139_regw(0x00, 0);
  c139_regw(0x01, 0x0f);
  c139_regw(0x02, 0);
  c139_regw(0x03, 0);
  c139_regw(0x04, 0);
  c139_regw(0x05, 0);
  c139_regw(0x06, 0);
  c139_regw(0x07, 0);

  c139_regw(0x01, 0x0d);

  c139_memw(0x00, data);
  c139_memw(0x01, data2);
  c139_tick();

  c139_regw(0x05, 2);
  c139_tick_x(0x7fff);

  util_dump(0x1000);
}

void util_finallap() {
  DEBUG_LOG = false;

  // c139 init - regs @ 0x8f34
  c139_regw(0x00, 0);
  c139_regw(0x01, 0x0f);
  c139_regw(0x02, 0);
  c139_regw(0x03, 0);
  c139_regw(0x04, 0);
  c139_regw(0x05, 0);
  c139_regw(0x06, 0);
  c139_regw(0x07, 0);

  c139_regw(0x01, 0x0d);

  // c130 init - mem @ 0x8d26
  c139_memw(0x00, 0xfe);
  c139_memw(0x01, 0x01);
  c139_memw(0x02, 0x01);
  c139_memw(0x03, 0x00);
  c139_memw(0x04, 0x00);
  c139_memw(0x05, 0x00);
  c139_memw(0x06, 0x00);
  c139_memw(0x07, 0x00);
  c139_memw(0x08, 0x00);
  c139_memw(0x09, 0x00);
  c139_memw(0x0a, 0x00);
  c139_memw(0x0b, 0x00);
  c139_memw(0x0c, 0x00);
  c139_memw(0x0d, 0x00);
  c139_memw(0x0e, 0x00);
  c139_memw(0x0f, 0x00);
  c139_memw(0x10, 0x00);
  c139_memw(0x11, 0x00);
  c139_memw(0x12, 0x00);
  c139_memw(0x13, 0x100);

  c139_regr(0);

  c139_regw(0x05, 0x14);

  c139_tick_x(0x7fff);
  Serial.println("done.");
}

void util_suzuka8h() {
  DEBUG_LOG = false;
  INTCOUNT = 0;

  c139_memclear();
  for (short addr = 0; addr < 0x1000; addr++) {
    c139_memw(addr, addr >> 4);
  }

  // c139 init - regs @ 0x5736 // 57ae
  c139_regw(0x00, 0x00);    // 0
  c139_regw(0x01, 0x0f);    // 2
  c139_regw(0x02, 0x00);    // 4
  c139_regw(0x03, 0x03);    // 6
  c139_regw(0x04, 0x00);    // 8
  c139_regw(0x05, 0x00);    // A
  c139_regw(0x06, 0x1000);  // C
  c139_regw(0x07, 0x1000);  // E

  // c139 init - regs @ 0x57c6 // 57ce
  c139_regw(0x00, 0);       // 0
  c139_regw(0x01, 9);       // 2
  c139_regw(0x02, 0);       // 4
  c139_regw(0x03, 0);       // 6
  c139_regw(0x04, 0);       // 8
  c139_regw(0x05, 0);       // A
  c139_regw(0x06, 0x1000);  // C
  c139_regw(0x07, 0x1000);  // E

  INTCOUNT = 0;
  Serial.println("-- should trigger int!");
  c139_regw(0x01, c139_regr(0x01) & 0x0b);

  c139_tick_x(0x100);

  if (INTCOUNT == 0) {
    Serial.println(" something went wrong! no int ");
    return;
  }

  INTCOUNT = 0;
  Serial.println("-- int handler #1...");
  if (c139_regr(0x00) != 0x0c) {
    // should be 0x0c
    Serial.println(" something went wrong! not 0x0c ");
    return;
  }

  c139_regw(0x07, 0x900);
  c139_regw(0x05, 0x25);
  c139_regw(0x01, 0x0d);

  Serial.println("-- should send now...");
  c139_tick_x(0x7fff);
  c139_tick_x(0x7fff);

  if (INTCOUNT == 0) {
    Serial.println(" something went wrong! no int ");
    return;
  }

  Serial.println("-- int handler #2...");
  c139_regr(0x00);
  Serial.println("done.");

  Serial.println("done.");
}

void util_ridgera2() {
  DEBUG_LOG = false;
  INTCOUNT = 0;

  c139_memclear();
  for (short addr = 0; addr < 0x1000; addr++) {
    c139_memw(addr, addr);
  }

  Serial.println("-- reset?");
  // c139 init - regs @ 0x1bdbc
  c139_regw(0x00, 0x00);    // 0
  c139_regw(0x01, 0x0F);    // 2
  c139_regw(0x02, 0x00);    // 4
  c139_regw(0x03, 0x11);    // 6
  c139_regw(0x04, 0x04);    // 8
  c139_regw(0x05, 0x04);    // A
  c139_regw(0x06, 0x0000);  // C
  c139_regw(0x07, 0x0000);  // E

  Serial.println("-- init");
  // c139 init - regs @ 0x1bde0
  c139_regw(0x00, 0x00);    // 0
  c139_regw(0x01, 0x0C);    // 2
  c139_regw(0x02, 0x00);    // 4
  c139_regw(0x03, 0x11);    // 6
  c139_regw(0x04, 0x00);    // 8
  c139_regw(0x05, 0x00);    // A
  c139_regw(0x06, 0x0000);  // C
  c139_regw(0x07, 0x1000);  // E

  Serial.println("-- and-9");
  // c139 init - regs @ 0x1bdfc
  c139_regw(0x03, 0x00);
  c139_regw(0x01, c139_regr(0x01) & 0x09);

  Serial.println("-- should trigger int!");
  c139_tick_x(0x100);

  if (INTCOUNT == 0) {
    Serial.println(" something went wrong! no int ");
    return;
  }

  INTCOUNT = 0;
  Serial.println("-- int handler #1...");
  if (c139_regr(0x00) != 0x0c) {
    // should be 0x0c
    Serial.println(" something went wrong! not 0x0c ");
    return;
  }
  c139_regr(0x06);

  c139_regw(0x07, 0x00);
  c139_regw(0x03, 0x01);

  c139_regw(0x00, 0x00);

  c139_regw(0x05, 0x24);
  c139_regw(0x01, c139_regr(0x01) & 0x08);
  c139_regw(0x03, 0x00);

  Serial.println("-- should send now...");
  c139_tick_x(0x7fff);
  c139_tick_x(0x3fff);

  if (INTCOUNT == 0) {
    Serial.println(" something went wrong! no int ");
    return;
  }

  Serial.println("-- int handler #2...");
  c139_regr(0x00);
  Serial.println("done.");
}

void util_testmode(short mode, short txsize, short rxsize) {
  DEBUG_LOG = false;
  INTCOUNT = 0;

  c139_memclear();
  for (short addr = 0; addr < 0x1000; addr++) {
    c139_memw(addr, addr);
  }

  Serial.println("-- reset");
  c139_regw(0x00, 0x00);    // 0
  c139_regw(0x01, 0x0F);    // 2
  c139_regw(0x02, 0x00);    // 4
  c139_regw(0x03, 0x00);    // 6
  c139_regw(0x04, 0x00);    // 8
  c139_regw(0x05, 0x00);    // A
  c139_regw(0x06, 0x1000);  // C
  c139_regw(0x07, 0x0000);  // E
  c139_regw(0x00, c139_regr(0x00) & 0x00);

  Serial.println("-- set size & mode");
  c139_regw(0x04, rxsize);
  c139_regw(0x05, txsize);
  c139_regw(0x01, mode);
  c139_regw(0x00, c139_regr(0x00) & 0x00);

  Serial.println("-- sending should start now...");
  c139_tick_x(0x7fff);
  c139_regw(0x00, c139_regr(0x00) & 0x00);

  Serial.println("-- receiving now (with sync)...");
  util_recv(0xAA, 0x0155);
  c139_tick_x(0x7fff);
  c139_regw(0x00, c139_regr(0x00) & 0x00);

  Serial.println("-- receiving now (no sync)...");
  util_recv(0xAA, 0x0055);
  c139_tick_x(0x7fff);
  c139_regw(0x00, c139_regr(0x00) & 0x00);

  Serial.println("done.");
}
// -----------------------------------

void tx_test(byte l, byte h) {
  byte newclock = l & CTRL_16MHz;
  byte newstate = h & AUX_TX;

  // ignore repeating cycles (unless state change!)
  if (txclock == newclock && txstate == newstate)
    return;
  txclock = newclock;

  // trigger on tx low
  if (!newstate && !txactive) {
    // tx now active?
    txactive = 1;
    txhigh = 0;
    txcount = 0;
    txindex = 0;
  }

  if (newstate != txstate) {
    //Serial.print("state change: ");
    //Serial.println(txcount);
    txcount = 0;
    txhigh = 0;
  }
  txstate = newstate;

  if (txactive) {
    txcount++;
  }

  if (newstate && txactive) {
    txhigh++;
  }

  if (txhigh > 0xf0) {
    txhigh = 0;
    txactive = 0;
    //Serial.println("tx high overflow...");
  }

  if (txactive && txcount == TX_SIZE) {
    tx_add(txstate == AUX_TX ? 1 : 0);
    txcount = 0;
  }

  /*
  // trigger on tx LOW
  if (txstate != newstate) {
    if (!newstate) {
      // if we were inactive, set active!
      if (!txactive) {
        txactive = 1;
        txhigh = 0;
        txcount = 0;
        txindex = 0;
      }
    }
    if (txcount == TX_SIZE)
      tx_add(txstate == AUX_TX ? 1 : 0);

    txstate = newstate;
    txcount = 0;
  }

  if (newstate && txactive)
    txhigh++;

  if (txhigh > 0xf0) {
    txhigh = 0;
    txactive = 0;
    Serial.println("tx high overflow...");
  }

  if (txactive)
    txcount++;

  if (txcount == TX_SIZE) {
    tx_add(txstate == AUX_TX ? 1 : 0);
    txcount = 1;
  }
*/
}

void tx_add(short state) {
  if (txcount == TX_SIZE) {
    if (txindex == 0) {
      if (txstate == 0) {
        // possibly new data...
        Serial.println("tx gone active");
        txhigh = 0;
      } else {
        txhigh = 0;
        txactive = 0;
        Serial.println("tx gone inactive");
        return;
      }
    }
    if (txindex > 0 && txindex < 10) {
      txdata[txindex] = state;
      txhigh = 0;
    }
    if (txindex == 10 && txstate == AUX_TX) {
      // end stop bit...
      short data = txdata[1] << 8 | txdata[2] << 7 | txdata[3] << 6 | txdata[4] << 5 | txdata[5] << 4 | txdata[6] << 3 | txdata[7] << 2 | txdata[8] << 1 | txdata[9];
      sprintf(buffer0, "tx data = %03x", data);
      Serial.println(buffer0);
      txindex = 0;
      return;
    }
  }
  txindex++;
}

void c139_memclear() {
  for (short addr = 0; addr < 0x2000; addr++) {
    c139_memw(addr, 0);
  }
}

void c139_reset() {
  COUNT_16MHZ = 0;
  COUNT_12MHZ = 0;
  CTRL = 0xff ^ CTRL_RES;
  PORTL = CTRL;
}

void c139_idle() {
  CTRL = 0xff;
  PORTL = CTRL;
}

void c139_sync16() {
  while (!(OSC_16MHZ_TICKS == 1 && OSC_16MHZ)) {
    c139_tick();
  }
}

void c139_sync12() {
  while (!(OSC_12MHZ_TICKS == 1 && OSC_12MHZ)) {
    c139_tick();
  }
}

void c139_tick_x(short x) {
  for (short i = 0; i < x; i++) {
    c139_tick();
  }
}

void c139_tick() {
  // ticks
  OSC_12MHZ_TICKS--;
  if (OSC_12MHZ_TICKS == 0) {
    OSC_12MHZ = !OSC_12MHZ;
    OSC_12MHZ_TICKS = OSC_12MHZ_DELAY;
    if (!OSC_12MHZ)
      COUNT_12MHZ++;
  }

  OSC_16MHZ_TICKS--;
  if (OSC_16MHZ_TICKS == 0) {
    OSC_16MHZ = !OSC_16MHZ;
    OSC_16MHZ_TICKS = OSC_16MHZ_DELAY;
    if (!OSC_16MHZ)
      COUNT_16MHZ++;
  }

  // prepare output
  byte ctrl_out = CTRL;
  if (OSC_12MHZ) {
    ctrl_out ^= CTRL_12MHz;
  }
  if (OSC_16MHZ) {
    ctrl_out ^= CTRL_16MHz;
  }

  // set output
  PORTL = ctrl_out;
  delayMicroseconds(1);

  c139_test();
}

void c139_test() {
  byte h = PINH;
  byte l = PINL;
  tx_test(l, h);

  if (!(l & CTRL_IRQ)) {
    Serial.println("INTERRUPT!");
    INTCOUNT++;
  }

  if (DEBUG_LOG) {
    sprintf(buffer0, "CYCLE: %04X %04X ADDR:%02X%02X DATA:%02X%02X CTRL:[ ", COUNT_16MHZ, COUNT_12MHZ, PINK & 0x1f, PINF, PINC & 0x1f, PINA);
    Serial.print(buffer0);

    Serial.print(l & CTRL_16MHz ? "16M " : "--- ");
    Serial.print(l & CTRL_12MHz ? "12M " : "--- ");
    Serial.print(l & CTRL_DT ? "-- " : "DT ");       // active low
    Serial.print(l & CTRL_DTACK ? "--- " : "ACK ");  // active low
    Serial.print(l & CTRL_IRQ ? "--- " : "INT ");    // active low
    Serial.print(l & CTRL_RES ? "--- " : "RES ");    // active low
    Serial.print(l & CTRL_RW ? "RD " : "WT ");
    Serial.print(l & CTRL_CS ? "-- " : "CS ");  // active low
    Serial.print("] AUX:[ ");

    Serial.print(h & AUX_RX ? "RX " : "-- ");
    Serial.print(h & AUX_TX ? "TX " : "-- ");
    Serial.println("]");
  }
}

short c139_memr(short addr) {
  // get out of reset and sync 12mhz clock
  if (!(PINL & CTRL_RES))
    c139_idle();
  c139_sync12();

  // address 00-07 on port F (analog pins 00-07)
  PORTF = addr & 0xff;

  // address 08-15 on port K (analog pins 08-15)
  PORTK = (addr >> 8) & 0xff;

  // CS low, RW high, DT high
  CTRL = 0xff;
  CTRL ^= CTRL_CS;

  short data = 0;
  delayMicroseconds(1);
  while ((PINL & CTRL_DTACK)) {
    c139_tick();
    data = ((PINC & 0x1f) << 8) | PINA;
  }
  if (DEBUG_LOG) {
    sprintf(buffer0, "reading MEMORY @ %04X : %04X", addr, data);
    Serial.println(buffer0);
  }

  //c139_tick_x(M68K_READ);
  c139_sync12();
  c139_idle();
  c139_tick_x(OSC_12MHZ_DELAY * 2);

  return data;
}

void c139_memw(short addr, short data) {
  // get out of reset and sync 12mhz clock
  if (!(PINL & CTRL_RES))
    c139_idle();
  c139_sync12();

  // address 00-07 on port F (analog pins 00-07)
  PORTF = addr & 0xff;

  // address 08-15 on port K (analog pins 08-15)
  PORTK = (addr >> 8) & 0xff;

  // data 00-07 on port A (digital pins 22-29)
  DDRA = 0xff;
  PORTA = data & 0xff;

  // data 08-15 on port C (digital pins 30-37)
  DDRC = 0xff;
  PORTC = (data >> 8) & 0xff;

  // CS low, RW low, DT high
  CTRL = 0xff;
  CTRL ^= CTRL_CS;
  CTRL ^= CTRL_RW;

  delayMicroseconds(1);
  while ((PINL & CTRL_DTACK)) {
    c139_tick();
  }
  if (DEBUG_LOG) {
    sprintf(buffer0, "writing MEMORY @ %04X : %04X", addr, data);
    Serial.println(buffer0);
  }

  //c139_tick_x(M68K_WRITE);
  c139_sync12();
  c139_idle();
  DDRA = 0x00;
  PORTA = DATA_PULLUPS;
  DDRC = 0x00;
  PORTC = DATA_PULLUPS;
  c139_tick_x(OSC_12MHZ_DELAY * 2);
}

short c139_regr(short addr) {
  // get out of reset and sync 12mhz clock
  if (!(PINL & CTRL_RES))
    c139_idle();
  c139_sync12();

  // address 00-07 on port F (analog pins 00-07)
  PORTF = addr & 0xff;

  // address 08-15 on port K (analog pins 08-15)
  PORTK = (addr >> 8) & 0xff;

  // CS low, RW high, DT low
  CTRL = 0xff;
  CTRL ^= CTRL_CS;
  CTRL ^= CTRL_DT;

  short data = 0;
  delayMicroseconds(1);
  while ((PINL & CTRL_DTACK)) {
    c139_tick();
    data = ((PINC & 0x1f) << 8) | PINA;
  }
  sprintf(buffer0, "reading REGISTER @ %04X : %04X", addr, data);
  Serial.println(buffer0);

  //c139_tick_x(M68K_READ);
  c139_sync12();
  c139_idle();
  c139_tick_x(OSC_12MHZ_DELAY * 2);

  return data;
}

void c139_regw(short addr, short data) {
  // get out of reset and sync 12mhz clock
  if (!(PINL & CTRL_RES))
    c139_idle();
  c139_sync12();

  // address 00-07 on port F (analog pins 00-07)
  PORTF = addr & 0xff;

  // address 08-15 on port K (analog pins 08-15)
  PORTK = (addr >> 8) & 0xff;

  // data 00-07 on port A (digital pins 22-29)
  DDRA = 0xff;
  PORTA = data & 0xff;

  // data 08-15 on port C (digital pins 30-37)
  DDRC = 0xff;
  PORTC = (data >> 8) & 0xff;

  // CS low, RW low, DT high
  CTRL = 0xff;
  CTRL ^= CTRL_CS;
  CTRL ^= CTRL_RW;
  CTRL ^= CTRL_DT;

  delayMicroseconds(1);
  while ((PINL & CTRL_DTACK)) {
    c139_tick();
  }
  sprintf(buffer0, "writing REGISTER @ %04X : %04X", addr, data);
  Serial.println(buffer0);

  c139_sync12();
  c139_tick_x(OSC_12MHZ_DELAY * 2);
  c139_idle();
  DDRA = 0x00;
  PORTA = DATA_PULLUPS;
  DDRC = 0x00;
  PORTC = DATA_PULLUPS;
  c139_tick_x(OSC_12MHZ_DELAY * 2);
}
