/*
  HID Keyboard for conf control on CH552g, with help from CH55xduino
  
  Flash using the bootloader (but see easier developer mode below):
  - press config button
  - unplug USB, replug USB
  - wait until end of led flashes
  - release BTN4
  - you are in flash mode

  Low level flash mode (without the bootloader):
  - press and keep RST hardware button on the PCB
  - press and keep PRG hardware button on the PCB
  - release RST button
  - wait a bit, then release PRG button

  Reach the hidden "developer mode" this way:
  - press and hold config button 4 seconds until all leds blink in a fast double flash
  - then keep the config button and press and keep one of:
    - left button: dump the current OS/meeting config via the keyboard HID
    - center button: reset the board (just keep pressing config button to end up in flash mode!)
    - right button: factory-reset the EEPROM
*/  

//For windows user, if you ever played with other HID device with the same PID C55D
//You may need to uninstall the previous driver completely        

#ifndef USER_USB_RAM
#error "This example needs to be compiled with a USER USB setting"
#endif

#define FREQ_SYS  24000000
#define XRAM_SIZE   0x0400
#define XRAM_LOC    0x0000
#define CODE_SIZE   0x3800
#define BOOT_ADDR   0x3800  // I got the address from include/bootloader.h:

#include "USBHIDKeyboard.h"

// Thresholds (milliseconds)
#define MAIN_LOOP_IDLE_MS 10
#define PRESS_QUICK   ((int16_t)25/MAIN_LOOP_IDLE_MS)
#define PRESS_LONG    ((int16_t)2000/MAIN_LOOP_IDLE_MS)
#define PRESS_RESET   ((int16_t)4000/MAIN_LOOP_IDLE_MS)
#define PRESS_DISABLE ((int16_t)40/MAIN_LOOP_IDLE_MS)

// Hardware pins for button press. BTN4 is the configuration button.

enum BtnPins
{
  BTN1_PIN= 34,
  BTN2_PIN= 14,
  BTN3_PIN= 32,
  BTN4_PIN= 11
};
typedef uint8_t BtnPin;

// Hardware pins for button leds
enum LedPins
{
  LED_PIN1= 17,
  LED_PIN2= 16,
  LED_PIN3= 15,
  //LED_PIN4= 33 // would require a jumper on the PCB
  LED_RESTORE= 254,
  LED_ALL= 255
};
typedef uint8_t LedPin;

// Constants for the main configuration (application and OS variants)
// The data are stored in a single EEPROM byte in two 4-bit fields
#define CFG_OS_SHIFT          0
#define CFG_OS_MASK           (0b11<<CFG_OS_SHIFT)

#define CFG_OS_LNX            (0b00<<CFG_OS_SHIFT)
#define CFG_OS_WIN            (0b01<<CFG_OS_SHIFT)
#define CFG_OS_MAC            (0b10<<CFG_OS_SHIFT)
#define CFG_OS_unused         (0b11<<CFG_OS_SHIFT)

#define CFG_APP_SHIFT         2
#define CFG_APP_MASK          (0b11<<CFG_APP_SHIFT)

#define CFG_APP_JITSI         (0b00<<CFG_APP_SHIFT)
#define CFG_APP_ZOOM          (0b01<<CFG_APP_SHIFT)
#define CFG_APP_SKYPE         (0b10<<CFG_APP_SHIFT)
#define CFG_APP_unused        (0b11<<CFG_APP_SHIFT)

// Uncomment to enable EEPROM
#define EEPROM_ENABLED        true
#define DEVELOPER_ENABLED     true

// Note: the CH552 is speced for up to 200 write cycles only! To implement
// wear leveling, we store the config byte in a circular ring within the EEPROM.
#define CFG_CANARY            0b11001101 // this MUST NOT be a valid config (here, bit 7 is set)
#define EEPROM_WL_SIZE        64         // length of the circular buffer in EEPROM
#define EEPROM_ADDR(a)        ((a+EEPROM_WL_SIZE)%EEPROM_WL_SIZE)

// Configuraton (application and OS), read/saved in EEPROM if EEPROM_ENABLED is true:
uint8_t configByte=            CFG_OS_WIN | CFG_APP_ZOOM;

// Lookup tables helpers
uint8_t buttonPins[]=          {BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN};
uint8_t ledPins[]=             {LED_PIN1, LED_PIN2, LED_PIN3, 0};
// How long a button was pressed
int16_t btnPressDuration[4]=   {0,0,0,0};
uint8_t btnState[4]=           {false, false, false, false};

void ledSet(LedPin ledPin, bool state);

// ===================================================================== LOW LEVEL

/**
 * The provided Arduino pause() seems broken, so here is ours:
 */
void pause(uint16_t ms)
{
  while(ms)
  {
    while( ( TKEY_CTRL & bTKC_IF ) == 0 );
    while( TKEY_CTRL & bTKC_IF );
    --ms;
  }
}

/**
 * Watchdog setup
 */
inline void wdSelect(uint8_t mode)
{
  SAFE_MOD = 0x55;
  SAFE_MOD = 0xaa;            // Enter Safe Mode
  if(mode)
    GLOBAL_CFG |= bWDOG_EN;   // Start watchdog reset
  else
    GLOBAL_CFG &= ~bWDOG_EN;  // Start watchdog only as a timer
  SAFE_MOD = 0x00;            // exit safe Mode
  WDOG_COUNT = 0;             // Watchdog assignment initial value
}

/**
 * Watchdog feed
 */
inline void wdFeed(uint8_t timReset)
{
  // 00H(6MHz)=2.8s
  // 80H(6MHz)=1.4s
  WDOG_COUNT = timReset; // Watchdog counter assignment
}

/**
 * Low level reboot into flash mode
 * This is shown by all 3 leds fading out while blinking
 */
void rebootToFlash()
{
  uint8_t t=0;
  while(t<100)
  {
    wdFeed(0);
    ledSet(LED_ALL, HIGH);
    pause(100-t);
    ledSet(LED_ALL, LOW);
    pause(t);
    t+=5;
  }
  
  EA = 0;  // Disable all interrupts
  __asm
  LJMP BOOT_ADDR // Jump to bootloader
  __endasm;
  while(1); 
}

// ===================================================================== LED AND BUTTONS

/**
 * Sets one or multiple leds. Can re-set theam according to button states.
 * LedPin is LED_PIN1, LED_PIN2, LED_PIN3, LED_ALL or LED_RESTORE
 * Use ledPins[i] for indexed access from 0-2 to LED_PIN<i>
 */
void ledSet(LedPin ledPin, bool state)
{
  if(ledPin==LED_ALL)
  {
    for(uint8_t i=0;i<3;++i)
      digitalWrite(ledPins[i], state?HIGH:LOW);
  }
  else if(ledPin==LED_RESTORE)
  {
    for(uint8_t li=0; li<3; ++li)
      ledSet(ledPins[li], btnState[li]);
  }
  else
    digitalWrite(ledPin, state?HIGH:LOW);
}

/**
 * Blink one of the leds
 * You probably want to call ledSet(LED_RESTORE) afterwards
 */
void ledBlinkFast(LedPin ledPin)
{
  ledSet(LED_ALL, false);
  for(uint8_t i=0;i<25;++i)
  {
    wdFeed(0);
    ledSet(ledPin, i&1);
    pause(15);
  }
}

/**
 * Return true when button is pressed (use BTNx_PIN, x=1,2,3,4)
 */
bool checkBtn(BtnPin btnPin)
{
  return !(digitalRead(btnPin));
}

// ===================================================================== EEPROM/CONFIG

/**
 * Clears the EEPROM
 */
void eepromReset()
{
  eeprom_write_byte(0,CFG_CANARY);
  eeprom_write_byte(1,configByte);
  for(uint8_t addr=2; addr<EEPROM_WL_SIZE; ++addr)
  {
    wdFeed(0);
    eeprom_write_byte(addr,0);
  }
}

/**
 * Check if EEPROM content is valid. Clear it is not.
 * we simply expect one and only one canary.
 */
void eepromCheck()
{
  #if EEPROM_ENABLED
    uint8_t canaries=0;
    for(uint8_t addr=0; addr<EEPROM_WL_SIZE && canaries<=1; ++addr)
    {
      wdFeed(0);
      if(eeprom_read_byte(addr)==CFG_CANARY)
        ++canaries;
    }
    if(canaries!=1)
      eepromReset();
  #endif
}

/**
 * Return the address of the canary in the EEPROM
 * (the CANARY is a byte which prefixes for the real data byte)
 */
uint8_t eepromCanaryAddress()
{
  #if EEPROM_ENABLED
    uint8_t addr= 0;
    while(addr<EEPROM_WL_SIZE-1 && eeprom_read_byte(addr) != CFG_CANARY)
      ++addr;
    return addr;
  #else
    return 0;
  #endif
}

/**
 * Read configuration from the EEPROM
 */
void configRead()
{
  eepromCheck();
  #if EEPROM_ENABLED
    uint8_t addr= eepromCanaryAddress();
    configByte= eeprom_read_byte(EEPROM_ADDR(addr+1));
  #endif
} 

/**
 * Save configuration into the EEPROM
 */
void configSave()
{
  #if EEPROM_ENABLED
    uint8_t addr= eepromCanaryAddress();
    eeprom_write_byte(addr,0); // kill the canary and move forward by one byte
    eeprom_write_byte(EEPROM_ADDR(addr+1), CFG_CANARY);  // overwrites the existing config
    eeprom_write_byte(EEPROM_ADDR(addr+2), configByte);  // write config on the next byte
  #endif
}

// ===================================================================== HELPERS

/**
 * User feedback when he configures the board:
 * - all leds blink very quickly for 1/2 second
 * - then one of the three led blinks
 *   - 5 times when configuring the OS (option 1, 2 or 3)
 *   - 3 times when configuring the application (option 1, 2 or 3)
 */
void configShow(bool kind, LedPin ledPin)
{
  ledSet(LED_ALL, false);
  pause(1000);

  uint8_t count= (kind==CFG_OS_MASK ? 11 : 7);
  while(--count)
  {
    wdFeed(0);
    ledSet(ledPin, HIGH);
    pause(250);
    ledSet(ledPin, LOW);
    pause(250);
  }  

  pause(1000);
  ledSet(LED_RESTORE, false);
}

/**
 * Reconfigure the board.
 * This is achieved by pressing btn4 followed by one of the 3 main buttons.
 */
void setConfiguration(uint8_t btnIndex)
{
  if(btnIndex>=3) return;

  if(btnPressDuration[btnIndex] < PRESS_LONG)
  {
    // config button + short press on btn 0-2: configure the application
    configByte&= ~CFG_APP_MASK;
    configByte|= ((btnIndex&0b11)<<CFG_APP_SHIFT);
    configShow(CFG_APP_MASK, ledPins[btnIndex]);
  }
  else // config button + long press: configure the OS
  {
    configByte&= ~CFG_OS_MASK;
    configByte|= ((btnIndex&0b11)<<CFG_OS_SHIFT);
    configShow(CFG_OS_MASK, ledPins[btnIndex]);
  }

  configSave();
}


/**
 * Action for short button press
 */
void buttonAction(uint8_t btnIndex)
{
  if(btnIndex>=3) return;

  // Toggle button state, and act accordingly
  bool newState= !btnState[btnIndex];
  btnState[btnIndex]= newState;

  uint8_t os=  (configByte & CFG_OS_MASK);   // CFG_OS_LNX, CFG_OS_WIN, CFG_OS_MAC
  uint8_t app= (configByte & CFG_APP_MASK);  // CFG_APP_JITSI, CFG_APP_ZOOM, CFG_APP_SKYPE

  //Keyboard_press(KEY_LEFT_CTRL);  // eg. to simulate dead keys

  // This is a dummy to show how it works
  uint8_t keyToSend= 0;
  switch(app)
  {
    case CFG_APP_JITSI:
      switch(btnIndex)
      {
        case 0:  keyToSend= (newState ? 'A' : 'a'); break; // note how A translates to Q due to _asciimap in USBHIDKeyboard.c :/
        case 1:  keyToSend= (newState ? 'B' : 'b'); break;
        case 2:  keyToSend= (newState ? 'C' : 'c'); break;
      }
      break;
    default:
      switch(btnIndex)
      {
        case 0:  keyToSend= (newState ? 'D' : 'd'); break;
        case 1:  keyToSend= (newState ? 'E' : 'e'); break;
        case 2:  keyToSend= (newState ? 'F' : 'f'); break;
      }
      break;
    /* todo ... */
  }

  if(keyToSend)
    Keyboard_write(keyToSend);

  //Keyboard_release(KEY_LEFT_CTRL);
}

// =====================================================================

/**
 * Handle button actions
 */
void handleButton(uint8_t bi)
{
  wdFeed(0);

  // Possibly debouce on->off bounces
  if(btnPressDuration[bi]<0)
  {
    ++btnPressDuration[bi];
    return;
  }

  // The small button helps configure the board
  bool configuring= checkBtn(BTN4_PIN);

  if(checkBtn(buttonPins[bi]))
  {
    // Track push duration. This helps software debouncing.
    if(btnPressDuration[bi]<32760)
      ++btnPressDuration[bi];
  }
  else // button was released (act on release)
  {
    if(bi==4) // config button
      ledSet(LED_RESTORE, false);
    else
    {
      // Only if it was pressed long enough (software debouncing)
      if(btnPressDuration[bi]>PRESS_QUICK)
      {
        if(configuring)
          setConfiguration(bi);
        else if(btnPressDuration[3]<PRESS_RESET) // we were configuring the board
          buttonAction(bi);
      }
    }
    btnPressDuration[bi]= -PRESS_DISABLE; // debounce "release" state (negative)
  }
  
  // Set button leds (no led for button 4)
  if(bi<3)
  {
    bool state= false;
    if(!configuring) // steady led state when not configuring the board
    {
      state= btnState[bi];
      //if(pressed && millis()%100<10)
      //  state=!state;  // tiny visual acknowledge that the button is depressed
    }
    else // config button if pressed
    {
      uint8_t period= 0;
      if(btnPressDuration[bi]>PRESS_LONG)
        period= 250;  // fast blink
      else if(btnPressDuration[bi]>PRESS_QUICK)
        period= 150;  // slow blink
      if(period>0)
        state= ((millis()/period)&1);
    }

    ledSet(ledPins[bi], state);
  }
}     

// ===================================================================== DEBUG STUFF

/**
 * Dumps existing config (selected OS and APP)
 * FIXME: the character table seems to be messed up (eg. '0'+n does not produce digits!)
 *   3OS=&
 *   3AP=&
 */
void configDump()
{
  uint8_t p=0;

  // Debug:
  Keyboard_write('#'); pause(p);
  Keyboard_write('O'); pause(p);
  Keyboard_write('S'); pause(p);
  Keyboard_write('='); pause(p);
  Keyboard_write('0' + ((configByte&CFG_OS_MASK)>>CFG_OS_SHIFT)); pause(p);
  Keyboard_write('\n'); pause(p);

  Keyboard_write('#'); pause(p);
  Keyboard_write('Q'); pause(p);
  Keyboard_write('P'); pause(p);
  Keyboard_write('='); pause(p);
  Keyboard_write('0' + ((configByte&CFG_APP_MASK)>>CFG_APP_SHIFT)); pause(p);
  Keyboard_write('\n');
}

void checkDevActions()
{
#if DEVELOPER_ENABLED
  if(btnPressDuration[3]<PRESS_RESET)
    return;

  // Notify the developer event with all leds blinking in a pattern (fast double blinks)
  ledSet(LED_ALL, (millis()/50 & 0b101));

  if(btnPressDuration[0]>PRESS_LONG)
  {
    // Show current configuration (left button)
    ledBlinkFast(LED_PIN1);
    configDump();
    btnPressDuration[0]= 1;
  }
  else if(btnPressDuration[1]>PRESS_LONG)
  {
    // Handle developer-friendly reset to flash mode (long press config + central button)
    // The board will reset because of the watchdog (keep config button down to enter flash mode)
    ledSet(LED_ALL, false);
    while(true)
      ledSet(LED_PIN2, (millis()/15)&1);
  }
  else if(btnPressDuration[2]>PRESS_LONG)
  {
    ledBlinkFast(LED_PIN3);
    // Reset EEPROM config
    eepromReset();
    btnPressDuration[2]= 1;
  }

#endif
} 

// =====================================================================

/**
 * Set up the board (boot)
 */
void setup()
{
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);

  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);

  // If config button is down while booting, enter flash mode (helps developing / upgrading)
  // Without this, you will need to populate and use the PRG SMT button on the PCB !
  #if DEVELOPER_ENABLED
    if(checkBtn(BTN4_PIN))
      rebootToFlash();
  #endif
 
  configRead();

  for(uint8_t round=0;round<10;++round)
  {
    for(uint8_t i=0;i<3; ++i)
    {
      ledSet(ledPins[i], true);
      pause(60-round*5);
      ledSet(ledPins[i], false);
    }
  }

  USBInit();
  pause(250);
}

/**
 * Main event loop
 */
void loop()
{
  // Watchdog reconfiguration (acts also as a watchdog feed)
  wdSelect(0x1);
  
  // Check each button
  for(uint8_t bi=0; bi<4; ++bi)
    handleButton(bi);

  checkDevActions();

  pause(MAIN_LOOP_IDLE_MS);
}

