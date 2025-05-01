
/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga16_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "sprites.h"

// Include protothreads
#include "pt_cornell_rp2040_v1_3.h"

//=====  DMA Config  ========================================

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size];

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size];

// Pointer to the address of the DAC data table
unsigned short *dac_pointer = &DAC_data[0];

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

// SPI configurations
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;

int data_chan = 0;
int ctrl_chan = 0;

//====================================================
// === the fixed point macros ========================================
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0)) // 2^15
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)(div_s64s64((((signed long long)(a)) << 15), ((signed long long)(b))))

// uS per frame
// #define FRAME_RATE 33000
#define FRAME_RATE 100000

#define LED_PIN 25

//=== ===
// Define constants
#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480
#define SCREEN_MIDLINE_X 320
// Keypad
#define BASE_KEYPAD_PIN 9
#define KEYROWS 4
#define NUMKEYS 12
#define KEYPAD2_PIN1 3
#define KEYPAD2_PIN2 4
#define KEYPAD2_PIN3 22


const fix15 GRAVITY = float2fix15(0.75);

unsigned int keycodes[12] = {0x28, 0x11, 0x21, 0x41, 0x12,
                             0x22, 0x42, 0x14, 0x24, 0x44,
                             0x18, 0x48};
unsigned int scancodes[4] = {0x01, 0x02, 0x04, 0x08};

unsigned int button = 0x70;

// Define background colors
#define GRASS_COLOR GREEN
#define DARK_GRASS DARK_GREEN
#define DIRT_COLOR DARK_ORANGE
#define STONE_COLOR ORANGE

#define GROUND_HEIGHT 60
#define GROUND_LEVEL 480 - GROUND_HEIGHT
#define GROUND_LEFT 148
#define GROUND_RIGHT 640 - 148

//=========Sound==========

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0 // 2^32 (a constant)
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
// volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// variable accumulator instead of a fixed one
// accumulator value changes based on current frequency
volatile unsigned int phase_incr_main_0;
// track the frequency (i.e. swoop/chirp)
volatile unsigned int current_frequency;
// variable to store 2^32 / Fs instead of calculating it every time
volatile unsigned int two32_fs = two32 / Fs;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size];

// Values output to DAC
int DAC_output_0;
int DAC_output_1;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1); // maximum amplitude
fix15 attack_inc;                   // rate at which sound ramps up
fix15 decay_inc;                    // rate at which sound ramps down
fix15 current_amplitude_0 = 0;      // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0;      // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
// #define ATTACK_TIME 250
#define ATTACK_TIME 250
// #define DECAY_TIME 250
#define DECAY_TIME 250
// #define BEEP_DURATION 6500
// #define BEEP_DURATION 125000
#define BEEP_DURATION 12500 // the goal is 0.125 sec

// State machine variables
volatile unsigned int count_0 = 0;

// button state: 0 = not pressed, 1 = maybe pressed, 2 = pressed, 3 = maybe not pressed
volatile unsigned int BUTTON_STATE = 0;

// Mode state: 0 = normal play sound, 1 = record sound, 2 = playback sound
volatile unsigned int MODE = 0;

// Track the pressed button
volatile unsigned int TRACKED_BUTTON = 0;

// SPI data
uint16_t DAC_data_1; // output value
uint16_t DAC_data_0; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define LDAC 8
#define LED 25
#define SPI_PORT spi0

// GPIO for timing the ISR
#define ISR_GPIO 2

// Playback array and variables.

// Max amount of sounds we can store and play back
#define max_sounds 50

unsigned int Theme_freq[32] = {587, 587, 622, 622, 440, 440, 0, 0, 175, 587, 247, 622, 440, 0, 0, 932, 1109, 1109, 622, 622, 440, 440, 0, 0, 1109, 587, 622, 247, 220, 0, 0, 294};
int Theme_id = 0;

static void play_sound()
{
  current_frequency = Theme_freq[Theme_id];

  phase_incr_main_0 = current_frequency * two32_fs;
  // DDS phase and sine table lookup
  phase_accum_main_0 += phase_incr_main_0;
  if (current_amplitude_0 > int2fix15(1))
  {
    current_amplitude_0 = int2fix15(1);
  }
  DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
                                     sin_table[phase_accum_main_0 >> 24])) +
                 2048;

  // Ramp up amplitude
  if (count_0 < ATTACK_TIME)
  {
    current_amplitude_0 = (current_amplitude_0 + attack_inc);
  }
  // Ramp down amplitude
  else if (count_0 > BEEP_DURATION - DECAY_TIME)
  {
    current_amplitude_0 = (current_amplitude_0 - decay_inc);
  }

  // Mask with DAC control bits
  DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));

  // SPI write (no spinlock b/c of SPI buffer)
  spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);

  // Increment the counter
  count_0 += 1;

  // note transition
  if (count_0 >= BEEP_DURATION)
  {
    count_0 = 0;

    Theme_id++;

    if (Theme_id >= 32)
    {
      Theme_id = 0;
    }
  }
}

// This timer ISR is called on core 0
static void alarm_irq(void)
{
  // Assert a GPIO when we enter the interrupt
  gpio_put(ISR_GPIO, 1);

  // Clear the alarm irq
  hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

  // Reset the alarm register
  timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

  play_sound();

  // De-assert the GPIO when we leave the interrupt
  gpio_put(ISR_GPIO, 0);
}

void drawGround()
{
  int groundTop = SCREEN_HEIGHT - GROUND_HEIGHT;

  fillRect(0, groundTop, SCREEN_WIDTH, GROUND_HEIGHT, BLACK);
}


static uint32_t last_update_time = 0;
static uint32_t elapsed_time_sec = 0;

static short tracked_key;

// Possible Player States: 0 = Idle, 1 = Attack, 2 = Hurt, 3 = Die

typedef struct
{
  short x;
  short y;

  bool flip;
  short state;
  short frame;
  short attack_hitbox;
  short hp;
  Anim *animations;
  
  short body;
  short shield;
} player;

typedef struct
{
  short x_off;
  short y_off;
  short w;
  short h;
} hitbox;

 // hitboxes:   0: stand body, 1: stand hit, 2: stand feet, 3: stand head, 4: drop hit, 5: crouch body, 6: crouch hit, 7: upward hit
const hitbox hitboxes[] = {{28, 160, 60, 160}, {-28, 128, 84, 56}, {28, 0, 60, 0}, {28, 160, 60, 0}, {28, 0, 60, 20}, {28, 124, 60, 124}, {-28, 28, 72, 28}, {8, 200, 32, 40}};
const short active_frames[] = {-1, 5, -1, -1};                      // active frames for each attack
short clouds_x = 320;

#define NUM_PLAYERS 2
player players[2] = {{640 / 4, GROUND_LEVEL, false, 0, 0, 0, 200, E, 0, 3}, {640 * 3 / 4, GROUND_LEVEL, true, 0, 0, 0, 200, A, 0, 3}};

void drawTitleScreen()
{
  fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE); // set background to white
  setCursor(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
  writeString("Press any key to start");
  // drawSprite(title_screen, 640, false, SCREEN_MIDLINE_X - 320, SCREEN_HEIGHT / 2 - 240, BLACK);
  // drawSprite(title_screen, 640, true, SCREEN_MIDLINE_X + 320, SCREEN_HEIGHT / 2 - 240, BLACK);
}

// Draw health bars for both players
void drawHealthBars()
{ 
  fillRect(12, 20, 4, 16, BLACK);   // left bar
  fillRect(224, 20, 4, 16, BLACK);  // right bar
  fillRect(20+players[0].hp, 28, 200-players[0].hp, 4, BLACK);  // empty part
  fillRect(20, 20, players[0].hp, 16, BLACK);  // filled part

  fillRect(412, 20, 4, 16, BLACK); // left bar 
  fillRect(624, 20, 4, 16, BLACK); // right bar
  fillRect(420, 28, 200-players[1].hp, 4, BLACK); //empty part
  fillRect(620-players[1].hp, 20, players[1].hp, 16, BLACK);  //filled part  
}

void drawShields()
{
  for(short i=0,x=12;i<4;i++, x+=52)
  {
    fillRect(x, 40, 4, 8, BLACK);
    fillRect(640-x-4, 40, 4, 8, BLACK);
  }
  for(short i=0,x=20;i<players[0].shield;i++, x+=52)
  {
    fillRect(x, 40, 40, 8, BLACK);
  }
  for(short i=0,x=580;i<players[1].shield;i++, x-=52)
  {
    fillRect(x, 40, 40, 8, BLACK);
  }
}

void eraseShields(bool p1)
{
  if(p1)
    for(short i=0,x=20;i<3;i++,x+=52)
    {
      fillRect(x, 40, 40, 8, WHITE);
    }
  else
    for(short i=0,x=476;i<3;i++,x+=52)
    {
      fillRect(x, 40, 40, 8, WHITE);
    }
}

void eraseHP(bool p1)
{
  if(p1)
  {
    fillRect(20, 20, 200, 16, WHITE);
    if(players[0].hp<0)
      players[0].hp=0;
  }
  else
  {
    fillRect(620-200, 20, 200, 16, WHITE);
    if(players[0].hp<0)
      players[0].hp=0;
  }
}

void drawLooped(const short arr[][2], short arr_len, short x, short y, char color)
{
  for (short i = 0; i < arr_len; i++)
  {
    if (x - arr[i][0]+4>640)
      fillRect(x - arr[i][0] -640, y - arr[i][1], 4, 4, color);
    else
      fillRect(x - arr[i][0], y - arr[i][1], 4, 4, color);
  }
}

void drawSprite(const short arr[][2], short arr_len, bool flip, short x, short y, char color)
{
  if (!flip)
    for (short i = 0; i < arr_len; i++)
      fillRect(x - arr[i][0], y - arr[i][1], 4, 4, color);
  else
    for (short i = 0; i < arr_len; i++)
      fillRect(x + arr[i][0], y - arr[i][1], 4, 4, color);
}

void drawFrame(player *p, char color)
{
  drawSprite(p->animations[p->state].f[p->frame].p, p->animations[p->state].f[p->frame].len, p->flip, p->x, p->y, color);
}

bool isOverlapping(short h1, short h2, short attacker)
{
  if(h1<0||h2<0)
    return false;
  // short h1x = players[attacker].x-hitboxes[h1].x_off;
  // short h1y = players[attacker].x-hitboxes[h1].y_off;

  short h1xL = players[attacker].x - hitboxes[h1].x_off;
  short h1xR = players[attacker].x + hitboxes[h1].x_off - hitboxes[h1].w;
  short h2xL = players[!attacker].x - hitboxes[h2].x_off;
  short h2xR = players[!attacker].x + hitboxes[h2].x_off - hitboxes[h2].w;

  short h1y1 = players[attacker].y - hitboxes[h1].y_off;
  short h1y2 = h1y1+hitboxes[h1].h;
  short h2y1 = players[!attacker].y - hitboxes[h2].y_off;
  short h2y2 = h2y1+hitboxes[h2].h;
  // short h1y = players[attacker].y;
  // short h2y = players[!attacker].y;

  short h1x1;
  short h1x2;
  short h2x1;
  short h2x2;

  if (!players[attacker].flip)
  {
    h1x1 = h1xL;
    h1x2 = h1xL + hitboxes[h1].w;
  }
  else
  {
    h1x1 = h1xR;
    h1x2 = h1xR + hitboxes[h1].w;
  }
  if (!players[!attacker].flip)
  {
    h2x1 = h2xL;
    h2x2 = h2xL + hitboxes[h2].w;
  }
  else
  {
    h2x1 = h2xR;
    h2x2 = h2xR + hitboxes[h2].w;
  }

  // return ((h1x>h2x && h1x<h2x+hitboxes[h2].w)||(h1x+hitboxes[h1].w>h2x && h1x<h2x+hitboxes[h2].w)) && ((h1y<h2y && h1y>h2y-hitboxes[h2].h)||(h1y-hitboxes[h1].h<h2y && h1y-hitboxes[h1].h>h2y-hitboxes[h2].h));
  bool x_overlap = (h1x1 >= h2x1 && h1x1 <= h2x2) || (h1x2 >= h2x1 && h1x2 <= h2x2) || (h2x1 >= h1x1 && h2x1 <= h1x2) || (h2x2>=h1x1 && h2x2<=h1x2);
  bool y_overlap = (h1y1 >= h2y1 && h1y1 <= h2y2) || (h1y2 >= h2y1 && h1y2 <= h2y2) || (h2y1 >= h1y1 && h2y1 <= h1y2) || (h2y2>=h1y1 && h2y2<=h1y2);
  bool r=x_overlap&&y_overlap;
  
  // if(r)
  // {
  //   drawRect(h1x1,h1y1,h1x2-h1x1,h1y2-h1y1, BLUE);
  //   drawRect(h2x1,h2y1,h2x2-h2x1,h1y2-h1y1, YELLOW);
  // }
  
  // if(x_overlap)
  //   printf("x_overlap\n");
  // if(y_overlap)
  //   printf("y_overlap:%d, %d, %d, %d\n", h1y1, h1y2, h2y1, h2y2);
  // else
  //   printf("missed y:%d, %d, %d, %d\n", h1y1, h1y2, h2y1, h2y2);
  // if(r)
  //   printf("overlap\n");
  return r;
}

#define LED 25
short getKey(bool p1)
{
  static int i;
  static uint32_t keypad;

  // Scan the keypad!
  short begin=p1?0:2;
  short end=p1?2:4;
  for (i = begin; i < end; i++)
  {
    // Set a row high
    gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                    (scancodes[i] << BASE_KEYPAD_PIN));
    // Small delay required
    sleep_us(1);
    // if(p1)
    // {
    //   printf("%d:%d,%d,%d\t",i,gpio_get(13),gpio_get(14),gpio_get(15));
    // }
    // Read the keycode
    keypad = p1?((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F):(gpio_get(KEYPAD2_PIN1) << 6)|(gpio_get(KEYPAD2_PIN2) << 5)|(gpio_get(KEYPAD2_PIN3) << 4);
    
    
    // Break if button(s) are pressed
    if (keypad & button)
      break;
  }
  // If we found a button . . .
  if (keypad & button)
  {
    // Look for a valid keycode.
    for (i = 0; i < NUMKEYS; i++)
    {
      if (keypad == keycodes[i])
        break;
    }
    // If we don't find one, report invalid keycode
    if (i == NUMKEYS)
      (i = -1);
  }
  // Otherwise, indicate invalid/non-pressed buttons
  else
    (i = -1);
  if (p1)
    return i;
  switch (i)
  {
  case 11:
    return 6;
  case 0:
    return 5;
  default:
    return i - 6;
  }
}

void handle_input_floating(short i)
{
  short tracked_key = getKey(i == 0);
  switch (tracked_key)
  {
    case 4: // left
    {
      bool overlap_before=isOverlapping(players[i].body,players[!i].body,i);
      players[i].x -= 20;
      if(players[i].x<GROUND_LEFT || (!overlap_before&&isOverlapping(players[i].body,players[!i].body,i)))
        players[i].x += 20;
      break;
    }
    case 6: // right
    {
      bool overlap_before=isOverlapping(players[i].body, players[!i].body, i);
      players[i].x += 20;
      if(players[i].x>GROUND_RIGHT || (!overlap_before&&isOverlapping(players[i].body, players[!i].body, i)))
        players[i].x -= 20;
      break;
    }
    default:
    {
      break;
    }
  }
}

void handle_input(short tracked_key, short i)
{
  switch (tracked_key)
  {
  case 4: // left
  {
    bool overlap_before=isOverlapping(players[i].body,players[!i].body,i);
    players[i].x -= 5;
    if(players[i].x<GROUND_LEFT||(!overlap_before&&isOverlapping(players[i].body,players[!i].body,i)))
      players[i].x += 5;
    players[i].state = players[i].flip ? 2 : 3;
    break;
  }
  case 6: // right
  {
    bool overlap_before=isOverlapping(players[i].body,players[!i].body,i);
    players[i].x += 5;
    if(players[i].x>GROUND_RIGHT||(!overlap_before&&isOverlapping(players[i].body,players[!i].body,i)))
      players[i].x -= 5;
    players[i].state = players[i].flip ? 3 : 2;
    break;
  }
  case 5: // down
  {
    players[i].frame = 0;
    players[i].state = 8; //crouch state;
    break;
  }
  case 3: //upward punch
  {
    players[i].frame = 0;
    players[i].state = 10; 
    break;
  }
  case 1: // attack
  {
    players[i].state = players[i].state==8?9:1; // crouch=>crouch attack, else stand attack
    players[i].frame = 0;
    break;
  }
  case 2: //jump
  {
    players[i].state = 5; // jump state
    players[i].frame = 0;
    break;
  }
  default:
  {
    players[i].state = 0; // idle state
    break;
  }
  }
}

// void ui_

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;

  fillRect(0, 0, 640, 480, WHITE); // set background to white

  // draw background at the start (initialize)
  drawSprite(rooftop, 741, false, 322, 480, BLACK);
  drawSprite(rooftop, 741, true, 318, 480, BLACK);

  while (1)
  {

    // Measure time at start of thread
    begin_time = time_us_32();

    // drawRect(80, 0 ,640-160, 480, BLACK);

    printf("returned:%d\n",getKey(false));

    players[0].body = players[0].state==11?-1:players[0].state==8||players[0].state==9? 5:0;// none if dead, crouch body if crouch OR crouch attack, stand body otherwise
    players[1].body = players[1].state==11?-1:players[1].state==8||players[1].state==9? 5:0;// none if dead, crouch body if crouch OR crouch attack, stand body otherwise
    
    drawSprite(P1, 13, false, players[0].x, players[0].y, WHITE); //erase previous frame UI
    drawSprite(P2, 15, false, players[1].x, players[1].y, WHITE); //erase previous frame UI
    // drawLooped(clouds, 479, clouds_x, 480, WHITE); //erase previous clouds
    drawLooped(clouds3, 403, clouds_x, 480, WHITE); //erase previous clouds
    clouds_x++;
    if(clouds_x>=960)
      clouds_x=320;
    for (int i = 0; i < NUM_PLAYERS; i++)
    {
      drawFrame(&players[i], WHITE); // player i, erase previous frame

      switch (players[i].state)
      {
        case 8: // crouch
        case 0: // Idle
        case 2: // Forward
        case 3: // Backward
        {
          players[i].frame++;
          if (players[i].frame >= players[i].animations[players[i].state].len)
            players[i].frame = 0;

          //start falling if above ground && feet not on the other player
          if(players[i].y<GROUND_LEVEL&&!isOverlapping(2,players[!i].body,i))
          {
            players[i].state=6;
            players[i].frame=0;
            break;
          }

          tracked_key = getKey(i == 0);
          handle_input(tracked_key, i); // get keypresses
          break;
        }
        case 1: // stand attack
        {
          players[i].frame++;
          if (players[i].frame >= players[i].animations[1].len)
            players[i].frame = 0;

          // check if attack active
          if (players[i].frame == active_frames[players[i].state])
          {
            if (isOverlapping(1, players[!i].body, i))
            {
              //check back block 
              if(players[!i].shield>0 && players[!i].state == 3)
              {
                players[!i].shield--;
                if(players[i].shield<3)
                  players[i].shield++;
                eraseShields(!i==0);
              }
              else
              {
                players[!i].frame = 0;
                players[!i].state = 4; // hurt state
                players[!i].hp -= 20;  // hurt state
                eraseHP(!i==0);
              }
            }
          }

          if (players[i].frame == 0)
            players[i].state = 0; // back to idle state
          break;
        }
        case 9: // crouch attack
        {
          players[i].frame++;
          if (players[i].frame >= players[i].animations[9].len)
          {
            players[i].frame = 0;
            players[i].state = 8; //back to crouch
          }
          // check if attack active
          if (players[i].frame == 2)
          {
            if (isOverlapping(6, players[!i].body, i)) 
            {
              //check crouch block 
              if(players[!i].shield>0 && players[!i].state == 8)
              {
                players[!i].shield--;
                if(players[i].shield<3)
                  players[i].shield++;
                eraseShields(!i==0);
              }
              else
              {
                players[!i].frame = 0;
                players[!i].state = 4; // hurt state
                players[!i].hp -= 20;  // hurt state
                eraseHP(!i==0);
              }
            }
          }
          break;
        }
        case 10: // upward attack
        {
          players[i].frame++;
          if (players[i].frame >= players[i].animations[10].len)
          {
            players[i].frame = 0;
            players[i].state = 0; //back to idle
          }
          // check if attack active
          if (players[i].frame == 3)
          {
            if (isOverlapping(7, players[!i].body, i))
            {
              players[!i].frame = 0;
              players[!i].state = 4; // hurt state
              players[!i].hp -= 20;  // hurt state
              eraseHP(!i==0);
            }
          }
          break;
        }
        case 4: // hurt state
        {
          if(players[i].hp<=0)
          {
            players[i].frame=0;
            players[i].state=11;// dead state
            break;
          }

          players[i].frame++;
          if (players[i].frame >= players[i].animations[4].len)
          {
            players[i].frame = 0;
            players[i].state = 0; // back to idle state
          }
          break;
        }
        case 5: // jump state
        {
          players[i].frame++;
          if(players[i].frame > 1)
          {
            bool overlap_before=isOverlapping(3,players[!i].body,i);//this.head vs. other.body
            players[i].y -= 40;
            bool overlap_after=isOverlapping(3,players[!i].body,i);//this.head vs. other.body
            
            if(!overlap_before&&overlap_after)
              players[i].y=players[!i].y+hitboxes[0].h;
            handle_input_floating(i);
          }
          if (players[i].frame >= players[i].animations[5].len)
          {
            players[i].frame = 0;
            players[i].state = 6; // fall state
          }
          break;
        }
        case 6: //fall state
        {
          players[i].y+=40;
          if(isOverlapping(2,players[!i].body,i)&&players[i].y<players[!i].y) //this.feet overlaps other.body && above other.feet
          {
            players[i].y=players[!i].y-hitboxes[players[!i].body].h;
            players[i].frame=0;
            players[i].state=7; // land state
            break;
          }  
          else if(players[i].y>=GROUND_LEVEL) //this.feet vs. ground
          {
            players[i].y=GROUND_LEVEL;
            players[i].frame=0;
            players[i].state=7; // land state
            break;
          }
          handle_input_floating(i);
          break;
        }
        case 7: //land state
        {
          players[i].frame++;
          if(players[i].frame==2)
          {
            if (isOverlapping(4, players[!i].body, i))
            {
              //check jump block 
              if(players[!i].shield>0 && players[!i].state == 5)
              {
                players[!i].shield--;
                if(players[i].shield<3)
                  players[i].shield++;
                eraseShields(!i==0);
                players[!i].state = 6; //fall state
                players[!i].frame = 0; //fall state
                players[i].state = 5; //jump state
                players[i].frame = 0; //jump state
              }
              else
              {
                players[!i].frame = 0;
                players[!i].state = 4; // hurt state
                players[!i].hp -= 20;  // hurt state
                eraseHP(!i==0);
              }
            }
          }
          if (players[i].frame >= players[i].animations[7].len)
          {
            players[i].frame = 0;
            players[i].state = 0; // idle state
          }
          break;
        }
        case 11: //dead state
        {
          if(players[i].frame+1<players[i].animations[11].len)
            players[i].frame++;
          //start falling if above ground && feet not on the other player
          if(players[i].y<GROUND_LEVEL&&!isOverlapping(2,players[!i].body,i))
          {
            players[i].y+=40;
            if(isOverlapping(2,players[!i].body,i)&&players[i].y<players[!i].y) //this.feet overlaps other.body && above other.feet
            {
              players[i].y=players[!i].y-hitboxes[players[!i].body].h;
            }  
            else if(players[i].y>=GROUND_LEVEL) //this.feet vs. ground
            {
              players[i].y=GROUND_LEVEL;
            }
          }
          break;
        }
        default:
        {
          break;
        }
      }

      if(players[i].state!=11)// turn towards the other player if not dead
        players[i].flip=players[i].x>players[!i].x;

      drawFrame(&players[i], BLACK); // player i, draw current frame
    }

    drawSprite(P1, 13, false, players[0].x, players[0].y, BLACK);
    drawSprite(P2, 15, false, players[1].x, players[1].y, BLACK);
    drawSprite(P1, 13, false, 264, 228, BLACK);
    drawSprite(P2, 15, false, 372, 228, BLACK);

    drawSprite(stars2, 15, false, 320, 480, BLACK);
    drawSprite(moon, 171, false, 320, 480, BLACK);
    drawLooped(clouds3, 403, clouds_x, 480, BLACK);
    drawLooped(clouds3_inside, 515, clouds_x, 480, WHITE);
    drawSprite(roof_decoration, 39, false, 324, 480, BLACK);
    drawSprite(roof_decoration, 39, true, 316, 480, BLACK);

    if (players[0].hp <= 0 || players[1].hp <= 0)
    {
      setCursor(SCREEN_WIDTH / 2 - 180, SCREEN_HEIGHT / 2 - 180);
      setTextSize(4);
      if (players[0].hp <= 0)
      {
        writeString("PLAYER 2 WINS!");
      }
      else
      {
        writeString("PLAYER 1 WINS!");
      }
    }

    // delay in accordance with frame rate
    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    if (spare_time < 0)
    {
      gpio_put(LED_PIN, 1);
    }
    else
    {
      gpio_put(LED_PIN, 0);
    }
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

// core 1
static PT_THREAD(protothread_core1(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);
  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;

  while (1)
  {
    // Measure time at start of thread
    begin_time = time_us_32();
    drawHealthBars();
    drawShields();

    spare_time = FRAME_RATE - (time_us_32() - begin_time);

    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

void core1_main()
{
  // Add animation thread
  pt_add_thread(protothread_core1);
  // Start the scheduler
  pt_schedule_start;
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
  set_sys_clock_khz(250000, true);
  // initialize stio
  stdio_init_all();

  // initialize VGA
  initVGA();

  //=============DMA

  // Initialize SPI channel (channel, baud rate set to 20MHz)
  spi_init(SPI_PORT, 20000000);

  // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Initialize the LED pin
  gpio_init(LED_PIN);
  // Configure the LED pin as an output
  gpio_set_dir(LED_PIN, GPIO_OUT);

  // Build sine table and DAC data table
  int i;
  for (i = 0; i < (sine_table_size); i++)
  {
    raw_sin[i] = (int)(2047 * sin((float)i * 6.283 / (float)sine_table_size) + 2047); // 12 bit
    DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff);
  }

  // Select DMA channels
  data_chan = dma_claim_unused_channel(true);
  ;
  ctrl_chan = dma_claim_unused_channel(true);
  ;

  // Setup the control channel
  dma_channel_config c = dma_channel_get_default_config(ctrl_chan); // default configs
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);           // 32-bit txfers
  channel_config_set_read_increment(&c, false);                     // no read incrementing
  channel_config_set_write_increment(&c, false);                    // no write incrementing
  channel_config_set_chain_to(&c, data_chan);                       // chain to data channel

  dma_channel_configure(
      ctrl_chan,                        // Channel to be configured
      &c,                               // The configuration we just created
      &dma_hw->ch[data_chan].read_addr, // Write address (data channel read address)
      &dac_pointer,                     // Read address (POINTER TO AN ADDRESS)
      1,                                // Number of transfers
      false                             // Don't start immediately
  );

  // Setup the data channel
  dma_channel_config c2 = dma_channel_get_default_config(data_chan); // Default configs
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);           // 16-bit txfers
  channel_config_set_read_increment(&c2, true);                      // yes read incrementing
  channel_config_set_write_increment(&c2, false);                    // no write incrementing
  // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
  // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
  dma_timer_set_fraction(0, 0x0017, 0xffff);
  // 0x3b means timer0 (see SDK manual)
  channel_config_set_dreq(&c2, 0x3b); // DREQ paced by timer 0
  // chain to the controller DMA channel
  // channel_config_set_chain_to(&c2, ctrl_chan);                        // Chain to control channel

  dma_channel_configure(
      data_chan,                 // Channel to be configured
      &c2,                       // The configuration we just created
      &spi_get_hw(SPI_PORT)->dr, // write address (SPI data register)
      DAC_data,                  // The initial read address
      sine_table_size,           // Number of transfers
      false                      // Don't start immediately.
  );

  // dma_start_channel_mask(1u << ctrl_chan) ;

  //================

  //============= ADC ============================
  adc_init();

  // Make sure GPIO is high-impedance, no pullups etc
  adc_gpio_init(26);

  // Select ADC input 0 (GPIO26)
  adc_select_input(0);

  //===================================================================

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_anim);

  ////////////////// KEYPAD INITS ///////////////////////
  // Initialize the keypad GPIO's
  gpio_init_mask((0x7F << BASE_KEYPAD_PIN));
  // Set row-pins to output
  gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN));
  // Set all output pins to low
  gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN));
  // Turn on pulldown resistors for column pins (on by default)
  gpio_pull_down((BASE_KEYPAD_PIN + 4));
  gpio_pull_down((BASE_KEYPAD_PIN + 5));
  gpio_pull_down((BASE_KEYPAD_PIN + 6));

  // pt_add_thread(protothread_keypad) ;

  //////////////// SOUND ////////////////
  // Initialize SPI channel (channel, baud rate set to 20MHz)
  spi_init(SPI_PORT, 20000000);
  // Format (channel, data bits per transfer, polarity, phase, order)
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  // Map SPI signals to GPIO ports
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

  // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
  gpio_init(LDAC);
  gpio_set_dir(LDAC, GPIO_OUT);
  gpio_put(LDAC, 0);

  // Setup the ISR-timing GPIO
  gpio_init(ISR_GPIO);
  gpio_set_dir(ISR_GPIO, GPIO_OUT);
  gpio_put(ISR_GPIO, 0);

  // Map LED to GPIO port, make it low
  gpio_init(LED);
  gpio_set_dir(LED, GPIO_OUT);
  gpio_put(LED, 0);

  // set up increments for calculating bow envelope
  attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME));
  decay_inc = divfix(max_amplitude, int2fix15(DECAY_TIME));

  // Build the sine lookup table
  // scaled to produce values between 0 and 4096 (for 12-bit DAC)
  int ii;
  for (ii = 0; ii < sine_table_size; ii++)
  {
    sin_table[ii] = float2fix15(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
  }

  // Enable the interrupt for the alarm (we're using Alarm 0)
  hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
  // Associate an interrupt handler with the ALARM_IRQ
  irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
  // Enable the alarm interrupt
  irq_set_enabled(ALARM_IRQ, true);
  // Write the lower 32 bits of the target time to the alarm register, arming it.
  timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

  // start scheduler
  pt_schedule_start;
}