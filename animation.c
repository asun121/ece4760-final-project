
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
#include "sprites.h"

// Include protothreads
#include "pt_cornell_rp2040_v1_3.h"

//===== Images ==============================================

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

const fix15 GRAVITY = float2fix15(0.75);

unsigned int keycodes[12] = {0x28, 0x11, 0x21, 0x41, 0x12,
                             0x22, 0x42, 0x14, 0x24, 0x44,
                             0x18, 0x48};
unsigned int scancodes[4] = {0x01, 0x02, 0x04, 0x08};

unsigned int button = 0x70;

// Define background colors
#define GRASS_COLOR     GREEN
#define DARK_GRASS      DARK_GREEN
#define DIRT_COLOR      DARK_ORANGE
#define STONE_COLOR     ORANGE

#define GROUND_HEIGHT 80

void drawGround()
{
  int groundTop = SCREEN_HEIGHT - GROUND_HEIGHT;

  fillRect(0, groundTop, SCREEN_WIDTH, GROUND_HEIGHT, BLACK);
}

// Draw Background with only ground and clouds
void drawBackground() {
    // Main ground platform - positioned below where players stand (3/4 of screen height)
    int groundTop = SCREEN_HEIGHT - 110;
    
    // Draw main grass layer
    fillRect(0, groundTop, SCREEN_WIDTH, 30, DARK_GRASS);
    
    // Draw dirt layer below grass
    // fillRect(0, groundTop + 30, SCREEN_WIDTH, 80, DIRT_COLOR);
    fillRect(0, groundTop + 30, SCREEN_WIDTH, 80, DIRT_COLOR);
    
    // Draw textured grass on top (small varied tufts)
    for (int i = 0; i < 60; i++) {
        int grassX = 10 + i * 11;
        int height = 5 + (i % 3) * 2; // Varied heights
        fillRect(grassX, groundTop - height, 3, height, DARK_GRASS);
    }
    
    // Add some stones/rocks scattered on the ground
    for (int i = 0; i < 12; i++) {
        int stoneX = 50 + i * 50;
        int stoneY = groundTop + 10 + (i % 3) * 5;
        int stoneSize = 4 + (i % 4) * 2;
        fillCircle(stoneX, stoneY, stoneSize, STONE_COLOR);
    }
    
    // Draw some ground texture/patterns
    for (int i = 0; i < 20; i++) {
        int x = 30 + i * 30;
        // Darker patches of dirt
        fillRect(x, groundTop + 35, 15, 10, YELLOW);
    }
    
    // Draw ground edge detail (slightly darker line at the top edge)
    drawLine(0, groundTop, SCREEN_WIDTH, groundTop, DARK_GRASS);
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
} player;

typedef struct
{
  short x_off;
  short y_off;
  short w;
  short h;
} hitbox;

// const hitbox hitboxes[] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}; // possible bounding hitboxes
// const hitbox hitboxes[] = {{12,40,25,40}};
//{-7*4,32*4,(-7--28)*4 (18-32)*4}
const hitbox hitboxes[] = {{28,160,60,160},{-28, 128, 84, 56}}; // 0: Default hitbox, 1:
const short active_frames[] = {-1, 5, -1, -1};                                         // active frames for each attack

#define NUM_PLAYERS 2

// player p1 = {640 / 2, 480 / 2, false, 0, 0, 0, 100, default_character};
// player p2 = {500, 480 / 2, true, 0, 0, 0, 100, default_character};

// player *players[2] = {&p1, &p2}; // array of players
player players[2] = {{640/4, SCREEN_HEIGHT - GROUND_HEIGHT, true, 0, 0, 0, 100, E},{640*3/4, SCREEN_HEIGHT - GROUND_HEIGHT, false, 0, 0, 0, 100, A}};

// typedef struct
// {
//   const short (*const *frames)[2]; //pointer to an array of
//   const short *pixel_counts; //pixel counts in each frame
//   const short size; //number of frames
// }Animation;


// Draw health bars for both players
void drawHealthBars() {
  char hp_text[10]; // Buffer for HP text
  
  // Player 1 health bar - left side
  fillRect(20, 20, 200, 20, WHITE); 
  fillRect(20, 20, players[0].hp * 2, 20, BLACK); 

  // Clear previous text area before writing new text
  fillRect(230, 25, 50, 10, WHITE); // Clear P1 text area
  
  // Display Player 1 (P1) HP value
  sprintf(hp_text, "HP: %d", players[0].hp);
  setTextSize(1);
  setCursor(230, 25); 
  writeString(hp_text);
  
  // Player 2 (P2) health bar - right side
  fillRect(420, 20, 200, 20, WHITE); 
  fillRect(420, 20, players[1].hp * 2, 20, BLACK);

  // Clear previous text area
  fillRect(370, 25, 50, 10, WHITE); // Clear P2 text area
  
  // Display Player 2 HP value
  sprintf(hp_text, "HP: %d", players[1].hp);
  setTextSize(1);
  setCursor(370, 25); // left of P2 health bar
  writeString(hp_text);
}


void drawSprite(const short arr[][2], short arr_len, bool flip, short x, short y, char color)
{
  if (flip)
    // for (short i = 0; i < arr_len; i++)
    //   drawPixel(x - arr[i][0], y - arr[i][1], color);
    for(short i=0;i<arr_len;i++)
  //   // fillRect(x-scale*arr[i][0],y-scale*arr[i][1],scale,scale,color);
      fillRect(x-((arr[i][0])<<2),y-((arr[i][1])<<2),4,4,color);
  else
    // for (short i = 0; i < arr_len; i++)
    //   drawPixel(x + arr[i][0], y - arr[i][1], color);
    for(short i=0;i<arr_len;i++)
      fillRect(x+((arr[i][0])<<2),y-((arr[i][1])<<2),4,4,color);
}

void drawFrame(player *p, char color)
{
  // drawRect(p->x - ((hitboxes[0].x_off)), p->y - ((hitboxes[0].y_off)), ((hitboxes[0].w)), ((hitboxes[0].h)), GREEN);
  // if(p->flip)
  //   drawRect(p->x - ((hitboxes[1].x_off)), p->y - ((hitboxes[1].y_off)), ((hitboxes[1].w)), ((hitboxes[1].h)), RED);
  // else
  //   drawRect(p->x + ((hitboxes[1].x_off-hitboxes[1].w)), p->y - ((hitboxes[1].y_off)), ((hitboxes[1].w)), ((hitboxes[1].h)), RED);
  drawSprite(p->animations[p->state].f[p->frame].p, p->animations[p->state].f[p->frame].len, p->flip, p->x, p->y, color);
}

bool isOverlapping(short h1, short h2, short attacker)
{
  // short h1x = players[attacker].x-hitboxes[h1].x_off;
  // short h1y = players[attacker].x-hitboxes[h1].y_off;

  short h1xL = players[attacker].x-hitboxes[h1].x_off;
  short h1xR = players[attacker].x+hitboxes[h1].x_off-hitboxes[h1].w;
  short h2xL = players[!attacker].x-hitboxes[h2].x_off;
  short h2xR = players[!attacker].x+hitboxes[h2].x_off-hitboxes[h2].w;

  short h1y = players[attacker].y-hitboxes[h1].y_off;
  short h2y = players[!attacker].y-hitboxes[h2].y_off;

  short h1x1;
  short h1x2;
  short h2x1;
  short h2x2;

  if (players[attacker].flip)
  {
    h1x1 = h1xL;
    h1x2 = h1xL+hitboxes[h1].w;
  }
  else
  {
    h1x1 = h1xR;
    h1x2 = h1xR+hitboxes[h1].w;
  }
  if (players[!attacker].flip)
  {
    h2x1 = h2xL;
    h2x2 = h2xL+hitboxes[h2].w;
  }
  else
  {
    h2x1 = h2xR;
    h2x2 = h2xR+hitboxes[h2].w;
  }
  
  // drawRect(h1x1,h1y,hitboxes[h1].w,hitboxes[h1].h, BLUE);
  // drawRect(h2x1,h2y,hitboxes[h2].w,hitboxes[h2].h, YELLOW);
  
  // return ((h1x>h2x && h1x<h2x+hitboxes[h2].w)||(h1x+hitboxes[h1].w>h2x && h1x<h2x+hitboxes[h2].w)) && ((h1y<h2y && h1y>h2y-hitboxes[h2].h)||(h1y-hitboxes[h1].h<h2y && h1y-hitboxes[h1].h>h2y-hitboxes[h2].h));
  return ((h1x1>h2x1 && h1x1<h2x2)||(h1x2>h2x1 && h1x2<h2x2)) && ((h1y<h2y && h1y>h2y-hitboxes[h2].h)||(h1y-hitboxes[h1].h<h2y && h1y-hitboxes[h1].h>h2y-hitboxes[h2].h));
}


#define LED 25
short getKey(bool p1)
{
  static int i;
  static uint32_t keypad;

  // Scan the keypad!
  for (i = 0; i < KEYROWS; i++)
  {
    // Set a row high
    gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                    (scancodes[i] << BASE_KEYPAD_PIN));
    // Small delay required
    sleep_us(1);
    // Read the keycode
    keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F);
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
  gpio_put(LED, i == -1 ? false : true);
  if(p1)
    return i;
  switch(i)
  {
    case 11:
      return 6;
    case 0:
      return 5;
    default:
      return i-6;
  }
}

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
  drawGround();
  // drawBackground();

  // bool skipped0=false;
  // bool skipped1=false;

  while (1)
  {

    // Measure time at start of thread
    begin_time = time_us_32();

    // slow approach - redraw background every frame
    // drawBackground(); 

    for (int i = 0; i < NUM_PLAYERS; i++)
    {
      drawFrame(&players[i], WHITE); // player 1, erase previous frame
      // Player 1 logic, add loop for player 2
      switch (players[i].state)
      {
      case 0: // Idle
      case 2: // Forward
      case 3: // Backward
      {
        tracked_key = getKey(i==0);
        //printf("%d\n", tracked_key);
        switch (tracked_key)
        {
        // case 2: // up
        // {
        //   p1.y-=1;
        //   break;
        // }
        // case 5: // down
        // {
        //   p1.y+=1;
        //   break;
        // }
        case 4: // left
        {
          players[i].x -= 5;
          players[i].state = players[i].flip?3:2;
          break;
        }
        case 6: // right
        {
          players[i].x += 5;
          players[i].state = players[i].flip?2:3;
          break;
        }
        case 1: // attack
        {
          players[i].state = 1; // attack state
          players[i].frame = 0;
          break;
        }
        default:
        {
          players[i].state = 0; // idle state
          break;
        }
        }
        players[i].frame++;
        // if (players[i].state == 0 && players[i].frame > 4 || ((players[i].state == 2 || players[i].state == 3) && players[i].frame > 6))
        if(players[i].frame >= players[i].animations[players[i].state].len)
          players[i].frame = 0;
        break;
      }
      case 1: // Attack
      {
        players[i].frame++;
        if (players[i].frame > 7)
          players[i].frame = 0;
      
        //check if attack active
        if(players[i].frame == active_frames[players[i].state])
        {

          if(isOverlapping(1,0,i)) 
          {
            players[!i].frame = 0;
            players[!i].state = 4; // hurt state
            players[!i].hp -= 10; // hurt state

          }
        }

        if (players[i].frame == 0)
          players[i].state = 0; // back to idle state
        break;
      }
      case 4: // hurt state
      {
        players[i].frame++;
        if (players[i].frame > 3)
          players[i].frame = 0;
        if (players[i].frame == 0)
          players[i].state = 0; // back to idle state
        break;
      }
      default:
      {
        break;
      }
      }

      drawFrame(&players[i], BLACK); // player 1, draw current frame
    }


    if (players[0].hp <= 0 || players[1].hp <= 0) {
      setCursor(SCREEN_WIDTH/2 - 200, SCREEN_HEIGHT/2 - 200);
      setTextSize(4);
      if (players[0].hp <= 0) {
        writeString("PLAYER 2 WINS!");
      } else {
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

  // start scheduler
  pt_schedule_start;
}
