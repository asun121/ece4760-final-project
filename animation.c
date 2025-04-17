
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
//#define FRAME_RATE 33000
#define FRAME_RATE 100000

#define LED_PIN 25

//=== ===
// Define constants
#define SCREEN_WIDTH 640
#define SCREEN_MIDLINE_X 320
// Keypad 
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

const fix15 GRAVITY = float2fix15(0.75);

unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;

unsigned int button = 0x70 ;

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
}player;

typedef struct
{
  short x_off;
  short y_off;
  short w;
  short h;
}hitbox;


const hitbox hitboxes[] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}; // possible bounding hitboxes
const short active_frames[] = {0, 0, 0, 0}; // active frames for each attack
//const short frames[][][][] = {{{{0}}}}; // pixels for each possible frame

player p1 = {640/2,480/2,false,0,0,0,100};


// const short (*const *Idle)[2] = {Idle_0,Idle_1,Idle_2,Idle_3}; 

// typedef struct
// {
//   const short (*const *frames)[2]; //pointer to an array of 
//   const short *pixel_counts; //pixel counts in each frame
//   const short size; //number of frames
// }Animation;

void drawSprite(const short arr[][2], short arr_len, short scale, bool flip, short x, short y, char color)
{
  if(flip)
    for(short i=0;i<arr_len;i++)
      drawPixel(x-arr[i][0],y-arr[i][1],color);
    // for(short i=0;i<arr_len;i++)
    //   // fillRect(x-scale*arr[i][0],y-scale*arr[i][1],scale,scale,color);
    //   fillRect(x-((arr[i][0])<<2),y-((arr[i][1])<<2),scale,scale,color);
  else  
    for(short i=0;i<arr_len;i++)
      drawPixel(x+arr[i][0],y-arr[i][1],color);
    // for(short i=0;i<arr_len;i++)
    //   fillRect(x-((arr[i][0])<<2),y-((arr[i][1])<<2),scale,scale,color);
}

void drawFrame(player *p, char color)
{
  if(p->state==0) //idle
  {
    if(p->frame==0)
      drawSprite(Idle_0, 182, 4, p->flip, p->x, p->y, color);
    else if(p->frame==1)
      drawSprite(Idle_1, 184, 4, p->flip, p->x, p->y, color);
    else if(p->frame==2)
      drawSprite(Idle_2, 198, 4, p->flip, p->x, p->y, color);
    else if(p->frame==3)
      drawSprite(Idle_3, 189, 4, p->flip, p->x, p->y, color);
    else
      drawSprite(Idle_4, 191, 4, p->flip, p->x, p->y, color);
  }

  else if(p->state==1)
  {
    if(p->frame==0)
      drawSprite(E_Attack_0, 222, 4, p->flip, p->x, p->y, color);
    else if(p->frame==1)
      drawSprite(E_Attack_1, 219, 4, p->flip, p->x, p->y, color);
    else if(p->frame==2)
      drawSprite(E_Attack_2, 222, 4, p->flip, p->x, p->y, color);
    else if(p->frame==3)
      drawSprite(E_Attack_3, 208, 4, p->flip, p->x, p->y, color);
    else if(p->frame==4)
      drawSprite(E_Attack_4, 187, 4, p->flip, p->x, p->y, color);
    else if(p->frame==5)
      drawSprite(E_Attack_5, 240, 4, p->flip, p->x, p->y, color);
    else if(p->frame==6)
      drawSprite(E_Attack_6, 229, 4, p->flip, p->x, p->y, color);
    else 
      drawSprite(E_Attack_7, 209, 4, p->flip, p->x, p->y, color);

  }

  else if(p->state==2)
  {
    if(p->frame==0)
      drawSprite(Forward_0, 155, 4, p->flip, p->x, p->y, color);
    else if(p->frame==1)
      drawSprite(Forward_1, 159, 4, p->flip, p->x, p->y, color);
    else if(p->frame==2)
      drawSprite(Forward_2, 160, 4, p->flip, p->x, p->y, color);
    else if(p->frame==3)
      drawSprite(Forward_3, 165, 4, p->flip, p->x, p->y, color);
    else if(p->frame==4)
      drawSprite(Forward_4, 160, 4, p->flip, p->x, p->y, color);
    else
      drawSprite(Forward_5, 174, 4, p->flip, p->x, p->y, color);
  }

  else if(p->state==3)
  {
    if(p->frame==0)
      drawSprite(Backward_0, 160, 4, p->flip, p->x, p->y, color);
    else if(p->frame==1)
      drawSprite(Backward_1, 157, 4, p->flip, p->x, p->y, color);
    else if(p->frame==2)
      drawSprite(Backward_2, 163, 4, p->flip, p->x, p->y, color);
    else if(p->frame==3)
      drawSprite(Backward_3, 182, 4, p->flip, p->x, p->y, color);
    else if(p->frame==4)
      drawSprite(Backward_4, 168, 4, p->flip, p->x, p->y, color);
    else
      drawSprite(Backward_5, 161, 4, p->flip, p->x, p->y, color);
  }
}

// This thread runs on core 0
static PT_THREAD (protothread_core_1(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int i ;
    static uint32_t keypad ;

    while(1) {

        // gpio_put(LED, !gpio_get(LED)) ;

        // Scan the keypad!
        for (i=0; i<KEYROWS; i++) {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            // Small delay required
            sleep_us(1) ; 
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            // Break if button(s) are pressed
            if (keypad & button) break ;
        }
        // If we found a button . . .
        if (keypad & button) {
            // Look for a valid keycode.
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ;
            }
            // If we don't find one, report invalid keycode
            if (i==NUMKEYS) (i = -1) ;
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else (i=-1) ;



        // Write key to VGA
        // if (i != prev_key) {
        //     prev_key = i ;
        //     fillRect(250, 20, 176, 30, RED); // red box
        //     sprintf(keytext, "%d", i) ;
        //     setCursor(250, 20) ;
        //     setTextSize(2) ;
        //     writeString(keytext) ;
        // }

        // Print key to terminal
        tracked_key=i;
        printf("\n%d", i) ;

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

// static uint32_t keypad ;
// short getKey()
// {
//   printf("in getKey()\n");
//   // Scan the keypad!
//   for (short i = 0; i < KEYROWS; i++)
//   {
//       // Set a row high
//       gpio_put_masked((0xF << BASE_KEYPAD_PIN),
//                       (scancodes[i] << BASE_KEYPAD_PIN));
//       // Small delay required
//       sleep_us(1);
//       // Read the keycode
//       keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F);
//       // Break if button(s) are pressed
//       // if (keypad & button)
//       // {
//       //     pressed = true;
//       //     break;
//       // }
//       // else
//       // {
//       //     pressed = false;
//       // }
//   }
//   // If we found a button . . .
//   if (keypad & button)
//   {
//       // Look for a valid keycode.
//       short i;
//       for (i = 0; i < NUMKEYS; i++)
//       {
//           if (keypad == keycodes[i])
//           {
//             printf("%d(pos0)\n",i);
//             return i;
//           }
            
//               // break;
//       }
//       // If we don't find one, report invalid keycode
//       if (i == NUMKEYS)
//       {
//         printf("%d(pos1)\n",-1);
//         return -1;
//       }
//           //  (i = -1);
//   }
//   // Otherwise, indicate invalid/non-pressed buttons
//   else
//       // (i = -1);
//       {
//         printf("%d(pos2)\n",-1);
//         return -1;
//       }

  // // debounce here
  // // We only take action when transitioning between maybe pressed to pressed state - only chirp on state transition
  // switch (BUTTON_STATE)
  // {
  // case 0:
  //     if (pressed)
  //     {
  //         BUTTON_STATE = 1;
  //     }
  //     break;
  // case 1:
  //     if (pressed)
  //     {
  //         BUTTON_STATE = 2;
  //         action = true;
  //     }
  //     else
  //     {
  //         BUTTON_STATE = 0;
  //     }
  //     break;
  // case 2:
  //     if (pressed)
  //     {
  //         BUTTON_STATE = 2;
  //     }
  //     else
  //     {
  //         BUTTON_STATE = 3;
  //     }
  //     break;
  // case 3:
  //     if (pressed)
  //     {
  //         BUTTON_STATE = 2;
  //     }
  //     else
  //     {
  //         BUTTON_STATE = 0;
  //     }
  //     break;

  // default:
  //     BUTTON_STATE = 0;
  //     break;
  // }
// }

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;

  fillRect(0,0,640,480,WHITE); // set background to white

  // bool skipped0=false;
  // bool skipped1=false;

  while (1)
  {
    // if(!skipped0)
    // {
    //   skipped0=true;
    //   continue;
    // }
    // else if(!skipped1)
    // {
    //   skipped1=true;
    //   continue;
    // }
    // else
    // {
    //   skipped0=false;
    //   skipped1=false;
    // }

    // Measure time at start of thread
    begin_time = time_us_32();

    drawFrame(&p1, WHITE); //player 1, erase previous frame
    
    
    // Player 1 logic, add loop for player 2
    switch(p1.state) {
      case 0: // Idle
      {}
      case 2: // Forward
      {}
      case 3: // Backward
      {
        // tracked_key = getKey();
        // printf("%d\n",tracked_key);
        switch(tracked_key) {
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
            p1.x-=1;
            p1.state=2;
            break;
          }
          case 6: // right
          {
            p1.x+=1;
            p1.state=3;
            break;
          }
          case 1: // attack
          {
            p1.state=1; // attack state
            p1.frame=0;
            break;
          }
          default:
          {
            p1.state=0; // idle state
            break;
          }
        }
        p1.frame++;
        if(p1.state==0 && p1.frame>4||((p1.state==2||p1.state==3) && p1.frame>6))
          p1.frame=0;
        break;
      }
      case 1: // Attack
      {
        p1.frame++;
        if(p1.frame>7)
          p1.frame=0;
        if(p1.frame==0)
          p1.state=0; // back to idle state
        break;
      }
      default:
      {
        break;
      }
    }

    
    drawFrame(&p1, BLACK); //player 1, draw current frame

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

// Animation on core 1
static PT_THREAD(protothread_anim1(struct pt *pt))
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
  pt_add_thread(protothread_anim1);
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
  pt_add_thread(protothread_anim1);
  pt_add_thread(protothread_anim);

  ////////////////// KEYPAD INITS ///////////////////////
  // Initialize the keypad GPIO's
  gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
  // Set row-pins to output
  gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
  // Set all output pins to low
  gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN)) ;
  // Turn on pulldown resistors for column pins (on by default)
  gpio_pull_down((BASE_KEYPAD_PIN + 4)) ;
  gpio_pull_down((BASE_KEYPAD_PIN + 5)) ;
  gpio_pull_down((BASE_KEYPAD_PIN + 6)) ;

  pt_add_thread(protothread_core_1) ;

  // start scheduler
  pt_schedule_start;
}

