#include "stm32f4xx_hal.h"
//#include "main.h"

//config variables
#define NUM_LED_COLUMNS (4)
#define NUM_LED_ROWS (4)
#define NUM_BTN_COLUMNS (4)
#define NUM_BTN_ROWS (4)
#define NUM_COLORS (1)

#define MAX_DEBOUNCE (3)

// Global variables
//static uint8_t LED_Buffer[16] = {0};

//static const uint16_t btncolumnpins[NUM_BTN_COLUMNS] = {Kout0_Pin, Kout1_Pin, Kout2_Pin, Kout3_Pin};
//static const uint16_t btnrowpins[NUM_BTN_ROWS]       = {Kin0_Pin, Kin1_Pin, Kin2_Pin, Kin3_Pin};
static const uint16_t btncolumnpins[NUM_BTN_COLUMNS] = {0x0001, 0x0002, 0x0004, 0x0008};
static const uint16_t btnrowpins[NUM_BTN_ROWS]       = {0x0100, 0x0080, 0x0040, 0x0010};
static const uint8_t ledcolumnpins[NUM_LED_COLUMNS] = {0, 1, 2, 3};
static const uint8_t colorpins[NUM_LED_ROWS]        = {7, 6, 5, 4};

static int8_t debounce_count[NUM_BTN_COLUMNS][NUM_BTN_ROWS];

void setuppins();

void flash(uint8_t spot);

void turnon();
void turnoff();
void poweron();
void poweroff();

void binflash(int input);

uint8_t rowscan(uint8_t, uint16_t);

uint16_t fullscan();

uint16_t waitscan();

uint16_t waitscanInt(int);

