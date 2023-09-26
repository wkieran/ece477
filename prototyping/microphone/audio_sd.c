#include "audio_sd.h"
#include "stdio.h"
#include "fatfs.h"

static uint8_t wav_file_header[44] = {0x52, 0x49, 0x46, 0x46, 0xa4, 0xa9, 0x03, 0x00, 0x57, 0x41, 0x56, 0x45, 0x66, 0x6d,
        0x74, 0x20, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x80, 0x7d, 0x00, 0x00, 0x00, 0xf4, 0x01, 0x00,
        0x04, 0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0x80, 0xa9, 0x03, 0x00};

static int sd_result;
static FATFS sdCard;
static FIL wavFile;
static uint32_t wav_file_size;

//mount sd card
void sd_card_init() {
  //mounting an sd card
  sd_result = f_mount(&sdCard, SDPath, 1);
  if(sd_result) {
    printf("ERROR : Failed mounting an SD card %d\n", sd_result);
    while(1);
  }
  else {
    printf("SUCCESS : Mounted an SD card\n");
  }
}

//create .wav file when button pressed
void start_recording(uint32_t freq) {
  uint8_t file_name[] = "test.txt";
  uint8_t temp_num;
  uint8_t test_text[] = "This is a test text.";

  uint32_t byte_rate = freq * 2 * 2;
  wav_file_header[24] = (uint8_t)freq;
  wav_file_header[25] = (uint8_t)(freq >> 8);
  wav_file_header[26] = (uint8_t)(freq >> 16);
  wav_file_header[27] = (uint8_t)(freq >> 24);
  wav_file_header[28] = (uint8_t)byte_rate;
  wav_file_header[29] = (uint8_t)(byte_rate >> 8);
  wav_file_header[30] = (uint8_t)(byte_rate >> 16);
  wav_file_header[31] = (uint8_t)(byte_rate >> 24);

  //define file name
  file_name[4] = file_num_digit % 10 + 48;
  file_num_digit /= 10;
  file_name[3] = file_num_digit % 10 + 48;
  file_num_digit /= 10;
  file_name[2] = file_num_digit % 10 + 48;
  printf("file name is %s.\n", file_name);
  file_counter++;

  //create file
  sd_result = f_open(&wavFile, (void*) file_name, FA_WRITE|FA_CREATE_ALWAYS);
  if(sd_result) {
    printf("ERROR : Failed creating file on an SD card %d\n", sd_result);
    while(1);
  }
  else {
    printf("SUCCESS : Opened file\n");
  }

  wav_file_size = 0;
}

//write recording to .wav file
void process_recording(uint8_t *data, uint16_t data_size) {
  uint16_t temp_num;

  printf("w");

  if(first_time == 0) {
    for(int i = 0; i < 44; i++) {
      *(data + i) = wav_file_header[i];
    }
    first_time = 1;
  }

  sd_result = f_write(&wavfile, (void*)data, data_size, (UINT*)&temp_num);
  if(sd_result) {
    printf("ERROR : can't wirte to the file %d \n", sd_result);
    while(1);
  }

  wav_file_size += data_size;
}

//close .wav file when button pressed
void stop_recording(){
  uint16_t temp_num;

  //update data size
  wav_file_size -= 8;
  wav_file_header[4] = (uint8_t)wav_file_size;
  wav_file_header[5] = (uint8_t)(wav_file_size >> 8);
  wav_file_header[6] = (uint8_t)(wav_file_size >> 16);
  wav_file_header[7] = (uint8_t)(wav_file_size >> 24);
  wav_file_size -= 36;
  wav_file_header[40] = (uint8_t)wav_file_size;
  wav_file_header[41] = (uint8_t)(wav_file_size >> 8);
  wav_file_header[42] = (uint8_t)(wav_file_size >> 16);
  wav_file_header[43] = (uint8_t)(wav_file_size >> 24);

  //move to the beginning of the file to update file format
  f_lseek(&wavFile, 0);
  f_write(&wavFile, (void*) wav_file_header, sizeof(wav_file_header), (UINT*) &temp_num);
  if(sd_result) {
    printf("ERROR : can't update the first sector %d \n", sd_result);
    while(1);
  }
  f_close(&wavFile);
  first_time = 0;
  printf("closed the file \n");
}
