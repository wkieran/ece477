#ifndef AUDIO_SD_H_
#define AUDIO_SD_H_
#include <stdio.h>
#include <fatfs.h>

#define WAV_WRITE_SAMPLE_COUNT 2048

typedef struct _WaveHeader{
	char riff[4];
	uint32_t size;
	char wave[4];
	char fmt[4];
	uint32_t fmt_size;
	uint16_t format; //1:PCM
	uint16_t channels; // channels
	uint32_t sampleRate;  // sample rate
	uint32_t rbc;//sampleRate*bitsPerSample*channels/8
	uint16_t bc; //bitsPerSample*channels/8
	uint16_t bitsPerSample; //bitsPerSample
	char data[4];
	uint32_t data_size;
} WAVE_HEADER;

void sd_card_init();
void start_recording(uint32_t frequency);
void write2wave_file(uint8_t *data, uint16_t data_size);
void stop_recording();


#endif /* AUDIO_SD_H_ */
