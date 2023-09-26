#ifndef INC_AUDIO_SD_H
#define INC_AUDIO_SD_H

void sd_card_init();
void start_recording(uint32_t freq);
void process_recording(uint8_t *data, uint16_t data_size);
void stop_recording();

#endif /* INC_AUDIO_SD_H */