static FATFS sdCard;
static FIL testFile;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void sd_card_init() {
  //mounting an sd card
  sd_result = f_mount(&sdCard, SDPath, 1);
  if(sd_result) {
    printf("ERROR : Failed mounting an SD card %d\n", sd_result);
    while(1);
  }
  else {
    printf("SUCCESS : Mounted an SD card\n")
  }
}

void start_recording(uint32_t freq) {
  //create .wav file when button pressed
}

void process_recording(uint8_t *data, uint16_t data_size) {
  //write recording to .wav file
}

void stop_recording(){
  //close .wav file when button pressed
}