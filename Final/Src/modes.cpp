#include "modes.h"
#include "stm32f4xx_it.h"
#include "lcd.h"
#include "sequencer.h"
#include "keypad.h"
#include "track.h"
#include <vector>
#include <string>

#define NULLTRACK 19
#define NULLBEAT 19
#define NULLDRUM 19

extern uint8_t switching;
extern int* rpress;
extern int* beatset;
extern int* rturn;
extern int debug1;
extern int debug2;
extern int debug3;

uint8_t selmode(){ // HOME MENU FOR SELECTING OPERATING MODE
	lcd_clear();
	uint8_t modem = 5;
	lcd_setquad(1);
	lcd_send_string("  SELECT  MODE  ");
	lcd_setquad(3);
	lcd_send_string("   1  2  3  4   ");
	binflash(0xF000); // flash top four buttons
	while(modem > 3){
		modem = fullscan(); // wait for user to give a mode to swtich to
	}
	return modem;
}

void mode0(sequencer &seq, TIM_HandleTypeDef *htim){ // PLAY MODE: HANDLES THE SEQUENCER AND INTERRUPT FOR PLAYING SEQUENCER BEATS. NO AUDIO YET
	lcd_clear();
	int PLAYING = 0;
	uint8_t playcheck = 0;
	int beat = 0;
	int oldtempo = 60;
	int tempo = 60;
	beatset = &beat;
	rturn = &tempo;
	uint8_t track = NULLTRACK;
	uint8_t nextrack;
	rpress = &PLAYING;

	lcd_setquad(1);
	lcd_send_string("PLAY MOD");
	lcd_setquad(2);
	lcd_send_string("E TRK XX");
	lcd_setquad(3);
	lcd_send_string("BPM  060");
	lcd_setquad(4);
	lcd_send_string(" VOL 100"); // may eliminate volume option; keep it only in editing mode but here only change speed

	/*
	 * Rendering entails going through all beat assignments for each drum and combining the audio files for each as necessary. This means taking the buffers
	 * for each drum and adding them together. There is an implementation in track.cpp which does this, but it uses the large AudioFile library. The exact same effect
	 * can be achieved if we can extract buffers on our own. In the end, there should be 16 2-second .wav files representing each beat, for EACH TRACK, so 64 2-second clips total.
	 * My current idea is to render all of them at once, saving each set of rendered beats back to the SD to then be read in real-time by the beatplayer interrupt which then plays
	 * the audio.
	 *
	 * My current design uses a folder of empty "template" beats which would be 2-second BLANK .wav files. Then, the render()/combine() functions essentially make a copy
	 * of these empty files to then "burn" the assigned drums to. So every time this operating mode renders files, it scraps the folder of "rendered" beat files and creates
	 * new beat files off of the blanks we have stored on the SD.
	 *
	seq.tracks[0].render();
	seq.tracks[1].render();
	seq.tracks[2].render();
	seq.tracks[3].render();
	*/
	binflash(0xF000); // light top row for track select

	while(track > 4){
		track = waitscan(); // wait for user to queue first track.
	}
	nextrack = track; // set next track; stays the same unless user presses button again
	lcd_setcursor(14);
	lcd_send_string("0"); // writing the track on the screen
	lcd_send_int(track + 1);

	while(switching == 0){
		for(int i = 0; i < 5; i++){
			nextrack = rowscan(nextrack, 0); // see if a track button is being pushed
		}
		debug1 = nextrack;
		debug2 = track;
		debug2 = beat;
		if((beat == 0) & (track != nextrack)){ // on beat 0, check for new track queue. Note it's zero on initialization, so it can be changed when paused.
			track = nextrack;
			lcd_setcursor(15);
			lcd_send_int(track + 1);
		}

		if (tempo != oldtempo){ // check whether tempo was changed via interrupt ("rturn" defined above)
			oldtempo = tempo;
			lcd_setcursor(21);
			if(tempo < 100){
				lcd_send_string("0"); // spacing for values less than 100
			}
			if(tempo < 10){
				lcd_send_string("0"); // spacing for values less than 10
			}
			lcd_send_int(tempo);
		}

		if(PLAYING){ // routines for when things are actually playing. Audio handling and beat playing are actually inside the htm13 interrupt in main.
			if(playcheck == 0){
				turnoff(); // cease the bitflash, now go to consecutive lighting mode
				poweron();
				HAL_TIM_Base_Start_IT(htim); // start the sequencer on first loop
				playcheck = 1;
			}
		}
		if(!PLAYING){ // if paused, stop the timer that generates beat interrupt and set beat to starting beat.
			if(playcheck == 1){
				binflash(0xF000); // light top row for track select
				HAL_TIM_Base_Stop_IT(htim);
				poweroff();
				beat = 0;
				playcheck = 0;
			}
		}
	}
	HAL_TIM_Base_Stop_IT(htim);
	turnoff();
	return;
}

void mode1(sequencer &seq){ // SEQUENCE MODE: USERS SET BIT PATTERNS FOR EACH DRUM FOR EACH TRACK. A TRACK'S "TRACKLIST" CONTAINS AN INT FOR EACH DRUM WHOSE BITS ARE ANALYZED
							// CURRENT ISSUE HERE IS THAT USING KNOB TO SWITCH TRACKS IS VERY BUGGY AND VALUES "LEAK" FROM ONE TRACK TO THE NEXT.
	lcd_clear();
	lcd_setquad(1);
	lcd_send_string("SEQUENCE MODE");
	lcd_setquad(3);
	lcd_send_string("TRACK 1 DRUM XX");


	uint16_t curdrum = NULLBEAT; // current instrument being programmed
	uint16_t asbeat; // current assignment to toggle
	int** asreg = new int*[4];
	asreg[0] = seq.tracks[0].tracklist;
	asreg[1] = seq.tracks[1].tracklist;
	asreg[2] = seq.tracks[2].tracklist;
	asreg[3] = seq.tracks[3].tracklist;
	int curtrack = 0;
	int nextrack = curtrack;
	int NEWBEAT = 0;
	rturn = &nextrack; // CHANGE CURRENT TRACK RECORDING TO WITH ENCODER
	rpress = &NEWBEAT;
	int reg;
	int rec;

	while(switching == 0){
		while(curdrum == NULLBEAT){
			if(switching == 1) break;
			binflash(0xFFFF);
			lcd_setcursor(29);
			lcd_send_string("XX");
			curdrum = waitscan();
		}
		turnoff();
		lcd_setcursor(29);
		if((curdrum + 1) < 10) lcd_send_string("0"); // update display with which drum you're assigning the beat pattern for
		lcd_send_int(curdrum + 1);
		HAL_Delay(50);
		// PLAY BEAT's SOUND HERE
		while(curdrum != NULLBEAT){
			binflash(seq.tracks[curtrack].tracklist[curdrum]); // flash new assignments to keypad
			if(NEWBEAT | switching) break;
			debug1 = curtrack;
			if(curtrack != nextrack){
				curtrack = nextrack;
				lcd_setcursor(22);
				lcd_send_int(curtrack + 1);
			}
			else{

				debug2 = curtrack; // DEBUGGING CHECKS
				debug1 = nextrack;

				HAL_Delay(110);

				asbeat = waitscanInt(asbeat); // scan for buttons to update beat assignments

				if(NEWBEAT) break;

				rec = 1 << (15 - asbeat); // isolate bit associated with that beat in the assignment register
				reg = asreg[curtrack][curdrum]; // work with the assignment register for this drum on this track
				if(switching == 1) break;

				if((asreg[curtrack][curdrum] >> (15 - asbeat)) & 0x1){ // boolean check if that beat is set or not set
					reg = reg - rec;
					(asreg[curtrack][curdrum]) = reg; // toggle beat assignment: if set, reset it
				}
				else{
					reg = reg + rec;
					asreg[curtrack][curdrum] = reg; // if it's zero, then set it
				}
				seq.tracks[curtrack].tracklist[curdrum] = reg; // place assignment register in for current drum
			}

		}
		if(NEWBEAT) curdrum = NULLBEAT;
		NEWBEAT = 0;
		if(switching == 1) break;
	}
	turnoff();
	return;
}

void mode2(){ // EDIT volume and pitch
	lcd_clear();

	uint16_t curdrum = NULLDRUM;
	uint16_t newdrum = curdrum;
	int volume = 100;
	int oldVolume = volume;
	int pitch = 0;
	int oldPitch = pitch;
	int editmode = 1;

	rpress = &editmode; // knob press changes whether you're editing volume or pitch

	curdrum = waitscan();
	newdrum = curdrum;

	if(switching == 1) return;

	while(curdrum != NULLDRUM){
		// OPERATING CODE
		curdrum = newdrum;

		// open that drum's buffer HERE

		// play that drum's sound HERE

		// once drum is selected, indicate which one on the LCD. Also write fields for Pitch (+/- X) and Volume ( X % )

		// delay?

		binflash(1 << curdrum); // only show whichever drum is selected

		while(1){
			if(editmode == 1) rturn = &pitch;
			else rturn = &volume;

			if(pitch != oldPitch){ // PITCH WAS CHANGED BY INTERRUPT
				// update screen with new value

				// manipulate buffer accordingly

				// play new sound

				oldPitch = pitch;
			}
			if(volume != oldVolume){ // VOLUME CHANGED BY INTERRUPT
				// update screen with new value

				// manipulate buffer

				// play new sound

				oldVolume = volume;
			}

			newdrum = fullscan();

			if((switching == 1) | (newdrum != NULLDRUM)) break;
		}
		if(switching == 1) break;
	}
	turnoff();
	return;
}

void mode3(){ // SET: I want to eliminate the need for this mode by incorporating an automatic recording trim
	while(switching == 0){
		// OPERATING CODE
		char * msg = "SETT MODE";

		while(1){
			if(switching == 1) break;
		}
	}
}

void mode4(){ // REC MODE: In the end, this should be a very similar architecture to EDIT mode, but instead of reading for value changes you're just selecting a button/channel and recording to it.
	char * msg = "REC MODE";
	uint16_t curdrum = NULLDRUM;
	uint16_t newdrum = curdrum;
	int recording;

	rpress = &recording; // knob press changes whether you're editing volume or pitch

	curdrum = waitscan();
	newdrum = curdrum;

	if(switching == 1) return;

	while(curdrum != NULLDRUM){
		// OPERATING CODE
		curdrum = newdrum;

		// open that drum's buffer HERE

		// play that drum's sound HERE

		// once drum is selected, indicate which one on the LCD. Also write fields for Pitch (+/- X) and Volume ( X % )

		// delay?

		binflash(1 << curdrum); // only show whichever drum is selected

		while(1){

			while(recording){
				// indicate recording on screen

			}

			newdrum = fullscan();

			if((switching == 1) | (newdrum != NULLDRUM)) break;
		}
		if(switching == 1) break;
	}
	turnoff();
	return;
}
