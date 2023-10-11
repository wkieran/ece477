#ifndef SEQUENCER_H
#define SEQUENCER_H
#include "track.h"

class sequencer{
public:
    track* tracks;
	sequencer();
	void csetup(); // set up clock for sequencer
	bool playing(); // check whether sequencer is meant to be playing
	void start();
	void stop();
	void tick(); // handler for sequencer ticks; responsible for playing
	void queue(int); // queue another track to play at next loop
	void checkq(); // move to next queued track (for use at end of loop)
	void tscale(); // scale tempo for sequencer clock
  track* gettrack(int);

	~sequencer();

private:
	int beat;
	bool isplaying;
	int curtrack;
	int nextrack;
};

#endif