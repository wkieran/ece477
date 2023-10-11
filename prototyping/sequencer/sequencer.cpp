#include "sequencer.h"

sequencer::sequencer(){
	isplaying = false;
	tracks = new track[4];
	curtrack = 1;
	nextrack = 1;
	beat = 0;
}

track* sequencer::gettrack(int i){
  return &tracks[i];
}

bool sequencer::playing(){
	return isplaying;
}

void sequencer::csetup(){
	// configure clock; on interrupt calls the tick handler.
}

void sequencer::tick(){
	/* retrieve current track (tracks[curtrack - 1])
	 * play beat X on that track through track's playBeat method
	 * light up LED X on button grid to indicate which beat is playing
	 * increment beat int
	 */

	track cur = tracks[curtrack - 1];
	cur.playBeat(beat);
	// light up beat button
	++beat;
}

void sequencer::queue(int next){
	nextrack = next;
}

void sequencer::checkq(){
	if(curtrack != nextrack) curtrack = nextrack;
}

void sequencer::start(){
	isplaying = true;
}

void sequencer::stop(){
	isplaying = false;
}

sequencer::~sequencer(){
	delete[] tracks;
}