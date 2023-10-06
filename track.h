#ifndef TRACK_H
#define TRACK_H
#include "AudioFile.h"

class track{
public:
	track();
	void playBeat(int);
	void writeBeat(int, int, int);
	void render();
    void combine(AudioFile<double>*, AudioFile<double>);
	~track();
private:
    AudioFile<double> * _track;
	int* tracklist;
};

#endif