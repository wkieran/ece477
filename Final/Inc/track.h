#ifndef TRACK_H
#define TRACK_H
//#include "AudioFile.h"

class track{
public:
	track();
	void playBeat(int);
	void writeBeat(int, int, int);
	void render();
    //void combine(AudioFile<double>*, AudioFile<double>);
	~track();
	int* tracklist;
    //AudioFile<double> * _track;
};

#endif
