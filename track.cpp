#include "track.h"

#include <bitset>

track::track(){
	tracklist = new int[16];
        for(int i = 0; i < 16; i++){
            tracklist[i] = 0;
        }
    _track = new AudioFile<double>[16];
}

void track::playBeat(int b){
	;
	// play beat through speaker
}

void track::writeBeat(int wd, int b, int wave){ // wave is drum number, b is beat number
    wave = wave - 1;
	int curbeat = tracklist[b];
	if(wd == 1){ // write
		curbeat = curbeat | (1 << (15 - wave)); // set bit to assign to track assignment
	}
	else if(wd == 0){ // delete
		curbeat = curbeat & (~(1 << (15 - wave))); // clear bit to remove from track assignment
	}
    tracklist[b] = curbeat;
    std::cout << "new assignment on beat " << b << std::endl;
    std::cout << std::bitset<16>(curbeat) << std::endl;
}

void track::combine(AudioFile<double>* base, AudioFile<double> newdrum){
    //AudioFile<double> base = *basep;
    int numSamp = (*base).getNumSamplesPerChannel();
    AudioFile<double>::AudioBuffer buffer;
    buffer.resize(1);
    buffer[0].resize(numSamp);
    for(int i = 0; i < numSamp; i++){
        buffer[0][i] = (*base).samples[0][i] + newdrum.samples[0][i];
    }
    (*base).setAudioBuffer(buffer);
}

void track::render(){
	//int* curbeat;
	for(int i = 0; i < 16; i++){ // loop through beat slots
		AudioFile<double> curbeat = _track[i];
        std::string filename = "beat" + std::to_string(i + 1) + ".wav";
        std::cout << "Beat: " << filename << std::endl << "                "; //PRINTING FOR DEBUG
        curbeat.load("beats/" + filename);
		for(int j = 0; j < 16; j++){ // loop through beat assignments
			if(((tracklist[i] >> (15 - j)) & 1) == 1){ // instrument goes here
                std::string drumname = "drum" + std::to_string(j + 1) + ".wav";

                std::cout << "    " << drumname; //                  PRINTING FOR DEBUG

                AudioFile<double> curdrum;
                curdrum.load("drums/" + drumname);
                combine(&curbeat, curdrum);
			}
		}
        std::cout << std::endl; //                               PRINTING FOR DEBUG
        curbeat.save("rendered/" + filename);
	}
}

track::~track(){
  delete[] tracklist;
  delete[] _track;
}