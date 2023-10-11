#include <stdio.h>
#include <iostream>

#include <math.h>
#include "AudioFile.h"
#include "sequencer.h"
#include "track.h"

int main(){
    /*
    AudioFile<double> drum1;
    drum1.load("example.wav");
    int BitDepth = drum1.getBitDepth();
    int SampleRate = drum1.getSampleRate();
    int NumChannels = drum1.getNumChannels();

    std::cout << "Bit Depth: " << BitDepth << std::endl;
    std::cout << "Sample Rate: " << SampleRate << std::endl;
    std::cout << "Num Channels? " << NumChannels << std::endl;
    */

    sequencer seq;
    
    for(int i = 0; i < 16; i += 4){
        seq.tracks[0].writeBeat(1, i, 1); // write Drum 1 (kick) to every fourth beat
    }
    for(int i = 0; i < 16; i++){
        seq.tracks[0].writeBeat(1, i, 2); // write Drum 2 (hihat) to every beat
    }
    for(int i = 2; i < 16; i += 4){
        seq.tracks[0].writeBeat(1, i, 3); // write Drum 3 (snare) to every fourth beat (offset)
    }
    seq.tracks[0].writeBeat(1, 12, 3); // write Drum 3 (snare) to beat 12 (all 3 drums on one beat)
    seq.tracks[0].writeBeat(0, 12, 3);
    seq.tracks[0].writeBeat(1, 12, 4);
    seq.tracks[0].writeBeat(1, 8, 5);
    seq.tracks[0].writeBeat(0, 8, 2);
    seq.tracks[0].render();

}