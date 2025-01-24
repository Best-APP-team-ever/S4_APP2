#ifndef SONG_HPP
#define SONG_HPP

// Tempo in beats-per-minute:
// #define TEMPO_BPM       120
//Varaible global
float tempo;
// Length of a 1/16 note in ms:
#define TEMPO_16T_MS(t)    ((60*250)/t)

// Note definition
// 
// 1/4  note -> noire
// 1/16 note -> double-croche
//
// Each note has a specific frequency (in Hz) and lasts a multiple of 1/16 notes
// defined by TEMPO_16T_MS (in ms).
// The tempo is defined by the length of a 1/4 note in beats per minute.
// A tempo of 60 means each 1/4 note lasts 1 second, meaning each 1/16 note lasts 250 ms.
struct Note
{
    float   freq;       // Frequency in Hz.
    int     duration;   // Duration in 1/16 notes.
};

// A song is simply defined as an array of note.
// It assumes notes are played in order (no polyphony).
// Note song[] = {
//     {164.81,    4},     // D3
//     {185.00,    4},     // F#3
//     {220.00,    4},     // A4
//     {185.00,    4}      // F#3
// };

// song to test if right notes are playing
Note song[] = {
    {440.00,    40},     // D3
    {185.00,    40},     // F#3
    {440.00,    40},     // A4
    {185.00,    40}      // F#3
};
// Twinkle, Twinkle, Little Star
// Note song[] = {
//     {130.81, 4},     // C3
//     {196.00, 4},     // G3
//     {196.00, 4},     // G3
//     {220.00, 4},     // A3
//     {196.00, 4},     // G3
//     {174.61, 4},     // F3
//     {164.81, 4},     // E3
//     {146.83, 4},     // D3
//     {130.81, 4},     // C3
//     {196.00, 4},     // G3
//     {174.61, 4},     // F3
//     {164.81, 4},     // E3
//     {146.83, 4},     // D3
//     {130.81, 4},     // C3
//     {196.00, 4},     // G3
//     {174.61, 4},     // F3
//     {164.81, 4},     // E3
//     {146.83, 4},     // D3
//     {130.81, 4},     // C3
//     {220.00, 4},     // A3
//     {196.00, 4},     // G3
//     {174.61, 4},     // F3
//     {164.81, 4},     // E3
//     {146.83, 4},     // D3
//     {130.81, 4},     // C3
//     {196.00, 4},     // G3
//     {174.61, 4},     // F3
//     {164.81, 4},     // E3
//     {146.83, 4},     // D3
//     {130.81, 4},     // C3
//     {220.00, 4},     // A3
//     {196.00, 4},     // G3
//     {174.61, 4},     // F3
//     {164.81, 4},     // E3
//     {146.83, 4},     // D3
// };

// Note frequency reference
// C3	    130.81
// C#3/Db3 	138.59	
// D3	    146.83	
// D#3/Eb3 	155.56	
// E3	    164.81
// F3	    174.61
// F#3/Gb3 	185.00
// G3	    196.00	
// G#3/Ab3 	207.65
// A3	    220.00
// A#3/Bb3 	233.08	
// B3	    246.94	

#endif
