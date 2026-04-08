## Author: Ramesh Yerraballi, Jonathan Valvano
## Use this in octave to convert a wav file into raw binary
##  samples with a chosen precision at 11.025kHz
## You will need to load the signal package first to use  the decimate function
## You can load it like this on the octave prompt:
##    pkg load signal
## Run in octave as:
##    WCbin ("filename", precision);
## The inputs are (i) the name of the wav file without the extention .wav.
##                (ii) the sample precision in bits (e.g., 4 to 8 bits)
## This will generate a filename.bin file that has raw binary data
## For example, say you have a sound file called boom.wav that you want to use in
## in your game.
##    1. Put the WCbin.m script and the boom.wav file in a folder (say sounds).
##    2. Open octave/matlab and cd to the sounds folder
##    3. Run script as: WCbin("boom", 8)
##       For 8-bit precision
##       This will produce a file named boom.bin, which contains raw samples
##       You can put the file on an SDC and read the data at run time
function WCbin(filename,precision)
  # load the file
  [SplsOrig, fs] = audioread(strcat(filename,'.wav'));
  # downsample to 11.025kHz
  Spls = decimate(SplsOrig,round(fs/11025));
  # trim the precision of each sample
  Spls = round((Spls+ 1)* (2^precision-1)/2);
  # write raw binary to file with bin extension
  file = fopen(strcat(filename,'.bin'), 'w');
  fwrite(file, Spls(1:length(Spls)));
  fclose(file);
end
