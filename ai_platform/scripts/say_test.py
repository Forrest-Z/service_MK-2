'''
Created on 2012. 2. 19.
This module is for playing mp3 (limited) and wav formatted audio file
@author: John
'''
import pygame
import time
 
def playsound(soundfile):
    """Play sound through default mixer channel in blocking manner.
       This will load the whole sound into memory before playback
    """   
    pygame.init()
    pygame.mixer.init()
    sound = pygame.mixer.Sound(soundfile)
    time.sleep(2)
    clock = pygame.time.Clock()
    sound.play()
    while pygame.mixer.get_busy():
        print("Playing... - func => playsound")
        clock.tick(1000)
 
def playmusic(soundfile):
    """Stream music with mixer.music module in blocking manner.
       This will stream the sound from disk while playing.
    """
    pygame.init()
    pygame.mixer.init()
    clock = pygame.time.Clock()
    pygame.mixer.music.load(soundfile)
    print(pygame.mixer.music)
    time.sleep(1.5)
    pygame.mixer.music.play()
    # time.sleep(1)
    print("play")
    while pygame.mixer.music.get_busy():
        print("Playing... - func => playingmusic")
        clock.tick(1000)
         
 
def stopmusic():
    """stop currently playing music"""
    pygame.mixer.music.stop()
 
def getmixerargs():
    pygame.mixer.init()
    freq, size, chan = pygame.mixer.get_init()
    return freq, size, chan
 
 
def initMixer():
    BUFFER = 3072  # audio buffer size, number of samples since pygame 1.8.
    FREQ, SIZE, CHAN = getmixerargs()
    pygame.mixer.init(FREQ, SIZE, CHAN, BUFFER)
 
def play_say(say_pygame,clock):
    say_pygame.play()
    while pygame.mixer.get_busy():
        print("Playing... - func => playingmusic")
        clock.tick(1000)

    print("playmusic")
    return 'done'

 
'''You definitely need test mp3 file (a.mp3 in example) in a directory, say under 'C:\\Temp'
   * To play wav format file instead of mp3,
      1) replace a.mp3 file with it, say 'a.wav'
      2) In try except clause below replace "playmusic()" with "playsound()"
     
'''
                     
try:
    initMixer()
    filename = './sound/hello.mp3'
    pygame.init()
    pygame.mixer.init()
    clock = pygame.time.Clock()
    hello_sound = pygame.mixer.Sound(filename)

    time.sleep(2)

    print("init_done")

    # play_say(hello_sound,clock)
    playsound(filename)
except KeyboardInterrupt:   # to stop playing, press "ctrl-c"
    stopmusic()
    print("\nPlay Stopped by user")

    print("unknown error")
     
print("Done")
