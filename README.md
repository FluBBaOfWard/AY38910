# AY38910 V0.6.5

An AY-3-8910 / YM2149 sound chip emulator for ARM32.

## How to use

First alloc chip struct, call reset then set in/out function pointers.
Call AY38910Mixer with length, destination and chip struct.
Produces signed 16bit mono.

The "in function" can take 2 arguments, the first is a possible output value
on the IO ports of the chip, the second argument is if the ports is in output
mode. Something like "ioACallback(u8 val, int inout)", if "inout" is not zero,
val is the current value output by the chip.   
You can define AY_UPSHIFT to a number, this is how many times the internal
sampling is doubled. You can add "-DAY_UPSHIFT=2" to the "make" file to
make the internal clock speed 4 times higher.
You can also define AYFILTER to a value between 0 & 8 or so to filter out
higher frequencies, default is 1.

## Projects that use this code

* <https://github.com/FluBBaOfWard/BlackTigerDS> (YM2203)
* <https://github.com/FluBBaOfWard/DoubleDribbleDS> (YM2203)
* <https://github.com/FluBBaOfWard/GhostsNGoblinsDS> (YM2203)
* <https://github.com/FluBBaOfWard/IronHorseDS> (YM2203)
* <https://github.com/FluBBaOfWard/S8DS>
* <https://github.com/FluBBaOfWard/SonSonDS>
* <https://github.com/FluBBaOfWard/SonSonGBA>
* <https://github.com/FluBBaOfWard/YM2203>
* <https://github.com/wavemotion-dave/ColecoDS>

## Credits

Fredrik Ahlström

X/Twitter @TheRealFluBBa

<https://www.github.com/FluBBaOfWard>
