# EasyBeacon

EasyBeacon is a work in progress based on the work of many\
and inspired by Bo OZ2M.

Fast PI4 Switching the ADF4350 had locking errors causing\
random suprious signals. 

The lastest version of the software. PI7ALK_ADF43XX_PWM_V1.0\
just uses PWM to create the four PI4 frequency's and the\
FSK-SPACE frequency 400 Hz below the Carrier\
With a 8 Bit PWM resolution this is less accurate,\
but decoding with PI-RX is still possible.

Not sure how this influences the decoding at very low levels.

### Added new software

The version PI7ALK_ADF43XX_6POT_V1.0 creates FSK and PI4\
sequences using 6 potmeters and CMOS switches in a\
differential mode. The CW switch is always closed.

The version PI7ALK_ADF43XX_6POT_V2.0 to creates FSK and PI4\
sequences using 6 potmeters and CMOS switches in a\
direct mode. Every switch is one frequency.

The version PI7ALK_ADF43XX_6POT_V2.1 fixes an programming\
error in the way FSK CW is create.
