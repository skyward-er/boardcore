These files contain a patched version of the harware i2c driver shipped with miosix. The patch makes available the possibility to send another start without having sent a stop
before (restart function, required by some sensors). Actually this driver CANNOT be integrated in miosix because it needs extensive testing
