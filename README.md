# PotatoMelt

PotatoMelt is a drive controller for Meltybrain (translational drift) robots, running on the arduino (atmega32) platform.

It's also my first Arduino project, so, brace yourself.

This is based incredibly heavily on the fantastic OpenMelt2: https://github.com/nothinglabs/openmelt2

If you're spinning this up in a VS Code workspace, there's a few things you'll need to do to get everything happy:
* Install the Arduino vs code extension
* Open the Library Manager and add:
** Adafruit SleepyDog Library
** SparkFun LIS331 Accelerometers
* Rebuild your intellisense configuration
* Add "USBCON" to the "defines" list in the Win32 section of your c_cpp_properties.json

My development hardware:
* Adafruit Itsybitsy 32u4 16mhz 5v
* Adafruit LIS331 accelerometer i2c board
* Generic IBus receiver
* Repeat Robotics dual AM32 ESC