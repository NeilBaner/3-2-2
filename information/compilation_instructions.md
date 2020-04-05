#Compilation Instructions

##alex-arduino.ino

* Make sure to import serialize.zip in Arduino IDE as a .ZIP library (Sketch -> Include Library -> Add .ZIP Library)

##alex-pi.cpp

* Use the following command in alex-pi directory:
    g++ alex-pi.cpp serialize.cpp serial.cpp -lpthread -o alex-pi

