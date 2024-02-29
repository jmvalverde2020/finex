#ifndef REFLEX_EXO_SPI_HPP_
#define REFLEX_EXO_SPI_HPP_

#include <iostream>
#include <bcm2835.h>
#include <stdlib.h>
#include "reflex-exo/Controller.hpp"
#include "rclcpp/rclcpp.hpp"

class SPI {
 public:
   SPI();

   //Initiate bc2835 and set up spi.
   bool init(reflex_exo::Controller::SharedPtr node);

   //Send data to the selected joint.
   void sendData(float voltage);

   //End bcm2835 and spi.
   void end();

 private:
   uint16_t data_;
   float last_voltage_;
   reflex_exo::Controller::SharedPtr node_;
};

#endif // REFLEX_EXO_SPI_HPP_