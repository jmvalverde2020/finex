#ifndef FINEX_SPI_HPP_
#define FINEX_SPI_HPP_

#include <iostream>
#include <bcm2835.h>
#include <stdlib.h>
#include "finex/Controller.hpp"
#include "rclcpp/rclcpp.hpp"

class SPI {
 public:
   SPI();

   //Initiate bc2835 and set up spi.
   bool init(finex::Controller::SharedPtr node);

   //Send data to the selected joint.
   void sendData(float voltage);

   //End bcm2835 and spi.
   void end();

 private:
   uint16_t data_;
   float last_voltage_;
   float MINV = -3.0, MAXV = 3.0;
   finex::Controller::SharedPtr node_;
};

#endif // FINEX_SPI_HPP_