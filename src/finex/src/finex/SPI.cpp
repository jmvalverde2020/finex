
#include "finex/SPI.hpp"

float VoutZero = 0.759;

enum {
  BUFF_SIZE = 2,
};

SPI::SPI() {
  last_voltage_ = 0;
}

void
printBuff(char *buff)
{
  for(int i = 0; i < BUFF_SIZE; i++) {
    printf("%02x", buff[i]);
  }
  printf("\n");
}

bool SPI::init(finex::Controller::SharedPtr node) {

  node_ = node;
  //Checking it is possible to acess bcm2835 library:
  if (!bcm2835_init()) {
    RCLCPP_ERROR(node_->get_logger(), "BCM2835 INIT ERROR: bcm2835_init failed. ¿Are you running as root?");
    return 0;
  }
  //Select the GPIO's used as Chanel Select as outputs. (Selected 29)
  bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_32, BCM2835_GPIO_FSEL_OUTP);

  //Unsellect channels.
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_32, HIGH);
  //Start SPI operations.
  if (!bcm2835_aux_spi_begin()) {
   RCLCPP_ERROR(node_->get_logger(), "SPI INIT ERROR: bcm2835_spi_init failed. ¿Are you running as root?");
   return 0;
  }
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // In AD5570: "Data is t$
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // CPOL = 0, CPHA = 1,  $
  //bcm2835_aux_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64); // RPi4 = 2.083 MHz ,$
  bcm2835_aux_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32); //RPi4= 8.333MHz
  //4 slaves implementation:
  bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
  return 1;
}

void SPI::sendData(float voltage) {

  // Motor Driver limitations
  voltage = std::clamp(voltage, MINV, MAXV);
  
  last_voltage_ = voltage;

  // printf("sending: %f\n", voltage);
  uint16_t data_ = static_cast<uint16_t>(voltage * 3364.2 + 30215.3); //AD5570 callibration
  char buf[2] = {static_cast<char>(data_ >> 8), static_cast<char>(data_ & 0xFF)};
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_32,LOW);

  // printBuff(buf);

  bcm2835_aux_spi_transfern(buf, sizeof(buf));

  // Unsellect channel.
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_32, HIGH);

  // Let the signal of the chanels select high for at least 45ns, AD5570 datasheet.
  bcm2835_st_delay(0, 0.5);
}

void SPI::end() {

    sendData(VoutZero);
    //END AUX SPI.
    bcm2835_aux_spi_end();
    //Close the library, deallocating any allocated memory and closing /dev/mem
    bcm2835_close();
}