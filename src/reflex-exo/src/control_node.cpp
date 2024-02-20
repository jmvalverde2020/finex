
#include "reflex-exo/control_node.hpp"

float last_voltage_ = 0;
float VOL = 2.5;

bool SPI_init() {
  //Checking it is possible to acess bcm2835 librry:
  if (!bcm2835_init()) {
    ROS_ERROR("BCM2835 INIT ERROR: bcm2835_init failed. ¿Are you running as root?");
    return 0;
  }
  //Select the GPIO's used as Chanel Select as outputs. (Selected 29)
  bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_29, BCM2835_GPIO_FSEL_OUTP);

  //Unsellect channels.
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_29, HIGH);
  //Start SPI operations.
  if (!bcm2835_aux_spi_begin()) {
   ROS_ERROR("SPI INIT ERROR: bcm2835_spi_init failed. ¿Are you running as root?");
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

void sendData(float voltage) {

  // Motor Driver limitations
  if (voltage >= 9)
    voltage = 9;
  if (voltage <= -9)
    voltage = -9;

  last_voltage_ = voltage;

  data_ = static_cast<uint16_t > (voltage * 3364.2 + 30215.3); //AD5570 callibration
  char buf[2] = {data_ >> 8, data_ & 0xFF};
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_29,LOW);

  bcm2835_aux_spi_transfern(buf, sizeof(buf));

  // Unsellect channel.
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_29, HIGH);

  // Let the signal of the chanels select high for at least 45ns, AD5570 datasheet.
  bcm2835_st_delay(0, 0.5);
}

void SPI_end() {

    //END AUX SPI.
    bcm2835_aux_spi_end();
    //Close the library, deallocating any allocated memory and closing /dev/mem
    bcm2835_close();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<reflex_exo::ControlNode>();
    if (!SPI_init()) {
        RCLCPP_ERROR(node->get_logger(), "SPI initialization failed");
    }


    while (rclcpp::ok()) {
        sendData(VOL);
        rclcpp::spin_some(node);
    }

    SPI_end();
    rclcpp::shutdown();
    return 0;
}