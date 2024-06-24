# FINEX User Manual

## 1. Installation

### 1.1 Install ROS2 (Raspberry and PC)

https://docs.ros.org/en/humble/Installation.html

### 1.2 Install required libraries (Raspberry)

```
> sudo apt update & sudo apt upgrade
> cd ~                  
> wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.58.tar.gz                       
> tar xvfz bcm2835-1.58.tar.gz;                      
> cd bcm2835-1.58;                       
> ./configure;                      
> make;        
> sudo make install
```

### 1.3 Clone ws (Raspberry and PC)

```
> cd ~                  
> git clone https://github.com/jmvalverde2020/finex.git                      
```

### 1.4 Compile workspace (Raspberry and PC)

```
> cd finex/
> colcon build
```
## 2. Raspberry execution

Turn on the power and the raspberry but not the motor switch

### 2.1 Config CAN interface (Raspberry)

```
> cd [ws directory]
> ./can_init.sh
```

### 2.2 Run sensor node (Raspberry)

```
> sudo su
> ./run_sensors.sh 
```

### 2.3 Run control node (Raspberry)

```
> sudo su
> ./run_control.sh 
```

## 3. PC execution
Wait until control node starts

### 3.1 Run interface node (PC)

```
> cd ~/finex/
> source install/setup.bash
> ros2 run finex_gui control_panel
```

Once the interface is shown, press STOP and turn on the motor switch
