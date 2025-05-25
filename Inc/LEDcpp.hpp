/**
 * @file LEDcpp.hpp
 * Head file of CLED class
 * Jast for STM32Cube C++ programming example
 * Keil ARM MDK V5.05
 * STM32 F4 Discovery board (STM32F407VGT6)
 * STM32CubeMX V4.9.0
 *
 * Copyright (C) Ian Jin
 * iancanada.mail@gmail.com
 * https://github.com/iancanada/
 * Twitter: @iancanadaTT
 *
 * This program is a free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LEDCPP_H
#define _LEDCPP_H
#include "stm32f4xx_hal.h"

class GPIO {
	uint32_t gpio_pin;
	public:
			GPIO(uint32_t GPIO_Pin);
			/*
					The init() method from this Class will override the init() method defined in AP_HAL::GPIO
					as we know, AP_HAL::GPIO's init() method is defined as pure virtual function, it means it must be implemented in the AP_HAL::GPIO's child classes
			*/
			void    init();
			void    pinMode(uint8_t pin, uint8_t output);
			uint8_t read(void);
			void    write(uint8_t pin, uint8_t value);
			void    toggle(uint8_t pin);
};

class CLed
{
  GPIO_TypeDef* _port;                                      //GPIOA,GPIOB....
  uint16_t _pin;                                            //GPIO_PIN_0,GPIO_PIN_1....
  uint16_t _toggleTime;                                     //Toggle time in ms
  uint16_t counter;                                         //Toggle time counter
  void on(){HAL_GPIO_WritePin(_port,_pin,GPIO_PIN_SET);}    //turn on LED
  void off(){HAL_GPIO_WritePin(_port,_pin,GPIO_PIN_RESET);} //turn off LED
  void toggle(){HAL_GPIO_TogglePin(_port,_pin);}            //toggle LED
public:
  CLed(GPIO_TypeDef* port,uint16_t pin,uint16_t toggleTim); //constructor
  void runToggle();                                         //run toggling LED from system tick every 1ms	
};

#endif

