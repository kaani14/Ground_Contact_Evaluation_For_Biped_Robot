/*
 * bmp388.h
 *
 *  Created on: 28-Jun-2021
 *      Author: kanishak
 */

#ifndef BMP388_H_
#define BMP388_H_

#include<string.h>
#include <stdio.h>
#include <stdlib.h>


class bmp388
{
	public:
		//bmp388();
		bmp388();
		~bmp388();
		void init(void);
		void calculate_coeff(void);
		void readout(void);
		void compensate(void);
		void chip_select(uint16_t pin);
		void spi(SPI_HandleTypeDef* spi1, SPI_HandleTypeDef* spi2, SPI_HandleTypeDef* spi3);
		float temperature_[3], pressure_[3];
		uint16_t pin_;
		SPI_HandleTypeDef *spi1_, *spi2_, *spi3_;
		char chip_id_r_1[1], chip_id_r_2[1], chip_id_r_3[1];

	private:
		void SPI_2_3_enable();
		void SPI_2_3_disable();
		const uint8_t chip_id_ = 0x80;
		const uint8_t cmd_rdy_ = 0x83;
		const uint8_t send_cmd_ = 0x7E;
		const uint8_t soft_rst_ = 0xB6;
		const uint8_t comp_coeff_r_ = 0xB1;
		const uint8_t pwr_ctrl_ = 0x9B;
		const uint8_t pwr_ctrl_w_ = 0x1B;
		const uint8_t pwr_config_ = 0x33;
		const uint8_t read_temp_ = 0x84;

		char spi_buf_1_[6][3], spi_buf_2_[1], spi_buf_3_[1];
		char comp_coeff_store_[22][3];

		float nvm_par_t1_[3], nvm_par_t2_[3], nvm_par_t3_[3];
		float nvm_par_p1_[3], nvm_par_p2_[3], nvm_par_p3_[3];
		float nvm_par_p4_[3], nvm_par_p5_[3], nvm_par_p6_[3];
		float nvm_par_p7_[3], nvm_par_p8_[3], nvm_par_p9_[3];
		float nvm_par_p10_[3], nvm_par_p11_[3];

		uint32_t temperature_read_out_[3], pressure_read_out_[3];

		float partial_data1_[3], partial_data2_[3], partial_data3_[3], partial_data4_[3];
		float partial_out1_[3], partial_out2_[3];

};



#endif /* BMP388_H_ */
