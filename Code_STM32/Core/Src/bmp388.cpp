/*
 * bmp388.cpp
 *
 *  Created on: 28-Jun-2021
 *      Author: kanishak
 */
#include "main.h"
#include "bmp388.h"


bmp388::bmp388()
{

}


bmp388::~bmp388()
{

}


void bmp388::chip_select(uint16_t pin)
{
	pin_ = pin;
}

void bmp388::spi(SPI_HandleTypeDef* spi1, SPI_HandleTypeDef* spi2, SPI_HandleTypeDef* spi3)
{
	spi1_ = spi1;
	spi2_ = spi2;
	spi3_ = spi3;
}


void bmp388::init(){


	  HAL_GPIO_WritePin(GPIOA, pin_, GPIO_PIN_RESET);
	  HAL_SPI_Transmit_DMA(spi1_, (uint8_t *)&chip_id_, 2);
	  SPI_2_3_enable();
	  HAL_SPI_Receive_DMA(spi1_, (uint8_t *)&chip_id_r_1, 1);
	  HAL_SPI_Receive_DMA(spi2_, (uint8_t *)&chip_id_r_2, 1);
	  HAL_SPI_Receive_DMA(spi3_, (uint8_t *)&chip_id_r_3, 1);
	  SPI_2_3_disable();
	  HAL_GPIO_WritePin(GPIOA, pin_, GPIO_PIN_SET);

	  HAL_Delay(2);

	  //Soft Reset
	  HAL_GPIO_WritePin(GPIOA, pin_, GPIO_PIN_RESET);
	  HAL_SPI_Transmit_DMA(spi1_, (uint8_t *)&send_cmd_, 1);
	  HAL_SPI_Transmit_DMA(spi1_, (uint8_t *)&soft_rst_, 1);
	  HAL_GPIO_WritePin(GPIOA, pin_, GPIO_PIN_SET);

	  HAL_Delay(2);

	  //Write to Power Control
	  HAL_GPIO_WritePin(GPIOA, pin_, GPIO_PIN_RESET);
	  HAL_SPI_Transmit_DMA(spi1_, (uint8_t *)&pwr_ctrl_w_, 1);
	  HAL_SPI_Transmit_DMA(spi1_, (uint8_t *)&pwr_config_, 1);
	  HAL_GPIO_WritePin(GPIOA, pin_, GPIO_PIN_SET);

	  HAL_Delay(2);

	  //Read Compensation Coeff of three sensors and store in 2D array
	  HAL_GPIO_WritePin(GPIOA, pin_, GPIO_PIN_RESET);
	  HAL_SPI_Transmit_DMA(spi1_, (uint8_t *)&comp_coeff_r_, 2);
	  SPI_2_3_enable();
	  for (int i = 0; i < 22; i++)
	  {
		  HAL_SPI_Receive_DMA(spi1_, (uint8_t*)&comp_coeff_store_[i][0], 1);
		  HAL_SPI_Receive_DMA(spi2_, (uint8_t*)&comp_coeff_store_[i][1], 1);
		  HAL_SPI_Receive_DMA(spi3_, (uint8_t*)&comp_coeff_store_[i][2], 1);
	  }
	  SPI_2_3_disable();
	  HAL_GPIO_WritePin(GPIOA, pin_, GPIO_PIN_SET);

	  HAL_Delay(2);
}


void bmp388::calculate_coeff()
{
	 for (int i = 0; i<3; i++)
	 {
	  nvm_par_t1_[i] = (uint16_t)(comp_coeff_store_[1][i] << 8 | comp_coeff_store_[0][i]) / 0.00390625f;
	  nvm_par_t2_[i] = (uint16_t)(comp_coeff_store_[3][i] << 8 | comp_coeff_store_[2][i]) / 1073741824.0f;
	  nvm_par_t3_[i] = (int8_t) comp_coeff_store_[4][i] / 281474976710656.0f;
	  nvm_par_p1_[i] = ((int16_t)(comp_coeff_store_[6][i] << 8 | comp_coeff_store_[5][i]) - 16384) / 1048576.0f;
	  nvm_par_p2_[i] = ((int16_t) (comp_coeff_store_[8][i] << 8 | comp_coeff_store_[7][i]) - 16384) / 536870912.0f;
	  nvm_par_p3_[i] = (int16_t) comp_coeff_store_[9][i] / 4294967296.0f;
	  nvm_par_p4_[i] = (int8_t) comp_coeff_store_[10][i] / 137438953472.0f;
	  nvm_par_p5_[i] = (uint16_t)(comp_coeff_store_[12][i] << 8 | comp_coeff_store_[11][i]) / 0.125f;
	  nvm_par_p6_[i] = (uint16_t)(comp_coeff_store_[14][i] << 8 | comp_coeff_store_[13][i]) / 64.0f;
	  nvm_par_p7_[i] = (int8_t) comp_coeff_store_[15][i] / 256.0f;
	  nvm_par_p8_[i] = (int8_t)comp_coeff_store_[16][i] / 32768.0f;;
	  nvm_par_p9_[i] = (int16_t)(comp_coeff_store_[18][i] << 8 | comp_coeff_store_[17][i]) / 281474976710656.0f;
	  nvm_par_p10_[i] = (int8_t) comp_coeff_store_[19][i] / 281474976710656.0f;
	  nvm_par_p11_[i] = (int8_t) comp_coeff_store_[20][i] / 36893488147419103232.0f;
	 }
}

//read raw values (6 bytes) of 3 sensors and store in a 2d array
void bmp388::readout()
{

	  HAL_GPIO_WritePin(GPIOA, pin_, GPIO_PIN_RESET);
	  HAL_SPI_Transmit_DMA(spi1_, (uint8_t *)&read_temp_, 2);
	  SPI_2_3_enable();
	  for (int i = 0; i < 6; i++)
	  {
		  HAL_SPI_Receive_DMA(spi1_, (uint8_t*)&spi_buf_1_[i][0], 1);
		  HAL_SPI_Receive_DMA(spi2_, (uint8_t*)&spi_buf_1_[i][1], 1);
		  HAL_SPI_Receive_DMA(spi3_, (uint8_t*)&spi_buf_1_[i][2], 1);
	  }
	  SPI_2_3_disable();
	  HAL_GPIO_WritePin(GPIOA, pin_, GPIO_PIN_SET);


	  for (int i = 0; i < 3; i++)
	  {
		  pressure_read_out_[i] = (uint32_t) spi_buf_1_[2][i] << 16 | (uint32_t)spi_buf_1_[1][i] << 8 | (uint32_t)spi_buf_1_[0][i];
		  temperature_read_out_[i] = (uint32_t) spi_buf_1_[5][i] << 16 | (uint32_t)spi_buf_1_[4][i] << 8  | (uint32_t)spi_buf_1_[3][i];
	  }
}


void bmp388::compensate()
{
	  for (int i = 0; i < 3; i++)
	  {
		  partial_data1_[i] = (float)(temperature_read_out_[i] - nvm_par_t1_[i]);
		  partial_data2_[i] = (float)(partial_data1_[i] * nvm_par_t2_[i]);
		  temperature_[i] = partial_data2_[i]+ (partial_data1_[i] * partial_data1_[i]) * nvm_par_t3_[i];

		  partial_data1_[i] = nvm_par_p6_[i] * temperature_[i];
		  partial_data2_[i] = nvm_par_p7_[i] * temperature_[i] * temperature_[i];
		  partial_data3_[i] = nvm_par_p8_[i] * temperature_[i] * temperature_[i] * temperature_[i];
		  partial_out1_[i] = nvm_par_p5_[i] + partial_data1_[i] + partial_data2_[i] + partial_data3_[i];

		  partial_data1_[i] = nvm_par_p2_[i] * temperature_[i];
		  partial_data2_[i] = nvm_par_p3_[i] * temperature_[i] * temperature_[i];
		  partial_data3_[i] = nvm_par_p4_[i] * temperature_[i] * temperature_[i] * temperature_[i];
		  partial_out2_[i] = (float)pressure_read_out_[i] * (nvm_par_p1_[i] +  partial_data1_[i] +  partial_data2_[i] +  partial_data3_[i]);

		  partial_data1_[i] = (float)pressure_read_out_[i] * (float)pressure_read_out_[i];
	  	  partial_data2_[i] = nvm_par_p9_[i] + nvm_par_p10_[i] * temperature_[i];
	  	  partial_data3_[i] =  partial_data1_[i] *  partial_data2_[i];
	  	  partial_data4_[i] =  partial_data3_[i] + ((float)pressure_read_out_[i] * (float)pressure_read_out_[i] * (float)pressure_read_out_[i])*nvm_par_p11_[i];
	  	  pressure_[i] = partial_out1_[i] + partial_out2_[i] + partial_data4_[i];
	  	 }
};

void bmp388::SPI_2_3_enable()
{
	  SPI2->CR1 &= ~SPI_CR1_SSI; //SPI2, software CS low
	  SPI2->CR1 |= SPI_CR1_SPE;
	  SPI3->CR1 &= ~SPI_CR1_SSI; //SPI3, software CS low
	  SPI3->CR1 |= SPI_CR1_SPE;
}

void bmp388::SPI_2_3_disable()
{
	  SPI2->CR1 &= ~SPI_CR1_SPE; //Disable SPI2
	  SPI2->CR1 |= SPI_CR1_SSI;
	  SPI3->CR1 &= ~SPI_CR1_SPE; //Disable SPI3
	  SPI3->CR1 |= SPI_CR1_SSI;
}
