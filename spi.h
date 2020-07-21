#ifndef __SPI_H 
#define __SPI_H

void LoRa_Interrupt(uint8_t);
void LoRa_SPIWrite(uint8_t addr, uint8_t* pcBuffer, uint8_t cNbBytes);
uint8_t LoRa_SPIRead(uint8_t addr, uint8_t* pcBuffer, uint8_t cNbBytes);
void spi_rx_fifo_clear(void);
void LoRa_SPI_init(void);

#endif