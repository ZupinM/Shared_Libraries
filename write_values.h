/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WRITE_VALUES_H
#define __WRITE_VALUES_H

#define ARROW_SHIFT 500     //pomik s tipkami v Heliosu

void write_values(unsigned char box, unsigned int IntTemp, float FloatTemp, unsigned char *StringTemp);
void write_value_limit(void);
unsigned int  decrypt (unsigned int rotate, unsigned int chiper);
void ClearStatus (void);


//----------------------------------
#endif /*__WRITE_VALUES_H*/