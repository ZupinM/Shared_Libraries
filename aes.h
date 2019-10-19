#ifndef __AES_H__
#define __AES_H__

#define ENCRYPT_KEY_SIZE        32

#define Nb            4

#define xtime(x)      ((x<<1)^(((x>>7)&1)*0x1b))

#define Multiply(x,y) (((y&1)*x)^ \
                      ((y>>1&1)*xtime(x))^ \
                      ((y>>2&1)*xtime(xtime(x)))^ \
                      ((y>>3&1)*xtime(xtime(xtime(x))))^ \
                      ((y>>4&1)*xtime(xtime(xtime(xtime(x))))))

int encryptData(char *pchDataBuffer, unsigned int uiDataLength);
int decryptData(char *pchDataBuffer, unsigned int uiDataLength);

#endif  // __AES_H__
