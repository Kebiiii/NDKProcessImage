#ifndef IMAGEEDITOR_ANDROID_BEAUTY_H
#define IMAGEEDITOR_ANDROID_BEAUTY_H

#include <string.h>
#include <math.h>

#define PI 3.14
#define ABS(a) ((a)<(0)?(-a):(a))
#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)<(b)?(a):(b))
#define div255(x) (x * 0.003921F)
#define COLOR_ARGB(a, r, g, b) ((a)<<24)|((b) << 16)|((g)<< 8)|(r)

#define IMAGE_JNI_METHOD(METHOD_NAME) \
  Java_com_kebii_utils_NativeImageUtils_##METHOD_NAME

const float YCbCrYRF = 0.299F;
const float YCbCrYGF = 0.587F;
const float YCbCrYBF = 0.114F;
const float YCbCrCbRF = -0.168736F;
const float YCbCrCbGF = -0.331264F;
const float YCbCrCbBF = 0.500000F;
const float YCbCrCrRF = 0.500000F;
const float YCbCrCrGF = -0.418688F;
const float YCbCrCrBF = -0.081312F;

const float RGBRYF = 1.00000F;
const float RGBRCbF = 0.0000F;
const float RGBRCrF = 1.40200F;
const float RGBGYF = 1.00000F;
const float RGBGCbF = -0.34414F;
const float RGBGCrF = -0.71414F;
const float RGBBYF = 1.00000F;
const float RGBBCbF = 1.77200F;
const float RGBBCrF = 0.00000F;

const int Shift = 20;
const int HalfShiftValue = 1 << (Shift - 1);

const int YCbCrYRI = (int)(YCbCrYRF * (1 << Shift) + 0.5);
const int YCbCrYGI = (int)(YCbCrYGF * (1 << Shift) + 0.5);
const int YCbCrYBI = (int)(YCbCrYBF * (1 << Shift) + 0.5);
const int YCbCrCbRI = (int)(YCbCrCbRF * (1 << Shift) + 0.5);
const int YCbCrCbGI = (int)(YCbCrCbGF * (1 << Shift) + 0.5);
const int YCbCrCbBI = (int)(YCbCrCbBF * (1 << Shift) + 0.5);
const int YCbCrCrRI = (int)(YCbCrCrRF * (1 << Shift) + 0.5);
const int YCbCrCrGI = (int)(YCbCrCrGF * (1 << Shift) + 0.5);
const int YCbCrCrBI = (int)(YCbCrCrBF * (1 << Shift) + 0.5);

const int RGBRYI = (int)(RGBRYF * (1 << Shift) + 0.5);
const int RGBRCbI = (int)(RGBRCbF * (1 << Shift) + 0.5);
const int RGBRCrI = (int)(RGBRCrF * (1 << Shift) + 0.5);
const int RGBGYI = (int)(RGBGYF * (1 << Shift) + 0.5);
const int RGBGCbI = (int)(RGBGCbF * (1 << Shift) + 0.5);
const int RGBGCrI = (int)(RGBGCrF * (1 << Shift) + 0.5);
const int RGBBYI = (int)(RGBBYF * (1 << Shift) + 0.5);
const int RGBBCbI = (int)(RGBBCbF * (1 << Shift) + 0.5);
const int RGBBCrI = (int)(RGBBCrF * (1 << Shift) + 0.5);

#define MAKE_RGB565(r,g,b) ((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))
#define MAKE_ARGB(a,r,g,b) ((a&0xff)<<24) | ((r&0xff)<<16) | ((g&0xff)<<8) | (b&0xff)

#define RGB565_R(p) ((((p) & 0xF800) >> 11) << 3)
#define RGB565_G(p) ((((p) & 0x7E0 ) >> 5) << 2)
#define RGB565_B(p) ( ((p) & 0x1F )    << 3)

#define RGB8888_A(p) (p & (0xff<<24) >> 24 )
#define RGB8888_R(p) (p & (0xff<<16) >> 16 )
#define RGB8888_G(p) (p & (0xff<<8) >> 8 )
#define RGB8888_B(p) (p & (0xff) )

typedef struct {
    uint8_t alpha, red, green, blue;
} ARGB;

uint64_t *mIntegralMatrix = NULL;
uint64_t *mIntegralMatrixSqr = NULL;
uint8_t *mSkinMatrix = NULL;
uint32_t *mImageData_rgb = NULL;
uint8_t *mImageData_yuv = NULL;

long getNowMs();
jintArray brightnessContrastChange(JNIEnv *env,jclass obj,jintArray buffer,jint width,jint height,jfloat brightness,jfloat constrat);
uint32_t * brightnessContrastChange(uint32_t *bitmapPixels, jint width, jint height, jfloat brightness, jfloat constrat);
uint32_t * gaussBlur(uint32_t *pInt, uint32_t width, uint32_t height, int i);
uint32_t * stackBlur(uint32_t *pInt, uint32_t width, uint32_t height, int i);
jobject generateBitmap(JNIEnv *env, uint32_t width, uint32_t height);

int *blur_ARGB_8888(int *pix, int w, int h, int radius);

void YCbCrToRGB(uint8_t* From, uint8_t* To, int length);

void RGBToYCbCr(uint8_t* From, uint8_t* To, int length);

int32_t convertArgbToInt(ARGB argb);

void convertIntToArgb(uint32_t pixel, ARGB* argb);

void initBeautiMatrix(uint32_t *pix, int width, int height);

void initSkinMatrix(uint32_t *pix, int width, int height);

void initIntegralMatrix(int width, int height);

void setSmooth(uint32_t *pix, float smoothValue, int width, int height);

void setWhiteSkin(uint32_t *pix, float whiteVal, int width, int height);

void freeMatrix();

void *do_mosaic(void *pix, void *out_pix, unsigned int width, unsigned int height, unsigned int stride,
                unsigned int out_stride, unsigned int radius);

#endif //IMAGEEDITOR_ANDROID_BEAUTY_H
