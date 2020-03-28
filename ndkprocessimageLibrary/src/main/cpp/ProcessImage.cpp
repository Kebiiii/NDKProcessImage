//
// Created by liuke on 2018/5/2.
//

//#include "com_kebii_ndkprocessimage_NativeImageUtils.h"
//
//#include <logutil.h>
#include <jni.h>
#include <string>
#include <tgmath.h>
#include <math.h>
#include <malloc.h>
#include "android/bitmap.h"
#include "logutil.h"

#define IMAGE_JNI_METHOD(METHOD_NAME) \
  Java_com_kebii_utils_NativeImageUtils_##METHOD_NAME

#define PI 3.14
#define ABS(a) ((a)<(0)?(-a):(a))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define MIN(a,b) ((a)<(b)?(a):(b))

jintArray brightnessContrastChange(JNIEnv *env,jclass obj,jintArray buffer,jint width,jint height,jfloat brightness,jfloat constrat);
uint32_t * brightnessContrastChange(uint32_t *bitmapPixels, jint width, jint height, jfloat brightness, jfloat constrat);
uint32_t * gaussBlur(uint32_t *pInt, uint32_t width, uint32_t height, int i);
uint32_t * stackBlur(uint32_t *pInt, uint32_t width, uint32_t height, int i);
jobject generateBitmap(JNIEnv *env, uint32_t width, uint32_t height);

long getNowMs(){
    struct timeval tv;
    gettimeofday(&tv, NULL);//  tv.tv_sec 很大，%36000 取得100个小时前的时间，不除也可以
    int sec = tv.tv_sec%36000;
    long t = sec*1000 + tv.tv_usec/1000;
    return t;
}

extern "C"
JNIEXPORT jintArray JNICALL IMAGE_JNI_METHOD(deepImage)
    (JNIEnv *env,jclass obj,jintArray buffer,jint width,jint height){
    return brightnessContrastChange(env,obj,buffer,width,height,0.2f,0.2f);
}


extern "C"
JNIEXPORT jobject JNICALL IMAGE_JNI_METHOD(brightnessContrastChange)(JNIEnv* env, jclass thiz,jobject bitmap,jfloat brightness,jfloat constrat) {
    AndroidBitmapInfo bitmapInfo;
    int ret;
    if ((ret = AndroidBitmap_getInfo(env, bitmap, &bitmapInfo)) < 0) {
        LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
        return NULL;
    }

    // 读取 bitmap 的像素内容到 native 内存
    void *bitmapPixels;
    if ((ret = AndroidBitmap_lockPixels(env, bitmap, &bitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    uint32_t newWidth = bitmapInfo.width;
    uint32_t newHeight = bitmapInfo.height;

    uint32_t * newBitmapPixels = brightnessContrastChange(static_cast<uint32_t *>(bitmapPixels), newWidth, newHeight, brightness, constrat);

    jobject newBitmap = generateBitmap(env, newWidth, newHeight);

    void *resultBitmapPixels;

    if ((ret = AndroidBitmap_lockPixels(env, newBitmap, &resultBitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    int pixelsCount = newWidth * newHeight;

//    memcpy((uint32_t *) resultBitmapPixels, bitmapPixels, sizeof(uint32_t) * pixelsCount);
    wmemcpy(reinterpret_cast<wchar_t *>((uint32_t *) resultBitmapPixels),
            reinterpret_cast<const wchar_t *>(newBitmapPixels), pixelsCount);

    AndroidBitmap_unlockPixels(env, newBitmap);

    delete[] newBitmapPixels;

    return newBitmap;
}


uint32_t * brightnessContrastChange
        (uint32_t *bitmapPixels, jint width, jint height, jfloat brightness, jfloat constrat){
    int newSize = width * height;
    if(brightness<0){
        brightness = 0.01f;
    }

    uint32_t *newBitmapPixels = new uint32_t[newSize];

    //开始处理
    int a,r,g,b;
    //调整的亮度
    int bab = (int) (255 - 255 * brightness);
    //对比度
    float ca = 1.0f + constrat;
    LOGE("brightness=%f,bab=%d,constrat=%f,ca=%f", brightness, bab,constrat,ca);
    ca *= ca;
    int cab = (int) (ca * 65536) + 1;
    int x = 0, y = 0;
    for (x = 0; x < width; x++) {
        for (y = 0; y < height; y++) {
            uint32_t color = bitmapPixels[y * width + x];
            a = (color >> 24) & 0xFF;
            r = (color >> 16) & 0xFF;
            g = (color >> 8) & 0xFF;
            b = color & 0xFF;
            int ri = r - bab;
            int gi = g - bab;
            int bi = b - bab;
            r = ri > 255 ? 255 : (ri < 0 ? 0 : ri);
            g = gi > 255 ? 255 : (gi < 0 ? 0 : gi);
            b = bi > 255 ? 255 : (bi < 0 ? 0 : bi);
            //变化对比度
            ri = r - 128;
            gi = g - 128;
            bi = b - 128;
            ri = (ri * cab) >> 16;
            gi = (gi * cab) >> 16;
            bi = (bi * cab) >> 16;
            ri = ri + 128;
            gi = gi + 128;
            bi = bi + 128;
            r = ri > 255 ? 255 : (ri < 0 ? 0 : ri);
            g = gi > 255 ? 255 : (gi < 0 ? 0 : gi);
            b = bi > 255 ? 255 : (bi < 0 ? 0 : bi);
            newBitmapPixels[y * width + x] = 0xff000000 | (r << 16) | (g << 8) | b;
        }
    }
//    wmemcpy(reinterpret_cast<wchar_t *>(bitmapPixels),
//            reinterpret_cast<const wchar_t *>(newBitmapPixels), newSize);
//    delete (newBitmapPixels);
    return newBitmapPixels;
}

jintArray brightnessContrastChange
        (JNIEnv *env,jclass obj,jintArray buffer,jint width,jint height,jfloat brightness,jfloat constrat){
    jint* source = (*env).GetIntArrayElements(buffer,0);
    int newSize = width * height;
    if(brightness<0){
        brightness = 0.01f;
    }

    //开始处理
    int a,r,g,b;
    //调整的亮度
    int bab = (int) (255 * brightness);
    //对比度
    float ca = 1.0f + constrat;
    ca *= ca;
    int cab = (int) (ca * 65536) + 1;
    int x = 0, y = 0;
    for (x = 0; x < width; x++) {
        for (y = 0; y < height; y++) {
            int color = source[y * width + x];
            a = (color >> 24) & 0xFF;
            r = (color >> 16) & 0xFF;
            g = (color >> 8) & 0xFF;
            b = color & 0xFF;
            int ri = r - bab;
            int gi = g - bab;
            int bi = b - bab;
            r = ri > 255 ? 255 : (ri < 0 ? 0 : ri);
            g = gi > 255 ? 255 : (gi < 0 ? 0 : gi);
            b = bi > 255 ? 255 : (bi < 0 ? 0 : bi);
            //变化对比度
            ri = r - 128;
            gi = g - 128;
            bi = b - 128;
            ri = (ri * cab) >> 16;
            gi = (gi * cab) >> 16;
            bi = (bi * cab) >> 16;
            ri = ri + 128;
            gi = gi + 128;
            bi = bi + 128;
            r = ri > 255 ? 255 : (ri < 0 ? 0 : ri);
            g = gi > 255 ? 255 : (gi < 0 ? 0 : gi);
            b = bi > 255 ? 255 : (bi < 0 ? 0 : bi);
            source[y * width + x] = 0xff000000 | (r << 16) | (g << 8) | b;
        }
    }
    jintArray result = (*env).NewIntArray(newSize);
    (*env).SetIntArrayRegion(result, 0, newSize, source);
    (*env).ReleaseIntArrayElements(buffer, source, 0);
    return result;
}

extern "C"
JNIEXPORT jbyteArray JNICALL IMAGE_JNI_METHOD(argb2Yuv)(JNIEnv* env, jclass thiz,jintArray argb, jint width, jint height) {
    int frameSize = width * height;
    int yIndex = 0;
    int uvIndex = frameSize;
    jbyteArray result = env->NewByteArray(frameSize*3/2);
//    unsigned char * yuv420sp = new unsigned char[frameSize*3/2];
    jbyte * yuv420sp = env->GetByteArrayElements(result,0);
    jint * source = env->GetIntArrayElements(argb,0);

    int a, R, G, B, Y, U, V;
    int index = 0;
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {

            a = (source[index] & 0xff000000) >> 24; // a is not used obviously
            R = (source[index] & 0xff0000) >> 16;
            G = (source[index] & 0xff00) >> 8;
            B = (source[index] & 0xff);

            // well known RGB to YUV algorithm
            Y = ( (  66 * R + 129 * G +  25 * B + 128) >> 8) +  16;
            U = ( ( -38 * R -  74 * G + 112 * B + 128) >> 8) + 128;
            V = ( ( 112 * R -  94 * G -  18 * B + 128) >> 8) + 128;

            // NV21 has a plane of Y and interleaved planes of VU each sampled by a factor of 2
            //    meaning for every 4 Y pixels there are 1 V and 1 U.  Note the sampling is every other
            //    pixel AND every other scanline.
            yuv420sp[yIndex++] = (jbyte) ((Y < 0) ? 0 : ((Y > 255) ? 255 : Y));
            if (j % 2 == 0 && index % 2 == 0) {
                yuv420sp[uvIndex++] = (jbyte) ((V < 0) ? 0 : ((V > 255) ? 255 : V));
                yuv420sp[uvIndex++] = (jbyte) ((U < 0) ? 0 : ((U > 255) ? 255 : U));
            }

            index ++;
        }
    }

    env->ReleaseIntArrayElements(argb, source, 0);

    return result;

}

extern "C"
JNIEXPORT jintArray JNICALL IMAGE_JNI_METHOD(yuv420sp2rgb)(JNIEnv* env, jclass thiz,jbyteArray yuv420sp, jint width, jint height) {

    int frameSize = width * height;
//    jintArray desArray = env->NewIntArray(frameSize);
    int *des = new int[frameSize];

    jbyte* source = env->GetByteArrayElements(yuv420sp,0);
    int brightness_factor = 1000;

    brightness_factor = 1192 * brightness_factor / 1000;

    for (int j = 0, yp = 0; j < height; j++) {
        int uvp = frameSize + (j >> 1) * width, u = 0, v = 0;
        for (int i = 0; i < width; i++, yp++) {
            int y = (0xff & ((int) source[yp])) - 16;
            if (y < 0)
                y = 0;
            if ((i & 1) == 0) {
                v = (0xff & source[uvp++]) - 128;
                u = (0xff & source[uvp++]) - 128;
            }

            int y1192 = brightness_factor * y; //1192 * y * brightness_factor /1000;
            int r = (y1192 + 1634 * v);
            int g = (y1192 - 833 * v - 400 * u);
            int b = (y1192 + 2066 * u);

            if (r < 0)
                r = 0;
            else if (r > 262143)
                r = 262143;
            if (g < 0)
                g = 0;
            else if (g > 262143)
                g = 262143;
            if (b < 0)
                b = 0;
            else if (b > 262143)
                b = 262143;

            des[yp] = 0xff000000 | ((r << 6) & 0xff0000) | ((g >> 2) & 0xff00) | ((b >> 10) & 0xff);
        }
    }

    jintArray result = env->NewIntArray(frameSize);
    env->SetIntArrayRegion(result, 0, frameSize, des);
    env->ReleaseByteArrayElements(yuv420sp, source, 0);

    delete []des;
    return result;
}


extern "C"
JNIEXPORT jobject JNICALL IMAGE_JNI_METHOD(gaussBlur)(JNIEnv* env, jclass thiz,jobject bitmap,jint radius) {

    AndroidBitmapInfo bitmapInfo;
    int ret;
    if ((ret = AndroidBitmap_getInfo(env, bitmap, &bitmapInfo)) < 0) {
        LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
        return NULL;
    }

    // 读取 bitmap 的像素内容到 native 内存
    void *bitmapPixels;
    if ((ret = AndroidBitmap_lockPixels(env, bitmap, &bitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    uint32_t newWidth = bitmapInfo.width;
    uint32_t newHeight = bitmapInfo.height;

//    long startTime1 = getNowMs();
//    uint32_t *newBitmapPixels = gaussBlur((uint32_t *) bitmapPixels,newWidth,newHeight,radius);
    uint32_t *newBitmapPixels = stackBlur((uint32_t *) bitmapPixels,newWidth,newHeight,radius);
//    LOGD("gaussBlur time=%ld", getNowMs()-startTime1);


    jobject newBitmap = generateBitmap(env, newWidth, newHeight);

    void *resultBitmapPixels;

    if ((ret = AndroidBitmap_lockPixels(env, newBitmap, &resultBitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    int pixelsCount = newWidth * newHeight;

//    memcpy((uint32_t *) resultBitmapPixels, bitmapPixels, sizeof(uint32_t) * pixelsCount);
    wmemcpy(reinterpret_cast<wchar_t *>((uint32_t *) resultBitmapPixels),
            reinterpret_cast<const wchar_t *>(newBitmapPixels), pixelsCount);

    AndroidBitmap_unlockPixels(env, newBitmap);

    delete[] newBitmapPixels;

    return newBitmap;
}

uint32_t * gaussBlur(uint32_t *pix, uint32_t width, uint32_t height, jint radius) {

    uint32_t *newBitmapPixels = new uint32_t[width*height];

    float sigma = (float) (1.0 * radius / 2.57);
    float deno = (float) (1.0 / (sigma * sqrt(2.0 * PI)));
    float nume = (float) (-1.0 / (2.0 * sigma * sigma));
    float *gaussMatrix = (float *) malloc(sizeof(float) * (radius + radius + 1));
    float gaussSum = 0.0;
    for (int i = 0, x = -radius; x <= radius; ++x, ++i) {
        float g = (float) (deno * exp(1.0 * nume * x * x));
        gaussMatrix[i] = g;
        gaussSum += g;
    }
    int len = radius + radius + 1;
    for (int i = 0; i < len; ++i)
        gaussMatrix[i] /= gaussSum;
    int *rowData = (int *) malloc(width * sizeof(int));
    int *listData = (int *) malloc(height * sizeof(int));
    for (int y = 0; y < height; ++y) {
//        memcpy(rowData, pix + y * width, sizeof(int) * width);
        wmemcpy(reinterpret_cast<wchar_t *>(rowData),
                reinterpret_cast<const wchar_t *>(pix + y * width), width);
        for (int x = 0; x < width; ++x) {
            float r = 0, g = 0, b = 0;
            gaussSum = 0;
            for (int i = -radius; i <= radius; ++i) {
                int k = x + i;
                if (0 <= k && k <= width) {
                    //得到像素点的rgb值
                    int color = rowData[k];
                    int cr = (color & 0x00ff0000) >> 16;
                    int cg = (color & 0x0000ff00) >> 8;
                    int cb = (color & 0x000000ff);
                    r += cr * gaussMatrix[i + radius];
                    g += cg * gaussMatrix[i + radius];
                    b += cb * gaussMatrix[i + radius];
                    gaussSum += gaussMatrix[i + radius];
                }
            }
            int cr = (int) (r / gaussSum);
            int cg = (int) (g / gaussSum);
            int cb = (int) (b / gaussSum);
            newBitmapPixels[y * width + x] = cr << 16 | cg << 8 | cb | 0xff000000;
        }
    }
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y)
            listData[y] = pix[y * width + x];
        for (int y = 0; y < height; ++y) {
            float r = 0, g = 0, b = 0;
            gaussSum = 0;
            for (int j = -radius; j <= radius; ++j) {
                int k = y + j;
                if (0 <= k && k <= height) {
                    int color = listData[k];
                    int cr = (color & 0x00ff0000) >> 16;
                    int cg = (color & 0x0000ff00) >> 8;
                    int cb = (color & 0x000000ff);
                    r += cr * gaussMatrix[j + radius];
                    g += cg * gaussMatrix[j + radius];
                    b += cb * gaussMatrix[j + radius];
                    gaussSum += gaussMatrix[j + radius];
                }
            }
            int cr = (int) (r / gaussSum);
            int cg = (int) (g / gaussSum);
            int cb = (int) (b / gaussSum);
            newBitmapPixels[y * width + x] = cr << 16 | cg << 8 | cb | 0xff000000;
        }
    }
    free(gaussMatrix);
    free(rowData);
    free(listData);
    return newBitmapPixels;
}

uint32_t * stackBlur(uint32_t* pix, uint32_t w, uint32_t h, jint radius) {

    uint32_t *newBitmapPixels = new uint32_t[w*h];

    int wm = w - 1;
    int hm = h - 1;
    int wh = w * h;
    int div = radius + radius + 1;
    // 指针
    int *r = (int *)malloc(wh * sizeof(int));
    int *g = (int *)malloc(wh * sizeof(int));
    int *b = (int *)malloc(wh * sizeof(int));
    int rsum, gsum, bsum, x, y, i, p, yp, yi, yw;

    int *vmin = (int *)malloc(MAX(w,h) * sizeof(int));

    int divsum = (div + 1) >> 1;
    divsum *= divsum;
    int *dv = (int *)malloc(256 * divsum * sizeof(int));
    for (i = 0; i < 256 * divsum; i++) {
        dv[i] = (i / divsum);
    }

    yw = yi = 0;

    int(*stack)[3] = (int(*)[3])malloc(div * 3 * sizeof(int));
    int stackpointer;
    int stackstart;
    int *sir;
    int rbs;
    int r1 = radius + 1;
    int routsum, goutsum, boutsum;
    int rinsum, ginsum, binsum;

    for (y = 0; y < h; y++) {
        rinsum = ginsum = binsum = routsum = goutsum = boutsum = rsum = gsum = bsum = 0;
        for (i = -radius; i <= radius; i++) {
            p = pix[yi + (MIN(wm, MAX(i, 0)))];
            sir = stack[i + radius];
            sir[0] = (p & 0xff0000) >> 16;
            sir[1] = (p & 0x00ff00) >> 8;
            sir[2] = (p & 0x0000ff);

            rbs = r1 - ABS(i);
            rsum += sir[0] * rbs;
            gsum += sir[1] * rbs;
            bsum += sir[2] * rbs;
            if (i > 0) {
                rinsum += sir[0];
                ginsum += sir[1];
                binsum += sir[2];
            }
            else {
                routsum += sir[0];
                goutsum += sir[1];
                boutsum += sir[2];
            }
        }
        stackpointer = radius;

        for (x = 0; x < w; x++) {

            r[yi] = dv[rsum];
            g[yi] = dv[gsum];
            b[yi] = dv[bsum];

            rsum -= routsum;
            gsum -= goutsum;
            bsum -= boutsum;

            stackstart = stackpointer - radius + div;
            sir = stack[stackstart % div];

            routsum -= sir[0];
            goutsum -= sir[1];
            boutsum -= sir[2];

            if (y == 0) {
                vmin[x] = MIN(x + radius + 1, wm);
            }
            p = pix[yw + vmin[x]];

            sir[0] = (p & 0xff0000) >> 16;
            sir[1] = (p & 0x00ff00) >> 8;
            sir[2] = (p & 0x0000ff);

            rinsum += sir[0];
            ginsum += sir[1];
            binsum += sir[2];

            rsum += rinsum;
            gsum += ginsum;
            bsum += binsum;

            stackpointer = (stackpointer + 1) % div;
            sir = stack[(stackpointer) % div];

            routsum += sir[0];
            goutsum += sir[1];
            boutsum += sir[2];

            rinsum -= sir[0];
            ginsum -= sir[1];
            binsum -= sir[2];

            yi++;
        }
        yw += w;
    }
    for (x = 0; x < w; x++) {
        rinsum = ginsum = binsum = routsum = goutsum = boutsum = rsum = gsum = bsum = 0;
        yp = -radius * w;
        for (i = -radius; i <= radius; i++) {
            yi = MAX(0, yp) + x;

            sir = stack[i + radius];

            sir[0] = r[yi];
            sir[1] = g[yi];
            sir[2] = b[yi];

            rbs = r1 - ABS(i);

            rsum += r[yi] * rbs;
            gsum += g[yi] * rbs;
            bsum += b[yi] * rbs;

            if (i > 0) {
                rinsum += sir[0];
                ginsum += sir[1];
                binsum += sir[2];
            }
            else {
                routsum += sir[0];
                goutsum += sir[1];
                boutsum += sir[2];
            }

            if (i < hm) {
                yp += w;
            }
        }
        yi = x;
        stackpointer = radius;
        for (y = 0; y < h; y++) {
            // Preserve alpha channel: ( 0xff000000 & pix[yi] )
            newBitmapPixels[yi] = (0xff000000 & pix[yi]) | (dv[rsum] << 16) | (dv[gsum] << 8) | dv[bsum];

            rsum -= routsum;
            gsum -= goutsum;
            bsum -= boutsum;

            stackstart = stackpointer - radius + div;
            sir = stack[stackstart % div];

            routsum -= sir[0];
            goutsum -= sir[1];
            boutsum -= sir[2];

            if (x == 0) {
                vmin[y] = MIN(y + r1, hm) * w;
            }
            p = x + vmin[y];

            sir[0] = r[p];
            sir[1] = g[p];
            sir[2] = b[p];

            rinsum += sir[0];
            ginsum += sir[1];
            binsum += sir[2];

            rsum += rinsum;
            gsum += ginsum;
            bsum += binsum;

            stackpointer = (stackpointer + 1) % div;
            sir = stack[stackpointer];

            routsum += sir[0];
            goutsum += sir[1];
            boutsum += sir[2];

            rinsum -= sir[0];
            ginsum -= sir[1];
            binsum -= sir[2];

            yi += w;
        }
    }
    //记得要释放掉
    free(r);
    free(g);
    free(b);
    free(vmin);
    free(dv);
    free(stack);
    return newBitmapPixels;
}


extern "C"
JNIEXPORT jobject JNICALL IMAGE_JNI_METHOD(rotateBitmap)
        (JNIEnv *env, jclass thiz, jobject bitmap) {
    LOGD("rotate bitmap");

    AndroidBitmapInfo bitmapInfo;
    int ret;
    if ((ret = AndroidBitmap_getInfo(env, bitmap, &bitmapInfo)) < 0) {
        LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
        return NULL;
    }

    // 读取 bitmap 的像素内容到 native 内存
    void *bitmapPixels;
    if ((ret = AndroidBitmap_lockPixels(env, bitmap, &bitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    uint32_t newWidth = bitmapInfo.height;
    uint32_t newHeight = bitmapInfo.width;

    uint32_t *newBitmapPixels = new uint32_t[newWidth * newHeight];

    int whereToGet = 0;

    // 弄明白 bitmapPixels 的排列，这里不同于二维数组了。
    for (int x = newWidth; x >= 0; --x) {
        for (int y = 0; y < newHeight; ++y) {
            uint32_t pixel = ((uint32_t *) bitmapPixels)[whereToGet++];
            newBitmapPixels[newWidth * y + x] = pixel;
        }
    }

    jobject newBitmap = generateBitmap(env, newWidth, newHeight);

    void *resultBitmapPixels;

    if ((ret = AndroidBitmap_lockPixels(env, newBitmap, &resultBitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    int pixelsCount = newWidth * newHeight;

//    memcpy((uint32_t *) resultBitmapPixels, newBitmapPixels, sizeof(uint32_t) * pixelsCount);
    wmemcpy(reinterpret_cast<wchar_t *>((uint32_t *) resultBitmapPixels),
            reinterpret_cast<const wchar_t *>(newBitmapPixels), pixelsCount);

    AndroidBitmap_unlockPixels(env, newBitmap);

    delete[] newBitmapPixels;
    return newBitmap;
}

extern "C"
JNIEXPORT jobject JNICALL IMAGE_JNI_METHOD(convertBitmap)
        (JNIEnv *env, jclass thiz, jobject bitmap) {
    LOGD("convert bitmap");
    AndroidBitmapInfo bitmapInfo;
    int ret;
    if ((ret = AndroidBitmap_getInfo(env, bitmap, &bitmapInfo)) < 0) {
        LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
        return NULL;
    }

    // 读取 bitmap 的像素内容到 native 内存
    void *bitmapPixels;
    if ((ret = AndroidBitmap_lockPixels(env, bitmap, &bitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    uint32_t newWidth = bitmapInfo.width;
    uint32_t newHeight = bitmapInfo.height;

    uint32_t *newBitmapPixels = new uint32_t[newWidth * newHeight];

    int whereToGet = 0;

    for (int y = 0; y < newHeight; ++y) {
        for (int x = 0; x < newWidth; x++) {
            uint32_t pixel = ((uint32_t *) bitmapPixels)[whereToGet++];
            newBitmapPixels[newWidth * (newHeight - 1 - y) + x] = pixel;
        }
    }


    jobject newBitmap = generateBitmap(env, newWidth, newHeight);

    void *resultBitmapPixels;

    if ((ret = AndroidBitmap_lockPixels(env, newBitmap, &resultBitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    int pixelsCount = newWidth * newHeight;

//    memcpy((uint32_t *) resultBitmapPixels, newBitmapPixels, sizeof(uint32_t) * pixelsCount);
    wmemcpy(reinterpret_cast<wchar_t *>((uint32_t *) resultBitmapPixels),
            reinterpret_cast<const wchar_t *>(newBitmapPixels), pixelsCount);

    AndroidBitmap_unlockPixels(env, newBitmap);

    delete[] newBitmapPixels;

    return newBitmap;
}

extern "C"
JNIEXPORT jobject JNICALL IMAGE_JNI_METHOD(mirrorBitmap)
        (JNIEnv *env, jclass thiz, jobject bitmap) {
    LOGD("mirror bitmap");
    AndroidBitmapInfo bitmapInfo;
    int ret;
    if ((ret = AndroidBitmap_getInfo(env, bitmap, &bitmapInfo)) < 0) {
        LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
        return NULL;
    }

    // 读取 bitmap 的像素内容到 native 内存
    void *bitmapPixels;
    if ((ret = AndroidBitmap_lockPixels(env, bitmap, &bitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    uint32_t newWidth = bitmapInfo.width;
    uint32_t newHeight = bitmapInfo.height;

    uint32_t *newBitmapPixels = new uint32_t[newWidth * newHeight];

    int whereToGet = 0;

    for (int y = 0; y < newHeight; ++y) {
        for (int x = newWidth - 1; x >= 0; x--) {
            uint32_t pixel = ((uint32_t *) bitmapPixels)[whereToGet++];
            newBitmapPixels[newWidth * y + x] = pixel;
        }
    }


    jobject newBitmap = generateBitmap(env, newWidth, newHeight);

    void *resultBitmapPixels;

    if ((ret = AndroidBitmap_lockPixels(env, newBitmap, &resultBitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    int pixelsCount = newWidth * newHeight;

//    memcpy((uint32_t *) resultBitmapPixels, newBitmapPixels, sizeof(uint32_t) * pixelsCount);
    wmemcpy(reinterpret_cast<wchar_t *>((uint32_t *) resultBitmapPixels),
            reinterpret_cast<const wchar_t *>(newBitmapPixels), pixelsCount);

    AndroidBitmap_unlockPixels(env, newBitmap);

    delete[] newBitmapPixels;

    return newBitmap;
}


jobject generateBitmap(JNIEnv *env, uint32_t width, uint32_t height) {

    jclass bitmapCls = env->FindClass("android/graphics/Bitmap");
    jmethodID createBitmapFunction = env->GetStaticMethodID(bitmapCls,
                                                            "createBitmap",
                                                            "(IILandroid/graphics/Bitmap$Config;)Landroid/graphics/Bitmap;");
    jstring configName = env->NewStringUTF("ARGB_8888");
    jclass bitmapConfigClass = env->FindClass("android/graphics/Bitmap$Config");
    jmethodID valueOfBitmapConfigFunction = env->GetStaticMethodID(
            bitmapConfigClass, "valueOf",
            "(Ljava/lang/String;)Landroid/graphics/Bitmap$Config;");

    jobject bitmapConfig = env->CallStaticObjectMethod(bitmapConfigClass,
                                                       valueOfBitmapConfigFunction, configName);

    jobject newBitmap = env->CallStaticObjectMethod(bitmapCls,
                                                    createBitmapFunction,
                                                    width,
                                                    height, bitmapConfig);

    return newBitmap;
}

