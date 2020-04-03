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
#include "ProcessImage.h"



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

extern "C"
JNIEXPORT jobject JNICALL IMAGE_JNI_METHOD(handleSmoothAndWhiteSkin)(JNIEnv *env, jobject obj, jobject bitmap,
                                                                                           jfloat smoothValue,jfloat whiteValue) {
    AndroidBitmapInfo info;
    void *pixels;
    int ret;

    if (bitmap == NULL){
        LOGD("bitmap is null");
        return NULL;
    }

    if ((ret = AndroidBitmap_getInfo(env, bitmap, &info)) < 0) {
        LOGI("AndroidBitmap_getInfo() failed ! error=%d", ret);
        return NULL;
    }

    if ((ret = AndroidBitmap_lockPixels(env, bitmap, &pixels)) < 0) {
        LOGI("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }

    jobject newBitmap = generateBitmap(env, info.width, info.height);
    void *resultBitmapPixels;
    if ((ret = AndroidBitmap_lockPixels(env, newBitmap, &resultBitmapPixels)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return NULL;
    }
    int pixelsCount = info.width * info.height;
    wmemcpy(reinterpret_cast<wchar_t *>((uint32_t *) resultBitmapPixels),
            reinterpret_cast<const wchar_t *>(pixels), pixelsCount);



    LOGI("Bitmap smooth and whiteskin handle");
    initBeautiMatrix((uint32_t *) resultBitmapPixels, info.width, info.height);

    LOGI("Bitmap smooth = %f and whiteSkin = %f", smoothValue,whiteValue);

    if (smoothValue > 0) {
        setSmooth((uint32_t *) resultBitmapPixels, smoothValue, info.width, info.height);
    }
    if (whiteValue > 0) {
        setWhiteSkin((uint32_t *) resultBitmapPixels, whiteValue, info.width, info.height);
    }

    AndroidBitmap_unlockPixels(env, bitmap);
    AndroidBitmap_unlockPixels(env, newBitmap);

    //free memory code
    freeMatrix();
    return newBitmap;
}


long getNowMs(){
    struct timeval tv;
    gettimeofday(&tv, NULL);//  tv.tv_sec 很大，%36000 取得100个小时前的时间，不除也可以
    int sec = tv.tv_sec%36000;
    long t = sec*1000 + tv.tv_usec/1000;
    return t;
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

void freeMatrix() {
    if (mIntegralMatrix != NULL) {
        free(mIntegralMatrix);
        mIntegralMatrix = NULL;
    }

    if (mIntegralMatrixSqr != NULL) {
        free(mIntegralMatrixSqr);
        mIntegralMatrixSqr = NULL;
    }

    if (mSkinMatrix != NULL) {
        free(mSkinMatrix);
        mSkinMatrix = NULL;
    }

    if (mImageData_rgb != NULL) {
        free(mImageData_rgb);
        mImageData_rgb = NULL;
    }

    if (mImageData_yuv != NULL) {
        free(mImageData_yuv);
        mImageData_yuv = NULL;
    }
}

void initBeautiMatrix(uint32_t *pix, int width, int height) {
    if (mImageData_rgb == NULL)
        mImageData_rgb = (uint32_t *)malloc(sizeof(uint32_t)*width * height);

    memcpy(mImageData_rgb, pix, sizeof(uint32_t) * width * height);

    if (mImageData_yuv == NULL)
        mImageData_yuv = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 4);

    RGBToYCbCr((uint8_t *) mImageData_rgb, mImageData_yuv, width * height);

    initSkinMatrix(pix, width, height);
    initIntegralMatrix(width, height);
}

void initSkinMatrix(uint32_t *pix, int w, int h) {
    LOGE("start - initSkinMatrix");
    if (mSkinMatrix == NULL)
        mSkinMatrix = (uint8_t *)malloc(sizeof(uint8_t) *w *h);
    //mSkinMatrix = new uint8_t[w * h];

    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            int offset = i * w + j;
            ARGB RGB;
            convertIntToArgb(pix[offset], &RGB);
            if ((RGB.blue > 95 && RGB.green > 40 && RGB.red > 20 &&
                 RGB.blue - RGB.red > 15 && RGB.blue - RGB.green > 15) ||//uniform illumination
                (RGB.blue > 200 && RGB.green > 210 && RGB.red > 170 &&
                 abs(RGB.blue - RGB.red) <= 15 && RGB.blue > RGB.red &&
                 RGB.green > RGB.red))//lateral illumination
                mSkinMatrix[offset] = 255;
            else
                mSkinMatrix[offset] = 0;
        }
    }
    LOGE("end - initSkinMatrix");
}

void initIntegralMatrix(int width, int height) {
    LOGI("initIntegral");
    LOGI("width = %d height = %d", width, height);

    if (mIntegralMatrix == NULL)
        mIntegralMatrix = (uint64_t *)malloc(sizeof(uint64_t) * width * height);
    //mIntegralMatrix = new uint64_t[width * height];
    if (mIntegralMatrixSqr == NULL)
        mIntegralMatrixSqr = (uint64_t *)malloc(sizeof(uint64_t) * width * height);
    //mIntegralMatrixSqr = new uint64_t[width * height];

    LOGI("malloc complete");

    //uint64_t *columnSum = new uint64_t[width];
    uint64_t *columnSum = (uint64_t *)malloc(sizeof(uint64_t) * width);
    //uint64_t *columnSumSqr = new uint64_t[width];
    uint64_t *columnSumSqr =(uint64_t *)malloc(sizeof(uint64_t) * width);

    columnSum[0] = mImageData_yuv[0];
    columnSumSqr[0] = mImageData_yuv[0] * mImageData_yuv[0];

    mIntegralMatrix[0] = columnSum[0];
    mIntegralMatrixSqr[0] = columnSumSqr[0];

    for (int i = 1; i < width; i++) {

        columnSum[i] = mImageData_yuv[3 * i];
        columnSumSqr[i] = mImageData_yuv[3 * i] * mImageData_yuv[3 * i];

        mIntegralMatrix[i] = columnSum[i];
        mIntegralMatrix[i] += mIntegralMatrix[i - 1];
        mIntegralMatrixSqr[i] = columnSumSqr[i];
        mIntegralMatrixSqr[i] += mIntegralMatrixSqr[i - 1];
    }

    for (int i = 1; i < height; i++) {
        int offset = i * width;

        columnSum[0] += mImageData_yuv[3 * offset];
        columnSumSqr[0] += mImageData_yuv[3 * offset] * mImageData_yuv[3 * offset];

        mIntegralMatrix[offset] = columnSum[0];
        mIntegralMatrixSqr[offset] = columnSumSqr[0];

        for (int j = 1; j < width; j++) {
            columnSum[j] += mImageData_yuv[3 * (offset + j)];
            columnSumSqr[j] += mImageData_yuv[3 * (offset + j)] * mImageData_yuv[3 * (offset + j)];

            mIntegralMatrix[offset + j] = mIntegralMatrix[offset + j - 1] + columnSum[j];
            mIntegralMatrixSqr[offset + j] = mIntegralMatrixSqr[offset + j - 1] + columnSumSqr[j];
        }
    }

    free(columnSum);
    free(columnSumSqr);
    //delete[] columnSum;
    //delete[] columnSumSqr;
    LOGI("initIntegral~end");
}

void setWhiteSkin(uint32_t *pix, float whiteVal, int width, int height) {
    if (whiteVal >= 1.0 && whiteVal <= 10.0) { //1.0~10.0
        float a = log(whiteVal);

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                int offset = i * width + j;
                ARGB RGB;
                convertIntToArgb(mImageData_rgb[offset], &RGB);
                if (a != 0) {
                    RGB.red = 255 * (log(div255(RGB.red) * (whiteVal - 1) + 1) / a);
                    RGB.green = 255 * (log(div255(RGB.green) * (whiteVal - 1) + 1) / a);
                    RGB.blue = 255 * (log(div255(RGB.blue) * (whiteVal - 1) + 1) / a);
                }
                pix[offset] = convertArgbToInt(RGB);
            }
        }
    }//end if
}

void setSmooth(uint32_t *pix, float smoothValue, int width, int height) {//磨皮操作
    if (mIntegralMatrix == NULL || mIntegralMatrixSqr == NULL || mSkinMatrix == NULL) {//预操作辅助未准备好
        LOGE("not init correctly");
        return;
    }

    LOGE("AndroidBitmap_smooth setSmooth start---- smoothValue = %f", smoothValue);

    RGBToYCbCr((uint8_t *) mImageData_rgb, mImageData_yuv, width * height);

    int radius = width > height ? width * 0.02 : height * 0.02;

    for (int i = 1; i < height; i++) {
        for (int j = 1; j < width; j++) {
            int offset = i * width + j;
            if (mSkinMatrix[offset] == 255) {
                int iMax = i + radius >= height - 1 ? height - 1 : i + radius;
                int jMax = j + radius >= width - 1 ? width - 1 : j + radius;
                int iMin = i - radius <= 1 ? 1 : i - radius;
                int jMin = j - radius <= 1 ? 1 : j - radius;

                int squar = (iMax - iMin + 1) * (jMax - jMin + 1);
                int i4 = iMax * width + jMax;
                int i3 = (iMin - 1) * width + (jMin - 1);
                int i2 = iMax * width + (jMin - 1);
                int i1 = (iMin - 1) * width + jMax;

                float m = (mIntegralMatrix[i4]
                           + mIntegralMatrix[i3]
                           - mIntegralMatrix[i2]
                           - mIntegralMatrix[i1]) / squar;

                float v = (mIntegralMatrixSqr[i4]
                           + mIntegralMatrixSqr[i3]
                           - mIntegralMatrixSqr[i2]
                           - mIntegralMatrixSqr[i1]) / squar - m * m;
                float k = v / (v + smoothValue);

                mImageData_yuv[offset * 3] = ceil(m - k * m + k * mImageData_yuv[offset * 3]);
            }
        }
    }
    YCbCrToRGB(mImageData_yuv, (uint8_t *) pix, width * height);

    LOGI("AndroidBitmap_smooth setSmooth END!----");
}

void convertIntToArgb(uint32_t pixel, ARGB* argb) {
    argb->red = ((pixel >> 16) & 0xff);
    argb->green = ((pixel >> 8) & 0xff);
    argb->blue = (pixel & 0xff);
    argb->alpha = (pixel >> 24);
}

int32_t convertArgbToInt(ARGB argb)
{
    return (argb.alpha << 24) | (argb.red << 16) | (argb.green << 8) | argb.blue;
}

void YCbCrToRGB(uint8_t* From, uint8_t* To, int length)
{
    if (length < 1) return;
    int Red, Green, Blue , alpha;
    int Y, Cb, Cr;
    int i,offset;
    for(i = 0; i < length; i++)
    {
        offset = (i << 1) + i;
        Y = From[offset];
        Cb = From[offset+1] - 128;
        Cr = From[offset+2] - 128;
        Red = Y + ((RGBRCrI * Cr + HalfShiftValue) >> Shift);
        Green = Y + ((RGBGCbI * Cb + RGBGCrI * Cr + HalfShiftValue) >> Shift);
        Blue = Y + ((RGBBCbI * Cb + HalfShiftValue) >> Shift);
        alpha = From[offset+3];

        if (Red > 255) Red = 255; else if (Red < 0) Red = 0;
        if (Green > 255) Green = 255; else if (Green < 0) Green = 0;
        if (Blue > 255) Blue = 255; else if (Blue < 0) Blue = 0;
        offset = i << 2;

        To[offset] = (uint8_t)Blue;
        To[offset+1] = (uint8_t)Green;
        To[offset+2] = (uint8_t)Red;
        To[offset+3] = alpha;
    }
}

void RGBToYCbCr(uint8_t* From, uint8_t* To, int length)
{
    if (length < 1) return;
    int Red, Green, Blue , alpha;
    int i,offset;
    for(i = 0; i < length; i++)
    {
        offset = i << 2;
        Blue = From[offset];
        Green = From[offset+1];
        Red = From[offset+2];
        alpha = From[offset + 3];

        offset = (i << 1) + i;
        To[offset] = (uint8_t)((YCbCrYRI * Red + YCbCrYGI * Green + YCbCrYBI * Blue + HalfShiftValue) >> Shift);
        To[offset+1] = (uint8_t)(128 + ((YCbCrCbRI * Red + YCbCrCbGI * Green + YCbCrCbBI * Blue + HalfShiftValue) >> Shift));
        To[offset+2] = (uint8_t)(128 + ((YCbCrCrRI * Red + YCbCrCrGI * Green + YCbCrCrBI * Blue + HalfShiftValue) >> Shift));
        To[offset + 3] = alpha;
    }
}

void *do_mosaic(void *pix, void *out_pix, unsigned int width, unsigned int height, unsigned int stride,
                unsigned int out_stride, unsigned int radius) {
    if (width == 0 || height == 0 || radius <= 1)
        return pix;

    uint32_t x, y;
    uint32_t a_total = 0;
    uint32_t r_total = 0;
    uint32_t g_total = 0;
    uint32_t b_total = 0;

    uint32_t limit_x = radius;
    uint32_t limit_y = radius;

    uint32_t i = 0;
    uint32_t j = 0;

    int32_t *src_pix = (int32_t *) pix;
    int32_t *out = (int32_t *) out_pix;

    for (y = 0; y < height; y += radius) {
        for (x = 0; x < width; x += radius) {
            //rgba *line = (rgba *) pix;
            limit_y = y + radius > height ? height : y + radius;
            limit_x = x + radius > width ? width : x + radius;

            // get average rgb
            a_total = 0;
            r_total = 0;
            g_total = 0;
            b_total = 0;
            uint32_t count = 0;
            for (j = y; j < limit_y; j++) {
                for (i = x; i < limit_x; i++) {
                    int32_t color = src_pix[j * width + i];

                    //考虑透明像素点
                    uint8_t a = ((color & 0xFF000000) >> 24);
                    uint8_t r = color & 0x000000FF;
                    uint8_t g = ((color & 0x0000FF00) >> 8);
                    uint8_t b = ((color & 0x00FF0000) >> 16);

                    r_total += r;
                    g_total += g;
                    b_total += b;
                    a_total += a;

                    count++;
                }//end for i
            }//end for j

            uint32_t r = r_total / count;
            uint32_t g = g_total / count;
            uint32_t b = b_total / count;
            uint32_t a = a_total / count;

            //ALOGE("total = %d  count = %d ", total , count);
            for (j = y; j < limit_y; j++) {
                for (i = x; i < limit_x; i++) {
                    out[j * width + i] = COLOR_ARGB(a,r,g,b);
                }//end for i
            }//end for j
        }//end for x
    }//end for y

    return pix;
}


