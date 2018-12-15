//
// Created by liuke on 2018/5/2.
//

//#include "com_kebii_ndkprocessimage_NativeImageUtils.h"
#include <jni.h>
#include <string>

#define IMAGE_JNI_METHOD(METHOD_NAME) \
  Java_com_kebii_utils_NativeImageUtils_##METHOD_NAME


extern "C"
JNIEXPORT jintArray JNICALL IMAGE_JNI_METHOD(getImage)
    (JNIEnv *env,jobject obj,jintArray buffer,jint width,jint height){
    jint* source = (*env).GetIntArrayElements(buffer,0);
    int newSize = width * height;
    float brightness = 0.2f;
    float constrat = 0.2f;

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
            //变化对比对
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
JNIEXPORT jbyteArray JNICALL IMAGE_JNI_METHOD(argb2Yuv)(JNIEnv* env, jobject thiz,jintArray argb, int width, int height) {
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
JNIEXPORT jintArray JNICALL IMAGE_JNI_METHOD(yuv420sp2rgb)(JNIEnv* env, jobject thiz,jbyteArray yuv420sp, int width, int height) {

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