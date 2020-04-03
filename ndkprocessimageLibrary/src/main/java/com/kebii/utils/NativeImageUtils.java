package com.kebii.utils;

import android.graphics.Bitmap;
import android.support.annotation.Keep;

/**
 * Created At 2018/5/2 15:02.
 *
 * @author larry
 */

public class NativeImageUtils {
    static {
        System.loadLibrary("kebii_imageutils");
    }

    @Keep
    public static native int[] deepImage(int[] buffer,int width,int height);

    /**
     *
     * @param bitmap 需要处理的图片
     * @param brightness 亮度 0.0f~1.0f
     * @param constrat 对比度-1.0f~1.0f
     * @return
     */
    @Keep
    public static native Bitmap brightnessContrastChange(Bitmap bitmap,float brightness,float constrat);

    @Keep
    public static native byte[] argb2Yuv(int[] data,int width, int height);

    @Keep
    public static native int[] yuv420sp2rgb(byte[] data,int width, int height);

    @Keep
    public static native Bitmap gaussBlur(Bitmap bitmap, int radius);

    @Keep
    public static native Bitmap rotateBitmap(Bitmap bitmap);

    @Keep
    public static native Bitmap convertBitmap(Bitmap bitmap);

    @Keep
    public static native Bitmap mirrorBitmap(Bitmap bitmap);

    /**
     * @param smoothValue 0~500
     * @param whiteValue  0~10
     */
    @Keep
    public static native Bitmap handleSmoothAndWhiteSkin(Bitmap bitmap,float smoothValue,float whiteValue);

}
