package com.kebii.utils;

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
    public static native int[] getImage(int[] buffer,int width,int height);
    @Keep
    public static native byte[] argb2Yuv(int[] data,int width, int height);
    @Keep
    public static native int[] yuv420sp2rgb(byte[] data,int width, int height);
    @Keep
    public static native byte[] rotateYUV420Degree180(byte[] data,int width, int height);
}
