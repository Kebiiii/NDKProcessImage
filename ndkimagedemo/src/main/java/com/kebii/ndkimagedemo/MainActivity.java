package com.kebii.ndkimagedemo;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;
import android.widget.SeekBar;

import com.kebii.utils.NativeImageUtils;

import java.nio.ByteBuffer;

public class MainActivity extends AppCompatActivity {

    SeekBar brightnessSeekBar;
    SeekBar contrastSeekBar;
    ImageView imageView;
    private Bitmap mOriginalBitmap;
    private Bitmap mBitmap;
    private float brightness = 1.0f;
    private float contrast = 0.0f;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        imageView = findViewById(R.id.imageView);
        mBitmap = BitmapFactory.decodeResource(getResources(), R.mipmap.case1);
        mOriginalBitmap = mBitmap;
        imageView.setImageBitmap(mBitmap);

        brightnessSeekBar = findViewById(R.id.brightness_see_bar);
        contrastSeekBar = findViewById(R.id.contrast_see_bar);
        brightnessSeekBar.setProgress(255);
        contrastSeekBar.setProgress(50);
        brightnessSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {

            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                brightness = seekBar.getProgress() / 255.0f;
            }
        });
        contrastSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {

            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                contrast = seekBar.getProgress() / 50.0f - 1.0f;
            }
        });
    }

    public void onClick(View view) {
        switch (view.getId()) {
            case R.id.srcBtn:
                srcImage();
                break;
            case R.id.ndkImageBtn:
                ndkImage();
                break;
            case R.id.rotateImageBtn:
                rotateImage();
                break;
            case R.id.mirrorImageBtn:
                mirrorImage();
                break;
            case R.id.convertImageBtn:
                convertImage();
                break;
            case R.id.gaussImageBtn:
                gaussImage();
                break;
            case R.id.brightnessContrastChangeBtn:
                brightnessContrastChange();
                break;
        }
    }

    private void brightnessContrastChange() {
        long startTime = System.currentTimeMillis();
        mBitmap = NativeImageUtils.brightnessContrastChange(mOriginalBitmap, brightness, contrast);
        imageView.setImageBitmap(mBitmap);
        Log.e("MainActivity", "brightnessContrastChange time = " + (System.currentTimeMillis() - startTime) + " ms" + mBitmap);
    }

    private void srcImage() {
        mBitmap = mOriginalBitmap;
        imageView.setImageBitmap(mBitmap);
    }

    /**
     * NDK 处理图片
     */
    private void ndkImage() {
        long startTime = System.currentTimeMillis();
        //利用ndk处理
        int width = mBitmap.getWidth();
        int height = mBitmap.getHeight();
        int[] buffer = new int[width * height];
//        //将图片像素放进数组
        mBitmap.getPixels(buffer, 0, width, 0, 0, width, height);
        //经过NDK处理
        int[] result = NativeImageUtils.deepImage(buffer, mBitmap.getWidth(), mBitmap.getHeight());

        //产生新的图片
        mBitmap = Bitmap.createBitmap(result, width, height, Bitmap.Config.ARGB_8888);
        imageView.setImageBitmap(mBitmap);
        Log.e("MainActivity", "ndkImage time = " + (System.currentTimeMillis() - startTime) + " ms");
    }

    /**
     * 旋转 处理图片
     */
    private void rotateImage() {
        long startTime = System.currentTimeMillis();
        mBitmap = NativeImageUtils.rotateBitmap(mBitmap);
        Log.e("MainActivity", "rotateBitmap time = " + (System.currentTimeMillis() - startTime) + " ms");
        imageView.setImageBitmap(mBitmap);
    }

    /**
     * 镜像 处理图片
     */
    private void mirrorImage() {
        long startTime = System.currentTimeMillis();
        mBitmap = NativeImageUtils.mirrorBitmap(mBitmap);
        Log.e("MainActivity", "mirrorImage time = " + (System.currentTimeMillis() - startTime) + " ms");
        imageView.setImageBitmap(mBitmap);
    }

    /**
     * 上下镜像 处理图片
     */
    private void convertImage() {
        long startTime = System.currentTimeMillis();
        mBitmap = NativeImageUtils.convertBitmap(mBitmap);
        Log.e("MainActivity", "convertImage time = " + (System.currentTimeMillis() - startTime) + " ms");
        imageView.setImageBitmap(mBitmap);
    }

    /**
     * 高斯模糊 处理图片
     */
    private void gaussImage() {
        long startTime = System.currentTimeMillis();
        mBitmap = NativeImageUtils.gaussBlur(mBitmap,30);
        Log.e("MainActivity", "gaussImage time = " + (System.currentTimeMillis() - startTime) + " ms");
        imageView.setImageBitmap(mBitmap);
    }

    private byte[] getByteFromBitmap(Bitmap bitmap){
        int bytes = bitmap.getByteCount();
        ByteBuffer buffer = ByteBuffer.allocate(bytes); // Create a new buffer
        bitmap.copyPixelsToBuffer(buffer); // Move the byte data to the buffer
        return buffer.array();
    }

}
