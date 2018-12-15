package com.kebii.ndkimagedemo;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;

import com.kebii.utils.JavaImageUtils;
import com.kebii.utils.NativeImageUtils;

import java.nio.ByteBuffer;

public class MainActivity extends AppCompatActivity {

    ImageView imageView;
    private Bitmap mBitmap;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(com.kebii.ndkprocessimage.R.layout.activity_main);
        imageView = findViewById(com.kebii.ndkprocessimage.R.id.imageView);
        mBitmap = BitmapFactory.decodeResource(getResources(), com.kebii.ndkprocessimage.R.mipmap.case1);

    }

    public void srcImage(View view) {
        mBitmap = BitmapFactory.decodeResource(getResources(), com.kebii.ndkprocessimage.R.mipmap.case1);
        imageView.setImageBitmap(mBitmap);
    }

    /**
     * JAVA 处理图片
     * @param view
     */
    public void javaImage(View view) {
        long startTime = System.currentTimeMillis();
        mBitmap = JavaImageUtils.getImage(mBitmap);
        imageView.setImageBitmap(mBitmap);
        Log.e("MainActivity", "javaImage time = " + (System.currentTimeMillis() - startTime) + " ms");
    }

    public void ndkImage(View view) {
        long startTime = System.currentTimeMillis();
        //利用ndk处理
        int width = mBitmap.getWidth();
        int height = mBitmap.getHeight();
        int[] buffer = new int[width * height];
//        //将图片像素放进数组
        mBitmap.getPixels(buffer, 0, width, 0, 0, width, height);
        //经过NDK处理
        int[] result = NativeImageUtils.getImage(buffer, mBitmap.getWidth(), mBitmap.getHeight());
        Log.e("MainActivity", "ndkImage getImage result.length= "+result.length + ",result:"
                + result[0]+","+result[1]+","+result[2]+","+result[3]+","+result[4]);
//        int[] result2 = NativeImageUtils.yuv420sp2rgb(getByteFromBitmap(mBitmap), mBitmap.getWidth(), mBitmap.getHeight());
//        Log.e("MainActivity", "ndkImage yuv420sp2rgb result2.length= "+result2.length + ",result2:"
//                + result2[0]+","+result2[1]+","+result2[2]+","+result2[3]+","+result2[4]);

        //产生新的图片
        mBitmap = Bitmap.createBitmap(result, width, height, Bitmap.Config.RGB_565);
        imageView.setImageBitmap(mBitmap);
        Log.e("MainActivity", "ndkImage time = " + (System.currentTimeMillis() - startTime) + " ms");
    }

    public byte[] getByteFromBitmap(Bitmap bitmap){
        int bytes = bitmap.getByteCount();
        ByteBuffer buffer = ByteBuffer.allocate(bytes); // Create a new buffer
        bitmap.copyPixelsToBuffer(buffer); // Move the byte data to the buffer
        return buffer.array();
    }

}
