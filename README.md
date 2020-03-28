# NDKProcessImage
## Introduction

使用 `NDKProcessImage` 实现对图片基本处理，可以用来实现对bitmap的对比度调节、亮度调节、旋转、镜像、高斯模糊、yuv和rgb格式互转等。


## Features

* 支持对比度调节
* 支持亮度调节
* 支持旋转
* 支持镜像
* 支持高斯模糊
* 支持yuv和rgb数据互转


## Getting started

在项目的根节点的 `build.gradle` 中添加如下代码
```
allprojects {
    repositories {
        ...
        maven { url 'https://jitpack.io' }
    }
}
```

在项目的 `build.gradle` 中添加
```
dependencies {
    implementation 'com.github.Kebiiii:NDKProcessImage:v1.0.2'
}
```


## Usage


1. 设置bitmap的亮度和对比度：
- 亮度范围：0-1.0f
- 对比度范围：-1.0f-1.0f
```
mBitmap = NativeImageUtils.brightnessContrastChange(mOriginalBitmap, brightness, contrast);
imageView.setImageBitmap(mBitmap);
```

2. 对图片的argb数据图像加深：相当于亮度设置为0.2f对比度设置0.2f：
```
int[] result = NativeImageUtils.deepImage(buffer, mBitmap.getWidth(), mBitmap.getHeight());
```

3. argb2Yuv： 图片的argb数据转为yuv格式：
```
byte[] yuvData = NativeImageUtils.argb2Yuv(data,width,height);
```
4. yuv420sp2rgb： 图片的yuv格式转为rgb数据：
```
int[] data = NativeImageUtils.yuv420sp2rgb(yuvData,width,height);
```

5. gaussBlur: 对bitmap图片进行高斯模糊处理：
- radius： 模糊半径
```
mBitmap = NativeImageUtils.gaussBlur(mBitmap,30);
imageView.setImageBitmap(mBitmap);
```

6. rotateBitmap: 对bitmap图片90度旋转顺时针：
```
mBitmap = NativeImageUtils.rotateBitmap(mBitmap);
imageView.setImageBitmap(mBitmap);
```

7. convertBitmap: 对bitmap图片上下镜像处理：
```
mBitmap = NativeImageUtils.convertBitmap(mBitmap);
imageView.setImageBitmap(mBitmap);
```

8. mirrorBitmap: 对bitmap图片左右镜像处理：
```
mBitmap = NativeImageUtils.mirrorBitmap(mBitmap);
imageView.mirrorBitmap(mBitmap);
```


### NDKProcessImage 图片处理工具混淆配置
```
-keep class com.kebii.utils.**{*;}
```

## 关于我

* **Email**: <545043382@qq.com>
* **CSDN**: <https://blog.csdn.net/Kebiiii/article/details/105164591>


### 你的 Statr 是我最大的动力，谢谢~~~

## License
--
    Copyright (C) 2018 545043382@qq.com

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
