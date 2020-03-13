# 注視駆動可変焦点双眼ルーペシステム

OpenVRベースのHMDでビデオシースルーするためのプログラムです。</br>
注視位置の奥行きに基づいてステレオカメラの焦点距離を調節することができます。</br>
プログラムはOpenVRのサンプル[*hellovr_opengl*](https://github.com/ValveSoftware/openvr/tree/master/samples)の改変により作成しました。

## 使用デバイス

- IDS社製 UI-3881LE AF 可変焦点レンズ調節可能カメラモジュール x2
  > [製品情報](https://jp.ids-imaging.com/store/products/cameras/ui-3881le-af.html)

- Corning社製 Varioptic Lenses C-S-25H0-026 液体レンズ x2
  > [製品情報](https://www.corning.com/jp/jp/innovation/corning-emerging-innovations/corning-varioptic-lenses/auto-focus-lens-modules-c-s-series/varioptic-C-S-25H0-026.html)

- USB3.1 拡張ボード x2
  > [この製品](https://www.asus.com/Motherboard-Accessories/USB_31_TYPEA_CARD/)が最も安定動作していた

- HTC Vive Pro
  > [製品情報](https://www.vive.com/jp/product/vive-pro/)

- Pupil Labs HTC Vive Binocular Add-on
  > [製品情報](https://pupil-labs.com/products/vr-ar/)

## 実行環境

| Item | Spec |
| :--- | :---: |
| OS | Windows10 64bit |
| CPU | Intel Core i7-9700K |
| RAM | 16 GB |
| GPU | NVIDIA GeForce RTX 2070 |

## 依存ライブラリ

- For HMD
  - [OpenVR](https://github.com/ValveSoftware/openvr)
  - SDL2
  - GLEW 1.11+
- For Camera
  - [uEye camera SDK](https://jp.ids-imaging.com/downloads.html)
- For Eye Tracking
  - ZeroMQ
  - cppzmq
  - msgpack for C/C++

HMD用のライブラリはOpenVRのサンプルに同梱されています。</br>
カメラSDKは入手するために無料の会員登録が必要です。カメラキャリブレーションにも利用するアプリケーションが含まれています。</br>
視線検出のためには[Pupil labsのソフトウェア](https://github.com/pupil-labs/pupil/releases)で予めアイトラッカと通信できるようにする必要があります。*Pupil service*を起動した状態で本プログラムを実行して下さい。

## ビルド

cmakeを利用してビルド可能です。</br>
しかし、CMakeFileの書き方に慣れていなかったため、ビルドに成功してもライブラリのパスが通りません。手動で設定してください。

### 手順

1. 依存ライブラリのインストール

    自らビルドする必要があるライブラリは**For Eye Tracking**に示した3つです。</br>
    ***CMAKE_PREFIX_PATH***という環境変数を作成し、ライブラリのビルド時にできるフォルダのパスを登録して下さい。
    ![cmake_path](https://user-images.githubusercontent.com/15144450/76413502-e425f500-63d8-11ea-80a2-fc6f9d94f0a1.png)

      > プログラムにパスを通してくれないので上手く機能していませんが、cmakeによるビルド時にライブラリを認識させるためです。[(参考サイト)](プログラムにパスを通してくれないので上手く機能していませんが)

2. cmakeによるビルド

    cmake GUIで*augfocus*フォルダを指定し、ビルドして下さい。</br>
    その際、Visual Studioのツールセットをv141(Visual Studio 2017のもの)に指定してください。</br>
    ![cmake](https://user-images.githubusercontent.com/15144450/76413465-d4a6ac00-63d8-11ea-8cf0-90b1b87fe6bd.png)




We will use the command-line on Unix and [Git Bash](https://git-for-windows.github.io/) on Windows.

First, move from the repository root to the samples directory to create a build directory:
```
cd samples
mkdir build; cd build
```

Then, depending on your system:

### Unix

Generate the CMake cache using Makefile:
```
cmake .. -G Makefile -DCMAKE_PREFIX_PATH=/opt/Qt/5.6/gcc_64/lib/cmake -DCMAKE_BUILD_TYPE=Release
```

To build type:
```
make -j4
```


### Windows

Generate the CMake cache using MSVC 12 for x64:
```
cmake .. -G "Visual Studio 12 2013 Win64" -DCMAKE_PREFIX_PATH=C:/Qt/5.6/msvc2013_64/lib/cmake
```

Alternatively, you can force the compilation on x86 architectures by using the **PLATFORM** property (*be sure to use the right generator and Qt binaries*):
```
cmake .. -G "Visual Studio 12 2013" -DCMAKE_PREFIX_PATH=C:/Qt/5.6/msvc2013/lib/cmake -DPLATFORM=32
```

To build, simply type:
```
cmake --build . --target all --config Release
```

*Note : using CMake, the build configuration type (ie. Debug, Release) is set at Build Time with MSVC and at Cache Generation Time with Makefile.*

## Usage


---
