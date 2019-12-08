# DynamixelWrapper C++ クラスライブラリ
---
Dynamixel のサーボモータ（現状では Pro+ のみ対応）のためのC++ラッパーライブラリ．一つのシリアルラインに多数のPro+サーボを接続する状態を想定している．現状では，シリアルラインを一つに限定している．
本ラッパーライブラリは，以下の３つのクラスから構成されている．

- DynamixelNetwork: シリアルラインを制御するクラス
- DynamixelServo: 各サーボに対応するインスタンスを生成する
- DynamixelRobotSystem: 全体のシステムとして管理するためのクラス


---

## DynamixelNetwork

シリアルラインを制御するクラス　DynamixelNetwork は　Singletonとして設計している．
ユーザが直接利用する必要はない．


## DynamixelServo

サーボ一つあたり一つずつインスタンスを作る．


## DynamixelRobotSystem

複数のサーボにより構成されるロボットシステムに関する記述をする．
一般的には継承クラスを作成して用いる．

-----


# ライブラリのビルド


CMakeLists.txt を編集して，プロジェクトパスを正しく設定する．
事前に DynamixelSdk をインストールしておく必要がある．

```
cd DynamixelWrapper
mkdir build
cd build
cmake ..
make
```


これにより，

- dxl_wrapper.a ... ラッパーライブラリ
- ccvtest  ...  SQ2-CCVに合わせたサーボシステムのデモプログラム

が作成される．














