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









