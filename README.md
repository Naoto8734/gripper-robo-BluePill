# gripper-robo-BluePill
STM32F103(BluePill)用の、CubeIDEのワークスペース。
回路図は以下の通り。
![circuit-board](gripper-robo-controller-board.png)
*28BYJ用のコネクタ及び配線は、使用しなくなったので除去済み*

### グリッパーロボ用のリポジトリ一覧

- [ロボのグリッパー部のリポジトリ(これ)](https://github.com/Naoto8734/gripper-robo-BluePill)
- [ロボのXY直動部のリポジトリ](https://github.com/Naoto8734/xy-axis-robo-BluePill)

## 書き込み方法
OSはUbuntu。[J-Link EDU](https://www.embitek.co.jp/product/jlink-edu.html)を使用し、STM32CubeIDEで生成したbinファイルを書き込み。
**BluePill側の電源(3.3v)を入れておくこと。**
参考：[J-LinkでコマンドラインからマイコンのFlashに書き込む](http://idken.net/posts/2019-07-14-jlinkflash/)

### BluePillとJ-Link EDUとのピン接続

| SWD(JLink) | Pin# | | BluePill |
 ---- | ---- | ---- | ---- | ---- 
| VTref | 1 | |3V3(SWD-Connector) |
| GND | 4 | | GND(SWD-Connector) |
| SWDIO | 7 | | DIO(SWD-Connector) |
| SWCLK | 9 | | DCLK(SWD-Connector) |
| RESET | 15 | | R |

## C++に対応させる方法
参考：[How to Use C++ with STM32CubeIDE - Shawn Hymel](https://shawnhymel.com/1941/how-to-use-c-with-stm32cubeide/)

1. `main.c`を`main.cpp`に名前を変更。
2. `Project Explorer`の画面で、該当のプロジェクトを右クリック。`Convert C++`を選択。
3. Build (これで終了。HALの関数の引数がいくつか変更されるので、Errorに注意。)

### CubeMXで設定を変更した場合の対処法
`main.cpp`に変更しても、CubeMXで生成されるのは`main.c`のままとなる。
そこで、`main.cpp`を`main.c`に改名し、CubeMXからコードを生成する。
これにより、UserCodeの部分がそのまま引き継がれるので、再度`main.cpp`に変更し直す。
面倒くさい。

### 自作クラスの追加方法
1. `Project Explorer`の画面で、該当のプロジェクトを右クリック。`New -> (C++) Class`を選択。
2. Class Name`の欄に、適当な名前を入力して、Finish。
3. Coreフォルダに`***.h`と`***.cpp`ができるので、それぞれIncとSrcフォルダに突っ込む。
4. 作られた`***.h`ファイルで`#include "main.h"`する。
5. `main.cpp`ファイルで`#include "DriveMotor.h"`する。

クラスなどの命名規則：[ja/CppStyleGuide - ROS Wiki](http://wiki.ros.org/ja/CppStyleGuide)
