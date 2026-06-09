# lecture_ros2_01

ロボットに簡単な動作をさせるサンプルコードです。

## simple_move.py

シミュレータ、もしくは、実機ロボットの`Nav`ボタンでナビゲーションを起動してから実行してください。

シミュレータの場合はVSCode上で`Ctrl+Shigt+@`でコマンドターミナルを表示し、以下のコマンドを入力して`Enter`キーを押します。

```shell
ros2 run lecture_ros2_01 simple_move.py
```

デフォルトでは5秒かけて1m前進し、3秒で90度旋回します。
プログラムを修正して色々な動きをさせてみましょう。

実機ロボットでも実行してみましょう。
デスクトップ上のタスクバーやドック（画面の端にある起動用アイコン）からターミナル（黒い画面のようなアイコン）を開き、次のコマンドを実行してください。

```shell
cd ~/ros2_ws/src
git clone https://github.com/KMiyawaki/lecture_ros2_01.git
cd lecture_ros2_01
./build.sh -a
code . # VSCodeインストール済みならこれで編集可能です。無ければ、適当なエディタでプログラムを編集してください。
# 実行はシミュレータと同じです。ロボットの「Nav」ボタンで自律移動アプリを起動してから実行してください。
```

## simple_navigation.py

シミュレータの`Nav`ボタンでナビゲーションを起動してから実行してください。
シミュレータは再起動してから実行した方がよいです。

シミュレータの場合はVSCode上で`Ctrl+Shigt+@`でコマンドターミナルを表示し、以下のコマンドを入力して`Enter`キーを押します。

```shell
ros2 run lecture_ros2_01 simple_navigation.py
```

4か所の座標を回ります。

単純に、目標ごとに停止しながら巡回するだけなら[FollowWaypoints](https://zenn.dev/katsuitoh/articles/21adc95b5f4c2d)の方が簡単です。
しかし、長距離の自律移動では、逐一停止させることはできません。
ゴールまでの距離が一定範囲内に入ったら次の目標を送信する必要があります。

[FollowPath](https://docs.nav2.org/configuration/packages/bt-plugins/actions/FollowPath.html)を使うこともかんがえられますが、これは障害物回避をしません。

また、横断歩道など特定のウェイポイントに来たら停止するなどの制御も必要となります。
挑戦してみてください。
