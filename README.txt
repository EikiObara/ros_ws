
# チュートリアル履修を推奨します（できれば原文のほうが情報新しいです．）

ROS　原文チュートリアル
http://wiki.ros.org/ROS/Tutorials

ROS 日本語版チュートリアル
http://wiki.ros.org/ja/ROS/Tutorials

# ROSのインストール

ここをみてください．
参考：http://wiki.ros.org/ja/ROS/Tutorials/InstallingandConfiguringROSEnvironment



# ROSのワークスペース初期化
mkdir <ros_ws>を行い，~/ros_ws内で

$ catkin_init_workspace
$ . ./update_command.sh

で初期化されます．

参考：http://wiki.ros.org/ja/ROS/Tutorials/InstallingandConfiguringROSEnvironment



#パッケージの作り方

$ catkin_create_pkg <作りたいパッケージ名> <必要なライブラリ群 ex. roscpp std_msgs>

package.xmlは<必要なライブラリ群>を与えてcatkin_create_pkgをした場合，最初から記述される．
<必要なライブラリ群>を設定し忘れたら，以下の参考を読みつつ，必要なライブラリを記述するんだよ～．

参考：http://wiki.ros.org/ja/ROS/Tutorials/CreatingPackage



# プログラムを変更したとき

$ . ./update_command.sh
を実行すると、ROS_ws内のすべてのプログラムが更新されます



# rosのノードのテスト
ROSを動かすためにはroscoreが起動している必要があるため，以下コマンド．

$roscore

その後，動かしたいノードを走らせる．

$rosrun <パッケージ名> <ノード名>



# launchファイルの走らせ方

$ roslaunch <パッケージ名> <launchファイル名>.launch

で走るよ．
roslaunchで動かす場合，roscoreもバックグラウンドで勝手に走るよ．便利だね！



# デバッグに使えるやつら
roscoreが走っている状態で．

現在動作しているノードを確認するには
$ rosrun rqt_graph rqt_graph

デバッグコードの確認には
$ rosrun rqt_console rqt_console

プログラムの動作レベルをいじったりするには
$ rosrun rqt_logger_level rqt_logger_level

を使うのが基本的なんだとさ．
