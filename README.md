# 概要

タートルボット 3 の動作テスト（docker, gazebo, wsl, gpu）

# 手順

## リポジトリをクローン

```bash
# ディレクトリの作成・移動
mkdir -p workspace/src
cd workspace/src

# リポジトリのクローン
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
```

## コンテナ起動

```bash
# docker compose build --no-cache

docker compose up -d

docker compose exec ros2 bash
```

## サンプル

### キーボード操作

ターミナル 1:シミュレータ起動

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

ターミナル 2:キーボードでのロボット操作

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### SLAM

ターミナル 1:シミュレータ起動

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

ターミナル 2:SLAM ノード起動

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

ターミナル 3:キーボードでのロボット操作

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

ターミナル 4:MAP 保存

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_world
```

### Nav2

ターミナル 1:シミュレータ起動

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

ターミナル 2:ナビゲータ起動

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true slam:=False autostart:=true map:=$HOME/maps/my_world.yaml
```

Nav2 解説記事: https://qiita.com/sfc_nakanishi_lab/items/028edb3a7d5ed0300e33  
例 1: https://www.youtube.com/watch?v=r4fIkcktZUM  
例 2: https://www.youtube.com/watch?v=ANgA5AWzWeo

### キーボード操作（マニュピュレータ）

ターミナル 1:シミュレータ起動

```bash
ros2 launch turtlebot3_manipulation_gazebo gazebo.launch.py
```

ターミナル 3:キーボードでのロボット操作

```bash
ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py use_sim_time:=true
```

ターミナル 3:キーボードでのロボット操作

```bash
ros2 run turtlebot3_manipulation_teleop turtlebot3_manipulation_teleop
```

## コンテナ終了

```bash
docker compose down
```
