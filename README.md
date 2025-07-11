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
```

## コンテナ起動

```bash
docker compose up -d --build
```

## シミュレーション開始&操作

ターミナル 1:

```bash
docker compose exec ros2 bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

ターミナル 2:

```bash
docker compose exec ros2 bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## コンテナ終了

```bash
docker compose down
```
