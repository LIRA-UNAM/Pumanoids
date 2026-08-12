# Packaging the GameController and Joystick Release

## Build and Stage the Artifacts
```
colcon build
cp -r install ./src/game_controller/utils/game_controller_package

```

## Create the Installer

```
cd ./src/game_controller/utils/game_controller_package

makeself ../game_controller_package ../game_controller_0.0.2.run "booster_game_controller" ./install.sh
```

## Update the Installed GameController Configuration
```
vim /opt/robocup/install/game_controller/share/game_controller/launch/launch.py
```
