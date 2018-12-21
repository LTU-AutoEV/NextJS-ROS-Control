# NextJS-ROS-Control

## Quickstart

```sh
curl -sL https://deb.nodesource.com/setup_11.x | sudo -E bash -
sudo apt-get install nodejs ros-<rosdistro>-rosbridge-suite
npm install
npm run dev
roslaunch rosbridge_server rosbridge_websocket.launch
```

Open [localhost:3000](http://localhost:3000/)

## Running

Debug build (auto-updating):
```sh
npm run dev
```

Optimized build:
```sh
npm run build
npm run start
```

## Updating Packages

This repo has versioned packages in `package.json`. These version are
guaranteed to work together, but they may be outdated. Up update everything to
latest, run `npm update`.
