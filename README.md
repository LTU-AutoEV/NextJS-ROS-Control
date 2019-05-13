# NextJS-ROS-Control

## Quickstart

### Install Node and rosbridge

```sh
curl -sL https://deb.nodesource.com/setup_11.x | sudo -E bash -
sudo apt-get install nodejs ros-<rosdistro>-rosbridge-suite
```

### Install package and dependencies 

```
git clone https://github.com/LTU-AutoEV/NextJS-ROS-Control.git
cd NextJS-ROS-Control
npm install
```

### Running

Launch rosbridge
```

roslaunch rosbridge_server rosbridge_websocket.launch
```

Launch the server (development mode)

```
npm run dev
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

## Example

To view a ros topic on the page, edit the `TopicDisplay` in the `pages/index.js` file.

For Example, to subscribe to a `std_msgs::String` topic on `/chatter`, change the line to:

```html
<TopicDisplay name="Chatter: " topic="/chatter" type="std_msgs/String" />
```

To test, run the following command:

```
rostopic pub -r 1 /chatter std_msgs/String "Hello world!"
```

