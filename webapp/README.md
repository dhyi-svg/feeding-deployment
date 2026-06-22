# Feeding Robot — Web Interface

Vue 3 single-page application that provides the caregiver-facing UI for the feeding robot. It communicates with the ROS backend over a rosbridge WebSocket connection.

---

## Architecture

```
webapp/
└── vue-ros-demo/       # Vue 3 SPA (all frontend code lives here)
    ├── src/
    │   ├── App.vue             # Root component; shared ROS connection + skill-plan overlay
    │   ├── main.js
    │   ├── router/
    │   │   ├── index.js        # Route definitions (all snake_case paths)
    │   │   └── routeMap.js     # Maps {state, status} ROS messages → route paths
    │   ├── config/
    │   │   └── parameterConfig.js  # ROS host/port and user display name
    │   ├── views/              # One .vue file per page (filenames match route paths)
    │   └── assets/             # Images used by views
    └── public/
        └── index.html
```

### ROS Topics

| Topic | Direction | Type | Purpose |
|---|---|---|---|
| `/robot_to_webapp` | robot → app | `std_msgs/String` | JSON `{state, status}` messages that drive page navigation |
| `/webapp_to_robot` | app → robot | `std_msgs/String` | JSON `{state, status}` messages reporting user actions |
| `/skill_plan` | robot → app | `std_msgs/String` | Latched skill plan displayed in the overlay |
| `/camera/image/compressed` | robot → app | `sensor_msgs/CompressedImage` | Camera feed (used on select pages) |
| `/shared_autonomy/takeover` | app → robot | `std_msgs/Empty` | Signals teleop takeover |
| `/shared_autonomy/done` | app → robot | `std_msgs/Empty` | Signals end of teleop |

### Message protocol

All `/robot_to_webapp` and `/webapp_to_robot` messages are JSON strings:

```json
{ "state": "<page_name>", "status": "<action>" }
```

`routeMap.js` maps every `{state, status}` pair the backend can send to the corresponding frontend route.

---

## Setup

### Prerequisites

- Node.js ≥ 18 and npm
- Vue CLI: `npm install -g @vue/cli`

### Install

```bash
cd webapp/vue-ros-demo
npm install
```

### Configure

Edit `src/config/parameterConfig.js` and set the IP address of the robot running rosbridge:

```js
const ROS_HOST = '192.168.1.2';   // robot's IP on the LAN
const ROS_PORT = 9090;
const USER = 'Hi Aimee';            // display name shown in the top bar
```

### Run (development)

```bash
# In one terminal — start rosbridge on the robot
roslaunch rosbridge_server rosbridge_websocket.launch   # ROS 1
# or
ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # ROS 2

# In another terminal — start the dev server
cd webapp/vue-ros-demo
npm run serve
```

Open `http://localhost:8080` in a browser. The app starts at `/#/task_selection`.

### Build (production)

```bash
cd webapp/vue-ros-demo
npm run build
```

Serve the `dist/` folder from any static file server or point the robot's web server at it.

---

## Useful ROS commands (debugging)

```bash
# Send a message to the webapp (simulates robot output)
rostopic pub -1 /robot_to_webapp std_msgs/String \
  "data: '{\"state\": \"bite_selection\", \"status\": \"jump\"}'"

# Watch messages from the webapp (user actions)
rostopic echo /webapp_to_robot

# Watch the skill plan
rostopic echo /skill_plan
```

---

## Notes

- The app uses hash-history routing (`/#/...`) so it works without a configured server rewrite rule.
- If rosbridge is not running, the app still navigates normally but cannot send or receive ROS messages. The browser may show a WebSocket connection error in the console.
- Microphone access (used on the transparency and adaptability pages) requires the browser to grant permission. Some browsers block this on plain `http://`; use `https://` or `localhost` in that case.
