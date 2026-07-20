# Feeding Robot — Web Interface

Vue 3 single-page application that provides the caregiver-facing UI for the feeding robot. It communicates with the ROS backend over a rosbridge WebSocket connection.

---

## Architecture

```
webapp/
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
cd webapp
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
cd webapp
npm run serve
```

Open `http://localhost:8080` in a browser. The app starts at `/#/task_selection`.

### Build (production)

```bash
cd webapp
npm run build
```

Serve the `dist/` folder from any static file server or point the robot's web server at it.

---

## Researcher intervention timer (port 8081)

`launch_app.sh` (what the `launch_app` alias runs) starts this webapp **plus**
`src/feeding_deployment/integration/researcher_timer.py` — a standalone Flask
page at `http://192.168.1.2:8081` where the researcher timestamps
interventions and explanations during a meal (events outside the system, so
the robot can't log them itself). It writes append-only records to
`log/<user>/day_NN/researcher_events.jsonl` on the robot's clock and depends
on nothing (no roscore / rosbridge / run.py), since interventions happen
exactly when the system is down. Ctrl-C on `launch_app` stops both servers.

Total feeding time is then computed offline:

```bash
cd ../src/feeding_deployment/integration
python compute_feeding_time.py --user <user>            # all days
python compute_feeding_time.py --user <user> --day 3    # one day
# feeding time = meal window - union(marked intervals)
```

---

## Meal review — edit the log after the meal (port 8082)

`launch_app.sh` starts `review_meal.py` in the background too, so it is always
live at `http://192.168.1.2:8082` for the whole session — reopen a day's log in
the browser to edit the note on each intervention / explanation / note, add
entries you missed live, and write free-form end-of-meal notes and your own
thoughts. (It can also be run standalone afterwards:)

```bash
cd ../src/feeding_deployment/integration
python review_meal.py                 # then open http://192.168.1.2:8082
```

Pick a session, edit, and **Save**. Everything is written to a *separate*
`log/<user>/day_NN/researcher_review.json`; the original append-only
`researcher_events.jsonl` is never modified. The first open of a session is
seeded from the original marks; after that it loads your saved review. Removing
an entry tombstones it (`deleted: true`) rather than dropping it.

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
