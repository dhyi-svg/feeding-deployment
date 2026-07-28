---
name: verify
description: Build, launch, and drive the feeding webapp to verify frontend changes at the rendered surface.
---

# Verifying webapp changes

- Build: `npm run build` in `webapp/`. In a worktree, symlink node_modules first:
  `ln -s <main-checkout>/webapp/node_modules webapp/node_modules` (remove the symlink before committing).
- Dev server: `npm run serve -- --port <port> --host 127.0.0.1`. It serves **HTTPS** with a
  self-signed cert — use `https://127.0.0.1:<port>` and ignore cert errors.
- Router is **hash mode**: routes are `https://127.0.0.1:<port>/#/<route>` (e.g. `/#/plate_release_confirm`).
  Route list: `webapp/src/router/index.js`.
- Drive with headless system Chrome (`/usr/bin/google-chrome`) via `npm i puppeteer-core --no-save`
  in a temp dir; launch with `--no-sandbox --ignore-certificate-errors` and `ignoreHTTPSErrors: true`.
- Without a ROS backend, pages render fine but log "Uncaught, unspecified 'error' event"
  (roslib websocket failure) — harmless for layout/UI checks. ROS-driven state (countdowns,
  images, preference steps) never arrives. `preference_correction` renders no content without ROS data.
- ROS points at the real robot (`192.168.1.2:9090`, `src/config/parameterConfig.js`) — never connect
  tests to it. To drive ROS-driven flows, inject a `window.WebSocket` stub via
  `page.evaluateOnNewDocument` that auto-opens and exposes a helper dispatching
  `{op:'publish', topic:'/robot_to_webapp', msg:{data: JSON.stringify({...})}}` to `onmessage` —
  the real roslib/handleRosMessage path then runs. Working example: the `verify_cancel.js`
  driver pattern (job tmp, session 2026-07-17). Message shapes: most confirm pages arm on
  `{state:'<page>', status:'jump', autocontinue_seconds:N}`; after_bite/after_drink on
  `{state:'auto_time', status:'<secs>'}`. Views publish to `/webapp_to_robot` (capture sends for asserts).
