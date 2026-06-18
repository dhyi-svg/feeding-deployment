const ROS_HOST = '192.168.1.2';
const ROS_PORT = 9090;

// Use a secure WebSocket (wss) whenever the page itself is served over HTTPS —
// required on iOS, and browsers block plain ws:// from an HTTPS page (mixed
// content). Falls back to ws:// for normal http dev. NOTE: wss requires
// rosbridge to be served over TLS (SSL on rosbridge_websocket, or a TLS proxy).
const ROS_PROTO =
  (typeof window !== 'undefined' && window.location && window.location.protocol === 'https:')
    ? 'wss'
    : 'ws';
const ROS_URL = `${ROS_PROTO}://${ROS_HOST}:${ROS_PORT}`;
const USER = 'Hi Zen';

export { ROS_URL, USER };
