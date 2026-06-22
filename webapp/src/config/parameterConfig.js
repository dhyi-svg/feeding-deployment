const ROS_HOST = '192.168.1.2';
const ROS_PORT = 9090;

const ROS_PROTO =
  (typeof window !== 'undefined' && window.location && window.location.protocol === 'https:')
    ? 'wss'
    : 'ws';
const ROS_URL = `${ROS_PROTO}://${ROS_HOST}:${ROS_PORT}`;
const USER = 'Hi Aimee';

export { ROS_URL, USER };
