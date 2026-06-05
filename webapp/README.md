# Feeding Project Frontend and Communication

Hello!

##### <u>For ROS(Backend):</u>

ROS1

`roslaunch rosbridge_server rosbridge_websocket.launch`

`rostopic pub /ServerComm std_msgs/String "data: '{\"state\": \"drink_transfer\", \"status\": \"completed\"}'"`

`rostopic echo /WebAppComm`

ROS2

`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

##### <u>For Web Pages (Frontend):</u>

If you don't have an environment for vue3 development, please install first. 

1. Install Node.js and npm. 
   Link:  [Node.js — Run JavaScript Everywhere (nodejs.org)](https://nodejs.org/en)
2. Install vue cli

   `npm install -g @vue/cli`

3. Install vue router 

   `npm install vue-router@4`

If you already have a Vue3 development environment(My version: vue/cli 5.0.8+vue-router 4.0.13), please execute the following command directly.

1. Clone or download:

    `git clone https://github.com/SKYETN/feedingpage.git`

2. Open the folder(eg `D:\pagefeeding> cd vue-ros-demo`) : 

   `cd vue-ros-demo`    

3. Install the Dependencies(eg `D:\pagefeeding\vue-ros-demo> npm i`) 

   `npm i`   

   If error, please execute `npm install --force`

4. Run(eg `D:\pagefeeding\vue-ros-demo> npm run serve`) : 

   `npm run serve`   

5. Open your browser and type in the address : 

   http://localhost:8080/#/home

6. Change the IP address in "pagefeeding\vue-ros-demo\src\config\parameterConfig.js" : 

   const ROS_URL = 'http://192.168.3.187:9090';

Other reminders:

1. If you keep ROS closed, but run and open the web page, sometimes the web page will show an error saying that it is not properly connected(Uncaught runtime errors). For this case, you can open ROS and refresh the webpage, or keep ROS closed and turn off the prompt directly in the top right corner (web can't send/receive messages now but still can perform other functions).
2. ROS and web pages communicate on port 9090 on the same LAN (can be different devices).
4. When using the voice function, please ensure that the browser has the necessary access permissions. For example, Edge browser may deny microphone access without displaying a prompt.

##### <u>Python Example:</u>

Please refer to the front-end API documentation

| name                                                      | function                                |
| --------------------------------------------------------- | --------------------------------------- |
| test_message_publisher | Post a bot text reply to the front end  |
| test_image_publisher_OnlyforTransparencyAndAdaptability   | Post a bot image reply to the front end |
| test_video_pulisher_OnlyforTransparencyAndAdaptability    | Post a bot video reply to the front end |
| test_video_subscriber_OnlyforGestureWorkflow              | Play the video passed by the front-end  |



