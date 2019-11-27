#### Start camera
`sudo /etc/init.d/motion start`
#### Stop camera
`sudo /etc/init.d/motion stop`
#### Restart camera
`sudo /etc/init.d/motion restart`
#### Config camera
`sudo nano /etc/motion/motion.conf`

Config documentation found [here](https://motion-project.github.io/motion_config.html) (although some options don't seem to work...)
#### View camera stream
1. Start camera
2. Connect to SCR Wi-Fi (if you haven't already)
3. Open CameraStream.html with your favorite web browser
4. You should now see the video stream from the camera in your browser