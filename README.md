# MEAM 5100 Final Project

This project was for MEAM 5100 Mechatronics at Penn. The goal of the project was to design, build, and program a semi autonomous robot that could complete various tasks autonomously, 
including wall following, beacon tracking (frequency detection), and moving to target coordinates. 

We used an ESP32 S2 Saola 1 board for our robot. We used 2 Time of Flight sensors for wall following, 2 photoresistors mounted on a servo for beacon tracking, and 2 Vive photodiodes 
for localization. The robot was controlled from a custom webpage over Wi-Fi. UDP communication was used to receive the coordinates of a target object the robot had to locate, move to, 
and then move 10 inches.

<video src="https://drive.google.com/file/d/1YkIFjRe0YPl5AZdIKdeFspM_RymJJNJw/view?usp=sharing" controls width="600"></video>

