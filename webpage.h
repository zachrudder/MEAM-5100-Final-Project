#include "pgmspace.h"
const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
<head>
    <title>MEAM 5100 Final Project</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            background-color: #f4f4f4;
            margin: 0;
            padding: 20px;
        }

        h1 {
            color: #333;
        }

        .arrow-button {
            background-color: #4CAF50;
            color: white;
            padding: 10px 15px;
            margin: 10px 5px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            transition: 0.3s;
            font-size: 20px;
        }

        .arrow-button:hover {
            background-color: #45a049;
        }

        .stop-button {
            background-color: red;
            color: white;
            padding: 10px 15px;
            margin: 10px 5px;
            border: none;
            border-radius: 50%; /* Makes the button circular */
            cursor: pointer;
            transition: 0.3s;
            font-size: 20px;
        }

        .stop-button:hover {
            background-color: darkred;
        }

        p {
            color: #555;
        }
    </style>
</head>
<body>

<h1>MEAM 5100 Final Project</h1>
  
<p>Press to turn on motors (always press at beginning).</p>
<button id="motorPowerButton" class="button" onclick="hitMotorPower()"> &#9650; Start motors. </button>

<p>Press to move robot.</p>
<button id="forwardButton" class="arrow-button" onclick="hitForward()"> &#9650; Move Forward </button> <!-- Up arrow -->

<button id="stopButton" class="stop-button" onclick="hitStop()"> Stop </button> <!-- Stop button -->

<button id="backButton" class="arrow-button" onclick="hitBack()"> &#9660; Move Backwards </button> <!-- Down arrow -->

<p>Press to turn robot.</p>
<button id="leftButton" class="arrow-button" onclick="hitLeft()"> &#9664; Turn Left </button> <!-- Left arrow -->

<button id="rightButton" class="arrow-button" onclick="hitRight()"> &#9654; Turn Right </button> <!-- Right arrow -->

<p>Press to enter an autonomous mode.</p>
<button id="wallFollowingButton" class="button" onclick="hitWallFollowing()"> &#9654; Perform Wall Following. </button> 

<button id="beaconTrackingButton" class="button" onclick="hitTrackBeacon()"> &#9654; Perform Beacon Tracking. </button> 

<button id="locatePoliceCar" class="button" onclick="hitPoliceCar()"> &#9654; Locate Police Car. </button> 



<script>      
    function hitLeft(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitLeft", true);
        xhttp.send();
    }
    
    function hitRight(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitRight", true);
        xhttp.send();
    }
    
    function hitForward(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitForward", true);
        xhttp.send();
    }
    
    function hitBack(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitBack", true);
        xhttp.send();
    }
    
    function hitStop(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitStop", true);
        xhttp.send();
    }
    
    function hitWallFollowing(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitWallFollowing", true);
        xhttp.send();
    }
    
    function hitTrackBeacon(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitTrackBeacon", true);
        xhttp.send();
    }
    
    function hitPoliceCar(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitPoliceCar", true);
        xhttp.send();
    }
    
    function hitMotorPower(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitMotorPower", true);
        xhttp.send();
    }
</script>

</body>
</html>
</html>




)===";