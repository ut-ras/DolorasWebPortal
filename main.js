var world_origin_x = CANVAS_WIDTH/2, 
    world_origin_y = CANVAS_HEIGHT/2,
    world_width = 25, 
    world_height = 25;

var CANVAS_WIDTH, CANVAS_HEIGHT;

var cur_angle = 0;

window.onload = startup;

function startup() {
    var canvas = document.getElementById("canvas");
    var context = canvas.getContext("2d");
    
    CANVAS_WIDTH = canvas.width;
    CANVAS_HEIGHT = canvas.height;

    context.setTransform(1,0,0,1,1,1);
    context.translate(CANVAS_WIDTH/2, CANVAS_HEIGHT/2);
    context.scale(CANVAS_WIDTH/world_width, -CANVAS_HEIGHT/world_height);

    init_websocket();
}

//
// Websocket stuff
//

function init_websocket() {
    var connection = new ros.Connection("ws://doloras.chickenkiller.com:9090");
    
    connection.setOnClose( function(e) {
        console.log('connection closed');
    });

    connection.setOnError( function(e) {
        console.log('error:',e);
    });

    connection.addHandler('/imu_data', function(data) {
        updateLidarDisplayPose(0, 0, data.yaw);
    });

    connection.addHandler('/scan', function(data) {
        var points = [];
        var max_range = data.range_max,
            min_angle = data.angle_min,
            max_angle = data.angle_max,
            angle_inc = data.angle_increment;

        var i = 0;
        for (var angle = min_angle; angle < max_angle; angle += angle_inc) {
            var range = data.ranges[i],
                isValid = true;

            if (range == 0) {
                range = max_range;
                isValid = false;
            }

            points.push([range*Math.cos(angle+Math.PI/2), 
                         range*Math.sin(angle+Math.PI/2),
                         isValid]);
            i++;
        }

        drawPoints(points);
    });

    connection.setOnOpen( function(e) {
        console.log('connected to ROS');
        connection.callService('/rosjs/subscribe','["/imu_data",-1]',function(e) {});
        connection.callService('/rosjs/subscribe','["/scan",-1]',function(e) {});
    });
}

//
// IMU udpate
//

function updateLidarDisplayPose(offsetx, offsety, yaw) {
    var context = document.getElementById("canvas").getContext("2d");
    context.setTransform(1,0,0,1,1,1);
    context.translate(CANVAS_WIDTH/2+offsetx, CANVAS_HEIGHT/2+offsety);
    context.scale(CANVAS_WIDTH/world_width, -CANVAS_HEIGHT/world_height);
    context.rotate(yaw);
}

//
// LIDAR display functions
//

function drawPoints(points) {
    var canvas = document.getElementById("canvas");
    var context = canvas.getContext("2d");

    context.fillStyle = "lightGray";
    context.fillRect(-world_width, -world_height, 2*world_width, 2*world_height);
    
    context.lineWidth = .1;
    context.strokeStyle = "gray";
    context.fillStyle = "black";

    for (var i = 0; i < points.length; i++) {
        context.beginPath();
        context.moveTo(0, 0);
        context.lineTo(points[i][0], points[i][1]);
        context.stroke();
    }
    
    for (var i = 0; i < points.length; i++) {
        if (points[i][2]) {
            context.beginPath();
            context.arc(points[i][0], points[i][1], .05, 0, 2*Math.PI, true);
            context.fill();
        }
    }

    context.lineWidth = .05;
    context.strokeStyle = "blue";
    context.fillStyle = "blue";
    drawOrigin(context);
}   

function drawOrigin(context) {
    context.beginPath();
    context.moveTo(0,0);
    context.lineTo(0, 1);
    context.lineTo(.2, .8);
    context.stroke();

    context.beginPath();
    context.moveTo(0, 1);    
    context.lineTo(-.2, .8);
    context.stroke();

    context.beginPath();
    context.arc(0, 0, .1, 0, 2*Math.PI, true);
    context.fill();
}

//
// Command/Control functions
//

var keyToDirMap = {};
keyToDirMap["W".charCodeAt(0)] = "forward";
keyToDirMap["S".charCodeAt(0)] = "backward";
keyToDirMap["A".charCodeAt(0)] = "left";
keyToDirMap["D".charCodeAt(0)] = "right";

var dirToValsMap = {
    "forward" :     {x:    0, y:  127},
    "backward" :    {x:    0, y: -127},
    "left" :        {x: -127, y:    0},
    "right" :       {x:  127, y:    0}
}

function keypressed(event) {
    var key = event.which;
    sendCommand(keyToDirMap[key]);
}

function sendCommand(dir) {
    var vals = dirToValsMap[dir];

    if (vals) {
        getFile(url+"?x="+vals.x+"&y="+vals.y, false, true);
    }
}
