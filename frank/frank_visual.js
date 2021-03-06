var IMAGE_DATA_URL = 'http://doloras.kicks-ass.org:8080/stream?topic=/green',//usb_cam/image_raw',
    YAW_OFFSET = 0;

var world_origin_x = CANVAS_WIDTH/2,
    world_origin_y = CANVAS_HEIGHT/2,
    world_width = 25,
    world_height = 25;

var CANVAS_WIDTH, CANVAS_HEIGHT;

var cur_angle = 0, cur_points = [], connection;

window.onload = main;

function main() {
    var canvas = document.getElementById("canvas");
    var context = canvas.getContext("2d");

    CANVAS_WIDTH = canvas.width;
    CANVAS_HEIGHT = canvas.height;

    context.setTransform(1,0,0,1,1,1);
    context.translate(CANVAS_WIDTH/2, CANVAS_HEIGHT/2);
    context.scale(CANVAS_WIDTH/world_width, -CANVAS_HEIGHT/world_height);

    init_websocket();
    init_image_viewer();

    setInterval(redraw, 100);
};

//
// Image viewer stuff
//

function init_image_viewer() {
  document.getElementById('image_viewer').setAttribute('src', IMAGE_DATA_URL);
}

//
// Websocket stuff
//

function init_websocket() {
    connection = new ros.Connection("ws://doloras.kicks-ass.org:9090");

    connection.setOnClose( function(e) {
        console.log('connection closed');
    });

    connection.setOnError( function(e) {
        console.log('error:',e);
    });

    connection.addHandler('/imu_data', function(data) {
//        console.log("IMU DATA: ",data.yaw);
        updateLidarDisplayPose(0, 0, data.yaw);
    });

    connection.addHandler('/scan', function(data) {
//        console.log("LIDAR DATA: ...");
        points = [];
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

            points.push([range*Math.cos(angle),
                         range*Math.sin(angle),
                         isValid]);
            i++;
        }

        cur_points = points;
    });

    connection.setOnOpen( function(e) {
        console.log('connected to ROS');

        // /imu_data gets published to at about 35 Hz
        connection.callService('/rosjs/subscribe','["/imu_data", '+Math.round(1000/35)+']',
            function(e) {
                console.log("connected to /imu_data!", e);
            }
        );

        // /scan gets published to at about 15 Hz
        connection.callService('/rosjs/subscribe','["/scan", '+Math.round(1000/15)+']',
            function(e) {
                console.log("connected to /scan!", e);
            }
       );
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
    context.rotate(yaw + YAW_OFFSET);
}

//
// LIDAR display functions
//

function redraw() {
    clearCanvas();
    drawPoints(cur_points);
    drawOrigin();
}

function clearCanvas() {
    var context = document.getElementById("canvas").getContext("2d");

    context.fillStyle = "lightGray";
    context.fillRect(-world_width, -world_height, 2*world_width, 2*world_height);
}

function drawPoints(points) {
    var context = document.getElementById("canvas").getContext("2d");

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
}

function drawOrigin(context) {
    var context = document.getElementById("canvas").getContext("2d");

    context.lineWidth = .05;
    context.strokeStyle = "blue";
    context.fillStyle = "blue";

    context.beginPath();
    context.moveTo(0, 0);
    context.lineTo(1, 0);
    context.lineTo(.8, .2);
    context.stroke();

    context.beginPath();
    context.moveTo(1, 0);
    context.lineTo(.8, -.2);
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
    "forward" :     {x: 128, y: 255},
    "backward" :    {x: 128, y:   0},
    "left" :        {x:   0, y: 128},
    "right" :       {x: 255, y: 128}
}

function keypressed(event) {
    var key = event.which;
    sendCommand(keyToDirMap[key]);
}

function sendCommand(dir) {
    console.log(dir);
    var vals = dirToValsMap[dir];

    if (vals) {
        var msg = '>SVXA:'+vals.x;
        connection.publish('/psoc_cmd', 'std_msgs/String', '{"data":"'+msg+'"}');
        msg = '>SVYA:'+vals.y;
        connection.publish('/psoc_cmd', 'std_msgs/String', '{"data":"'+msg+'"}');
    }
}
