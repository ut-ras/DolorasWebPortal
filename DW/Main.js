(function () {
    var RATE = 10, // Hz
        NUM_OBSTACLES = 100;

    var plotEverything = function (data) {
        Plotter.clear();

        Plotter.plotPoint(data.x, data.y, CLEARANCE_MAX, "lightBlue");
        Plotter.plotPoint(data.x, data.y, SIZE_RADIUS, "lightGreen");
        Plotter.plotCircle(data.x, data.y, SIZE_RADIUS, "darkGreen");
        Plotter.drawAxises(1, 1);

        for (var i = 0; i < data.cloud.length; i++) {
            Plotter.plotPoint(data.cloud[i].x, data.cloud[i].y, 0.05, "black");
            Plotter.plotCircle(data.cloud[i].x, data.cloud[i].y, data.cloud[i].r, "gray");
        }

        Plotter.plotPoint(data.goalx, data.goaly, 0.1, "blue");
        Plotter.plotArrow(data.x, data.y, data.heading, "red");
    };

    var processData = function (data) {
        // move goal to our reference frame
        var x = data.goalx - data.x,
            y = data.goaly - data.y,
            t = -data.heading;

        data.goalx = x*Math.cos(t) - y*Math.sin(t);
        data.goaly = x*Math.sin(t) + y*Math.cos(t);

        // set our pose to be the center of the frame
        data.x = 0;
        data.y = 0;
        data.heading = 0;

        // convert pdata to cloud
        data.cloud = []
        for (var i = 0; i < data.pdata.ranges.length; i++) {
            var dist = data.pdata.ranges[i],
                angle = data.pdata.angles[i];

            if (dist < CLEARANCE_MAX && dist > SIZE_RADIUS) {
                data.cloud.push({
                    x: dist*Math.cos(angle),
                    y: dist*Math.sin(angle),
                    r: SIZE_RADIUS + BUFFER_SPACE
                    });
            }
        }
    };

    var goalx_intercept = false,
        goaly_intercept = false;

    var step = function (data) {
        if (INTERCEPT_GOAL && goalx_intercept !== false) {
            data.goalx = goalx_intercept;
            data.goaly = goaly_intercept;
        }

        plotEverything(data);

        var action_out = {
            linear: 0,
            angular: 0
            };

        DynamicWindow(
            action_out,
            data.x,
            data.y,
            data.heading,
            data.v,
            data.w,
            data.cloud,
            data.goalx,
            data.goaly,
            1.0/RATE,
            Plotter
            );

        sendCommand(action_out.linear, action_out.angular);

        return action_out;
    };

    var readWeights = function () {
        LINEAR_WEIGHT = document.getElementById("linearVel").value + 0;
        CLEARANCE_WEIGHT = document.getElementById("clearance").value + 0;
        GOAL_DIR_WEIGHT = document.getElementById("goalDir").value + 0;
        GOAL_DIST_WEIGHT = document.getElementById("goalDist").value + 0;
    };

    document.getElementById("linearVel").value = "" + LINEAR_WEIGHT;
    document.getElementById("clearance").value = "" + CLEARANCE_WEIGHT;
    document.getElementById("goalDir").value = "" + GOAL_DIR_WEIGHT;
    document.getElementById("goalDist").value = "" + GOAL_DIST_WEIGHT;

    Plotter.init(document.getElementById("canvas"), .5, .5, 50, 50);

    if (USING_ROS) {
        if (INTERCEPT_GOAL) {
            canvas.onmousedown = function (event) {
                event.preventDefault();

                var coords = Plotter.getPlotCoords(event.offsetX, event.offsetY);
                goalx_intercept = coords[0];
                goaly_intercept = coords[1];
            }
        }

        setInterval(readWeights, 1000)

        setTimeout(
            function f() {
                acquireData(function (data) {
                    if (!data.timeout) {
                        processData(data);
                        step(data);
                    } else {
                        console.log("encountered a goal timeout! (/goal isn't being published to)");
                    }

                    setTimeout(f, 1);
                });

                readWeights();
            },
            1000/RATE
            );
    } else {
        canvas.onmousedown = function (event) {
            event.preventDefault();

            readWeights();

            var coords = Plotter.getPlotCoords(event.offsetX, event.offsetY);
            data.goalx = coords[0];
            data.goaly = coords[1];
        };

        var cloud = [];

        var data = {
            x: 0,
            y: 0,
            heading: 0,
            v: 0,
            w: 0,
            goalx: 0,
            goaly: 0,
            cloud: cloud,
            };

        for (var i = 0; i < NUM_OBSTACLES; i++) {
            do {
                var x = Math.random()*10 - 5,
                    y = Math.random()*10 - 5;
            } while (glib.euclid(x, y, data.goalx, data.goaly) < SIZE_RADIUS);

            cloud.push({x:x, y:y, r:SIZE_RADIUS});
        }

        setTimeout(
            function f() {
                var res = step(data);

                data.v = res.linear;
                data.w = res.angular;

                var kine = glib.calcTrajectoryStepFromTime(
                    data.x,
                    data.y,
                    data.heading,
                    data.v,
                    data.w,
                    1.0/RATE
                    );

                data.x = kine[0];
                data.y = kine[1];
                data.heading = kine[2];

                Plotter.plotArrow(data.x, data.y, data.heading, "red");

                setTimeout(f, 1000/RATE);
            },
            1000/RATE
        );
    }
})();
