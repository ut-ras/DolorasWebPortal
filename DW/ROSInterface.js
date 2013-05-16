var acquireData, sendCommand;

(function () {
    var ros = new ROSLIB.Ros({
            url : 'ws://' + DOMAIN + ':9090'
            });

        getPosClient = new ROSLIB.Service({
            ros : ros,
            name : '/getPos',
            serviceType : 'ReactiveDecisionMaker/GetPos'
            }),

        getHeadingClient = new ROSLIB.Service({
            ros : ros,
            name : '/getHeading',
            serviceType : 'ReactiveDecisionMaker/GetHeading'
            }),

        getPDataClient = new ROSLIB.Service({
            ros : ros,
            name : '/getPlanarData',
            serviceType : 'ReactiveDecisionMaker/GetPlanarData'
            }),

        getGoalClient = new ROSLIB.Service({
            ros : ros,
            name : '/getGoal',
            serviceType : 'ReactiveDecisionMaker/GetGoal'
            }),

        getTwistClient = new ROSLIB.Service({
            ros : ros,
            name : '/getTwist',
            serviceType : 'ReactiveDecisionMaker/GetTwist'
            }),

        velCmdTopic = new ROSLIB.Topic({
            ros : ros,
            name : '/vel_cmd_tbf',
            messageType : 'geometry_msgs/Twist'
            });

    acquireData = function (callback) {
        console.log("acquiring data");
        var data = {};

        var gotPos = false,
            gotHeading = false,
            gotPData = false,
            gotGoal = false,
            gotTwist = false;

        var posReq = new ROSLIB.ServiceRequest({}),
            headingReq = new ROSLIB.ServiceRequest({}),
            pdataReq = new ROSLIB.ServiceRequest({}),
            goalReq = new ROSLIB.ServiceRequest({}),
            twistReq = new ROSLIB.ServiceRequest({});

        var returnIfDone = function () {
            if (gotPos && gotHeading && gotPData && gotGoal && gotTwist) {
                callback(data);
            }
        };

        getPosClient.callService(posReq, function (result) {
            data.x = result.pos.x;
            data.y = result.pos.y;
            gotPos = true;

            returnIfDone();
        });

        getHeadingClient.callService(headingReq, function (result) {
            data.heading = result.heading;
            gotHeading = true;

            returnIfDone();
        });

        getPDataClient.callService(pdataReq, function (result) {
            data.pdata = result.pdata;
            gotPData = true;

            returnIfDone();
        });

        getGoalClient.callService(goalReq, function (result) {
            data.goalx = result.goal.x;
            data.goaly = result.goal.y;
            data.timeout = (result.goal.z === -10);
            gotGoal = true;

            returnIfDone();
        });

        getTwistClient.callService(twistReq, function (result) {
            data.v = result.twist.linear.x;
            data.w = result.twist.angular.z;
            gotTwist = true;

            returnIfDone();
        });
    };

    sendCommand = function (linear, angular) {
        var twist = new ROSLIB.Message({
            linear : {
                x : linear,
                y : 0.0,
                z : 0.0
            },
            angular : {
                x : 0.0,
                y : 0.0,
                z : angular
            }
        });

        velCmdTopic.publish(twist);
    };
})();

/*
// Tests

acquireData(function(data) {
    console.log(data);
});

sendCommand(1, 2);
*/
