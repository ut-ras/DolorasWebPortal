var clib = (function() {
    var lib = {},

        LARGE_DISTANCE = 1e6,
        SMALL_ENOUGH = 1e-6;

    lib.makeCircle = function (x, y, r) {
        return {
            type: "circle",
            x: x,
            y: y,
            r: r
        };
    };

    lib.makePoint = function (x, y) {
        return {
            type: "point",
            x: x,
            y: y
        };
    };

    lib.makeVector = function (x, y, dir) {
        return {
            type: "vector",
            x: x,
            y: y,
            dir: dir
        }
    };

    lib.makePose = function (x, y, dir, v, w) {
        return {
            x: x,
            y: y,
            dir: dir,
            v: v,
            w: w
        };
    };

    //
    // This will parameterize two lines from each (x0, y0, theta) pair with:
    //  x(t) = x0 + t*cos(theta)
    //  y(t) = y0 + t*sin(theta)
    // Then it calculates the intersection of the two lines, and returns
    //  the parameter t on the second line for that intersection.
    //
    // Returns false if no intersection exists
    //
    var getParameterOfIntersection = function (x0, y0, theta0, x1, y1, theta1) {
        // return false if the angles are parallel
        if (Math.abs(Math.sin(theta0 - theta1)) < SMALL_ENOUGH) {
            return false;
        }

        return -(Math.sin(theta0)*(x1 - x0) + Math.cos(theta0)*(y0 - y1)) /
            Math.sin(theta0 - theta1);
    }

    lib.getParameterOfIntersection = getParameterOfIntersection;

    var circleIntersections = function (c1, c2) {
        var dist = glib.euclid(c1.x, c1.y, c2.x, c2.y),
            angle = Math.atan2(c2.y - c1.y, c2.x - c1.x),

            small = (c1.r < c2.r) ? c1.r : c2.r,
            big = (c1.r > c2.r) ? c1.r : c2.r;

        if (c1.r === c2.r && c1.x === c2.x && c1.y === c2.y) {
            return [];
        } else if (dist === small + big || dist === big - small) {
            return [
                lib.makePoint(c1.x + c1.r*Math.cos(angle),
                              c1.y + c1.r*Math.sin(angle))
            ];
        } else if (dist < small + big && dist > big - small) {
            // using the law of cosines
            var angleOffset = Math.acos((c1.r*c1.r + dist*dist - c2.r*c2.r)/(2*c1.r*dist));

            return [
                lib.makePoint(c1.x + c1.r*Math.cos(angle + angleOffset),
                              c1.y + c1.r*Math.sin(angle + angleOffset)),
                lib.makePoint(c1.x + c1.r*Math.cos(angle - angleOffset),
                              c1.y + c1.r*Math.sin(angle - angleOffset))
            ];
        }

        return [];
    };

    // returns circle, vector, or point
    lib.calculateTrajetory = function (pose) {
        if (Math.abs(pose.v) < SMALL_ENOUGH) {
            return lib.makePoint(pose.x, pose.y);
        } else if (Math.abs(pose.w) <= SMALL_ENOUGH) {
            return lib.makeVector(pose.x, pose.y, pose.dir);
        }

        // otherwise, v and w are both nonzero:
        var radius = Math.abs(pose.v/pose.w);

        if (pose.v > 0) {
            if (pose.w > 0) {
                return lib.makeCircle(
                    pose.x + radius*Math.cos(pose.dir + Math.PI/2),
                    pose.y + radius*Math.sin(pose.dir + Math.PI/2),
                    radius
                );
            } else {
                return lib.makeCircle(
                    pose.x + radius*Math.cos(pose.dir - Math.PI/2),
                    pose.y + radius*Math.sin(pose.dir - Math.PI/2),
                    radius
                );
            }
        } else {
            throw "exception: not implemented yet!"
        }
    };

    lib.trajectoryIntersection = function (pose, traj, circle) {
        if (traj.type === "circle") {
            var points = circleIntersections(traj, circle);

            if (points.length === 0) {
                return false;
            } else if (points.length === 1) {
                return points[0];
            } else if (points.length === 2) {
                // need to find the closest of the two points to the starting point
                // along the trajectory
                var curAngle = glib.boundAngle0to2Pi(Math.atan2(
                        pose.y - traj.y,
                        pose.x - traj.x
                    )),
                    angle1 = glib.boundAngle0to2Pi(Math.atan2(
                        points[0].y - traj.y,
                        points[0].x - traj.x
                    )),
                    angle2 = glib.boundAngle0to2Pi(Math.atan2(
                        points[1].y - traj.y,
                        points[1].x - traj.x
                    ));

                if (pose.w > 0) {
                    angle1 = (angle1 < curAngle) ? angle1 + Math.PI*2 : angle1;
                    angle2 = (angle2 < curAngle) ? angle2 + Math.PI*2 : angle2;

                    if (angle1 < angle2) {
                        return {
                            point: points[0],
                            delta: traj.r*(angle1 - curAngle)
                            };
                    } else {
                        return {
                            point: points[1],
                            delta: traj.r*(angle2 - curAngle)
                            };
                    }
                } else {
                    angle1 = (angle1 > curAngle) ? angle1 - Math.PI*2 : angle1;
                    angle2 = (angle2 > curAngle) ? angle2 - Math.PI*2 : angle2;

                    if (angle1 > angle2) {
                        return {
                            point: points[0],
                            delta: traj.r*(curAngle - angle1)
                            };
                    } else {
                        return {
                            point: points[1],
                            delta: traj.r*(curAngle - angle2)
                            };
                    }
                }
            }
        } else if (traj.type === "vector") {
            var vector = lib.makeVector(circle.x, circle.y, traj.dir + Math.PI/2);

            var s = getParameterOfIntersection(
                vector.x, vector.y, vector.dir,
                traj.x, traj.y, traj.dir
            );

            // if the intersection doesn't occur or if it occurs in the
            //  opposite direction, return false
            if (false === s) {
                return false;
            }

            var intersection = lib.makePoint(
                traj.x + s*Math.cos(traj.dir),
                traj.y + s*Math.sin(traj.dir)
                );

            // if the intersection occured inside the circle, check the real
            //  intersection on the circle's perimeter
            var a = glib.euclid(intersection.x, intersection.y, circle.x, circle.y);

            if (a < circle.r) {
                var r = circle.r,
                    b = Math.sqrt(r*r - a*a),
                    dist = glib.euclid(intersection.x, intersection.y, traj.x, traj.y);

                if (s < 0) {
                    dist *= -1;
                }

                if (dist - b > 0) {
                    dist -= b;
                } else if (dist + b > 0) {
                    dist += b;
                } else {
                    return false;
                }

                return {
                    delta: dist,
                    point: lib.makePoint(
                                traj.x + dist*Math.cos(traj.dir),
                                traj.y + dist*Math.sin(traj.dir)
                                )
                }
            }
        }

        return false;
    };

    lib.calcIntersection = function (x, y, theta, linear, angular, circs) {
        var pose = lib.makePose(
            x,
            y,
            theta,
            linear,
            angular
            ),

            traj = lib.calculateTrajetory(pose);

        var closest = {
            delta: LARGE_DISTANCE,
            point: false
            };

        for (var i = 0; i < circs.length; i++) {
            var intersection = lib.trajectoryIntersection(pose, traj, circs[i]);

            if (intersection) {
                if (intersection.delta < closest.delta) {
                    closest = intersection;
                }
            }
        }

        closest.traj = traj;

        return closest;
    };

    lib.calcClearance = function (x, y, theta, linear, angular, circs) {
        var res = lib.calcIntersection(x, y, theta, linear, angular, circs);

        if (res.point) {
            return res.delta;
        }

        return false;
    };

    return lib;
})();



