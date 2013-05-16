DynamicWindow = function (
    action_out,
    cur_x,
    cur_y,
    cur_dir,
    cur_linear,
    cur_angular,
    mycloud,
    goal_x,
    goal_y,
    deltaTime,
    plotter
    )
{
    var distToGoal = glib.euclid(goal_x, goal_y, cur_x, cur_y);

    if (distToGoal < CLOSE_ENOUGH_TO_GOAL) {
        action_out.linear = 0;
        action_out.angular = 0;
        return;
    }

    var best_weight = -1,
        best_linear = null,
        best_angular = null,
        best_traj = null,
        best_found = false;

    for (var angular = ANGULAR_MIN;
             angular <= ANGULAR_MAX;
             angular += ANGULAR_INC)
    {
        if (angular < cur_angular - ANGULAR_ACCEL ||
            angular > cur_angular + ANGULAR_ACCEL) {
            continue;
        }

        for (var linear = LINEAR_MIN;
                 linear <= LINEAR_MAX;
                 linear += LINEAR_INC)
        {
            if (linear < cur_linear - LINEAR_ACCEL ||
                linear > cur_linear + LINEAR_ACCEL) {
                continue;
            }

            //
            // get clearance and normalize
            //
            var res = clib.calcIntersection(
                cur_x,
                cur_y,
                cur_dir,
                linear,
                angular,
                mycloud
                );

            var clearance = CLEARANCE_MAX,
                color = "gray";

            if (res.point && res.delta < CLEARANCE_MAX) {
                clearance = res.delta;
                color = "red";

                if (clearance < CLEARANCE_MIN) {
                    continue;
                }
            }

            var clearanceNorm = clearance/CLEARANCE_MAX;

            //
            // get normalized goal direction weight
            //
            var newDir = cur_dir + angular*deltaTime,
                goalDir = Math.atan2(goal_y - cur_y, goal_x - cur_x),
                goalDirDif = Math.PI - Math.abs(glib.angleDif(goalDir, newDir)),
                goalDirDifNorm = goalDirDif/Math.PI;

            //
            // get normalized linear velocity weight
            //
            var linearNorm = linear/LINEAR_MAX;

            //
            // calculate long-term goal distance
            //
            if (res.traj.type === "circle") {
                var d = glib.euclid(goal_x, goal_y, res.traj.x, res.traj.y);
                goaldist = Math.abs(d - res.traj.r);

                // console.log("circle: ", goaldist);
            } else if (res.traj.type === "vector") {
                var goal_param = clib.getParameterOfIntersection(
                        cur_x, cur_y, res.traj.dir,
                        goal_x, goal_y, res.traj.dir + Math.PI/2.0
                        );

                if (goal_param === false) {
                    throw "the lines are perpendicular--but no intersection?!";
                } else {
                    var traj_param = clib.getParameterOfIntersection(
                        goal_x, goal_y, res.traj.dir + Math.PI/2.0,
                        cur_x, cur_y, res.traj.dir
                        );

                    if (traj_param < 0) {
                        goaldist = GOAL_DIST_MAX;
                    } else {
                        goaldist = Math.abs(goal_param);
                    }
                }
            } else if (res.traj.type === "point") {
                goaldist = distToGoal;
            }

            if (goaldist > GOAL_DIST_MAX) {
                goaldist = GOAL_DIST_MAX;
            }

            var goalDistNorm = 1 - goaldist/GOAL_DIST_MAX;

            // hack to get what I want out of this piece of shit.
            if (goalDir < -Math.PI/2 || goalDir > Math.PI/2) {
                goalDistNorm = 0;
            }


            //
            // get the final weight, compare it with what we've seen so far
            //
            var weight = clearanceNorm*CLEARANCE_WEIGHT +
                         goalDirDifNorm*GOAL_DIR_WEIGHT +
                         linearNorm*LINEAR_WEIGHT +
                         goalDistNorm*GOAL_DIST_WEIGHT;

            if (weight > best_weight) {
                best_weight = weight;
                best_linear = linear;
                best_angular = angular;
                best_traj = res.traj;
                best_found = true;
            }

            //
            // plot trajectory & intersection point, if there was one
            //
            if (plotter) {
                if (res.point) {
                    Plotter.plotPoint(res.point.x, res.point.y, .1, color);
                }

                if (res.traj.type === "circle") {
                    Plotter.plotCircle(res.traj.x, res.traj.y, res.traj.r, color);
                } else if (res.traj.type === "vector") {
                    Plotter.plotVector(res.traj.x, res.traj.y, res.traj.dir, color);
                } else if (res.traj.type === "point") {
                    Plotter.plotCircle(res.traj.x, res.traj.y, .03, color);
                }
            }
        }
    }

    if (!best_found || (Math.abs(best_linear) <= 1e-6 && Math.abs(best_angular) <= 1e-6)) {
        action_out.linear = 0;
        action_out.angular = 0;
    }

    if (distToGoal < SLOW_DOWN_RADIUS) {
        best_linear *= (distToGoal/SLOW_DOWN_RADIUS);
    }

    if (best_found) {
        var color = "white";
        if (best_traj.type === "circle") {
            Plotter.plotCircle(best_traj.x, best_traj.y, best_traj.r, color, 3);
        } else if (best_traj.type === "vector") {
            Plotter.plotVector(best_traj.x, best_traj.y, best_traj.dir, color, 3);
        } else if (best_traj.type === "point") {
            Plotter.plotPoint(best_traj.x, best_traj.y, .1, color);
        }
    }

    action_out.linear = best_linear;
    action_out.angular = best_angular;
}
