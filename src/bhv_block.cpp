//
// Created by arash on 10/25/19.
//

#include "bhv_block.h"
#include "strategy.h"
#include <rcsc/action/basic_actions.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/action/neck_turn_to_low_conf_teammate.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/intercept_table.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <vector>

#include "neck_offensive_intercept_neck.h"

#define DLOG_FOR_TARGET_POINT

using namespace rcsc;
using namespace std;

bool
bhv_block::execute(PlayerAgent *agent) {
    const WorldModel &wm = agent->world();
    const int opp_min = wm.interceptTable()->opponentReachCycle();
    opponent_pass = false;
    static Vector2D target_point;
    PlayerObject fastest_opponent = *wm.interceptTable()->fastestOpponent();
    Vector2D predict, predictInertia;

    dlog.addText(Logger::CLEAR, __FILE__" opp_min is : %d", opp_min);

    if (wm.interceptTable()->fastestOpponent() == NULL)
        return false;
    if (wm.opponentsFromBall().front() == NULL)
        return false;


    if (wm.ball().pos().x > fastest_opponent.pos().x + 1) {
        opponent_pass = true;
    }

    int real_opp_min = opp_min;
    if (opponent_pass) {
        real_opp_min += opp_min / 2;
        if (opp_min > 4) {
            real_opp_min -= 1;
        }
    }

    if (!doPredict(wm, wm.ball().inertiaPoint(real_opp_min), &predictInertia, true))
        return false;

    if (wm.ball().inertiaPoint(opp_min).absY() > ServerParam::i().pitchHalfWidth()
        || wm.ball().inertiaPoint(opp_min).absX() > ServerParam::i().pitchHalfLength()) {
        predictInertia = wm.ball().inertiaPoint(opp_min);
    }

    if (wm.ball().pos().dist(target_point) < 8 && target_point.dist(predictInertia) < 4) {
        target_point = (target_point + predictInertia) / 2;
    } else {
        target_point = predictInertia;
    }
    double dash_power = ServerParam::i().maxDashPower();
    double dist_thr = wm.ball().distFromSelf() * 0.1;
    if (dist_thr < 1.0) dist_thr = 1.0;

#ifdef DLOG_FOR_TARGET_POINT
    dlog.addText(Logger::TEAM,
                 __FILE__": Bhv_BasicMove target=(%.1f %.1f) dist_thr=%.2f",
                 target_point.x, target_point.y,
                 dist_thr);

    agent->debugClient().addMessage("BasicMove%.0f", dash_power);
    agent->debugClient().setTarget(target_point);
    agent->debugClient().addCircle(target_point, dist_thr);
#endif

    if (!Body_GoToPoint(target_point, dist_thr, dash_power
    ).execute(agent)) {
        Body_TurnToBall().execute(agent);
    }

    if (wm.existKickableOpponent()
        && wm.ball().distFromSelf() < 18.0) {
        agent->setNeckAction(new Neck_TurnToBall());
    } else {
        agent->setNeckAction(new Neck_TurnToBallOrScan());
    }
    return true;
}

/*-------------------------------------------------------------------*/

bool
bhv_block::doPredict(const WorldModel &wm, Vector2D center, Vector2D *predict, bool draw) {
    const int opp_min = wm.interceptTable()->opponentReachCycle();

    int n = 60;
    vector<Vector2D> nodes;
    double alfa = 360.0 / n;
    double maxRate = -1000;
    int maxNode = 0;

    for (int i = n / 4; i <= (3 * n) / 4; i++) {
        nodes.push_back(Vector2D(center.x, center.y) + Vector2D::polar2vector(0.7, i * alfa));
        double rate;
        if (!rateThisPoint(wm, nodes.at(i - n / 4), &rate)) {
            *predict = center;
            return true;
        }
        if (maxRate < rate) {
            maxRate = rate;
            maxNode = i - n / 4;
        }
    }
    if (draw) {
        dlog.addCircle(Logger::CLEAR, nodes.at(maxNode), 0.2, "blue");
    }
    cycle_opponent++;

    double my_speed = 0.92;
    double my_cycle = wm.self().pos().dist(nodes.at(maxNode)) / my_speed;
    int predict_opp_min = opp_min;
    if (opponent_pass) {
        predict_opp_min -= opp_min / 2;
        if (wm.ball().inertiaPoint(opp_min).absY() > ServerParam::i().penaltyAreaHalfWidth() &&
            wm.ball().inertiaPoint(opp_min).x < 0) {
            predict_opp_min -= 3;
        }
    }
    if (my_cycle <= cycle_opponent + predict_opp_min) {
        *predict = nodes.at(maxNode);
        return true;
    } else {
        return doPredict(wm, nodes.at(maxNode), predict, draw);
    }

}

/*-------------------------------------------------------------------*/

bool
bhv_block::rateThisPoint(const WorldModel &wm, Vector2D point, double *rate) {
    Vector2D goal = Vector2D(-ServerParam::i().pitchHalfLength(), 0);
    *rate = 400 + 200 - point.x;
    if (point.absY() > ServerParam::i().penaltyAreaHalfWidth()) {
        *rate = 400 - 3 * point.x;
    }

    if (point.dist(goal) < 40) {
        *rate += max(0.0, 40.0 - point.dist(goal));
    } else if (wm.self().pos().dist(point) <= 8 && !opponent_pass) {
        if (point.absY() < ServerParam::i().penaltyAreaHalfWidth())
            return true;
        if (abs(wm.self().pos().absY() - point.absY()) < 1)
            return true;
        double distY = abs(wm.self().pos().absY() - point.absY());
        distY = min(1.0, distY * 0.10);
        if (!opponent_pass) {
            //  distY = 1;
        }
        if (wm.self().pos().absY() < point.absY()) {
            *rate = point.absY() * distY;
        } else {
            *rate = -point.absY() * distY;
        }
        *rate -= 3 * point.x;

    }
    if (point.absY() > ServerParam::i().pitchHalfWidth() - 1
        || point.absX() > ServerParam::i().pitchHalfLength() - 1) {
        *rate -= 400;////OUT
    }

    return true;
}

