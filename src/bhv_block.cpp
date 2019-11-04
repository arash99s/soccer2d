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

using namespace rcsc;
using namespace std;

bool
bhv_block::execute(PlayerAgent *agent) {
    const WorldModel &wm = agent->world();
    const int opp_min = wm.interceptTable()->opponentReachCycle();
    bool opponent_pass = false;
    Vector2D target_point;
    PlayerObject fastest_opponent = *wm.interceptTable()->fastestOpponent();
    Vector2D predict, predictInertia;

    if (wm.interceptTable()->fastestOpponent() == NULL)
        return false;
    if (wm.opponentsFromBall().front() == NULL)
        return false;


    if (wm.ball().inertiaPoint(opp_min).x > fastest_opponent.pos().x) {
        opponent_pass = true;
    }


    dlog.addText(Logger::CLEAR, __FILE__"opponent pass is :%d", opponent_pass);
    dlog.addText(Logger::CLEAR, __FILE__"opponent cycle :%d", wm.interceptTable()->opponentReachCycle());


    if (opponent_pass && !doPredict(wm, fastest_opponent.pos(), &predict))
        return false;
    if (!opponent_pass && !doPredict(wm, wm.ball().inertiaPoint(opp_min), &predictInertia))
        return false;

    if (wm.ball().inertiaPoint(opp_min).absY() > ServerParam::i().pitchHalfWidth() + 1
        || wm.ball().inertiaPoint(opp_min).absX() > ServerParam::i().pitchHalfLength() + 1) {
        predictInertia = wm.ball().inertiaPoint(opp_min);
    }

    if (opponent_pass) {
        target_point = predict;
    } else {
        target_point = predictInertia;
    }

    double dash_power = ServerParam::i().maxDashPower();
    double dist_thr = wm.ball().distFromSelf() * 0.1;
    if (dist_thr < 1.0) dist_thr = 1.0;

    dlog.addText(Logger::TEAM,
                 __FILE__": Bhv_BasicMove target=(%.1f %.1f) dist_thr=%.2f",
                 target_point.x, target_point.y,
                 dist_thr);

    agent->debugClient().addMessage("BasicMove%.0f", dash_power);
    agent->debugClient().setTarget(target_point);
    agent->debugClient().addCircle(target_point, dist_thr);

    if (!Body_GoToPoint(target_point, dist_thr, dash_power
    ).execute(agent)) {
        Body_TurnToBall().execute(agent);
    }
    if (opponent_pass) {
        agent->setViewAction(new View_Wide());
    }
    agent->setNeckAction(new Neck_TurnToBall());

    return true;
}

/*-------------------------------------------------------------------*/

bool
bhv_block::doPredict(const WorldModel &wm, Vector2D center, Vector2D *predict) {
    const int opp_min = wm.interceptTable()->opponentReachCycle();
    if (wm.self().distFromBall() < 2 && wm.self().pos().x < wm.ball().pos().x) {
        *predict = wm.ball().inertiaPoint(opp_min);
        return true;
    }
    int n = 60;
    vector<Vector2D> nodes;
    vector<double> rateNodes;
    double alfa = 360.0 / n;
    double maxRate = -1000;
    int maxNode = 0;

    for (int i = 0; i < n; i++) {
        nodes.push_back(Vector2D(center.x, center.y) + Vector2D::polar2vector(0.7, i * alfa));
        double rate;
        if (!rateThisPoint(wm, nodes.at(i), &rate)) {
            *predict = center;
            return true;
        }
        if (maxRate < rate) {
            maxRate = rate;
            maxNode = i;
        }
        rateNodes.push_back(rate);
    }
    cycle_opponent++;

    double my_speed = 0.92;
    double my_cycle = wm.self().pos().dist(nodes.at(maxNode)) / my_speed;
    if (my_cycle <= cycle_opponent + opp_min) {
        *predict = nodes.at(maxNode);
        return true;
    } else {
        dlog.addCircle(Logger::CLEAR, nodes.at(maxNode), 0.2, "blue");
        return doPredict(wm, nodes.at(maxNode), predict);
    }

}

/*-------------------------------------------------------------------*/

bool
bhv_block::rateThisPoint(const WorldModel &wm, Vector2D point, double *rate) {
    Vector2D goal = Vector2D(-ServerParam::i().pitchHalfLength(), 0);
    *rate = 200 - point.x;
    if (point.absY() > ServerParam::i().penaltyAreaHalfWidth()) {
        *rate = -3 * point.x;
    }

    if (point.dist(goal) < 40) {
        *rate += max(0.0, 40.0 - point.dist(goal));
    } else if(wm.self().pos().dist(point) <= 5){
        if (point.absY() < ServerParam::i().penaltyAreaHalfWidth())
            return true;
        if (abs(wm.self().pos().absY() - point.absY()) < 1)
            return true;
        double distY = abs(wm.self().pos().absY() - point.absY());
        distY = min(1.0, distY * 0.25);
        distY = 1;
        if (wm.self().pos().absY() < point.absY()) {
            *rate = point.absY() * distY;
        } else {
            *rate = -point.absY() * distY;
        }
        *rate -= point.x;

    }
    if (point.absY() > ServerParam::i().pitchHalfWidth()
        || point.absX() > ServerParam::i().pitchHalfLength() - 2) {
        *rate -= 100;////OUT
    }

    return true;
}

