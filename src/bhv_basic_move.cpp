// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_basic_move.h"
#include "bhv_block.h"
#include "strategy.h"

#include "bhv_basic_tackle.h"

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/body_intercept.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/action/neck_turn_to_low_conf_teammate.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/intercept_table.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include "neck_offensive_intercept_neck.h"

using namespace rcsc;
using namespace std;

/*-------------------------------------------------------------------*/
/*!

 */
bool
Bhv_BasicMove::execute(PlayerAgent *agent) {
    Vector2D(0, 0);
    dlog.addText(Logger::TEAM,
                 __FILE__": Bhv_BasicMove");

    //-----------------------------------------------
    // tackle
    if (Bhv_BasicTackle(0.8, 80.0).execute(agent)) {
        return true;
    }

    const WorldModel &wm = agent->world();
    /*--------------------------------------------------------*/
    // chase ball
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    const int opp_min = wm.interceptTable()->opponentReachCycle();
    if (!wm.existKickableTeammate()
        && (self_min <= 3
            || (self_min <= mate_min
                && self_min < opp_min + 1)
        )
            ) {
        dlog.addText(Logger::TEAM,
                     __FILE__": intercept");
        Body_Intercept().execute(agent);
        agent->setNeckAction(new Neck_OffensiveInterceptNeck());

        return true;
    }
    bhv_block block;
    if (wm.existKickableTeammate() || mate_min < opp_min + 1) {//////////ball is our
        goToFormation(agent);
        return true;
    }
    if (!isNearestToBallInertia(wm)) {
        goToFormation(agent);
        return true;
    }
    if (!block.execute(agent)) {
        goToFormation(agent);
        return true;
    }
    return true;
}

/*-------------------------------------------------------------------*/
void
Bhv_BasicMove::goToFormation(PlayerAgent *agent) {
    const WorldModel &wm = agent->world();
    Vector2D target_point;
    target_point = Strategy::i().getPosition(wm.self().unum());
    dlog.addText(Logger::CLEAR,
                 __FILE__": go to formation");
    double dash_power = Strategy::get_normal_dash_power(wm);
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
    if (wm.existKickableOpponent()
        && wm.ball().distFromSelf() < 18.0) {
        agent->setNeckAction(new Neck_TurnToBall());
    } else {
        agent->setNeckAction(new Neck_TurnToBallOrScan());
    }
}

/*-------------------------------------------------------------------*/
bool
Bhv_BasicMove::isNearestToBallInertia(const WorldModel &wm) {
    const int opp_min = wm.interceptTable()->opponentReachCycle();
    const ServerParam &SP = ServerParam::i();
    Vector2D goal = Vector2D(-SP.pitchHalfLength(), 0);
    Vector2D ball = wm.ball().inertiaPoint(opp_min);

    double minimum = 1000;
    int minimum_player = 0;
    for (int i = 2; i <= 11; i++) {
        if (wm.ourPlayer(i) == NULL || wm.ourPlayer(i)->unum() < 0)
            continue;

        double m = wm.ourPlayer(i)->pos().dist(ball);
        m += wm.ourPlayer(i)->pos().x / 3;
        if (ball.absY() < SP.penaltyAreaHalfWidth() && ball.x < (-SP.pitchHalfLength() + SP.penaltyAreaLength())) {
            m += wm.ourPlayer(i)->pos().dist(goal);
        }
        if (m < minimum) {
            minimum = m;
            minimum_player = i;
        }
    }
    double minimum1 = 1000;
    int minimum_player1 = 0;
    for (int i = 2; i <= 11; i++) {
        if (wm.ourPlayer(i) == NULL || wm.ourPlayer(i)->unum() < 0)
            continue;

        double m = wm.ourPlayer(i)->pos().dist(ball);
        m += wm.ourPlayer(i)->pos().x / 3;
        if (ball.absY() < SP.penaltyAreaHalfWidth() && ball.x < (-SP.pitchHalfLength() + SP.penaltyAreaLength())) {
            m += wm.ourPlayer(i)->pos().dist(goal);
        }
        if (m < minimum1 && m > minimum) {
            minimum1 = m;
            minimum_player1 = i;
        }
    }
    dlog.addText(Logger::CLEAR, __FILE__"must be block: %d = %f", minimum_player , minimum);
    dlog.addText(Logger::CLEAR, __FILE__"must be block: %d = %f", minimum_player1 , minimum1);

    if(abs(minimum - minimum1)<2.5){
        if(minimum_player1 < minimum_player){
            minimum_player = minimum_player1;
        }
    }
    dlog.addText(Logger::CLEAR, __FILE__"realy must be block: %d = %f", minimum_player , minimum);
    return minimum_player == wm.self().unum();
}