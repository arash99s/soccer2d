//
// Created by arash on 10/25/19.
//

#ifndef SOCCER2D_BHV_BLOCK_H
#define SOCCER2D_BHV_BLOCK_H

#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>
#include <rcsc/player/world_model.h>


class bhv_block {
public:
    bhv_block() {}

    bool execute(rcsc::PlayerAgent *agent);

    bool doPredict(const rcsc::WorldModel &wm, rcsc::Vector2D center, rcsc::Vector2D *predict, int unum, bool draw);


private:
    bool rateThisPoint(const rcsc::WorldModel &wm, rcsc::Vector2D point, double *rate);

    int cycle_opponent = 0;

    bool opponent_pass = false;
};


#endif //SOCCER2D_BHV_BLOCK_H
