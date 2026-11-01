#include "route_v7.h"
#include <math.h>
#include <string.h>

/* Direction values are the unchanged national position.h N=0,S=1,E=2,W=3.
 * Every translation is position_move(direction, 10.0f), never timed PWM.
 * World-frame waypoints share one origin across both laps. */
const rv_step_t rv_plan[12] = {
    {RV_MOVE_ACTION,0,1,1,   0, 0,10,10,0},
    {RV_MOVE_ACTION,2,1,2,   0,10,10,10,0},
    {RV_MOVE_ACTION,1,1,3,   0,10, 0,10,0},
    {RV_MOVE_ACTION,3,1,4,   0, 0, 0,10,0},
    {RV_MOVE_ACTION,0,2,1,   0, 0,10,10,0},
    {RV_TURN_ACTION,0,2,2, -90, 0,10,0,0},
    {RV_MOVE_ACTION,2,2,2, -90,10,10,10,0},
    {RV_TURN_ACTION,0,2,3,-180,10,10,0,0},
    {RV_MOVE_ACTION,1,2,3,-180,10, 0,10,0},
    {RV_TURN_ACTION,0,2,4,  90,10, 0,0,0},
    {RV_MOVE_ACTION,3,2,4,  90, 0, 0,10,0},
    {RV_TURN_ACTION,0,2,4,   0, 0, 0,0,0}
};
static float wrapped(float a) {
    while(a>180.0f) a-=360.0f;
    while(a<=-180.0f) a+=360.0f;
    return a;
}
uint16_t rv_crc16(const uint8_t *p, unsigned n) {
    uint16_t crc=0xffffu;
    unsigned i,bit;
    for(i=0;i<n;i++) {
        crc^=(uint16_t)((uint16_t)p[i]<<8);
        for(bit=0;bit<8;bit++) crc=(uint16_t)((crc&0x8000u)?((uint32_t)crc<<1)^0x1021u:(uint32_t)crc<<1);
    }
    return crc;
}
static void default_plan(rv_controller_t *r) {
    memcpy(r->plan,rv_plan,sizeof(rv_plan));r->count=12;r->speed=r->move_speed=80;r->next_speed=0;
    r->turn_speed=40; /* Legacy routes keep their existing angular limit. */
    r->config_ready=1;r->config_ack=0;r->configured_actions=r->configured_headings=0;
}
void rv_init(rv_controller_t *r) {memset(r,0,sizeof(*r));r->protocol=8;default_plan(r);}
static uint8_t held(const rv_controller_t *r) {return r->state==RV_HOLDING || r->state==RV_HELD;}
static uint8_t bypassing(const rv_controller_t *r) {return r->state>=RV_BYPASS_RIGHT && r->state<=RV_BYPASS_LEFT;}
static uint8_t active(const rv_controller_t *r) {
    return r->state==RV_ARMED || r->state==RV_COUNTDOWN ||
           r->state==RV_MOVE || r->state==RV_TURN || r->state==RV_GAP || bypassing(r);
}
void rv_command(rv_controller_t *r, uint8_t op, uint32_t session, uint16_t seq) {
    uint16_t delta;
    /* STOP is always accepted, including a different session or sequence. */
    if(op==RV_STOP) {
        r->state=RV_STOPPED;r->action=RV_NO_ACTION;r->resume_requested=0;r->next_speed=0;r->protocol=8;
        r->bypass_requested=r->bypass_active=0;
        if(r->fault>=RV_BYPASS_SHORT) r->fault=RV_OK;
        return;
    }
    if(!session) return;
    if(op==RV_ARM) {
        if(active(r) || held(r) || r->state==RV_PAUSED || session==r->session) return;
        r->session=session;r->sequence=seq;r->state=RV_ARMED;r->fault=RV_OK;
        default_plan(r);r->bypass_requested=r->bypass_active=0;r->step=0;return;
    }
    if(session!=r->session || (!active(r) && !held(r) && r->state!=RV_PAUSED)) return;
    delta=(uint16_t)(seq-r->sequence);
    if(!delta || delta>=0x8000u) return;
    if(op==RV_HEARTBEAT) {r->sequence=seq;return;}
    if(r->protocol>=6u && op==RV_HOLD) {
        if(r->state==RV_ARMED) return;
        r->sequence=seq;r->bypass_requested=0;
        if(!held(r)) {
            r->held_state=r->state;r->held_at=r->ticks;r->held_stable_ticks=r->stable_ticks;
            r->state=RV_HOLDING;r->stable_ticks=0;
        }
        return;
    }
    if(r->protocol>=6u && op==RV_RELEASE && r->state==RV_HELD) {
        r->bypass_requested=0;
        r->sequence=seq;r->state=r->held_state;r->stage_at+=r->ticks-r->held_at;
        r->stable_ticks=r->held_stable_ticks;
        if(r->fault>=RV_BYPASS_SHORT) r->fault=RV_OK;
        return;
    }
    if(r->protocol>=8u && op==RV_BYPASS && r->state==RV_HELD && !r->bypass_active) {
        r->sequence=seq;
        if(r->held_state!=RV_MOVE) {r->fault=RV_BYPASS_NOT_MOVE;return;}
        r->bypass_requested=1;return;
    }
    if(r->protocol>=6u && op>=RV_CONTINUE_SPEED && op<RV_CONTINUE_SPEED+10u && r->state==RV_PAUSED) {
        if(r->step+1u>=r->count || r->plan[r->step+1u].kind!=RV_MOVE_ACTION) return;
        r->sequence=seq;r->next_speed=(uint8_t)(48u+8u*(op-RV_CONTINUE_SPEED));
        r->resume_requested=1;return;
    }
    if(op==RV_CONTINUE && r->state==RV_PAUSED) {
        r->sequence=seq;r->resume_requested=1;return;
    }
    if(op==RV_START && r->state==RV_ARMED && r->config_ready) {
        r->sequence=seq;r->state=RV_COUNTDOWN;
        r->stage_at=r->ticks;r->stable_ticks=0;
    }
}
static void config(rv_controller_t *r,uint8_t op,uint32_t session,uint16_t value) {
    unsigned index;uint16_t mask;int16_t heading=(int16_t)value;
    if(r->state!=RV_ARMED || session!=r->session || !session) return;
    if(op==RV_CFG_BEGIN) {
        if(!value || value>12u) return;
        r->count=(uint8_t)value;r->config_ready=0;r->speed_set=0;
        r->configured_actions=r->configured_headings=0;memset(r->plan,0,sizeof(r->plan));
    } else {
        if(r->config_ready) return;
        if(op==RV_CFG_SPEED) {
            if(value<8u || value>(r->protocol>=5u?160u:80u)) return;
            r->speed=(uint8_t)value;r->speed_set=1;
        } else if(op==RV_CFG_TURN_SPEED) {
            if(r->protocol<7u || value<8u || value>80u) return;
            r->turn_speed=(uint8_t)value;
        } else if(op==RV_CFG_COMMIT) {
            mask=(uint16_t)((1u<<r->count)-1u);
            if(value!=0x1234u || !r->speed_set || r->configured_actions!=mask || r->configured_headings!=mask) return;
            r->config_ready=1;
        } else if(op>=RV_CFG_MOVE && op<RV_CFG_MOVE+r->count) {
            index=(unsigned)(op-RV_CFG_MOVE);
            if(!(value&0x3fffu) || (value&0x3fffu)>2000u) return;
            r->plan[index].kind=RV_MOVE_ACTION;r->plan[index].turn_degrees=0.0f;r->plan[index].direction=(uint8_t)(value>>14);
            r->plan[index].grids=(float)(value&0x3fffu)*.01f;
            r->configured_actions|=(uint16_t)(1u<<index);
        } else if(op>=RV_CFG_SPIN && op<RV_CFG_SPIN+r->count) {
            if(value!=900u && value!=1800u && value!=3600u && value!=7200u) return;
            index=(unsigned)(op-RV_CFG_SPIN);r->plan[index].kind=RV_TURN_ACTION;
            r->plan[index].turn_degrees=(float)value*.1f;
            r->configured_actions|=(uint16_t)(1u<<index);r->configured_headings|=(uint16_t)(1u<<index);
        } else if((op>=RV_CFG_HEADING && op<RV_CFG_HEADING+r->count) ||
                  (op>=RV_CFG_TURN && op<RV_CFG_TURN+r->count)) {
            if(heading< -18000 || heading>18000) return;
            index=(unsigned)(op&15u);r->plan[index].heading=(float)heading*.01f;
            r->configured_headings|=(uint16_t)(1u<<index);
            if(op>=RV_CFG_TURN) {r->plan[index].kind=RV_TURN_ACTION;r->plan[index].turn_degrees=0.0f;r->configured_actions|=(uint16_t)(1u<<index);}
        } else return;
    }
    r->config_ack=op;r->sequence=value;
}
void rv_feed(rv_controller_t *r, uint8_t b) {
    uint8_t *p=r->rx;
    if(r->rx_length<12u) p[r->rx_length++]=b;
    if(r->rx_length<12u) return;
    if(p[0]==0xc7u && p[1]==0x7cu && (p[2]==2u || p[2]==3u || p[2]==4u || p[2]==5u || p[2]==6u || p[2]==7u || p[2]==8u || (p[2]==1u && p[3]==RV_STOP)) &&
       rv_crc16(p,10)==(uint16_t)(p[10]|((uint16_t)p[11]<<8))) {
        uint32_t session=(uint32_t)p[4]|((uint32_t)p[5]<<8)|
                         ((uint32_t)p[6]<<16)|((uint32_t)p[7]<<24);
        uint16_t value=(uint16_t)(p[8]|((uint16_t)p[9]<<8));
        if(p[2]>=3u && p[3]>=RV_CFG_BEGIN && p[3]<RV_CONTINUE_SPEED) config(r,p[3],session,value);
        else {
            rv_command(r,p[3],session,value);
            if(p[3]==RV_ARM && r->session==session && r->state==RV_ARMED) r->protocol=p[2];
        }
        r->rx_length=0;
    } else {memmove(p,p+1,11);r->rx_length=11;}
}
static void begin_step(rv_controller_t *r,const rv_input_t *in) {
    r->state=(r->plan[r->step].kind==RV_MOVE_ACTION)?RV_MOVE:RV_TURN;
    r->action=r->plan[r->step].kind;r->stage_at=r->ticks;
    r->stable_ticks=0;
    if(r->plan[r->step].kind==RV_MOVE_ACTION) {
        r->move_speed=r->next_speed?r->next_speed:r->speed;r->next_speed=0;
    }
    r->turn_previous_yaw=in->yaw;r->turn_progress=0.0f;
    r->target_heading=r->plan[r->step].turn_degrees>0.0f?wrapped(in->yaw-45.0f):r->plan[r->step].heading;
}
static void finish_step(rv_controller_t *r) {
    if(r->step+1u==r->count) {r->state=RV_DONE;return;}
    /* Keep the completed index. Only a fresh CONTINUE advances exactly once.
     * No heartbeat is required while paused: every motor output is disabled. */
    r->state=RV_PAUSED;r->stage_at=r->ticks;r->resume_requested=0;
}
static const float direction_x[4]={0.0f,0.0f,1.0f,-1.0f};
static const float direction_y[4]={1.0f,-1.0f,0.0f,0.0f};
static const uint8_t right_of[4]={2u,3u,1u,0u};
static void bypass_leg(rv_controller_t *r,uint8_t state) {
    uint8_t forward=r->plan[r->step].direction,right=right_of[forward];
    r->state=state;r->stable_ticks=0;r->stage_at=r->ticks;r->action=RV_TARGET_ACTION;
    r->command_direction=state==RV_BYPASS_FORWARD?forward:(state==RV_BYPASS_RIGHT?right:(uint8_t)(right^1u));
    r->command_x=r->bypass_x;r->command_y=r->bypass_y;
    if(state!=RV_BYPASS_LEFT) {r->command_x+=2.5f*direction_x[right];r->command_y+=2.5f*direction_y[right];}
    if(state!=RV_BYPASS_RIGHT) {r->command_x+=5.0f*direction_x[forward];r->command_y+=5.0f*direction_y[forward];}
}
void rv_tick(rv_controller_t *r, const rv_input_t *in) {
    float heading_error;
    r->ticks++;r->action=RV_NO_ACTION;r->reset_origin=0;
    if(held(r)) {
        if(r->bypass_requested && r->state==RV_HELD) {
            uint8_t d=r->plan[r->step].direction;
            float remaining=(in->goal_x-in->x)*direction_x[d]+(in->goal_y-in->y)*direction_y[d];
            r->bypass_requested=0;
            if(remaining<5.0f) {r->fault=RV_BYPASS_SHORT;return;}
            r->fault=RV_OK;r->bypass_active=1;r->bypass_x=in->x;r->bypass_y=in->y;
            r->bypass_goal_x=in->goal_x;r->bypass_goal_y=in->goal_y;r->saved_move_speed=r->move_speed;
            if(r->move_speed>40u) r->move_speed=40u;
            bypass_leg(r,RV_BYPASS_RIGHT);return;
        }
        /* Keep multi-turn progress, including the small coast after HOLD. */
        if(r->held_state==RV_TURN && r->plan[r->step].turn_degrees>0.0f) {
            r->turn_progress-=wrapped(in->yaw-r->turn_previous_yaw);r->turn_previous_yaw=in->yaw;
        }
        if(r->state==RV_HOLDING) {
            if(fabsf(in->wheel_peak)<=5.0f) r->stable_ticks++;else r->stable_ticks=0;
            if(r->stable_ticks>=5u) r->state=RV_HELD;
        }
        return;
    }
    if(bypassing(r)) {
        if(!in->moving && fabsf(in->wheel_peak)<=5.0f) {
            if(r->state==RV_BYPASS_RIGHT) bypass_leg(r,RV_BYPASS_FORWARD);
            else if(r->state==RV_BYPASS_FORWARD) bypass_leg(r,RV_BYPASS_LEFT);
            else {
                r->bypass_active=0;r->move_speed=r->saved_move_speed;r->state=RV_MOVE;
                r->command_direction=r->plan[r->step].direction;
                r->command_x=r->bypass_goal_x;r->command_y=r->bypass_goal_y;r->action=RV_TARGET_ACTION;
            }
        }
        return;
    }
    if(r->state==RV_PAUSED) {
        if(r->resume_requested) {r->resume_requested=0;r->step++;begin_step(r,in);}
        return;
    }
    if(r->state==RV_COUNTDOWN) {
        if((uint32_t)(r->ticks-r->stage_at)>=150u) {
            rv_input_t origin=*in;origin.yaw=0.0f;
            r->reset_origin=1;r->step=0;begin_step(r,&origin);
        }
        return;
    }
    if(r->state==RV_GAP) {
        /* Two seconds between laps; one second at the other corners. */
        if((uint32_t)(r->ticks-r->stage_at)>=(r->step==4u?100u:50u)) begin_step(r,in);
        return;
    }
    if(r->state!=RV_MOVE && r->state!=RV_TURN) return;
    if(r->state==RV_TURN) {
        if(r->plan[r->step].turn_degrees>0.0f) {
            float remaining;
            r->turn_progress-=wrapped(in->yaw-r->turn_previous_yaw);r->turn_previous_yaw=in->yaw;
            remaining=r->plan[r->step].turn_degrees-r->turn_progress;heading_error=fabsf(remaining);
            if(remaining>45.0f) remaining=45.0f;
            if(remaining< -45.0f) remaining=-45.0f;
            r->target_heading=wrapped(in->yaw-remaining);
        } else heading_error=fabsf(wrapped(in->yaw-r->plan[r->step].heading));
        if(heading_error<=1.0f && fabsf(in->wheel_peak)<=5.0f) r->stable_ticks++;
        else r->stable_ticks=0;
        if(r->stable_ticks>=15u) finish_step(r);
        return;
    }
    if(!in->moving) {
        if(fabsf(in->wheel_peak)<=5.0f) finish_step(r);
        return;
    }
}
uint8_t rv_motor_enabled(const rv_controller_t *r) {
    return (uint8_t)(r->state==RV_MOVE || r->state==RV_TURN || bypassing(r));
}
void rv_status(const rv_controller_t *r, uint8_t p[18]) {
    uint16_t crc;unsigned i;
    p[0]=0xa7;p[1]=0x7a;p[2]=r->protocol;p[3]=r->state;
    for(i=0;i<4;i++) p[4+i]=(uint8_t)(r->session>>(8*i));
    p[8]=(uint8_t)r->sequence;p[9]=(uint8_t)(r->sequence>>8);
    p[10]=r->step;p[11]=(r->protocol>=3u && r->state==RV_ARMED)?r->config_ack:r->fault;
    if(r->protocol>=8u && held(r) && r->bypass_active) p[11]=(uint8_t)(0x10u+r->held_state-RV_BYPASS_RIGHT);
    for(i=0;i<4;i++) p[12+i]=(uint8_t)(r->ticks>>(8*i));
    crc=rv_crc16(p,16);p[16]=(uint8_t)crc;p[17]=(uint8_t)(crc>>8);
}
