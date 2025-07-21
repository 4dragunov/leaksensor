#ifndef FSMLIST_H_INCLUDED
#define FSMLIST_H_INCLUDED

#include <tinyfsm.hpp>

#include "sensors-board.h"


using fsm_list = tinyfsm::FsmList<DataSampler, Channel>;

/** dispatch event to both "Motor" and "Elevator" */
template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}

#endif
