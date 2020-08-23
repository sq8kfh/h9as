/*
 * h9avr-can v0.1
 *
 * Created by SQ8KFH on 2017-08-07.
 *
 * Copyright (C) 2017-2020 Kamil Palkowski. All rights reserved.
 */

#ifndef CAN_H
#define CAN_H

#include <avr/io.h>
#include <avr/eeprom.h>
#include "h9msg.h"

extern volatile uint16_t can_node_id;

void CAN_init(void);
void CAN_send_turned_on_broadcast(void);

void CAN_set_mob_for_remote_node1(uint16_t remote_node_id);
void CAN_set_mob_for_remote_node2(uint16_t remote_node_id);
void CAN_set_mob_for_remote_node3(uint16_t remote_node_id);

void CAN_put_msg(h9msg_t *cm);
uint8_t CAN_get_msg(h9msg_t*cm);

void CAN_init_new_msg(h9msg_t *mes);
void CAN_init_response_msg(const h9msg_t *req, h9msg_t *res);

extern volatile uint16_t can_node_id;

#endif /*CAN_H*/
