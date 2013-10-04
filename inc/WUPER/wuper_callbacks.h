/*
 * wuper_callbacks.h
 *
 *  Created on: 2013.09.30
 *      Author: Giedrius
 */

#ifndef WUPER_CALLBACKS_H_
#define WUPER_CALLBACKS_H_

#include "function_def.h"

#include "main.h"

/*
 * CDC stream callbacks
 */


/*
 * Spirit stream callbacks
 */

SFPResult PingReceiveCallback(SFPFunction *func);
SFPResult PongReceiveCallback(SFPFunction *func);

#endif /* WUPER_CALLBACKS_H_ */
