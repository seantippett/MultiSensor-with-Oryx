/**
 * @file os_port_config.h
 * @brief RTOS port configuration file
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.2
 **/

#ifndef _OS_PORT_CONFIG_H
#define _OS_PORT_CONFIG_H

#define EVAL_LICENSE_TERMS_ACCEPTED

//Select underlying RTOS
#define USE_FREERTOS

//Trace output redirection
int fprintf_custom(void *stream, const char *format, ...);
#define TRACE_PRINTF(...) osSuspendAllTasks(), fprintf_custom(stderr, __VA_ARGS__), osResumeAllTasks()

//Delay routines
#define usleep(delay) {volatile uint32_t n = delay * 200; while(n > 0) {n--;}}
#define msleep(delay) {volatile uint32_t n = delay; while(n > 0) {usleep(1000); n--;}}
#define sleep(delay) {volatile uint32_t n = delay; while(n > 0) {msleep(1000); n--;}}

#endif
