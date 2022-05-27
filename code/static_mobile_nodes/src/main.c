#include "main.h"

K_THREAD_DEFINE(main_thread, STACK_SIZE_MAIN_THREAD, 
            thread_main_thread, NULL, NULL, NULL,
            THREAD_PRIORITY_MAIN, 0, 50);

void thread_main_thread(void) {

    while (1) {
        k_msleep(1000);
    }
}