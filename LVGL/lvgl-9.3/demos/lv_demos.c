#include "ameba_soc.h"
#include "os_wrapper.h"

#include "lvgl.h"
#include "lv_ameba_hal.h"
#include "demos/lv_demos.h"

#define LOG_TAG "LV-Demos"

void lv_demos_task(void *param) {
    UNUSED(param);

    RTK_LOGI(LOG_TAG, "###### lv_demos_task ######\n");

    lv_init();
    lv_ameba_hal_init();

    /*Open a demo or an example*/
#if defined(CONFIG_LV_DEMO_WIDGETS)
    lv_demo_widgets();
#endif
#if defined(CONFIG_LV_DEMO_STRESS)
    lv_demo_stress();
#endif
#if defined(CONFIG_LV_DEMO_MUSIC)
    lv_demo_music();
#endif
#if defined(CONFIG_LV_DEMO_BENCHMARK)
    lv_demo_benchmark();
#endif

    /* To hide the memory and performance indicators in the corners
     * disable `LV_USE_MEM_MONITOR` and `LV_USE_PERF_MONITOR` in `lv_conf.h`*/

    while(1) {
        /* Periodically call the lv_task handler.
         * It could be done in a timer interrupt or an OS task too.*/
        uint32_t time_till_next = lv_task_handler();

        /* handle LV_NO_TIMER_READY. Another option is to `sleep` for longer */
        if(time_till_next == LV_NO_TIMER_READY)
            time_till_next = LV_DEF_REFR_PERIOD;

        /* delay to avoid unnecessary polling */
        rtos_time_delay_ms(time_till_next);
    }

    lv_deinit();

    rtos_task_delete(NULL);
}

u32 lv_demos(u16 argc, u8  *argv[]) {
    UNUSED(argc);
    UNUSED(argv);
    rtos_task_create(NULL, "lv_demos_task", lv_demos_task, NULL, 1024 * 32, 1);
    return TRUE;
}

CMD_TABLE_DATA_SECTION
const COMMAND_TABLE cmd_table_lv_demos[] = {
    {"lv_demos", lv_demos},
};
