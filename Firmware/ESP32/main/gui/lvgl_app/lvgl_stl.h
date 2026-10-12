#ifndef __lvgl_stl_H
#define __lvgl_stl_H


#ifdef __cplusplus
extern "C" {
#endif

    /*********************
    *      INCLUDES
    *********************/
#include "../lvgl/lvgl.h"
    /*********************
    *      DEFINES
    *********************/

    /**********************
    *      TYPEDEFS
    **********************/

    /**********************
    * GLOBAL PROTOTYPES
    **********************/
    void lv_amin_start(void* obj, \
        int32_t start_value, \
        int32_t end_value, \
        uint32_t repeat_count, \
        uint32_t duration, \
        uint32_t delay, \
        lv_anim_exec_xcb_t exec_cb, \
        lv_anim_path_cb_t path_cb);
    void lv_obj_create_delayed(lv_obj_t* obj, uint32_t SysTick_Delay_ms);

    void lv_fun_param_delayed(void (*fun)(uint8_t), uint32_t SysTick_Delay_ms, uint8_t user_data);
    void lv_fun_delayed(void (*fun)(), uint32_t SysTick_Delay_ms);
    void lv_style_init_simple(lv_style_t* style);

    /* LVGL 动画回调统一使用 void * / int32_t，兼容新版编译器的严格类型检查。 */
    void lv_anim_set_x_cb(void* obj, int32_t value);
    void lv_anim_set_y_cb(void* obj, int32_t value);
    void lv_anim_set_height_cb(void* obj, int32_t value);
    void lv_anim_bar_set_value_cb(void* obj, int32_t value);
    void lv_obj_opa_cb(void* obj, int32_t value);
    void lv_fun_ready_cb(lv_anim_t* a);
    void lv_fun_param_ready_cb(lv_anim_t* a);
    void lv_obj_create_anim_ready_cb(lv_anim_t* a);
    /**********************
    *      MACROS
    **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif




#endif
