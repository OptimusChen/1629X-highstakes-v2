#include "main.h"
#include "s.hpp"
#include "pros/misc.hpp"
#include "liblvgl/lvgl.h"

using namespace pros;

Controller cm(E_CONTROLLER_MASTER);

namespace sec {
    int auton;
    const char *btnmMap[] = {"Blue WP (+)","Blue Elims (+)","BpRn2R","\n","Red WP (+)","Red Elims (+)","RpBn2R","\n","None","Skills",""}; // up to 10 autons
    std::vector<std::string> textMap; // up to 10 autons
    lv_obj_t * label;

    static void event_handler(lv_event_t * e){
        lv_event_code_t code = lv_event_get_code(e);
        lv_obj_t * obj = lv_event_get_target(e);
        if(code == LV_EVENT_CLICKED) {
            uint32_t id = lv_btnmatrix_get_selected_btn(obj);
            const char * txt = lv_btnmatrix_get_btn_text(obj, id);

            auton = id;

            lv_btnmatrix_set_selected_btn(obj, id);

            // lv_label_set_text_fmt(label, "SELECTED: %s", textMap[auton]);

            cm.set_text(1, 0, textMap[auton]);
        }
    }

    void init(int hue, int default_auton, const char **autons){
        auton = 6;
        textMap = {"Blue WP (+)","Blue Elims (+)","Blue (-)","Red WP (+)","Red Elims (+)","Red (-)","None","Skills",""};

        label = lv_label_create(lv_scr_act());
        // lv_label_set_text_fmt(label, "SELECTED: %s", textMap[auton]);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 20);

        cm.set_text(1, 0, textMap[auton]);

        lv_obj_t * btnm1 = lv_btnmatrix_create(lv_scr_act());
        lv_btnmatrix_set_map(btnm1, btnmMap);

        lv_obj_set_size(btnm1, 480, 272 - 60);

        for (int i = 0; i < 8; i++) {
            lv_btnmatrix_set_btn_width(btnm1, i, 20);    
        }

        lv_obj_align(btnm1, LV_ALIGN_CENTER, 0, 30); // Adjust to leave space for the label
        lv_obj_add_event_cb(btnm1, event_handler, LV_EVENT_ALL, NULL);

        for (int i = 0; i < textMap.size(); i++) {
            textMap[i].append("             ");
        }
    }
}