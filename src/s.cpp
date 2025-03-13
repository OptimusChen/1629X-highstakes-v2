#include "main.h"
#include "s.hpp"
#include "pros/misc.hpp"
#include "liblvgl/lvgl.h"
#include "intake.hpp"

using namespace pros;

Controller cm(E_CONTROLLER_MASTER);

namespace sec {
    int auton;
    int color;
    const char *btnmMap[] = {"Red Rush","Blue Rush","Red AWP","\n","Blue AWP","RPBN","BPRN","\n","Skills","Backward",""}; // up to 10 autons
    const char *toolsMap[] = {"Color","Calibrate","MCL","","","","","","","","","",""}; // up to 10 autons
    const char *textMap[] = {"Red Rush","Blue Rush","Red AWP","Blue AWP","RPBN","BPRN","Skills","Backward",""}; // up to 10 autons
    lv_obj_t* colorLabel;
    lv_obj_t* selectedLabel;
    lib::Robot* robot;

    static void autonsEventHandler(lv_event_t * e){
        lv_event_code_t code = lv_event_get_code(e);
        lv_obj_t * obj = lv_event_get_target(e);
        if(code == LV_EVENT_CLICKED) {
            uint32_t id = lv_btnmatrix_get_selected_btn(obj);
            const char * txt = lv_btnmatrix_get_btn_text(obj, id);

            auton = id;

            lv_btnmatrix_set_selected_btn(obj, id);

            std::cout << auton << std::endl;

            const char* selectedName = textMap[auton];

            lv_label_set_text_fmt(selectedLabel, "Selected: %s", selectedName);

            switch (auton) {
                case 0:
                    robot->set_pose(-51, 29, 20);
                    break;
                case 1:
                    robot->set_pose(51, 20, 152);
                    break;
                case 2:
                    robot->set_pose(-58, 14, 235);
                    break;
                case 3:
                    robot->set_pose(58, 14, 145);
                    break;
                case 4:
                    // placeholder
                    robot->set_pose(-58, 14, 235);
                    break;
                case 5:
                    // placeholder
                    robot->set_pose(58, 14, 145);
                    break;
                case 6:
                    robot->set_pose(-61, 0, 0);
                    break;
                case 7:
                    robot->set_pose(0, 0, 0);
                    break;
            }
        }
    }

    static void toolsEventHandler(lv_event_t * e){
        lv_event_code_t code = lv_event_get_code(e);
        lv_obj_t * obj = lv_event_get_target(e);
        if(code == LV_EVENT_CLICKED) {
            uint32_t id = lv_btnmatrix_get_selected_btn(obj);
            
            switch (id) {
                case 0:
                    if (color == BLUE) {
                        lv_label_set_text_fmt(colorLabel, "Color: Red");
                        lv_obj_set_style_bg_color(colorLabel, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);  // Set background color
                        lv_obj_set_style_border_color(colorLabel, lv_palette_darken(LV_PALETTE_RED, 2), LV_PART_MAIN);
                        color = RED;
                    } else {
                        lv_label_set_text_fmt(colorLabel, "Color: Blue");
                        lv_obj_set_style_bg_color(colorLabel, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);  // Set background color
                        lv_obj_set_style_border_color(colorLabel, lv_palette_darken(LV_PALETTE_BLUE, 2), LV_PART_MAIN);
                        color = BLUE;
                    }
                    break;
                case 1:
                    robot->poseSet = true;
                    robot->initialize_particle_filter();
                    robot->set_pose_mode(MCL);
                    break;
                case 2:
                    autonomous();
                    return;
                    if (robot->poseMode == ODOM) {
                        robot->set_pose_mode(MCL);
                    } else {
                        robot->set_pose_mode(ODOM);
                    }
                    break;
            }
        }
    }

    void createColorLabel() {
        colorLabel = lv_label_create(lv_scr_act());
        lv_label_set_text_fmt(colorLabel, "Color: Blue");

        // Enable background color
        lv_obj_set_style_bg_opa(colorLabel, LV_OPA_COVER, LV_PART_MAIN);  // Ensure opacity is set
        lv_obj_set_style_bg_color(colorLabel, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);  // Set background color

        // Optionally, set padding/border for better appearance
        lv_obj_set_style_pad_all(colorLabel, 10, LV_PART_MAIN);
        lv_obj_set_style_border_width(colorLabel, 2, LV_PART_MAIN);
        lv_obj_set_style_border_color(colorLabel, lv_palette_darken(LV_PALETTE_BLUE, 2), LV_PART_MAIN);

        lv_obj_align(colorLabel, LV_ALIGN_TOP_RIGHT, 0, 0);
    }

    void createSelectedLabel() {
        selectedLabel = lv_label_create(lv_scr_act());
        lv_label_set_text_fmt(selectedLabel, "Selected: %s", textMap[auton]);

        // Enable background color
        lv_obj_set_style_bg_opa(selectedLabel, LV_OPA_COVER, LV_PART_MAIN);  // Ensure opacity is set
        lv_obj_set_style_bg_color(selectedLabel, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);  // Set background color

        // Optionally, set padding/border for better appearance
        lv_obj_set_style_pad_all(selectedLabel, 10, LV_PART_MAIN);
        lv_obj_set_style_border_width(selectedLabel, 2, LV_PART_MAIN);
        lv_obj_set_style_border_color(selectedLabel, lv_palette_darken(LV_PALETTE_GREY, 2), LV_PART_MAIN);

        lv_obj_align(selectedLabel, LV_ALIGN_TOP_RIGHT, 0, 40);
    }

    void init(lib::Robot* bot, int hue, int default_auton, const char **autons){
        auton = 6;
        color = BLUE;
        robot = bot;

        lv_obj_t* background = lv_bar_create(lv_scr_act());

        lv_obj_t* autonsMatrix = lv_btnmatrix_create(lv_scr_act());
        lv_btnmatrix_set_map(autonsMatrix, btnmMap);

        lv_obj_t* toolsMatrix = lv_btnmatrix_create(lv_scr_act());
        lv_btnmatrix_set_map(toolsMatrix, toolsMap);
        lv_obj_set_size(toolsMatrix, toolsMatrix->class_p->width_def - 40, toolsMatrix->class_p->height_def);

        lv_obj_align(autonsMatrix, LV_ALIGN_BOTTOM_LEFT, 0, 0); 
        lv_obj_align(toolsMatrix, LV_ALIGN_BOTTOM_RIGHT, 0, 0); 
        lv_obj_align(background, LV_ALIGN_TOP_RIGHT, 0, 0);

        createColorLabel();
        createSelectedLabel();

        lv_obj_add_event_cb(autonsMatrix, autonsEventHandler, LV_EVENT_ALL, NULL);
        lv_obj_add_event_cb(toolsMatrix, toolsEventHandler, LV_EVENT_ALL, NULL);
    }
}