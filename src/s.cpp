#include "main.h"
#include "s.hpp"
#include "pros/misc.hpp"
#include "liblvgl/lvgl.h"
#include "intake.hpp"

using namespace pros;

Controller cm(E_CONTROLLER_MASTER);

LV_IMG_DECLARE(field);

const char* autonNames[] = {
    "RSAWP",     // 0
    "BSAWP",     // 1
    "R5p1wB",    // 2
    "R5p1wL",    // 3
    "R5p1aB",    // 4
    "R5p1aL",    // 5
    "R6B",       // 6
    "R6L",       // 7
    "B5p1wB",    // 8
    "B5p1wL",    // 9
    "B5p1aB",    // 10
    "B5p1aL",    // 11
    "B6B",       // 12
    "B6L",       // 13
    "R-6.1C",    // 14
    "R-6.1L",    // 15
    "R-6.1X",    // 16  (X = No Sweep)
    "R-6C",      // 17
    "R-6L",      // 18
    "R-6X",      // 19
    "B-6.1C",    // 20
    "B-6.1L",    // 21
    "B-6.1X",    // 22
    "B-6C",      // 23
    "B-6L",      // 24
    "B-6X"       // 25
};


lv_obj_t* redNegBtn;
lv_obj_t* redPosBtn;
lv_obj_t* blueNegBtn;
lv_obj_t* bluePosBtn;

// RED POS
lv_obj_t* redPos_5p1w_BakerBtn;
lv_obj_t* redPos_5p1w_LadderBtn;
lv_obj_t* redPos_5p1a_BakerBtn;
lv_obj_t* redPos_5p1a_LadderBtn;
lv_obj_t* redPos_6_BakerBtn;
lv_obj_t* redPos_6_LadderBtn;

// BLUE POS
lv_obj_t* bluePos_5p1w_BakerBtn;
lv_obj_t* bluePos_5p1w_LadderBtn;
lv_obj_t* bluePos_5p1a_BakerBtn;
lv_obj_t* bluePos_5p1a_LadderBtn;
lv_obj_t* bluePos_6_BakerBtn;
lv_obj_t* bluePos_6_LadderBtn;

// RED NEG
lv_obj_t* redNeg_6p1CornerBtn;
lv_obj_t* redNeg_6p1LadderBtn;
lv_obj_t* redNeg_6p1CornerNoSweepBtn;

lv_obj_t* redNeg_6CornerBtn;
lv_obj_t* redNeg_6LadderBtn;
lv_obj_t* redNeg_6CornerNoSweepBtn;

// BLUE NEG
lv_obj_t* blueNeg_6p1CornerBtn;
lv_obj_t* blueNeg_6p1LadderBtn;
lv_obj_t* blueNeg_6p1CornerNoSweepBtn;

lv_obj_t* blueNeg_6CornerBtn;
lv_obj_t* blueNeg_6LadderBtn;
lv_obj_t* blueNeg_6CornerNoSweepBtn;

// Solo Win Point
lv_obj_t* redNeg_sawpBtn;
lv_obj_t* blueNeg_sawpBtn;

lv_obj_t* titleLabel;

std::vector<lv_obj_t*> redNegButtons;
std::vector<lv_obj_t*> redPosButtons;
std::vector<lv_obj_t*> blueNegButtons;
std::vector<lv_obj_t*> bluePosButtons;
std::vector<lv_obj_t*> sawpButtons;

#define RED_NEG 0
#define RED_POS 1
#define BLUE_NEG 2
#define BLUE_POS 3

namespace sec {
    int auton;
    int color = RED;
    lib::Robot* robot;


    static void showButtons(std::vector<lv_obj_t*> buttons) {
        for (auto btn : buttons) {
            lv_obj_clear_flag(btn, LV_OBJ_FLAG_HIDDEN);
        }
    }
    static void hideButtons(std::vector<lv_obj_t*> buttons) {
        for (auto btn : buttons) {
            lv_obj_add_flag(btn, LV_OBJ_FLAG_HIDDEN);
        }
    }
    static void set_state(std::vector<lv_obj_t*> buttons, lv_state_t state) {
        for (auto btn : buttons) {
            lv_obj_add_state(btn, state);
        }
    }
    static void remove_state(std::vector<lv_obj_t*> buttons, lv_state_t state) {
        for (auto btn : buttons) {
            lv_obj_clear_state(btn, state);
        }
    }
    static void showAndHideOtherButtons(int show) {
        switch (show)
        {
        case RED_NEG:
            showButtons(redNegButtons);
            hideButtons(redPosButtons);
            hideButtons(blueNegButtons);
            hideButtons(bluePosButtons);
            break;
        case RED_POS:
            hideButtons(redNegButtons);
            showButtons(redPosButtons);
            hideButtons(blueNegButtons);
            hideButtons(bluePosButtons);
            break;
        case BLUE_NEG:
            hideButtons(redNegButtons);
            hideButtons(redPosButtons);
            showButtons(blueNegButtons);
            hideButtons(bluePosButtons);
            break;
        case BLUE_POS:
            hideButtons(redNegButtons);
            hideButtons(redPosButtons);
            hideButtons(blueNegButtons);
            showButtons(bluePosButtons);
            break;
        }
    }

    static bool equals(lv_obj_t *btn1, lv_obj_t *btn2) {
        return btn1->coords.x1 == btn2->coords.x1 && btn1->coords.y1 == btn2->coords.y1;
    }

    static void secondButtonsEventHandler(lv_event_t * e) {
        lv_obj_t *btn = lv_event_get_target(e);

        if (equals(btn, redNeg_sawpBtn)) {
            auton = 0;
        } else if (equals(btn, blueNeg_sawpBtn)) {
            auton = 1;
        } else if (equals(btn, redPos_5p1w_BakerBtn)) {
            auton = 2;
        } else if (equals(btn, redPos_5p1w_LadderBtn)) {
            auton = 3;
        } else if (equals(btn, redPos_5p1a_BakerBtn)) {
            auton = 4;
        } else if (equals(btn, redPos_5p1a_LadderBtn)) {
            auton = 5;
        } else if (equals(btn, redPos_6_BakerBtn)) {
            auton = 6;
        } else if (equals(btn, redPos_6_LadderBtn)) {
            auton = 7;
        } else if (equals(btn, bluePos_5p1w_BakerBtn)) {
            auton = 8;
        } else if (equals(btn, bluePos_5p1w_LadderBtn)) {
            auton = 9;
        } else if (equals(btn, bluePos_5p1a_BakerBtn)) {
            auton = 10;
        } else if (equals(btn, bluePos_5p1a_LadderBtn)) {
            auton = 11;
        } else if (equals(btn, bluePos_6_BakerBtn)) {
            auton = 12;
        } else if (equals(btn, bluePos_6_LadderBtn)) {
            auton = 13;
        } else if (equals(btn, redNeg_6p1CornerBtn)) {
            auton = 14;
        } else if (equals(btn, redNeg_6p1LadderBtn)) {
            auton = 15;
        } else if (equals(btn, redNeg_6p1CornerNoSweepBtn)) {
            auton = 16;
        } else if (equals(btn, redNeg_6CornerBtn)) {
            auton = 17;
        } else if (equals(btn, redNeg_6LadderBtn)) {
            auton = 18;
        } else if (equals(btn, redNeg_6CornerNoSweepBtn)) {
            auton = 19;
        } else if (equals(btn, blueNeg_6p1CornerBtn)) {
            auton = 20;
        } else if (equals(btn, blueNeg_6p1LadderBtn)) {
            auton = 21;
        } else if (equals(btn, blueNeg_6p1CornerNoSweepBtn)) {
            auton = 22;
        } else if (equals(btn, blueNeg_6CornerBtn)) {
            auton = 23;
        } else if (equals(btn, blueNeg_6LadderBtn)) {
            auton = 24;
        } else if (equals(btn, blueNeg_6CornerNoSweepBtn)) {
            auton = 25;
        }
        
        lv_label_set_text_fmt(titleLabel, "Selected Auton: %s", autonNames[auton]);
    }

    static void makeButtons(int x, std::vector<lv_obj_t*> &buttons, int type) {
        int i = 0;
        int realIndex = 0;
        int j = 0;
        for (auto button : buttons) {
            lv_obj_set_size(button, 70, 40);
            lv_obj_set_style_pad_left(button, 2, LV_PART_MAIN);  // Or 0 for no padding
            lv_obj_set_style_pad_right(button, 2, LV_PART_MAIN);  // Or 0 for no padding

            lv_obj_t *label = lv_label_create(button);
            lv_label_set_text(label, "no auto");
            auto align = LV_ALIGN_LEFT_MID;
            if (type == BLUE_NEG || type == BLUE_POS) {
                align = LV_ALIGN_RIGHT_MID;
            }
            lv_obj_align(button, align, x + 72 * i, -40 + 42 * j);
            i++;
            if (i % 2 == 0) {
                i = 0;
                j++;
            }

            realIndex++;

            if (type == RED_POS || type == RED_NEG) {
                lv_obj_set_style_bg_color(button, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
            } else {
                lv_obj_set_style_bg_color(button, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
            }

            lv_obj_add_event_cb(button, secondButtonsEventHandler, LV_EVENT_CLICKED, NULL);
        }
    }

    static void mainButtonsEventHandler(lv_event_t * e){
        lv_obj_t *btn = lv_event_get_target(e);
        
        if (equals(btn, redNegBtn)) {
            std::cout << "Red Neg" << std::endl;
            showAndHideOtherButtons(RED_NEG);

            lv_obj_add_state(redNegBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(redPosBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(blueNegBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(bluePosBtn, LV_STATE_CHECKED);

            makeButtons(8, redNegButtons, RED_NEG);
        } else if (btn == redPosBtn) {
            std::cout << "Red Pos" << std::endl;
            showAndHideOtherButtons(RED_POS);

            lv_obj_add_state(redPosBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(redNegBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(blueNegBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(bluePosBtn, LV_STATE_CHECKED);

            makeButtons(8, redPosButtons, RED_POS);
        } else if (btn == blueNegBtn) {
            std::cout << "Blue Neg" << std::endl;
            showAndHideOtherButtons(BLUE_NEG);

            lv_obj_add_state(blueNegBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(redNegBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(redPosBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(bluePosBtn, LV_STATE_CHECKED);

            makeButtons(-80, blueNegButtons, BLUE_NEG);
        } else if (btn == bluePosBtn) {
            std::cout << "Blue Pos" << std::endl;
            showAndHideOtherButtons(BLUE_POS);

            lv_obj_add_state(bluePosBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(redNegBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(redPosBtn, LV_STATE_CHECKED);
            lv_obj_clear_state(blueNegBtn, LV_STATE_CHECKED);

            makeButtons(-80, bluePosButtons, BLUE_POS);
        } else {
            std::cout << "Unknown Button Pressed" << std::endl;
        }
    }

    void init(lib::Robot* bot, int hue, int default_auton, const char **autons){
        auton = 9;
        color = RED;
        robot = bot;

        lv_scr_act();

        lv_obj_t* background = lv_obj_create(lv_scr_act());

        // 2. Set it to full screen
        lv_obj_set_size(background, LV_PCT(100), LV_PCT(100));

        lv_obj_t* center_img = lv_img_create(lv_scr_act());
        lv_img_set_src(center_img, &field);
        lv_obj_align(center_img, LV_ALIGN_TOP_MID, 0, 25);

        titleLabel = lv_label_create(lv_scr_act());

        redNegBtn = lv_btn_create(lv_scr_act());
        //--
        redNeg_6p1CornerBtn = lv_btn_create(lv_scr_act());
        redNeg_6p1LadderBtn = lv_btn_create(lv_scr_act());
        redNeg_6p1CornerNoSweepBtn = lv_btn_create(lv_scr_act());
        redNeg_6CornerBtn = lv_btn_create(lv_scr_act());
        redNeg_6LadderBtn = lv_btn_create(lv_scr_act());
        redNeg_6CornerNoSweepBtn = lv_btn_create(lv_scr_act());
        redNegButtons.push_back(redNeg_6p1CornerBtn);
        redNegButtons.push_back(redNeg_6p1LadderBtn);
        redNegButtons.push_back(redNeg_6p1CornerNoSweepBtn);
        redNegButtons.push_back(redNeg_6CornerBtn);
        redNegButtons.push_back(redNeg_6LadderBtn);
        redNegButtons.push_back(redNeg_6CornerNoSweepBtn);

        redPosBtn = lv_btn_create(lv_scr_act());
        //--
        redPos_5p1w_BakerBtn = lv_btn_create(lv_scr_act());
        redPos_5p1w_LadderBtn = lv_btn_create(lv_scr_act());
        redPos_5p1a_BakerBtn = lv_btn_create(lv_scr_act());
        redPos_5p1a_LadderBtn = lv_btn_create(lv_scr_act());
        redPos_6_BakerBtn = lv_btn_create(lv_scr_act());
        redPos_6_LadderBtn = lv_btn_create(lv_scr_act());
        redPosButtons.push_back(redPos_5p1w_BakerBtn);
        redPosButtons.push_back(redPos_5p1w_LadderBtn);
        redPosButtons.push_back(redPos_5p1a_BakerBtn);
        redPosButtons.push_back(redPos_5p1a_LadderBtn);
        redPosButtons.push_back(redPos_6_BakerBtn);
        redPosButtons.push_back(redPos_6_LadderBtn);

        blueNegBtn = lv_btn_create(lv_scr_act());
        //--
        blueNeg_6p1CornerBtn = lv_btn_create(lv_scr_act());
        blueNeg_6p1LadderBtn = lv_btn_create(lv_scr_act());
        blueNeg_6p1CornerNoSweepBtn = lv_btn_create(lv_scr_act());
        blueNeg_6CornerBtn = lv_btn_create(lv_scr_act());
        blueNeg_6LadderBtn = lv_btn_create(lv_scr_act());
        blueNeg_6CornerNoSweepBtn = lv_btn_create(lv_scr_act());
        blueNegButtons.push_back(blueNeg_6p1CornerBtn);
        blueNegButtons.push_back(blueNeg_6p1LadderBtn);
        blueNegButtons.push_back(blueNeg_6p1CornerNoSweepBtn);
        blueNegButtons.push_back(blueNeg_6CornerBtn);
        blueNegButtons.push_back(blueNeg_6LadderBtn);
        blueNegButtons.push_back(blueNeg_6CornerNoSweepBtn);

        bluePosBtn = lv_btn_create(lv_scr_act());
        //--
        bluePos_5p1w_BakerBtn = lv_btn_create(lv_scr_act());
        bluePos_5p1w_LadderBtn = lv_btn_create(lv_scr_act());
        bluePos_5p1a_BakerBtn = lv_btn_create(lv_scr_act());
        bluePos_5p1a_LadderBtn = lv_btn_create(lv_scr_act());
        bluePos_6_BakerBtn = lv_btn_create(lv_scr_act());
        bluePos_6_LadderBtn = lv_btn_create(lv_scr_act());
        bluePosButtons.push_back(bluePos_5p1w_BakerBtn);
        bluePosButtons.push_back(bluePos_5p1w_LadderBtn);
        bluePosButtons.push_back(bluePos_5p1a_BakerBtn);
        bluePosButtons.push_back(bluePos_5p1a_LadderBtn);
        bluePosButtons.push_back(bluePos_6_BakerBtn);
        bluePosButtons.push_back(bluePos_6_LadderBtn);

        // Solo Win Point
        redNeg_sawpBtn = lv_btn_create(lv_scr_act());
        blueNeg_sawpBtn = lv_btn_create(lv_scr_act());
        sawpButtons.push_back(redNeg_sawpBtn);
        sawpButtons.push_back(blueNeg_sawpBtn);
        
        hideButtons(redNegButtons);
        hideButtons(redPosButtons);
        hideButtons(blueNegButtons);
        hideButtons(bluePosButtons);
        hideButtons(sawpButtons);
    
        lv_label_set_text(titleLabel, "Select Auton:");
        lv_obj_align(titleLabel, LV_ALIGN_TOP_MID, 0, 5);
    
        lv_obj_set_width(redNegBtn, 100);
        lv_obj_set_height(redNegBtn, LV_SIZE_CONTENT);
        lv_obj_t *redNegBtnLabel = lv_label_create(redNegBtn);
        lv_label_set_text(redNegBtnLabel, "Red Neg");
        lv_obj_align(redNegBtn, LV_ALIGN_TOP_LEFT, 8, 15);
        lv_obj_add_event_cb(redNegBtn, mainButtonsEventHandler, LV_EVENT_CLICKED, NULL);
    
        lv_obj_set_width(redPosBtn, 100);
        lv_obj_set_height(redPosBtn, LV_SIZE_CONTENT);
        lv_obj_t *redPosBtnLabel = lv_label_create(redPosBtn);
        lv_label_set_text(redPosBtnLabel, "Red Pos");
        lv_obj_align(redPosBtn, LV_ALIGN_BOTTOM_LEFT, 8, -15);
        lv_obj_add_event_cb(redPosBtn, mainButtonsEventHandler, LV_EVENT_CLICKED, NULL);
    
        lv_obj_set_width(blueNegBtn, 100);
        lv_obj_set_height(blueNegBtn, LV_SIZE_CONTENT);
        lv_obj_t *blueNegBtnLabel = lv_label_create(blueNegBtn);
        lv_label_set_text(blueNegBtnLabel, "Blue Neg");
        lv_obj_align(blueNegBtn, LV_ALIGN_TOP_RIGHT, -8, 15);
        lv_obj_add_event_cb(blueNegBtn, mainButtonsEventHandler, LV_EVENT_CLICKED, NULL);
    
        lv_obj_set_width(bluePosBtn, 100);
        lv_obj_set_height(bluePosBtn, LV_SIZE_CONTENT);
        lv_obj_t *bluePosBtnLabel = lv_label_create(bluePosBtn);
        lv_label_set_text(bluePosBtnLabel, "Blue Pos");
        lv_obj_align(bluePosBtn, LV_ALIGN_BOTTOM_RIGHT, -8, -15);
        lv_obj_add_event_cb(bluePosBtn, mainButtonsEventHandler, LV_EVENT_CLICKED, NULL);

        int i = 0;
        for (auto btn : sawpButtons) {
            lv_obj_set_size(btn, 70, 40);
            lv_obj_set_style_pad_left(btn, 2, LV_PART_MAIN);  // Or 0 for no padding
            lv_obj_set_style_pad_right(btn, 2, LV_PART_MAIN);  // Or 0 for no padding

            auto name = "R SAWP";
            if (i == 1) {
                name = "B SAWP";
            }

            int mult = i == 0 ? 1 : -1;
            lv_obj_t *label = lv_label_create(btn);
            lv_label_set_text(label, name);
            lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, -30 + i * 72, 0);

            lv_obj_clear_flag(btn, LV_OBJ_FLAG_HIDDEN);

            if (i == 0) {
                lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
            } else {
                lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
            }

            lv_obj_add_event_cb(btn, secondButtonsEventHandler, LV_EVENT_CLICKED, NULL);

            i++;
        }

        redNegBtn = redNegBtn;
        redPosBtn = redPosBtn;
        blueNegBtn = blueNegBtn;
        bluePosBtn = bluePosBtn;
    }
}