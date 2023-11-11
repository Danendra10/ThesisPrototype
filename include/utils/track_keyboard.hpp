#ifndef TRACK_KEYBOARD_HPP_
#define TRACK_KEYBOARD_HPP_

#include <termios.h>
#include <sys/ioctl.h>

uint8_t game_status;       // this is the base robot game status that later will be multiplied by robot base action
uint8_t robot_base_action; // This is base action of robot, the state machine will be multplier by this

enum robot_state
{
    //---General Cmd
    status_iddle_0 = 0,
    status_iddle = 83,   // S | 0x53
    status_iddle_2 = 32, // Space | 0x20
    status_start = 115,  // s | 0x73

    //---Home Cmd
    status_preparation_kickoff_home = 75,     // K | 0x4B
    status_preparation_freekick_home = 70,    // F | 0x46
    status_preparation_goalkick_home = 71,    // G | 0x47
    status_preparation_cornerkick_home = 67,  // C | 0x43
    status_preparation_penaltykick_home = 80, // P | 0x50
    status_preparation_throwin_home = 84,     // T | 0x54

    //---All Cmd
    status_preparation_dropball = 78, // N | 0x4E
    status_callibration = 35,         // # | 0x23
    status_park = 76,                 // L | 0x4C

    //---Away Cmd
    status_preparation_kickoff_away = 107,     // k | 0x6B
    status_preparation_freekick_away = 102,    // f | 0x66
    status_preparation_goalkick_away = 103,    // g | 0x67
    status_preparation_cornerkick_away = 99,   // c | 0x63
    status_preparation_penaltykick_away = 112, // p | 0x70
    status_preparation_throwin_away = 116,     // t | 0x74

    //---Keyboard Manual
    keyboard_forward = 106,       // j | 0x6A
    keyboard_left = 98,           // b | 0x62
    keyboard_backward = 110,      // n | 0x6E
    keyboard_right = 109,         // m | 0x6D
    keyboard_right_rotation = 48, // 0 | 0x30
    keyboard_left_rotation = 57,  // 9 | 0x39

    //---Start home Cmds
    game_kickoff_home = status_preparation_kickoff_home + 128,
    game_freekick_home = status_preparation_freekick_home + 128,
    game_goalkick_home = status_preparation_goalkick_home + 128,
    game_cornerkick_home = status_preparation_cornerkick_home + 128,
    game_penaltykick_home = status_preparation_penaltykick_home + 128,
    game_throwin_home = status_preparation_throwin_home + 128,
    game_dropball = status_preparation_dropball + 128,

    //---Start away Cmds
    game_kickoff_away = status_preparation_kickoff_away + 128,
    game_freekick_away = status_preparation_freekick_away + 128,
    game_goalkick_away = status_preparation_goalkick_away + 128,
    game_cornerkick_away = status_preparation_cornerkick_away + 128,
    game_penaltykick_away = status_preparation_penaltykick_away + 128,
    game_throwin_away = status_preparation_throwin_away + 128,

    //---Callibration use 'o'
    game_callibration = 111,

    //---Game Status
    reset_action = 0,
    is_passing = 10,
    is_passing_inside_state = 15,
    is_receiving = 20,
    is_receiving_inside_state = 25,
    is_catching_ball = 30,
    pre_passing = 35,
};
int8_t kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized)
    {
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

void KeyboardHandler()
{
    static uint8_t prev_key = 0;
    if (kbhit() > 0)
    {
        char key = std::cin.get();

        switch (key)
        {
        case 'S':
            if (game_status > 127 && game_status <= 255)
                game_status -= 128;
            game_status = 0;
            break;
        case ' ':
            if (game_status > 127 && game_status <= 255)
                game_status -= 128;
            // game_status = 0;
            break;
        case 's':
            if (game_status > 0 && game_status <= 127)
                game_status += 128;

            break;
        case '$':
            game_status = 155;
            break;
        case '%':
            game_status = 156;
            break;
        case '^':
            game_status = 157;
            break;
        case '&':
            game_status = 158;
            break;
        case '#':
            game_status = status_callibration;
            break;
        case 'K':
            game_status = status_preparation_kickoff_home;
            break;
        case 'F':
            game_status = status_preparation_freekick_home;
            break;
        case 'C':
            game_status = status_preparation_cornerkick_home;
            break;
        case 'G':
            game_status = status_preparation_goalkick_home;
            break;
        case 'P':
            game_status = status_preparation_penaltykick_home;
            break;
        case 'T':
            game_status = status_preparation_throwin_home;
            break;
        case 'k':
            game_status = status_preparation_kickoff_away;
            break;
        case 'f':
            game_status = status_preparation_freekick_away;
            break;
        case 'c':
            game_status = status_preparation_cornerkick_away;
            break;
        case 'g':
            game_status = status_preparation_goalkick_away;
            break;
        case 'p':
            game_status = status_preparation_penaltykick_away;
            break;
        case 't':
            game_status = status_preparation_throwin_away;
            break;
        case 'j':
            game_status = keyboard_forward;
            break;
        case 'b':
            game_status = keyboard_left;
            break;
        case 'n':
            game_status = keyboard_backward;
            break;
        case 'm':
            game_status = keyboard_right;
            break;
        }
        robot_base_action = (key != 'S' && key != ' '); // Set base_act to 0 if key == S or space
    }
}
#endif