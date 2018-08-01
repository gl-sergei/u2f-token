extern void shutdown (void);
extern void set_backlight (int on);
extern int pbutton (void);
extern int joystick (void);

#define JOYSTICK_L(x) ((x) & 1)
#define JOYSTICK_R(x) ((x) & 2)
#define JOYSTICK_U(x) ((x) & 4)
#define JOYSTICK_D(x) ((x) & 8)
