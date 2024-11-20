#include <oled.h>
#include <oled.h>

int main(void)
{
    oled_init(OLED_DISP_ON);
    oled_clrscr();

    oled_charMode(DOUBLESIZE);
    oled_puts("  PROJEKT");

    oled_charMode(NORMALSIZE);

    oled_drawLine(0, 15, 120, 15, WHITE);

    oled_gotoxy(1, 3);
    oled_puts("Current:");

    oled_gotoxy(16, 3);
    oled_puts("[A]");

    oled_display();

    while (1) {
        ;
    }

    return 0;
}
