#include "ft8_ram.h"
#include "ft8_port.h"

ft8_ram_t *g_ft8_ram = NULL;

bool ft8_ram_init(void)
{
    if (g_ft8_ram == NULL)
    {
        g_ft8_ram = (ft8_ram_t *)ft8_port_calloc_large(1, sizeof(ft8_ram_t));
        if (g_ft8_ram == NULL)
        {
            return false;
        }
        ft8_fft1024_init();
    }
    return true;
}
