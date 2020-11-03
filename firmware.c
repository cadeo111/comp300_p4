#include <stdint.h>
#include <stdbool.h>


// a pointer to this is a null pointer, but the compiler does not
// know that because "sram" is a linker symbol from sections.lds.
extern uint32_t sram;

#define reg_spictrl (*(volatile uint32_t*)0x02000000)
#define reg_uart_clkdiv (*(volatile uint32_t*)0x02000004)
#define reg_uart_data (*(volatile uint32_t*)0x02000008)
#define reg_gpio (*(volatile uint32_t*)0x03000000)

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss,_heap_start;

uint32_t set_irq_mask(uint32_t mask); asm (
    ".global set_irq_mask\n"
    "set_irq_mask:\n"
    ".word 0x0605650b\n"
    "ret\n"
);

void main() {
    set_irq_mask(0xff);

    // zero out .bss section
    for (uint32_t *dest = &_sbss; dest < &_ebss;) {
        *dest++ = 0;
    }

    // switch to dual IO mode
    reg_spictrl = (reg_spictrl & ~0x007F0000) | 0x00400000;



    uint32_t second_counter = 0x0000;
    uint32_t alarm = 0x00000000;
    uint32_t minutes, min_tens, min_ones, seconds, sec_tens, sec_ones, display
    bool up_down = 0;
    bool second_toggle = 0;
    
    while (1) {
      while((second_toggle & 1) == (reg_gpio & 1));/// waits until change

      second_toggle = reg_gpio & 1; // LSB reg_gpio[0] is the second_toggle
      up_down = (reg_gpio & 0b10) >> 1; // reg_gpio[1] is the up_down switch

      if(up_down ==1)
      	second_counter += 1;
      else
	      second_counter -= 1;
        
      if (second_counter == 0x0008){ 
          alarm = 0xF0000000; 
      }

        minutes = ((second_counter / 60) % 60); //gets the number of minutes (<60)
        min_tens = minutes / 10;    //gets 10's digit of minute timer
        min_ones = minutes % 10;    //gets 1's digit of minute timer

        seconds = second_counter % 60;//gets number of seconds (<60)
        sec_tens = seconds / 10;    //gets 10's digit of seconds
        sec_ones = seconds % 10;    //gets 1's digit of seconds

        display = ((min_tens & 0x0F) << 12) | (min_ones << 8) | (sec_tens << 4) | sec_ones; //sets the display variable to each of the digits
        
        reg_gpio = (display & 0xFFFF) | alarm; //low order 16 bits will be displayed

      //**commented for minute timer reg_gpio = second_counter; // low order 16 bits will be displayed

  } // end of while(1)
} // end of main program
